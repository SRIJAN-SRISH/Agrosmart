/**
 * health_task.cpp — AgroSmart Node B System Supervisor
 *
 * Responsibilities (runs every HEALTH_CHECK_INTERVAL_MS):
 *   1. Heap Guardian       — WARN at 32 KB; reboot after 3× CRITICAL at 16 KB
 *   2. Bus Pinger          — live I2C-0, I2C-1, SD, WiFi, rain GPIO probes
 *   3. Task Watchdog       — per-task ping timeout; reboot after 2× stale
 *   4. Node C Battery Gate — hysteresis 11.2 V ↓ / 11.8 V ↑; dropout = gate close
 *   5. Queue Depth Monitor — fill % for all 9 queues
 *   6. Stack Watermarks    — logged once at 60 s uptime
 *   7. SD Storage Guard    — flag critical when free < SD_STORAGE_CRITICAL_PCT
 *   8. Controlled Reboot   — log reason → flush SD → esp_restart()
 *   9. UART Dispatch       — PKT_HEALTH frame to Pi Pico over Serial1
 *
 * SPI bus: health_task does NOT drive SPI directly. It calls lockSPI()
 * only to read SD.cardSize() / SD.usedBytes() — non-destructive reads
 * that complete in <5 ms. All LoRa SPI is owned by lora_task.
 */

#include "health_task.h"
#include "cloud_task.h"
#include "config.h"
#include "queues.h"
#include "sync.h"
#include "structs.h"
#include "enums.h"
#include "logger.h"

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <SD.h>
#include <Preferences.h>
#include <freertos/semphr.h>
#include <esp_system.h>

// ─────────────────────────────────────────────────────────────
// Pin references — declared in pins.h
// ─────────────────────────────────────────────────────────────
#include "pins.h"

// ─────────────────────────────────────────────────────────────
// I2C probe addresses
// ─────────────────────────────────────────────────────────────
#define I2C0_BMP280_ADDR   0x76
#define I2C0_RTC_ADDR      0x68
#define I2C1_PCA9685_ADDR  0x40

// ─────────────────────────────────────────────────────────────
// Node liveness timeouts
// ─────────────────────────────────────────────────────────────
#define NODE_A_TIMEOUT_MS  (10UL * 60UL * 1000UL)   // 10 min
#define NODE_C_TIMEOUT_MS  ( 5UL * 60UL * 1000UL)   //  5 min

// ─────────────────────────────────────────────────────────────
// Queue capacities — must match Agrosmart_nodeB.ino xQueueCreate() calls
// Index order: sensor, command, feedback, log, decision,
//              heartbeat, cloud, sd, irrigationLog
// ─────────────────────────────────────────────────────────────
static const uint8_t QUEUE_CAPACITY[HEALTH_QUEUE_COUNT] = {
    10,   // sensorQueue
    10,   // commandQueue
    10,   // feedbackQueue
    50,   // logQueue
    50,   // sdLogQueue
     5,   // heartbeatQueue
    50,   // cloudQueue (25-hour WiFi outage buffer at 30-min poll)
    10,   // sdQueue
    10    // irrigationLogQueue
};

static const uint8_t QUEUE_WARN_PCT[HEALTH_QUEUE_COUNT] = {
    80,   // sensorQueue
    80,   // commandQueue
    80,   // feedbackQueue
    70,   // logQueue
    70,   // sdLogQueue
   100,   // heartbeatQueue (fire-and-forget; full is expected under load)
    80,   // cloudQueue
    80,   // sdQueue
    80    // irrigationLogQueue
};

// ─────────────────────────────────────────────────────────────
// Global state — g_taskHandles is defined in Agrosmart_nodeB.ino
// ─────────────────────────────────────────────────────────────
static SystemHealthState  g_health;
static SemaphoreHandle_t  g_healthMutex = nullptr;

// Ping timestamps — written by healthPing(), read by vHealthTask
static volatile uint32_t  g_taskPingMs[HTASK_COUNT] = { 0 };

// Battery gate — hysteresis state
static volatile bool      g_nodeCBatteryOk    = true;   // safe default
static volatile bool      g_sdStorageCritical = false;

// Stack watermark logged once
static bool               g_stackLogged = false;

// ─────────────────────────────────────────────────────────────
// NVS helpers — namespace "health", keys "reboot_cnt" / "reboot_rsn"
// ─────────────────────────────────────────────────────────────
static Preferences nvs;

static uint32_t nvsReadRebootCount()
{
    nvs.begin("health", false);
    uint32_t c = nvs.getUInt("reboot_cnt", 0);
    nvs.end();
    return c;
}

static void nvsWriteReboot(uint32_t count, uint8_t reason)
{
    nvs.begin("health", false);
    nvs.putUInt("reboot_cnt",  count);
    nvs.putUChar("reboot_rsn", reason);
    nvs.end();
}

static uint8_t nvsReadRebootReason()
{
    nvs.begin("health", true);
    uint8_t r = nvs.getUChar("reboot_rsn", REBOOT_UNKNOWN);
    nvs.end();
    return r;
}

// ─────────────────────────────────────────────────────────────
// CRC-8 (XOR accumulation) for UART framing
// ─────────────────────────────────────────────────────────────
static uint8_t crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) crc ^= data[i];
    return crc;
}

// ─────────────────────────────────────────────────────────────
// UART dispatch — generic framed packet to Pi Pico over Serial2 (UART2)
// Frame: [0x7E][TYPE][LEN_LO][LEN_HI][DATA...][CRC8][0x7F]
// ─────────────────────────────────────────────────────────────
static void dispatchFrame(uint8_t type, const uint8_t *data, uint16_t len)
{
    Serial2.write(PKT_START);
    Serial2.write(type);
    Serial2.write((uint8_t)(len & 0xFF));
    Serial2.write((uint8_t)(len >> 8));

    for (uint16_t i = 0; i < len; i++) Serial2.write(data[i]);

    Serial2.write(crc8(data, len));
    Serial2.write(PKT_END);
}

static void dispatchHealthPacket()
{
    dispatchFrame(PKT_HEALTH, (const uint8_t *)&g_health, (uint16_t)sizeof(SystemHealthState));
}

// Called by decision_task on every processed sensor record — PKT_SENSOR (0x02)
void dispatchSensorPacket(const MasterSensorRecord &r)
{
    dispatchFrame(PKT_SENSOR, (const uint8_t *)&r, (uint16_t)sizeof(MasterSensorRecord));
}

// Called by decision_task on every finalized decision — PKT_DECISION (0x03)
void dispatchDecisionPacket(const IrrigationDecision &d)
{
    dispatchFrame(PKT_DECISION, (const uint8_t *)&d, (uint16_t)sizeof(IrrigationDecision));
}

void dispatchConfigPacket(const FarmProfile &p)
{
    dispatchFrame(PKT_CONFIG, (const uint8_t *)&p, (uint16_t)sizeof(FarmProfile));
}

// ─────────────────────────────────────────────────────────────
// I2C bus probe — returns true if device ACKs at address
// ─────────────────────────────────────────────────────────────
static bool probeI2C(TwoWire &bus, uint8_t addr)
{
    bus.beginTransmission(addr);
    return (bus.endTransmission() == 0);
}

// ─────────────────────────────────────────────────────────────
// Log a health event to logQueue (non-blocking, best-effort)
// ─────────────────────────────────────────────────────────────
static void healthLog(uint8_t severity, const char *msg)
{
    SystemEventLog entry;
    memset(&entry, 0, sizeof(entry));
    entry.severity     = severity;
    entry.boot_time_ms = (uint32_t)millis();
    strncpy(entry.source,  "HEALTH_TASK", sizeof(entry.source)  - 1);
    strncpy(entry.message, msg,           sizeof(entry.message) - 1);
    pushLog(entry);
}

// ─────────────────────────────────────────────────────────────
// Controlled reboot — persists metadata, gives sd_task 500 ms,
// then calls esp_restart()
// ─────────────────────────────────────────────────────────────
static void controlledReboot(uint8_t reason)
{
    LOG_ERROR("HEALTH", "*** CONTROLLED REBOOT — reason=0x%02X ***", reason);
    healthLog(ERR_LEVEL_4_CATASTROPHIC, "Controlled reboot initiated");

    uint32_t nextCount = g_health.reboot_count + 1;
    nvsWriteReboot(nextCount, reason);

    // Give sd_task a window to flush whatever it has in flight
    vTaskDelay(pdMS_TO_TICKS(500));

    LOG_INFO("HEALTH", "Restarting now.");
    esp_restart();
}

// ─────────────────────────────────────────────────────────────
// Queue depth — returns 0–100 %
// ─────────────────────────────────────────────────────────────
static uint8_t queueFillPct(QueueHandle_t q, uint8_t capacity)
{
    if (q == nullptr || capacity == 0) return 0;
    UBaseType_t waiting = uxQueueMessagesWaiting(q);
    return (uint8_t)((waiting * 100U) / capacity);
}

// ─────────────────────────────────────────────────────────────
// Build bus_status_mask from individual bools
// ─────────────────────────────────────────────────────────────
static uint8_t buildBusMask(const SystemHealthState &h)
{
    uint8_t m = 0;
    if (h.i2c0_ok)      m |= (1 << BUS_BIT_I2C0);
    if (h.i2c1_ok)      m |= (1 << BUS_BIT_I2C1);
    if (h.hspi_sd_ok)   m |= (1 << BUS_BIT_HSPI);
    if (h.vspi_lora_ok) m |= (1 << BUS_BIT_VSPI);
    if (h.rain_gpio_ok) m |= (1 << BUS_BIT_RAIN);
    if (h.wifi_ok)      m |= (1 << BUS_BIT_WIFI);
    if (h.node_a_alive) m |= (1 << BUS_BIT_NODE_A);
    if (h.node_c_alive) m |= (1 << BUS_BIT_NODE_C);
    return m;
}

// ─────────────────────────────────────────────────────────────
// Build global_health_flag — OR of all active fault bits
// Re-uses the enums.h HEALTH_NODEB_* constants for the MasterSensorRecord flag byte.
// ─────────────────────────────────────────────────────────────
static uint8_t buildGlobalFlag(const SystemHealthState &h)
{
    uint8_t f = 0x00;
    if (!h.i2c0_ok)           f |= HEALTH_NODEB_BMP280_FAULT;
    if (!h.i2c0_ok)           f |= HEALTH_NODEB_RTC_FAULT;
    if (!h.hspi_sd_ok)        f |= HEALTH_NODEB_SD_FAULT;
    if (h.sd_storage_critical) f |= HEALTH_NODEB_SD_FAULT;
    if (h.heap_severity >= HEAP_WARN) f |= 0x08;   // internal heap warning bit
    if (!h.nodec_battery_ok)  f |= 0x40;           // battery gate closed bit
    return f;
}

// ─────────────────────────────────────────────────────────────
// printHealthState — full Serial dump (useful for UART debug session)
// ─────────────────────────────────────────────────────────────
void printHealthState(const SystemHealthState &h)
{
    LOG_DEBUG("HEALTH", "=== SystemHealthState Dump ===");
    LOG_DEBUG("HEALTH", "uptime_s: %lu | reboot_cnt: %lu | reason: 0x%02X | fw: %s", (unsigned long)h.uptime_s, (unsigned long)h.reboot_count, h.reboot_reason_last, h.firmware_version);
    LOG_DEBUG("HEALTH", "heap_free: %lu | heap_min: %lu | severity: %u", (unsigned long)h.heap_free_bytes, (unsigned long)h.heap_free_min_ever, h.heap_severity);
    LOG_DEBUG("HEALTH", "bus_mask: 0x%02X | i2c0:%d i2c1:%d sd:%d lora:%d rain:%d wifi:%d(rssi:%d)", h.bus_status_mask, h.i2c0_ok, h.i2c1_ok, h.hspi_sd_ok, h.vspi_lora_ok, h.rain_gpio_ok, h.wifi_ok, h.wifi_rssi);
    LOG_DEBUG("HEALTH", "node_a:%d(last:%lu) rssi:%d | node_c:%d(last:%lu) rssi:%d", h.node_a_alive, h.node_a_last_seen_ms ? millis() - h.node_a_last_seen_ms : 0UL, h.node_a_rssi_last, h.node_c_alive, h.node_c_last_seen_ms ? millis() - h.node_c_last_seen_ms : 0UL, h.node_c_rssi_last);
    LOG_DEBUG("HEALTH", "nodec_batt: %.2fV ok:%d seen:%d", h.nodec_battery_v, h.nodec_battery_ok, h.nodec_seen);
    LOG_DEBUG("HEALTH", "task_mask: 0x%02X", h.task_alive_mask);
    const char *tnames[HTASK_COUNT] = {"LORA","UART","DEC","SD","CLOUD","ENV"};
    for (int i = 0; i < HTASK_COUNT; i++) {
        LOG_DEBUG("HEALTH", "  [%d] %-6s min=%u B %s", i, tnames[i], h.task_stack_min[i], (h.task_alive_mask & (1 << i)) ? "ALIVE" : "STALE");
    }
    const char *qnames[HEALTH_QUEUE_COUNT] = {"sensor","command","feedback","log","decision","heartbeat","cloud","sd","irrigLog"};
    for (int i = 0; i < HEALTH_QUEUE_COUNT; i++) {
        LOG_DEBUG("HEALTH", "  Q %-10s : %3u%%", qnames[i], h.queue_fill_pct[i]);
    }
    LOG_DEBUG("HEALTH", "sd_total: %luMB | sd_used: %luMB | used_pct: %u%% | crit: %d", (unsigned long)h.sd_total_mb, (unsigned long)h.sd_used_mb, h.sd_used_pct, h.sd_storage_critical);
    LOG_DEBUG("HEALTH", "global_health_flag: 0x%02X", h.global_health_flag);
}

// ═════════════════════════════════════════════════════════════
// Public API
// ═════════════════════════════════════════════════════════════

void healthPing(HealthTaskId id)
{
    if (id < HTASK_COUNT)
        g_taskPingMs[id] = millis();
}

void reportNodeASeen(int8_t rssi)
{
    if (xSemaphoreTake(g_healthMutex, pdMS_TO_TICKS(10)) == pdPASS) {
        g_health.node_a_last_seen_ms = millis();
        g_health.node_a_rssi_last    = rssi;
        xSemaphoreGive(g_healthMutex);
    }
}

void reportNodeCSeen(float battV, uint8_t statusFlag, int8_t rssi)
{
    if (xSemaphoreTake(g_healthMutex, pdMS_TO_TICKS(10)) == pdPASS) {
        g_health.node_c_last_seen_ms = millis();
        g_health.node_c_rssi_last    = rssi;
        g_health.nodec_battery_v     = battV;
        g_health.nodec_status_flag   = statusFlag;
        g_health.nodec_seen          = true;
        xSemaphoreGive(g_healthMutex);
    }

    // Hysteresis gate — outside mutex (atomic bool write on ESP32)
    if (battV < BATTERY_LOW_VOLTAGE) {
        g_nodeCBatteryOk = false;
        LOG_WARN("HEALTH", "Node C battery %.2f V — GATE CLOSED (irrigation blocked)", battV);
        healthLog(ERR_LEVEL_3_CRITICAL, "Node C battery below threshold — irrigation blocked");
    } else if (!g_nodeCBatteryOk && battV >= BATTERY_RECOVERY_VOLTAGE) {
        g_nodeCBatteryOk = true;
        LOG_INFO("HEALTH", "Node C battery %.2f V — GATE OPEN (recovered)", battV);
        healthLog(ERR_LEVEL_1_INFO, "Node C battery recovered — irrigation unblocked");
    }
}

bool isNodeCBatteryOk()    { return g_nodeCBatteryOk; }
bool isSdStorageCritical() { return g_sdStorageCritical; }

SystemHealthState getSystemHealth()
{
    SystemHealthState copy;
    if (g_healthMutex && xSemaphoreTake(g_healthMutex, pdMS_TO_TICKS(50)) == pdPASS) {
        memcpy(&copy, &g_health, sizeof(copy));
        xSemaphoreGive(g_healthMutex);
    } else {
        memset(&copy, 0, sizeof(copy));
    }
    return copy;
}

// ═════════════════════════════════════════════════════════════
// vHealthTask — main FreeRTOS task (Core 0, Priority 1)
// ═════════════════════════════════════════════════════════════
void vHealthTask(void *pvParameters)
{
    (void)pvParameters;

    LOG_INFO("HEALTH", "Task starting...");

    // ── Create health mutex ──────────────────────────────────
    g_healthMutex = xSemaphoreCreateMutex();
    if (!g_healthMutex) {
        LOG_ERROR("HEALTH", "FATAL: mutex alloc failed. Task halted.");
        vTaskDelete(nullptr);
        return;
    }

    // NOTE: no digital rain-gauge GPIO init here — the real hardware
    // (SEN0575, environment_task.cpp) is I2C-based, on the same Wire1 bus
    // as the PCA9685. RAIN_DT_PIN/RAIN_CR_PIN were leftover from an earlier
    // pulse-counting rain gauge design that was superseded; pins.h never
    // defined them and nothing wires them today. See the rain_gpio_ok
    // probe below, which was updated to match.

    // Pi Pico UART2 link is opened once in setup() before any task starts —
    // see Agrosmart_nodeB.ino.

    // ── Load NVS boot metadata ───────────────────────────────
    memset(&g_health, 0, sizeof(g_health));
    g_health.reboot_count       = nvsReadRebootCount();
    g_health.reboot_reason_last = nvsReadRebootReason();
    strncpy(g_health.firmware_version, FIRMWARE_VERSION,
            sizeof(g_health.firmware_version) - 1);
    g_health.heap_free_min_ever = ESP.getFreeHeap();
    g_nodeCBatteryOk = true;   // safe default — gate open until first heartbeat

    // Stamp all ping slots to now so tasks get a full grace period on boot
    uint32_t now = millis();
    for (int i = 0; i < HTASK_COUNT; i++) g_taskPingMs[i] = now;

    LOG_INFO("HEALTH", "Boot #%lu  last_reason=0x%02X  fw=%s",
             (unsigned long)g_health.reboot_count,
             g_health.reboot_reason_last,
             g_health.firmware_version);

    // ── Main supervisor loop ─────────────────────────────────
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(HEALTH_CHECK_INTERVAL_MS));

        now = millis();

        if (xSemaphoreTake(g_healthMutex, pdMS_TO_TICKS(200)) != pdPASS) {
            LOG_WARN("HEALTH", "mutex timeout — skipping cycle");
            continue;
        }

        // ── 1. Uptime ────────────────────────────────────────
        g_health.uptime_s = now / 1000UL;

        // ── 2. Heap Guardian ─────────────────────────────────
        uint32_t freeHeap = ESP.getFreeHeap();
        g_health.heap_free_bytes = freeHeap;
        if (freeHeap < g_health.heap_free_min_ever)
            g_health.heap_free_min_ever = freeHeap;

        if (freeHeap < HEAP_CRITICAL_THRESHOLD) {
            g_health.heap_severity = HEAP_CRITICAL;
            g_health.consecutive_critical_heap++;
            LOG_ERROR("HEALTH", "CRITICAL heap x%u — %lu bytes free",
                      g_health.consecutive_critical_heap, (unsigned long)freeHeap);
            healthLog(ERR_LEVEL_3_CRITICAL, "Heap critical");
        } else if (freeHeap < HEAP_WARNING_THRESHOLD) {
            g_health.heap_severity = HEAP_WARN;
            g_health.consecutive_critical_heap = 0;
            LOG_WARN("HEALTH", "WARN heap — %lu bytes free", (unsigned long)freeHeap);
            healthLog(ERR_LEVEL_2_OPERATIONAL, "Heap warning");
        } else {
            g_health.heap_severity = HEAP_OK;
            g_health.consecutive_critical_heap = 0;
        }

        // ── 3. Hardware Bus Pinging ───────────────────────────
        // I2C-0: BMP280 + RTC — healthy if EITHER device ACKs
        g_health.i2c0_ok = probeI2C(Wire,  I2C0_BMP280_ADDR) ||
                            probeI2C(Wire,  I2C0_RTC_ADDR);

        // I2C-1: PCA9685 PWM driver
        g_health.i2c1_ok = probeI2C(Wire1, I2C1_PCA9685_ADDR);

        // HSPI — SD size reads (brief lock; non-destructive)
        if (lockSPI(100)) {
            uint64_t cardSz = SD.cardSize();
            g_health.hspi_sd_ok = (cardSz > 0);
            if (g_health.hspi_sd_ok) {
                uint64_t usedBytes = SD.usedBytes();
                g_health.sd_total_mb = (uint32_t)(cardSz    / (1024ULL * 1024ULL));
                g_health.sd_used_mb  = (uint32_t)(usedBytes / (1024ULL * 1024ULL));
                g_health.sd_used_pct = (g_health.sd_total_mb > 0)
                    ? (uint8_t)((g_health.sd_used_mb * 100UL) / g_health.sd_total_mb) : 0;
                bool crit = (g_health.sd_total_mb > 0) &&
                            ((100UL - g_health.sd_used_pct) < (uint8_t)SD_STORAGE_CRITICAL_PCT);
                g_health.sd_storage_critical = crit;
                g_sdStorageCritical = crit;
            } else {
                g_health.sd_storage_critical = false;
                g_sdStorageCritical = false;
            }
            unlockSPI();
        }

        // Rain gauge (SEN0575) is I2C-based on Wire1, not a digital GPIO —
        // it shares that bus with the PCA9685. This is an interim proxy
        // (bus-level, not sensor-specific): if I2C-1 is reachable, the
        // rain sensor's transport layer is at least alive. A dedicated
        // probe of the SEN0575's own I2C address would be more precise,
        // but this is honest about what's actually being checked today,
        // unlike the old GPIO pins it replaces, which checked nothing —
        // pins.h never defined them.
        g_health.rain_gpio_ok = g_health.i2c1_ok;

        // WiFi — read cached state set by cloud_task to avoid driver contention
        g_health.wifi_ok   = g_wifiConnected;
        g_health.wifi_rssi = g_health.wifi_ok ? (int8_t)WiFi.RSSI() : 0;

        // LoRa — inferred from LORA task liveness (no direct SPI probe here)
        g_health.vspi_lora_ok = (g_health.task_alive_mask & (1 << HTASK_LORA)) != 0;

        // ── 4. Node Liveness ──────────────────────────────────
        g_health.node_a_alive = (g_health.node_a_last_seen_ms != 0) &&
                                 (now - g_health.node_a_last_seen_ms < NODE_A_TIMEOUT_MS);
        g_health.node_c_alive = (g_health.node_c_last_seen_ms != 0) &&
                                 (now - g_health.node_c_last_seen_ms < NODE_C_TIMEOUT_MS);

        // Node C dropout safety — close gate if heartbeat silent >5 min
        if (g_health.nodec_seen &&
            (now - g_health.node_c_last_seen_ms > NODE_C_TIMEOUT_MS)) {
            if (g_nodeCBatteryOk) {
                g_nodeCBatteryOk = false;
                g_health.nodec_battery_ok = false;
                LOG_WARN("HEALTH", "Node C silent >5 min — battery gate CLOSED");
                healthLog(ERR_LEVEL_2_OPERATIONAL, "Node C dropout — battery gate closed");
            }
        }
        g_health.nodec_battery_ok = g_nodeCBatteryOk;

        // ── 5. Task Watchdog ──────────────────────────────────
        uint8_t aliveMask = 0;
        for (int i = 0; i < HTASK_COUNT; i++) {
            // g_taskPingMs[] is written concurrently by other tasks, possibly
            // on a different core, with no synchronization on the array
            // itself. If a task's ping lands microseconds after this cycle's
            // `now` was snapshotted, g_taskPingMs[i] > now — a perfectly
            // healthy task, not a stale one — and the unsigned subtraction
            // below would otherwise wrap around to ~UINT32_MAX, which
            // previously caused immediate false "billions of ms silent"
            // readings and a fast, spurious REBOOT_WATCHDOG loop.
            uint32_t pingMs = g_taskPingMs[i];
            bool pingAheadOfNow = pingMs > now;
            uint32_t silence = (pingMs == 0 || pingAheadOfNow) ? 0 : (now - pingMs);
            bool alive = (pingMs == 0) || pingAheadOfNow || (silence < TASK_TIMEOUT_MS[i]);
            if (alive) {
                aliveMask |= (1 << i);
                g_health.task_stale_count[i] = 0;
            } else {
                g_health.task_stale_count[i]++;
                const char *tn[HTASK_COUNT] = {"LORA","UART","DEC","SD","CLOUD","ENV","LED"};
                LOG_WARN("HEALTH", "Task %d (%s) stale x%u — %lu ms silent",
                         i, tn[i], g_health.task_stale_count[i], (unsigned long)silence);
                char msg[64];
                snprintf(msg, sizeof(msg), "Task %s stale x%u", tn[i], g_health.task_stale_count[i]);
                healthLog(ERR_LEVEL_3_CRITICAL, msg);
            }

            // Stack high-water mark
            if (g_taskHandles[i] != nullptr) {
                uint32_t wm = uxTaskGetStackHighWaterMark(g_taskHandles[i]);
                if (g_health.task_stack_min[i] == 0 || wm < g_health.task_stack_min[i])
                    g_health.task_stack_min[i] = (uint16_t)wm;
            }
        }
        g_health.task_alive_mask = aliveMask;

        // Stack watermark audit — logged once at 60 s uptime
        if (!g_stackLogged && g_health.uptime_s >= 60) {
            g_stackLogged = true;
            const char *tn[HTASK_COUNT] = {"LORA","UART","DEC","SD","CLOUD","ENV","LED"};
            LOG_INFO("HEALTH", "── Stack Watermarks (60 s audit) ──");
            for (int i = 0; i < HTASK_COUNT; i++) {
                LOG_INFO("HEALTH", "  %-6s : %u bytes free", tn[i], g_health.task_stack_min[i]);
            }
            healthLog(ERR_LEVEL_1_INFO, "Stack watermark audit complete");
        }

        // ── 6. Queue Depths ───────────────────────────────────
        QueueHandle_t *queues[HEALTH_QUEUE_COUNT] = {
            &sensorQueue, &commandQueue, &feedbackQueue, &logQueue, &sdLogQueue,
            &heartbeatQueue, &cloudQueue, &sdQueue, &irrigationLogQueue
        };
        const char *qnames[HEALTH_QUEUE_COUNT] = {
            "sensor","command","feedback","log","sdLog",
            "heartbeat","cloud","sd","irrigLog"
        };
        for (int i = 0; i < HEALTH_QUEUE_COUNT; i++) {
            uint8_t pct = queueFillPct(*queues[i], QUEUE_CAPACITY[i]);
            g_health.queue_fill_pct[i] = pct;
            if (pct >= QUEUE_WARN_PCT[i]) {
                char msg[48];
                snprintf(msg, sizeof(msg), "Queue %s at %u%%", qnames[i], pct);
                LOG_WARN("HEALTH", "%s", msg);
                healthLog(ERR_LEVEL_2_OPERATIONAL, msg);
            }
        }

        // ── 7. Composite flags ────────────────────────────────
        g_health.bus_status_mask    = buildBusMask(g_health);
        g_health.global_health_flag = buildGlobalFlag(g_health);

        xSemaphoreGive(g_healthMutex);

        // ── 8. Reboot decisions (outside mutex) ───────────────
        if (g_health.consecutive_critical_heap >= 3) {
            controlledReboot(REBOOT_HEAP);
        }
        for (int i = 0; i < HTASK_COUNT; i++) {
            if (g_health.task_stale_count[i] >= 2) {
                controlledReboot(REBOOT_WATCHDOG);
            }
        }

        // ── 9. UART dispatch to Pi Pico ───────────────────────
        dispatchHealthPacket();

        // ── Cycle summary (always on — useful for field debug via USB) ─────────
        LOG_DEBUG("HEALTH", "cycle — heap=%lu B  bus=0x%02X  tasks=0x%02X  flag=0x%02X",
                  (unsigned long)g_health.heap_free_bytes,
                  g_health.bus_status_mask,
                  g_health.task_alive_mask,
                  g_health.global_health_flag);
    }
}
