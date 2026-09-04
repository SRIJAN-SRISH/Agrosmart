#pragma once
#ifndef HEALTH_TASK_H
#define HEALTH_TASK_H

#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "structs.h"

// ═══════════════════════════════════════════════════════════════
// Bus status byte — bit positions in bus_status_mask
// 0xFF = all systems healthy
// ═══════════════════════════════════════════════════════════════
#define BUS_BIT_I2C0    0   // Wire  : BMP280(0x76) + RTC(0x68)
#define BUS_BIT_I2C1    1   // Wire1 : PCA9685(0x40)
#define BUS_BIT_HSPI    2   // HSPI  : SD card
#define BUS_BIT_VSPI    3   // VSPI  : LoRa radio (inferred from task liveness)
#define BUS_BIT_RAIN    4   // Wire1 : Rain gauge (SEN0575, I2C) — proxied via
                            //         i2c1_ok today, see health_task.cpp
#define BUS_BIT_WIFI    5   // WiFi  : WL_CONNECTED
#define BUS_BIT_NODE_A  6   // Node A packet seen within 10 min
#define BUS_BIT_NODE_C  7   // Node C heartbeat seen within 5 min

// ═══════════════════════════════════════════════════════════════
// Task IDs — index into TASK_TIMEOUT_MS[], task_stack_min[], etc.
// Must match the order tasks are created in Agrosmart_nodeB.ino.
// ═══════════════════════════════════════════════════════════════
enum HealthTaskId : uint8_t {
    HTASK_LORA  = 0,
    HTASK_UART  = 1,
    HTASK_DEC   = 2,
    HTASK_SD    = 3,
    HTASK_CLOUD = 4,
    HTASK_ENV   = 5,
    HTASK_LED   = 6,
    HTASK_COUNT = 7
};

// ═══════════════════════════════════════════════════════════════
// Reboot reasons — stored in NVS, survive power cycle
// ═══════════════════════════════════════════════════════════════
enum RebootReason : uint8_t {
    REBOOT_NONE     = 0x00,
    REBOOT_HEAP     = 0x01,
    REBOOT_WATCHDOG = 0x02,
    REBOOT_MANUAL   = 0x03,
    REBOOT_UNKNOWN  = 0xFF
};

// ═══════════════════════════════════════════════════════════════
// Heap severity levels
// ═══════════════════════════════════════════════════════════════
enum HeapSeverity : uint8_t {
    HEAP_OK       = 0,
    HEAP_WARN     = 1,
    HEAP_CRITICAL = 2
};

// ═══════════════════════════════════════════════════════════════
// Watchdog silence thresholds per task (ms)
// Single stale = WARN. Two consecutive stale = controlled reboot.
// ═══════════════════════════════════════════════════════════════
static const uint32_t TASK_TIMEOUT_MS[HTASK_COUNT] = {
    15000,   // LORA
    15000,   // UART
    15000,   // DECISION
    30000,   // SD   (writes can be slow)
    60000,   // CLOUD (WiFi reconnect window)
    90000,   // ENV   (long sensor poll interval)
    10000    // LED   (50 ms loop → stale if silent 10 s)
};

// ═══════════════════════════════════════════════════════════════
// UART packet types — sent to Pi Pico over Serial2 (UART2, GPIO17)
// Frame: [0x7E][TYPE][LEN_LO][LEN_HI][DATA...][CRC8][0x7F]
// ═══════════════════════════════════════════════════════════════
#define PKT_START       0x7E
#define PKT_END         0x7F
#define PKT_HEALTH      0x01
#define PKT_SENSOR      0x02
#define PKT_DECISION    0x03
#define PKT_CONFIG      0x04   // FarmProfile — static, sent once at boot + on STATUS_REQUEST

// ═══════════════════════════════════════════════════════════════
// Number of FreeRTOS queues monitored
// Order: sensor, command, feedback, log, sdLog,
//        heartbeat, cloud, sd, irrigationLog
// ═══════════════════════════════════════════════════════════════
#define HEALTH_QUEUE_COUNT  9

// ═══════════════════════════════════════════════════════════════
// Master health state — mutex-protected, one global instance
// ═══════════════════════════════════════════════════════════════
struct SystemHealthState {

    // ── Group 1: System & Runtime ─────────────────────────────
    uint32_t uptime_s;
    uint32_t reboot_count;          // NVS — survives restart
    uint8_t  reboot_reason_last;    // RebootReason enum
    char     firmware_version[8];   // from FIRMWARE_VERSION in config.h

    // ── Group 2: Memory ───────────────────────────────────────
    uint32_t heap_free_bytes;
    uint32_t heap_free_min_ever;    // worst-case since boot
    uint8_t  heap_severity;         // HeapSeverity enum

    // ── Group 3: Hardware buses (physical layer) ──────────────
    uint8_t  bus_status_mask;       // bit map — see BUS_BIT_* above
    bool     i2c0_ok;               // I2C-0 probe result (BMP280 + RTC)
    bool     i2c1_ok;               // I2C-1 probe result (PCA9685)
    bool     hspi_sd_ok;            // SD card mounted and readable
    bool     vspi_lora_ok;          // LoRa alive (inferred from LORA task bit)
    bool     rain_gpio_ok;          // rain GPIO reads a valid logic level
    bool     wifi_ok;               // WiFi station connected
    int8_t   wifi_rssi;             // dBm, 0 if disconnected

    // ── Group 4: Edge node liveness ───────────────────────────
    uint32_t node_a_last_seen_ms;   // millis() of last Node A packet
    uint32_t node_c_last_seen_ms;   // millis() of last Node C heartbeat
    bool     node_a_alive;          // seen within 10 min
    bool     node_c_alive;          // seen within 5 min
    int8_t   node_a_rssi_last;      // dBm of last Node A packet
    int8_t   node_c_rssi_last;      // dBm of last Node C heartbeat

    // ── Group 5: Node C battery gate (safety-critical) ────────
    float    nodec_battery_v;
    bool     nodec_battery_ok;      // hysteresis gate — read by decision_task
    bool     nodec_seen;            // any heartbeat arrived since boot?
    uint8_t  nodec_status_flag;     // raw status byte from Node C

    // ── Group 6: FreeRTOS watchdog ────────────────────────────
    uint8_t  task_alive_mask;       // bit=1 alive, bit=0 stale
    uint16_t task_stack_min[HTASK_COUNT]; // bytes free at worst point

    // ── Group 7: Queue depths ─────────────────────────────────
    uint8_t  queue_fill_pct[HEALTH_QUEUE_COUNT]; // 0–100%

    // ── Group 8: SD storage ───────────────────────────────────
    uint32_t sd_total_mb;
    uint32_t sd_used_mb;
    uint8_t  sd_used_pct;
    bool     sd_storage_critical;

    // ── Group 9: Master diagnostic flag ───────────────────────
    uint8_t  global_health_flag;    // OR of all fault bits — 0x00 = healthy

    // ── Internal (not transmitted over UART) ──────────────────
    uint8_t  consecutive_critical_heap;      // reboot triggered at 3
    uint8_t  task_stale_count[HTASK_COUNT];  // per-task consecutive stale reads
    uint8_t  pending_reboot_reason;          // set before calling esp_restart()
};

// ═══════════════════════════════════════════════════════════════
// Task handles — defined in Agrosmart_nodeB.ino.
// health_task reads these for uxTaskGetStackHighWaterMark().
// ═══════════════════════════════════════════════════════════════
extern TaskHandle_t g_taskHandles[HTASK_COUNT];

// ═══════════════════════════════════════════════════════════════
// Public API
// ═══════════════════════════════════════════════════════════════

// FreeRTOS task entry point
void vHealthTask(void *pvParameters);

// Full human-readable dump to Serial
void printHealthState(const SystemHealthState &h);

// Each task calls this at the top of its main loop to reset its watchdog slot
void healthPing(HealthTaskId id);

// lora_task calls on every valid Node A LoRa packet
void reportNodeASeen(int8_t rssi);

// lora_task calls on every Node C heartbeat
void reportNodeCSeen(float battV, uint8_t statusFlag, int8_t rssi);

// decision_task calls before enqueuing any NodeCCommand
bool isNodeCBatteryOk();

// sd_task calls before writing non-critical rows
bool isSdStorageCritical();

// Any task can get a full mutex-safe snapshot
SystemHealthState getSystemHealth();

// decision_task calls these once per processed record — PKT_SENSOR / PKT_DECISION
// frames to the Pi Pico over Serial2. Serial2 is opened once in setup()
// (Agrosmart_nodeB.ino) before any task starts, so these are safe from first call.
void dispatchSensorPacket(const MasterSensorRecord &r);
void dispatchDecisionPacket(const IrrigationDecision &d);

// Sent once from setup() at boot, and again on PILINK_CMD_STATUS_REQUEST
void dispatchConfigPacket(const FarmProfile &p);

#endif // HEALTH_TASK_H
