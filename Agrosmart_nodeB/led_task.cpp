/**
 * @file    led_task.cpp
 * @brief   AgroSmart Node B — PCA9685 LED Status Driver Task (Production)
 *
 * Hardware:
 *   PCA9685 at 0x40 on I2C-1 (Wire1: SDA=33, SCL=32)
 *   Up to 16 channels; 8 used for system status (LED_CH_* in pins.h)
 *
 * Pattern engine:
 *   Each channel has an assigned LedPattern. The engine updates all channels
 *   every LED_UPDATE_PERIOD_MS (50 ms / 20 Hz) using a shared tick counter.
 *   Breathing uses a 60-step sine table to avoid float math in the loop.
 *
 * Health state → LED mapping (re-evaluated every LED_HEALTH_POLL_MS):
 *
 *   CH_SYSTEM_OK    BREATHE(slow) = all healthy
 *                   BLINK_FAST    = global_health_flag != 0
 *                   OFF           = PCA9685 init failed (self-fault)
 *
 *   CH_FAULT        SOLID  = global_health_flag != 0
 *                   OFF    = healthy
 *
 *   CH_LORA         SOLID  = Node A or Node C alive
 *                   BLINK_SLOW = last seen but timed out (stale)
 *                   OFF    = never seen since boot
 *
 *   CH_WIFI         SOLID  = WiFi connected
 *                   BLINK_SLOW = disconnected
 *
 *   CH_NODE_A       SOLID  = alive (seen <10 min)
 *                   DIM    = timed out
 *                   OFF    = never seen
 *
 *   CH_NODE_C       SOLID  = alive (seen <5 min)
 *                   DIM    = timed out
 *                   OFF    = never seen
 *
 *   CH_IRRIGATION   BLINK_FAST = irrigation in progress (pending_volume > 0)
 *                   OFF        = idle
 *
 *   CH_WARNING      BLINK_SLOW = battery low OR heap warn OR SD warn
 *                   OFF        = no warnings
 *
 * Safety:
 *   If PCA9685 init fails, the task logs to logQueue and enters a slow retry
 *   loop (every 5 s) rather than deleting itself. This allows the LED driver
 *   to recover if the I2C bus glitched at boot.
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "led_task.h"
#include "pins.h"
#include "health_task.h"
#include "queues.h"
#include "sync.h"
#include "enums.h"
#include "structs.h"
#include "tasks.h"
#include "logger.h"

// ─────────────────────────────────────────────────────────────────────────────
// PCA9685 register map (subset used here)
// ─────────────────────────────────────────────────────────────────────────────
#define PCA_MODE1         0x00
#define PCA_PRESCALE      0xFE
#define PCA_LED0_ON_L     0x06   // base: 4 bytes per channel (ON_L, ON_H, OFF_L, OFF_H)
#define PCA_ALLLED_ON_L   0xFA
#define PCA_MODE1_SLEEP   0x10
#define PCA_MODE1_AI      0x20   // auto-increment
#define PCA_MODE1_RESTART 0x80

// ─────────────────────────────────────────────────────────────────────────────
// LED patterns
// ─────────────────────────────────────────────────────────────────────────────
enum LedPattern : uint8_t {
    PAT_OFF        = 0,
    PAT_SOLID      = 1,
    PAT_DIM        = 2,
    PAT_BLINK_SLOW = 3,   // ~1 Hz: 600 ms on / 400 ms off
    PAT_BLINK_FAST = 4,   // ~4 Hz: 100 ms on / 150 ms off
    PAT_BREATHE    = 5    // sinusoidal 3-second ramp
};

// ─────────────────────────────────────────────────────────────────────────────
// 60-step half-sine LUT for the breathing pattern.
// Values are 12-bit (0–4095). Period = 60 ticks × 50 ms = 3 s.
// ─────────────────────────────────────────────────────────────────────────────
static const uint16_t kBreatheLUT[60] = {
       0,  142,  284,  424,  561,  693,  819,  938, 1048, 1148,
    1238, 1317, 1383, 1437, 1478, 1505, 1518, 1518, 1505, 1478,
    1437, 1383, 1317, 1238, 1148, 1048,  938,  819,  693,  561,
     424,  284,  142,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0
};
// Steps 33–59 are zero so the LED is fully off for half the cycle,
// giving a crisp "breathe in, pause" effect rather than a continuous sine.

// ─────────────────────────────────────────────────────────────────────────────
// Per-channel state
// ─────────────────────────────────────────────────────────────────────────────
static LedPattern s_pattern[8] = { PAT_OFF };
static uint8_t    s_breatheTick = 0;   // shared across all BREATHE channels

// ─────────────────────────────────────────────────────────────────────────────
// Low-level PCA9685 I2C helpers (direct register writes — no external library)
// ─────────────────────────────────────────────────────────────────────────────
static void pcaWrite(uint8_t reg, uint8_t val)
{
    if (!lockI2C1(50)) return;
    Wire1.beginTransmission(LED_PCA9685_ADDR);
    Wire1.write(reg);
    Wire1.write(val);
    Wire1.endTransmission();
    unlockI2C1();
}

static uint8_t pcaRead(uint8_t reg)
{
    if (!lockI2C1(50)) return 0xFF;
    Wire1.beginTransmission(LED_PCA9685_ADDR);
    Wire1.write(reg);
    Wire1.endTransmission(false);
    Wire1.requestFrom((uint8_t)LED_PCA9685_ADDR, (uint8_t)1);
    uint8_t val = Wire1.available() ? Wire1.read() : 0xFF;
    unlockI2C1();
    return val;
}

// Set one channel's 12-bit PWM duty (0=off, 4095=full on)
static void pcaSetChannel(uint8_t ch, uint16_t duty)
{
    if (ch > 15) return;
    duty &= 0x0FFF;

    if (!lockI2C1(50)) return;
    uint8_t reg = PCA_LED0_ON_L + (ch * 4);
    Wire1.beginTransmission(LED_PCA9685_ADDR);
    Wire1.write(reg);
    if (duty == 4095) {
        Wire1.write(0x00);  Wire1.write(0x10);
        Wire1.write(0x00);  Wire1.write(0x00);
    } else if (duty == 0) {
        Wire1.write(0x00);  Wire1.write(0x00);
        Wire1.write(0x00);  Wire1.write(0x10);
    } else {
        Wire1.write(0x00);
        Wire1.write(0x00);
        Wire1.write((uint8_t)(duty & 0xFF));
        Wire1.write((uint8_t)(duty >> 8));
    }
    Wire1.endTransmission();
    unlockI2C1();
}

// All channels off immediately (safe-state)
static void pcaAllOff()
{
    if (!lockI2C1(50)) return;
    Wire1.beginTransmission(LED_PCA9685_ADDR);
    Wire1.write(PCA_ALLLED_ON_L);
    Wire1.write(0x00); Wire1.write(0x00);
    Wire1.write(0x00); Wire1.write(0x10);
    Wire1.endTransmission();
    unlockI2C1();
}

// Initialise PCA9685: set PWM frequency to ~1 kHz
// Returns true if the device acknowledged on I2C.
static bool pcaInit()
{
    // Probe — if no ACK, chip is absent or bus is down
    if (!lockI2C1(200)) return false;
    Wire1.beginTransmission(LED_PCA9685_ADDR);
    bool present = (Wire1.endTransmission() == 0);
    unlockI2C1();
    if (!present) return false;

    // Sleep mode to allow prescaler write
    pcaWrite(PCA_MODE1, PCA_MODE1_SLEEP | PCA_MODE1_AI);

    // Prescaler = round(25 MHz / (4096 × freq)) − 1
    // For 1000 Hz: round(25e6 / (4096 × 1000)) − 1 = 5
    pcaWrite(PCA_PRESCALE, 5);

    // Wake up
    pcaWrite(PCA_MODE1, PCA_MODE1_AI);
    delayMicroseconds(500);

    // Restart PWM
    pcaWrite(PCA_MODE1, PCA_MODE1_RESTART | PCA_MODE1_AI);

    pcaAllOff();
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Pattern engine — compute PWM value for one channel given current tick
// ─────────────────────────────────────────────────────────────────────────────
static uint16_t patternPwm(LedPattern pat, uint32_t tick)
{
    switch (pat) {
        case PAT_OFF:        return 0;
        case PAT_SOLID:      return LED_PWM_FULL;
        case PAT_DIM:        return LED_PWM_DIM;

        case PAT_BLINK_SLOW: {
            // 1 Hz: 20-tick period at 50 ms/tick
            // ON for 12 ticks (600 ms), OFF for 8 ticks (400 ms)
            uint32_t phase = tick % 20;
            return (phase < 12) ? LED_PWM_FULL : 0;
        }

        case PAT_BLINK_FAST: {
            // ~4 Hz: 5-tick period
            // ON for 2 ticks (100 ms), OFF for 3 ticks (150 ms)
            uint32_t phase = tick % 5;
            return (phase < 2) ? LED_PWM_FULL : 0;
        }

        case PAT_BREATHE:
            return kBreatheLUT[s_breatheTick];

        default: return 0;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Health → LED mapping
// Called every LED_HEALTH_POLL_MS to update s_pattern[] from live health state.
// ─────────────────────────────────────────────────────────────────────────────
static void applyHealthState(const SystemHealthState &h)
{
    bool anyFault   = (h.global_health_flag != 0);
    bool heapWarn   = (h.heap_severity >= HEAP_WARN);
    bool sdWarn     = h.sd_storage_critical;
    bool batLow     = !h.nodec_battery_ok && h.nodec_seen;
    bool wifiUp     = h.wifi_ok;
    bool nodeAAlive = h.node_a_alive;
    bool nodeCSeen  = h.nodec_seen;
    bool nodeCAlive = h.node_c_alive;
    bool loraAlive  = nodeAAlive || nodeCAlive;
    bool loraSeen   = (h.node_a_last_seen_ms != 0) || (h.node_c_last_seen_ms != 0);

    // CH_SYSTEM_OK — green heartbeat
    s_pattern[LED_CH_SYSTEM_OK] = anyFault ? PAT_BLINK_FAST : PAT_BREATHE;

    // CH_FAULT — red solid when any fault active
    s_pattern[LED_CH_FAULT] = anyFault ? PAT_SOLID : PAT_OFF;

    // CH_LORA — blue: solid=alive, slow blink=seen but stale, off=never seen
    if      (loraAlive) s_pattern[LED_CH_LORA] = PAT_SOLID;
    else if (loraSeen)  s_pattern[LED_CH_LORA] = PAT_BLINK_SLOW;
    else                s_pattern[LED_CH_LORA] = PAT_OFF;

    // CH_WIFI — cyan: solid=connected, slow blink=disconnected
    s_pattern[LED_CH_WIFI] = wifiUp ? PAT_SOLID : PAT_BLINK_SLOW;

    // CH_NODE_A — white: solid=alive, dim=timed out, off=never seen
    if      (nodeAAlive)                     s_pattern[LED_CH_NODE_A] = PAT_SOLID;
    else if (h.node_a_last_seen_ms != 0)     s_pattern[LED_CH_NODE_A] = PAT_DIM;
    else                                     s_pattern[LED_CH_NODE_A] = PAT_OFF;

    // CH_NODE_C — yellow: solid=alive, dim=timed out, off=never seen
    if      (nodeCAlive)  s_pattern[LED_CH_NODE_C] = PAT_SOLID;
    else if (nodeCSeen)   s_pattern[LED_CH_NODE_C] = PAT_DIM;
    else                  s_pattern[LED_CH_NODE_C] = PAT_OFF;

    // CH_IRRIGATION — teal fast blink when a command is pending (pump running)
    // Inferred from task_alive_mask: DEC task sets pending; we see it via
    // queue depth proxy. Direct flag: use nodec_status_flag bit if available.
    // Simple proxy: if Node C is alive AND last feedback hasn't cleared, blink.
    // More reliable: sdQueue depth > 0 means recent sensor+decision data flowing.
    // Best approach: check nodec_status_flag bit 0 = pump running (convention).
    bool pumpRunning = nodeCAlive && (h.nodec_status_flag & 0x01);
    s_pattern[LED_CH_IRRIGATION] = pumpRunning ? PAT_BLINK_FAST : PAT_OFF;

    // CH_WARNING — amber slow blink for any soft warning
    bool warnActive = heapWarn || sdWarn || batLow;
    s_pattern[LED_CH_WARNING] = warnActive ? PAT_BLINK_SLOW : PAT_OFF;
}

// ─────────────────────────────────────────────────────────────────────────────
// vLedTask
// ─────────────────────────────────────────────────────────────────────────────
void vLedTask(void *pvParameters)
{
    (void)pvParameters;

    LOG_INFO("LED", "Task starting — PCA9685 on Wire1 (SDA=33 SCL=32)");
    // Wire1 is initialized centrally in setup() (Agrosmart_nodeB.ino), before
    // any task starts — not by vEnvironmentTask, which doesn't run at all in
    // DEVELOPMENT_MODE. No Wire1.begin() needed here.

    // ── PCA9685 init — retry every 5 s on failure ─────────────────────────────
    bool pcaReady = false;
    while (!pcaReady) {
        pcaReady = pcaInit();
        if (!pcaReady) {
            LOG_WARN("LED", "PCA9685 not found — retrying in 5 s");
            SystemEventLog entry;
            memset(&entry, 0, sizeof(entry));
            entry.severity     = ERR_LEVEL_2_OPERATIONAL;
            entry.boot_time_ms = (uint32_t)millis();
            strncpy(entry.source,  "LED_TASK", sizeof(entry.source)  - 1);
            strncpy(entry.message, "PCA9685 init failed — I2C-1 fault", sizeof(entry.message) - 1);
            pushLog(entry);

            healthPing(HTASK_LED);
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }

    LOG_INFO("LED", "PCA9685 online. Status LEDs active.");

    uint32_t tick         = 0;
    uint32_t lastPollMs   = 0;

    // Initial state: all off until first health poll
    for (int i = 0; i < 8; i++) s_pattern[i] = PAT_OFF;

    for (;;) {
        healthPing(HTASK_LED);

        uint32_t now = millis();

        // ── Re-read health state every LED_HEALTH_POLL_MS ────────────────────
        if ((now - lastPollMs) >= LED_HEALTH_POLL_MS) {
            lastPollMs = now;
            SystemHealthState h = getSystemHealth();
            applyHealthState(h);

            // If PCA9685 disappeared mid-run (I2C glitch), detect and re-init
            uint8_t mode = pcaRead(PCA_MODE1);
            if (mode == 0xFF) {
                LOG_WARN("LED", "PCA9685 lost — attempting re-init");
                pcaReady = pcaInit();
                if (!pcaReady) {
                    vTaskDelay(pdMS_TO_TICKS(LED_UPDATE_PERIOD_MS));
                    continue;
                }
            }
        }

        // ── Drive all 8 channels ──────────────────────────────────────────────
        for (uint8_t ch = 0; ch < 8; ch++) {
            uint16_t pwm = patternPwm(s_pattern[ch], tick);
            pcaSetChannel(ch, pwm);
        }

        // Advance breathe LUT pointer
        s_breatheTick = (s_breatheTick + 1) % 60;
        tick++;

        vTaskDelay(pdMS_TO_TICKS(LED_UPDATE_PERIOD_MS));
    }
}
