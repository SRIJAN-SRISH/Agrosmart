/**
 * Agrosmart_NodeA.ino  —  Production Firmware v2.0
 */

#include <Arduino.h>
#include <string.h>
#include <math.h>
#include "config.h"
#include "structs.h"
#include "pins.h"
#include "leds.h"
#include "modbus_core.h"
#include "lora_comms.h"

// ═══════════════════════════════════════════════════════════════════════════
// State
// ═══════════════════════════════════════════════════════════════════════════

static uint32_t g_recordSeq  = 0;
static uint32_t g_lastTxMs   = 0;
static uint32_t g_lastHbMs   = 0;
static uint32_t g_bootMs     = 0;

// ═══════════════════════════════════════════════════════════════════════════
// Main Sensor + TX Cycle
// ═══════════════════════════════════════════════════════════════════════════

static void runCycle() {
    Serial.println("────────────────────────────────────────────────────────────");
    Serial.printf("[CYCLE] #%lu  uptime=%lus\n",
                  ++g_recordSeq, (millis() - g_bootMs) / 1000UL);

    MasterSensorRecord r;
    memset(&r, 0, sizeof(r));

    r.boot_time_ms    = g_bootMs;
    r.timestamp_epoch = 0;
    r.timestamp_valid = 0;
    r.record_sequence = g_recordSeq;
    r.health_flag     = HEALTH_OK;
    r.gps_valid       = 0;
    r.latitude        = 0.0f;
    r.longitude       = 0.0f;
    r.air_temp        = NAN;
    r.humidity        = NAN;
    r.pressure        = NAN;
    r.rainfall        = NAN;

    snprintf(r.record_id, sizeof(r.record_id), "NA_%08lX_%04lX",
             (unsigned long)millis(), (unsigned long)g_recordSeq);

    // ── 1. Read soil sensor ───────────────────────────────────────────────
    ledSet(PIN_LED_YELLOW, true);
    bool soilOk = readSoilSensor(r);
    ledSet(PIN_LED_YELLOW, false);

    if (soilOk) {
        ledSet(PIN_LED_GREEN, true);
        ledSet(PIN_LED_RED,   false);
    } else {
        ledSet(PIN_LED_GREEN, false);
        ledSet(PIN_LED_RED,   true);
    }

    // ── 2. Transmit ───────────────────────────────────────────────────────
    bool txOk = false;
    if (loraIsReady()) {
        ledSet(PIN_LED_YELLOW, true);
        txOk = transmitPacket(r);
        ledSet(PIN_LED_YELLOW, false);

        if (!txOk) {
            r.health_flag |= HEALTH_NODEA_COMM_FAULT;
            ledShowFault(!soilOk, true);
        }
    } else {
        Serial.println("[LORA]  Not ready — skipping TX");
    }

    Serial.printf("[CYCLE] Done  health=0x%02X  soil=%s  tx=%s\n",
                  r.health_flag,
                  soilOk ? "OK"    : "FAULT",
                  txOk   ? "OK"    : "FAULT");
}

// ═══════════════════════════════════════════════════════════════════════════
// Setup
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);
    g_bootMs = millis();

    Serial.println("\n══════════════════════════════════════════");
    Serial.println("   Agrosmart Node A  —  Firmware v2.0");
    Serial.println("══════════════════════════════════════════");

    // ── Subsystems ─────────────────────────────────────────────────────────
    ledInit();
    ledBootRipple();
    initModbus();
    
    if (!initLora()) {
        ledAllOff();
        ledSet(PIN_LED_RED, true);
        while (1) delay(1000);
    }

    Serial.printf("[INIT]  Cycle interval: %lu s\n\n", TX_INTERVAL_MS / 1000UL);

    // First cycle immediately on boot
    runCycle();
    g_lastTxMs = millis();
    g_lastHbMs = millis();
}

// ═══════════════════════════════════════════════════════════════════════════
// Loop
// ═══════════════════════════════════════════════════════════════════════════

void loop() {
    uint32_t now = millis();

    // White heartbeat pulse every 30 s — confirms system is alive between TX cycles
    if (now - g_lastHbMs >= HEARTBEAT_INTERVAL_MS) {
        g_lastHbMs = now;
        ledPulse(PIN_LED_WHITE, 80);
    }

    // Main cycle
    if (now - g_lastTxMs >= TX_INTERVAL_MS) {
        g_lastTxMs = now;
        runCycle();
    }

    delay(50);
}
