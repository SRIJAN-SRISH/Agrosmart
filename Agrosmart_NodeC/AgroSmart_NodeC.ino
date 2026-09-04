// ============================================================
// AgroSmart_NodeC.ino — Entry point
//
// A single non-blocking loop, not FreeRTOS tasks — Node C's only
// concurrency needs (LoRa RX, the pump-cycle state machine tick, the
// heartbeat timer) all run fine cooperatively in one loop(), as
// validated on the bench. See nodeC_architecture.md for the full
// design rationale.
//
// Module responsibilities:
//   actuators.h/.cpp — relay control, flow meter, ACS712, the
//                       managePumpCycle-equivalent state machine
//   sensors.h/.cpp   — INA219 battery/system monitor
//   comms.h/.cpp     — LoRa init, packet validation, dispatch,
//                       feedback/heartbeat transmission
// ============================================================

#include <Arduino.h>
#include <esp_task_wdt.h>
#include <string.h>
#include "config.h"
#include "diagnostics.h"
#include "actuators.h"
#include "sensors.h"
#include "comms.h"

// ============================================================
// BENCH_NO_LORA — temporary switch for testing ACS712/INA219/relay/
// valve/flow-meter BEFORE the LoRa module is physically wired in.
//
// initComms() halts the sketch on LoRa.begin() failure by design (see
// comms.cpp) — correct behavior for the field, useless for a bench
// session with no radio connected yet. With this defined, initComms()/
// tickComms() are skipped entirely and replaced with a manual Serial
// command harness that drives the SAME startPumpCycle()/tickActuators()/
// emergencyStop()/popCompletedFeedback() API comms.cpp will call in
// production — this is not a separate rewritten test sketch, it
// exercises the real actuators.cpp/sensors.cpp code paths.
//
// REMOVE this #define once the LoRa module is wired in and confirmed —
// do not leave a live deployment running with comms disabled.
// ============================================================
// #define BENCH_NO_LORA

#ifdef BENCH_NO_LORA
static void printBenchHelp() {
    Serial.println("[BENCH] LoRa not connected — comms.cpp skipped for this build.");
    Serial.println("[BENCH] Send 'r' + Enter -> run a 2.0L test pump cycle (zone 1)");
    Serial.println("[BENCH] Send 's' + Enter -> force emergency stop");
    Serial.println("[BENCH] Battery voltage prints every 5s.\n");
}
#endif

// Hardware watchdog — the actual fail-safe of last resort. If loop()
// ever freezes (brownout, EMI-induced hang, an unforeseen bug) while
// the pump relay is energized, nothing else in this firmware can turn
// it off — tickActuators() simply never runs again. This forces a full
// chip reboot if loop() doesn't check in within WDT_TIMEOUT_MS, which
// resets GPIO16/17 to their boot-safe inactive level as a side effect
// of the reboot itself.
//
// API note: arduino-esp32 core 3.x (confirmed: this board package is on
// 3.3.10) requires the struct-based esp_task_wdt_init(const
// esp_task_wdt_config_t*) signature — the old 2.x
// esp_task_wdt_init(timeout_s, panic) two-argument form no longer
// exists. Config struct is built in setup() below.
#define WDT_TIMEOUT_MS 5000

void setup() {
    Serial.begin(115200);
    delay(500);

    // As early as possible — captures this boot's reset reason and
    // persists the reboot counter even if something later in setup()
    // halts (e.g. comms.cpp's LoRa-failure halt loop).
    initDiagnostics();

    Serial.println("\n============================================");
    Serial.printf(  "  %s — %s\n", PROJECT_NAME, NODE_ROLE);
    Serial.printf(  "  Firmware %s | %s\n", FIRMWARE_VERSION, NODE_ID);
    Serial.println("============================================\n");

    // Order matters: actuators before sensors before comms.
    //   - initActuators() forces relays inactive first — nothing else
    //     should be able to race it to the pump/valve pins.
    //   - initSensors() brings up I2C/INA219 next — independent of
    //     actuators, but comms.cpp's first heartbeat needs it ready.
    //   - initComms() goes last and halts on failure — by this point
    //     everything it might report on (relay state, battery reading)
    //     already exists.
    initActuators();
    initSensors();
#ifdef BENCH_NO_LORA
    printBenchHelp();
#else
    initComms();
#endif

    // Watchdog registered last, after every blocking init call has
    // already completed — LoRa.begin() and the ACS712/INA219 bring-up
    // routines run fine within WDT_TIMEOUT_MS, but there's no reason
    // to let setup() itself race the same clock as the running loop.
    esp_task_wdt_config_t wdtConfig = {
        .timeout_ms = WDT_TIMEOUT_MS,
        .idle_core_mask = 0,      // don't subscribe the idle tasks, only loop()
        .trigger_panic = true     // panic + reboot on timeout
    };
    esp_task_wdt_init(&wdtConfig);
    esp_task_wdt_add(NULL);       // watch the current (loop) task

    Serial.println("\n[SETUP] Node C ready. Listening for commands from Node B.\n");
}

void loop() {
    tickActuators();

    // Heap health — outside the BENCH_NO_LORA branch so it's active in
    // both bench and production builds, on its own timer independent of
    // the heartbeat/battery print cadences below.
    static uint32_t lastHeapPrintMs = 0;
    if (millis() - lastHeapPrintMs >= 60000UL) {
        lastHeapPrintMs = millis();
        tickDiagnosticsHeap();
    }

#ifdef BENCH_NO_LORA
    // ── Manual test harness — same actuators.h API comms.cpp uses ───────
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'r') {
            if (!isActuatorIdle()) {
                Serial.println("[BENCH] Busy — cycle already in progress.");
            } else {
                NodeCCommand testCmd;
                memset(&testCmd, 0, sizeof(testCmd));
                strncpy(testCmd.command_id, "BENCH_TEST", sizeof(testCmd.command_id) - 1);
                testCmd.target_volume_l = 2.0f;   // small test dose
                testCmd.max_runtime_sec = 60;     // generous bench ceiling
                testCmd.zone_id = 1;
                if (!startPumpCycle(testCmd)) {
                    Serial.println("[BENCH] startPumpCycle() rejected the command.");
                }
            }
        } else if (c == 's') {
            Serial.println("[BENCH] Forcing emergency stop.");
            emergencyStop();
        }
    }

    NodeCFeedback fb;
    if (popCompletedFeedback(fb)) {
        Serial.printf("[BENCH] Cycle result: status=0x%02X  volume=%.3fL  "
                      "flow=%.2fL/min  runtime=%us  batt=%.2fV\n",
                      fb.status_flag, fb.delivered_volume_l, fb.flow_rate_lpm,
                      fb.runtime_sec, fb.battery_voltage);
    }

    static uint32_t lastBattPrintMs = 0;
    if (millis() - lastBattPrintMs >= 5000) {
        lastBattPrintMs = millis();
        Serial.printf("[BENCH] Battery: %.2fV\n", readBatteryVoltage());
    }
#else
    tickComms();
#endif

    esp_task_wdt_reset();   // "I'm alive" — must run every iteration or the
                              // chip reboots within WDT_TIMEOUT_MS
}
