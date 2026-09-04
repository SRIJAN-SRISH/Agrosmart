/**
 * @file    mock_task.cpp
 * @brief   AgroSmart Node B — Hardware-Free Simulation Task
 *
 * Compiled ONLY when DEVELOPMENT_MODE is defined in config.h.
 * Produces zero symbols in a production build.
 *
 * What this simulates
 * -------------------
 *  Node A  : Injects a MasterSensorRecord into sensorQueue every
 *            SENSOR_POLL_INTERVAL_MS. VWC is set low enough (12.5%)
 *            to reliably trigger the Decision Engine's deficit gate.
 *
 *  Node C  : After each injection, watches commandQueue. If the
 *            Decision Engine queued an irrigation command, pulls it,
 *            waits 2 s (pump runtime simulation), then pushes a
 *            NodeCFeedback ACK into feedbackQueue.
 *
 * What this does NOT touch
 * ------------------------
 *  decision_task.cpp, cloud_task.cpp, and sd_task.cpp are entirely
 *  unaware of this file. They consume from the same queues they would
 *  read from real hardware — the pipeline is identical.
 */

#include "config.h"

#ifdef DEVELOPMENT_MODE

#include <Arduino.h>
#include <string.h>
#include <math.h>

#include "mock_task.h"
#include "structs.h"
#include "enums.h"
#include "queues.h"
#include "esp_random.h"
#include "health_task.h"
#include "logger.h"

// ─────────────────────────────────────────────────────────────────────────────
// Pre-field safety gate test scenarios
//
// Every other test done tonight only exercises the HAPPY PATH (VWC always
// low enough to trigger irrigation). That proves the decision engine can
// say "yes" — it does NOT prove it correctly says "no" when it's supposed
// to. Set MOCK_SCENARIO to one of these, reflash, watch ONE mock cycle,
// then move to the next. Revert to MOCK_SCENARIO_HAPPY_PATH before the
// field test tomorrow — these are deliberately abnormal readings.
//
// Untested by this switch (documented, not silently ignored):
//   - REASON_LOW_BATTERY: isNodeCBatteryOk() is only ever set false by a
//     real/mock NodeCHeartbeat via reportNodeCSeen(), which mock_task.cpp
//     never calls. Would need a separate temporary reportNodeCSeen(10.5f,...)
//     injection to exercise — lower priority than the two below since it's
//     also the gate least likely to silently regress (it's a single bool
//     read, not agronomic math).
//   - REASON_DAILY_LIMIT / REASON_STABILIZATION_DELAY: require accumulating
//     state across many real cycles (daily_total_l) or stab_delay_min > 0
//     (deliberately 0 in DEVELOPMENT_MODE's default profile) — impractical
//     to exercise in a single quick pass tonight.
// ─────────────────────────────────────────────────────────────────────────────
#define MOCK_SCENARIO_HAPPY_PATH   0   // VWC=12.5%, no rain, HEALTH_OK — irrigates every cycle
#define MOCK_SCENARIO_RAIN_LOCK    1   // Field already saturated — must BLOCK, reason=RAIN_LOCK
#define MOCK_SCENARIO_SENSOR_FAULT 2   // health_flag set — must BLOCK, reason=SENSOR_FAULT

// ⚠️ SET THIS, REFLASH, TEST ONE SCENARIO AT A TIME. REVERT TO 0 BEFORE FIELD TEST. ⚠️
#define MOCK_SCENARIO MOCK_SCENARIO_HAPPY_PATH

// ─────────────────────────────────────────────────────────────────────────────
// Simulation constants
// ─────────────────────────────────────────────────────────────────────────────

// Approximate UTC epoch for 2026-06-26 00:00:00.
// Using a real calendar base means the SD task writes dated folders
// (/DATA/2026/06/26.csv) instead of /DATA/nodate.csv.
static const uint32_t MOCK_BASE_EPOCH = 1782432000UL;

// Simulated flow rate used to compute mock runtime_sec.
static const float MOCK_FLOW_RATE_LPM = 20.0f;

// How long we wait after injecting sensor data before we check whether the
// Decision Engine emitted a command. The decision loop ticks every 50 ms;
// 600 ms gives it 12 ticks of margin — plenty even with median-filter warm-up.
static const uint32_t CMD_POLL_WAIT_MS = 600UL;

// ─────────────────────────────────────────────────────────────────────────────
// Internal helpers
// ─────────────────────────────────────────────────────────────────────────────

// Returns a monotonically increasing epoch derived from millis() so the
// Decision Engine's daily-limit rollover and SD date logic behave naturally.
static inline uint32_t mockEpoch(void) {
    return MOCK_BASE_EPOCH + (millis() / 1000UL);
}

// ─────────────────────────────────────────────────────────────────────────────
// vMockTask
// ─────────────────────────────────────────────────────────────────────────────

void vMockTask(void* pvParameters) {
    (void)pvParameters;

    LOG_INFO("MOCK", "*** DEVELOPMENT_MODE ACTIVE — hardware simulation running ***");

    // Let every other task finish initialising before we start injecting data.
    vTaskDelay(pdMS_TO_TICKS(3000));

    static uint32_t sequence = 0;
    // Hardware random boot token — changes on every power-on so record_ids
    // are unique across reboots even when millis() resets to 0.
    static uint32_t bootToken = esp_random();

    for (;;) {
        // MockTask replaces vLoraTask, vUartTask, vEnvironmentTask in DEVELOPMENT_MODE.
        // Ping all three slots so health_task watchdog sees them as alive.
        healthPing(HTASK_LORA);
        healthPing(HTASK_UART);
        healthPing(HTASK_ENV);

        sequence++;

        // =====================================================================
        // PART 1 — Simulate Node A (Sensor Injection)
        // =====================================================================

        MasterSensorRecord rec;
        memset(&rec, 0, sizeof(rec));

        uint32_t now = mockEpoch();

        // Primary key — must be unique per record, matches pattern used by
        // the real UART task so decision_task / cloud_task see no difference.
        // bootToken (random) + sequence guarantees uniqueness across reboots
        snprintf(rec.record_id, sizeof(rec.record_id), "REC_%08lX_%04lu", (unsigned long)bootToken, (unsigned long)sequence);

        rec.timestamp_epoch  = now;
        rec.boot_time_ms     = millis();
        rec.record_sequence  = sequence;
        rec.timestamp_valid  = 1;   // Non-zero → decision_task updates current_epoch

        // ── Soil parameters (Node A RS-485 sensor) ──────────────────────────
        // Base case (VWC 12.5%) is deliberately well below the deficit
        // trigger threshold. With the mock profile (fc=32%, wp=14%, mad=50%,
        // root=0.4m):
        //   TAW   = (0.32 - 0.14) * 400mm = 72 mm
        //   RAW   = 72 * 0.50            = 36 mm
        //   gate  = RAW + ACTIVATION_MARGIN_MM = 38 mm
        //   deficit at 12.5% VWC = (0.32 - 0.125) * 400 = 78 mm  → TRIGGERS
        rec.vwc_percent  = 12.5f;
        rec.soil_temp    = 24.0f;
        rec.ec           = 1.2f;
        rec.ph           = 6.8f;
        rec.nitrogen     = 45.0f;
        rec.phosphorus   = 30.0f;
        rec.potassium    = 180.0f;

        // ── GPS (Node A NEO-M8N — not available in simulation) ──────────────
        rec.gps_valid  = 0;
        rec.latitude   = 0.0f;
        rec.longitude  = 0.0f;

        // ── Local environment (Node B I2C — BMP280 + SEN0575) ───────────────
        rec.air_temp  = 32.0f;
        rec.humidity  = 40.0f;   // Validated by decision_task: must be 0–100
        rec.pressure  = 1013.25f;
        rec.rainfall  = 0.0f;    // Zero ensures rain_lock gate does NOT fire

        rec.health_flag = HEALTH_OK;   // Must be 0x00 or decision bails early

        // ── Safety gate scenario overrides — see MOCK_SCENARIO above ────────
#if MOCK_SCENARIO == MOCK_SCENARIO_RAIN_LOCK
        // Gate condition (decision_task.cpp): rainfall >= rain_lock_mm (5.0
        // default) AND filteredVwc >= field_capacity_vwc (32.0 default) —
        // both must be true, so both are overridden here.
        rec.vwc_percent = 35.0f;
        rec.rainfall    = 6.0f;
        LOG_INFO("MOCK", "*** SCENARIO: RAIN_LOCK — expect [DECISION] RAIN LOCK, "
                         "NO command issued ***");
#elif MOCK_SCENARIO == MOCK_SCENARIO_SENSOR_FAULT
        rec.health_flag = HEALTH_NODEA_SOIL_FAULT;
        LOG_INFO("MOCK", "*** SCENARIO: SENSOR_FAULT — expect [DECISION] BLOCKED: "
                         "sensor health_flag=0x01, NO command issued ***");
#else
        LOG_INFO("MOCK", "*** SCENARIO: HAPPY_PATH — expect [DECISION] IRRIGATE "
                         "every cycle ***");
#endif

        LOG_INFO("MOCK", "Injecting record #%lu | id=%s | VWC=%.1f%% | AirT=%.1fC | "
                         "rainfall=%.1fmm | health_flag=0x%02X",
                         (unsigned long)sequence, rec.record_id,
                         rec.vwc_percent, rec.air_temp, rec.rainfall, rec.health_flag);

        // sensorQueue — P1 policy (FMEA): brief wait, then drop newest if full.
        // As the injector we accept losing our own newest sample; the next
        // cycle will produce a fresher one anyway.
        if (sensorQueue != nullptr) {
            if (xQueueSend(sensorQueue, &rec, pdMS_TO_TICKS(10)) != pdPASS) {
                LOG_WARN("MOCK", "sensorQueue full — record dropped (P1 discard)");
            }
        }

        // =====================================================================
        // PART 2 — Simulate Node C (Command ACK)
        // =====================================================================

        // Give the Decision Engine time to wake up, process the record,
        // run the deficit math, and push a NodeCCommand if warranted.
        vTaskDelay(pdMS_TO_TICKS(CMD_POLL_WAIT_MS));

        NodeCCommand cmd;
        memset(&cmd, 0, sizeof(cmd));

        // Non-blocking receive — if the Decision Engine did not issue a command
        // (e.g. deficit was below threshold), we skip the ACK simulation
        // entirely and move straight to the next injection cycle.
        if (commandQueue != nullptr &&
            xQueueReceive(commandQueue, &cmd, pdMS_TO_TICKS(200)) == pdPASS) {

            LOG_INFO("MOCK", "Node C received command: id=%s | target=%.1f L | max=%u s",
                     cmd.command_id, cmd.target_volume_l, cmd.max_runtime_sec);

            // Simulate pump running (physical delay)
            vTaskDelay(pdMS_TO_TICKS(2000));

            // Build feedback struct — mirrors what the real Node C firmware sends
            NodeCFeedback feedback;
            memset(&feedback, 0, sizeof(feedback));

            strncpy(feedback.command_id, cmd.command_id, sizeof(feedback.command_id) - 1);
            feedback.delivered_volume_l = cmd.target_volume_l;
            feedback.flow_rate_lpm      = MOCK_FLOW_RATE_LPM;
            feedback.battery_voltage    = 12.6f;

            // Compute runtime: clamp to max_runtime_sec so the mock doesn't
            // report an impossible runtime for large volumes.
            float runtime_f = (cmd.target_volume_l / MOCK_FLOW_RATE_LPM) * 60.0f;
            feedback.runtime_sec = (uint16_t)fminf(runtime_f, (float)cmd.max_runtime_sec);

            // NODEC_PUMP_STOPPED (0x02): pump completed its run cleanly.
            feedback.status_flag = NODEC_PUMP_STOPPED;

            LOG_INFO("MOCK", "Node C ACK: delivered=%.1f L | runtime=%u s",
                     feedback.delivered_volume_l, feedback.runtime_sec);

            // feedbackQueue — P0 policy (FMEA): block indefinitely, NEVER drop.
            // Irrigation audit integrity depends on this reaching decision_task.
            if (feedbackQueue != nullptr) {
                xQueueSend(feedbackQueue, &feedback, portMAX_DELAY);
            }
        } else {
            LOG_INFO("MOCK", "No command from Decision Engine this cycle (deficit below gate or gate blocked)");
        }

        // =====================================================================
        // Wait for the next sensor injection cycle — chunked, with periodic
        // health pings during the wait.
        //
        // MockTask stands in for vLoraTask/vUartTask/vEnvironmentTask in
        // DEVELOPMENT_MODE, so it's responsible for pinging their watchdog
        // slots too. Previously that ping only happened once per full loop
        // iteration — fine for a task whose own cadence matches its
        // watchdog timeout, but HTASK_LORA's timeout (15000ms,
        // TASK_TIMEOUT_MS in health_task.h) is far shorter than
        // SENSOR_POLL_INTERVAL_MS (60000ms in DEV mode). That mismatch
        // guaranteed LORA would exceed its timeout on almost every health
        // cycle and eventually force a REBOOT_WATCHDOG, regardless of
        // whether anything was actually wrong. Pinging every
        // MOCK_PING_INTERVAL_MS during the wait keeps all three slots
        // comfortably inside their real timeouts.
        // =====================================================================
        LOG_INFO("MOCK", "Next injection in %lu ms", (unsigned long)SENSOR_POLL_INTERVAL_MS);
        const uint32_t MOCK_PING_INTERVAL_MS = 5000UL;   // well under HTASK_LORA's 15000ms timeout
        uint32_t waited = 0;
        while (waited < SENSOR_POLL_INTERVAL_MS) {
            uint32_t chunk = SENSOR_POLL_INTERVAL_MS - waited;
            if (chunk > MOCK_PING_INTERVAL_MS) chunk = MOCK_PING_INTERVAL_MS;
            vTaskDelay(pdMS_TO_TICKS(chunk));
            waited += chunk;
            healthPing(HTASK_LORA);
            healthPing(HTASK_UART);
            healthPing(HTASK_ENV);
        }
    }
}

#endif // DEVELOPMENT_MODE
