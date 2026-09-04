#include <Arduino.h>
#include <string.h>
#include <math.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <Preferences.h>
#include "config.h"
#include "queues.h"
#include "sync.h"
#include "config_manager.h"
#include "enums.h"
#include "structs.h"
#include "tasks.h"
#include "esp_random.h"
#include "health_task.h"
#include "environment_task.h"
#include "logger.h"

// =====================================================
// Persistent State
// =====================================================
static uint32_t current_epoch       = 0;
static uint32_t last_day_number     = 0;
static bool     day_initialized     = false;

// Random boot token — ensures decision_id uniqueness even at epoch=0 (cold boot,
// RTC not yet synced). Changes on every power-on so rows never collide.
static uint32_t decisionBootToken   = 0;

static float    daily_total_l       = 0.0f;
static float    pending_volume_l    = 0.0f;
static uint32_t last_irrigation_epoch = 0;

// =====================================================
// NVS: pending_volume_l survives power loss mid-irrigation
// Namespace "decision", key "pending_l" (float stored as uint32 raw bits)
// =====================================================
static Preferences decisionNvs;

static void nvsSavePending(float val)
{
    decisionNvs.begin("decision", false);
    uint32_t bits;
    memcpy(&bits, &val, 4);
    decisionNvs.putUInt("pending_l", bits);
    decisionNvs.end();
}

static float nvsLoadPending()
{
    decisionNvs.begin("decision", true);
    uint32_t bits = decisionNvs.getUInt("pending_l", 0);
    decisionNvs.end();
    float val;
    memcpy(&val, &bits, 4);
    // Reject NaN / negative / unreasonably large values from corrupt NVS
    if (isnan(val) || val < 0.0f || val > 100000.0f) return 0.0f;
    return val;
}

// =====================================================
// Validation Helpers
// =====================================================
static bool validateFarmProfile(const FarmProfile &p)
{
    if (p.field_capacity_vwc <= p.wilting_point_vwc)  return false;
    if (p.mad_percent        < MIN_VALID_MAD_PERCENT  ||
        p.mad_percent        > MAX_VALID_MAD_PERCENT)  return false;
    if (p.irrigation_efficiency < MIN_VALID_EFFICIENCY_PERCENT ||
        p.irrigation_efficiency > MAX_VALID_EFFICIENCY_PERCENT) return false;
    if (p.root_depth_m  <= 0.0f || p.field_area_m2 <= 0.0f) return false;
    if (p.max_dose_l    <= 0.0f || p.max_daily_l   <= 0.0f) return false;
    return true;
}

static bool validateSensorRanges(const MasterSensorRecord &r)
{
    if (r.vwc_percent < MIN_VALID_VWC_PERCENT ||
        r.vwc_percent > MAX_VALID_VWC_PERCENT)           return false;
    if (r.humidity    < MIN_VALID_HUMIDITY_PERCENT ||
        r.humidity    > MAX_VALID_HUMIDITY_PERCENT)       return false;
    if (r.air_temp    < MIN_VALID_TEMP_C ||
        r.air_temp    > MAX_VALID_TEMP_C)                 return false;
    // Reject NAN environment data — BMP280 or SEN0575 offline
    if (isnan(r.air_temp) || isnan(r.pressure))           return false;
    return true;
}

// 5-sample circular median filter on VWC — dampens sensor spike noise
static float runMedianFilter(float value)
{
    static float   buffer[VWC_MEDIAN_WINDOW] = {0};
    static uint8_t index = 0;
    static uint8_t count = 0;

    buffer[index] = value;
    index = (index + 1) % VWC_MEDIAN_WINDOW;
    if (count < VWC_MEDIAN_WINDOW) count++;

    // Return raw value until window is full (warm-up phase)
    if (count < VWC_MEDIAN_WINDOW) return value;

    float sorted[VWC_MEDIAN_WINDOW];
    memcpy(sorted, buffer, sizeof(sorted));
    for (uint8_t i = 0; i < VWC_MEDIAN_WINDOW - 1; i++) {
        for (uint8_t j = 0; j < VWC_MEDIAN_WINDOW - i - 1; j++) {
            if (sorted[j] > sorted[j + 1]) {
                float tmp    = sorted[j];
                sorted[j]    = sorted[j + 1];
                sorted[j + 1] = tmp;
            }
        }
    }
    return sorted[VWC_MEDIAN_WINDOW / 2];
}

static void logDecisionEvent(const IrrigationDecision &d, uint8_t severity, const char *msg)
{
    if (logQueue == nullptr) return;

    SystemEventLog log;
    memset(&log, 0, sizeof(log));
    log.timestamp_epoch  = d.timestamp_epoch;
    log.boot_time_ms     = d.boot_time_ms;
    log.record_sequence  = d.record_sequence;
    log.timestamp_valid  = d.timestamp_valid;
    log.severity         = severity;
    strncpy(log.source, "DECISION_TASK", sizeof(log.source) - 1);
    snprintf(log.message, sizeof(log.message) - 1,
             "%s | R:0x%02X | RAW:%.1f | DEF:%.1f | L:%.1f",
             msg, d.decision_reason, d.raw_mm, d.deficit_mm, d.gross_required_l);
    pushLog(log);
}

// =====================================================
// Decision Task
// =====================================================
void vDecisionTask(void *pvParameters)
{
    // Abort at boot if the farm profile loaded from SD is structurally invalid
    if (!validateFarmProfile(activeProfile)) {
        LOG_ERROR("DECISION", "FATAL: activeProfile invalid — task halted.");
        vTaskDelete(NULL);
        return;
    }

    decisionBootToken = esp_random();

    // Restore pending volume from NVS — survives power cut mid-irrigation.
    // If the pump was running when power was lost, we still know how much
    // was commanded so we don't over-irrigate on the next boot.
    pending_volume_l = nvsLoadPending();
    if (pending_volume_l > 0.0f) {
        LOG_INFO("DECISION", "NVS: restored pending_volume_l=%.1f L",
                 pending_volume_l);
    }

    MasterSensorRecord sensorRecord;
    NodeCFeedback      feedback;
    uint32_t           commandCounter = 0;

    while (true)
    {
        // ── 1. Process all pending Node C feedback first ──────────────────────
        // feedbackQueue P0: receive every ACK before processing new sensor data
        // so daily_total_l and pending_volume_l are current when the gate runs.
        if (feedbackQueue != nullptr) {
            while (xQueueReceive(feedbackQueue, &feedback, 0) == pdPASS) {
                // Use delivered volume, but always clamp pending to zero
                // (covers partial deliveries and fault/zero-volume ACKs)
                daily_total_l  += feedback.delivered_volume_l;
                float reduce    = (feedback.delivered_volume_l > 0.0f)
                                  ? feedback.delivered_volume_l
                                  : pending_volume_l;  // fault ACK: clear all pending
                pending_volume_l -= reduce;
                if (pending_volume_l < 0.0f) pending_volume_l = 0.0f;
                nvsSavePending(pending_volume_l);  // persist — 0 clears the gate
                last_irrigation_epoch = current_epoch;

                // Forward to SD /IRRIGATION CSV (P2: fire-and-forget)
                if (irrigationLogQueue != nullptr) {
                    IrrigationFeedbackLog irrLog;
                    irrLog.timestamp_epoch = current_epoch;
                    irrLog.feedback        = feedback;
                    xQueueSend(irrigationLogQueue, &irrLog, 0);
                }

                LOG_INFO("DECISION", "Node C ACK: delivered=%.1f L | runtime=%u s | "
                         "daily_total=%.1f L | pending=%.1f L",
                         feedback.delivered_volume_l, feedback.runtime_sec,
                         daily_total_l, pending_volume_l);
            }
        }

        // ── 2. Wait for next sensor record (100 ms timeout) ──────────────────
        if (sensorQueue == nullptr ||
            xQueueReceive(sensorQueue, &sensorRecord, pdMS_TO_TICKS(100)) != pdPASS)
        {
            healthPing(HTASK_DEC);   // still alive even when queue is dry
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // Ping AFTER successfully receiving — proves the loop is iterating
        healthPing(HTASK_DEC);

        // Epoch priority: 1) Node A RTC (most accurate — GPS-synced)
        //                 2) Node B DS3231 (local fallback)
        //                 3) Keep last known epoch (better than 0)
        if (sensorRecord.timestamp_valid != 0) {
            current_epoch = sensorRecord.timestamp_epoch;
        } else {
            uint32_t nodeBEpoch = getNodeBEpoch();
            if (nodeBEpoch != 0) current_epoch = nodeBEpoch;
            // else: keep current_epoch as-is — stale but better than zero
        }

        // ── 3. Build skeleton decision record ────────────────────────────────
        IrrigationDecision decision{};

        // Unique PK: use epoch when valid; fall back to boot token + millis()
        // so cold-boot decisions (epoch=0) never collide across reboots.
        if (current_epoch != 0) {
            snprintf(decision.decision_id, sizeof(decision.decision_id),
                     "DEC_%08lX_%04lX",
                     (unsigned long)current_epoch,
                     (unsigned long)(decisionBootToken & 0xFFFF));
        } else {
            snprintf(decision.decision_id, sizeof(decision.decision_id),
                     "DEC_%04lX%08lX",
                     (unsigned long)(decisionBootToken & 0xFFFF),
                     (unsigned long)millis());
        }
        strncpy(decision.record_id,      sensorRecord.record_id,
                sizeof(decision.record_id) - 1);
        decision.timestamp_epoch  = current_epoch;
        decision.boot_time_ms     = sensorRecord.boot_time_ms;
        decision.record_sequence  = sensorRecord.record_sequence;
        decision.timestamp_valid  = sensorRecord.timestamp_valid;

        // ── 4. Safety gate cascade ────────────────────────────────────────────
        // Each gate sets reason + status then falls through to dispatch.
        // No nested else-chains — first failing gate wins.

        bool doIrrigate = false;  // set true only if ALL gates pass

        if (sensorRecord.health_flag != HEALTH_OK) {
            // Node A or Node B sensor fault — do not trust the data
            decision.decision_reason = REASON_SENSOR_FAULT;
            decision.status_flag     = IRRIGATION_BLOCKED_SAFETY;
            LOG_WARN("DECISION", "BLOCKED: sensor health_flag=0x%02X",
                     sensorRecord.health_flag);
        }
        else if (!validateSensorRanges(sensorRecord)) {
            // Out-of-range reading or NAN from offline BMP280/ENV sensor
            decision.decision_reason = REASON_SENSOR_FAULT;
            decision.status_flag     = IRRIGATION_BLOCKED_SAFETY;
            LOG_WARN("DECISION", "BLOCKED: sensor value out of valid range");
        }
        else if (!isNodeCBatteryOk()) {
            // Battery gate closed by health_task (voltage < 11.2 V or Node C dropout)
            decision.decision_reason = REASON_LOW_BATTERY;
            decision.status_flag     = IRRIGATION_BLOCKED_SAFETY;
            LOG_WARN("DECISION", "BLOCKED: Node C battery gate closed");
        }
        else {
            // ── 5. Agronomic math ─────────────────────────────────────────────
            uint32_t current_day = current_epoch / SECONDS_PER_DAY;
            if (!day_initialized || current_day != last_day_number) {
                day_initialized = true;
                last_day_number = current_day;
                daily_total_l   = 0.0f;
                LOG_INFO("DECISION", "Day rollover — daily total reset. Day=%lu",
                         (unsigned long)current_day);
            }

            float filteredVwc = runMedianFilter(sensorRecord.vwc_percent);
            float fc  = activeProfile.field_capacity_vwc  / 100.0f;
            float wp  = activeProfile.wilting_point_vwc   / 100.0f;
            float mad = activeProfile.mad_percent          / 100.0f;
            float eff = activeProfile.irrigation_efficiency / 100.0f;
            float vwc = filteredVwc                        / 100.0f;

            decision.taw_mm     = (fc - wp) * activeProfile.root_depth_m * 1000.0f;
            decision.raw_mm     = decision.taw_mm * mad;
            decision.deficit_mm = fmaxf(0.0f, (fc - vwc) * activeProfile.root_depth_m * 1000.0f);

            // VPD — requires valid air_temp (already guarded above by validateSensorRanges)
            float es_kpa  = 0.6108f * expf((17.27f * sensorRecord.air_temp) /
                                            (sensorRecord.air_temp + 237.3f));
            decision.vpd_kpa = es_kpa * (1.0f - (sensorRecord.humidity / 100.0f));

            // ── 6. Operational gates (in priority order) ──────────────────────

            // Gate A: Rain lock — field already saturated
            if (sensorRecord.rainfall >= activeProfile.rain_lock_mm &&
                filteredVwc >= activeProfile.field_capacity_vwc)
            {
                decision.decision_reason = REASON_RAIN_LOCK;
                decision.status_flag     = IRRIGATION_BLOCKED_SAFETY;
                LOG_WARN("DECISION", "RAIN LOCK: rainfall=%.1f mm  VWC=%.1f%%",
                         sensorRecord.rainfall, filteredVwc);
            }
            // Gate B: Stabilization delay — soil still absorbing previous dose
            else if (last_irrigation_epoch != 0 && current_epoch > last_irrigation_epoch &&
                     (current_epoch - last_irrigation_epoch) <
                     (uint32_t)(activeProfile.stab_delay_min * 60UL))
            {
                uint32_t remaining = (activeProfile.stab_delay_min * 60UL) -
                                     (current_epoch - last_irrigation_epoch);
                decision.decision_reason = REASON_STABILIZATION_DELAY;
                decision.status_flag     = IRRIGATION_NOT_REQUIRED;
                LOG_INFO("DECISION", "STAB DELAY: %lu s remaining",
                         (unsigned long)remaining);
            }
            // Gate C: Daily volume limit
            else if ((daily_total_l + pending_volume_l) >= activeProfile.max_daily_l)
            {
                decision.decision_reason = REASON_DAILY_LIMIT;
                decision.status_flag     = IRRIGATION_NOT_REQUIRED;
                LOG_WARN("DECISION", "DAILY LIMIT: used=%.1f + pending=%.1f >= max=%.1f L",
                         daily_total_l, pending_volume_l, activeProfile.max_daily_l);
            }
            // Gate D: Deficit not yet large enough to justify irrigation
            else if (decision.deficit_mm < (decision.raw_mm + ACTIVATION_MARGIN_MM))
            {
                decision.decision_reason = REASON_BELOW_THRESHOLD;
                decision.status_flag     = IRRIGATION_NOT_REQUIRED;
                LOG_INFO("DECISION", "BELOW THRESHOLD: deficit=%.1f mm < gate=%.1f mm",
                         decision.deficit_mm, decision.raw_mm + ACTIVATION_MARGIN_MM);
            }
            else {
                // ── All gates passed — calculate dose and issue command ────────
                float avail  = activeProfile.max_daily_l -
                               (daily_total_l + pending_volume_l);
                float liters = fminf((decision.deficit_mm / eff) *
                                      activeProfile.field_area_m2,
                                      activeProfile.max_dose_l);
                liters = fminf(liters, avail);

                if (liters > 0.0f) {
                    decision.gross_required_l    = liters;
                    decision.irrigation_required = 1;
                    decision.decision_reason     = REASON_DEFICIT_EXCEEDED;
                    decision.status_flag         = IRRIGATION_IN_PROGRESS;

                    NodeCCommand cmd{};
                    cmd.target_volume_l = liters;
                    cmd.max_runtime_sec = MAX_IRRIGATION_RUNTIME_SEC;
                    cmd.zone_id         = activeProfile.default_zone_id;
                    snprintf(cmd.command_id, sizeof(cmd.command_id),
                             "CMD_%08lX_%04lX",
                             (unsigned long)current_epoch,
                             (unsigned long)(++commandCounter));

                    if (commandQueue != nullptr &&
                        xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(10)) == pdPASS)
                    {
                        pending_volume_l += liters;
                        nvsSavePending(pending_volume_l);  // persist before LoRa TX
                        doIrrigate = true;
                        LOG_INFO("DECISION", "IRRIGATE: %.1f L → cmd=%s | "
                                 "pending=%.1f L  daily_used=%.1f L",
                                 liters, cmd.command_id,
                                 pending_volume_l, daily_total_l);
                    } else {
                        // commandQueue full — LoRa task is behind, do not double-count
                        decision.decision_reason = REASON_INTERNAL_FAULT;
                        decision.status_flag     = IRRIGATION_FAILED;
                        LOG_ERROR("DECISION", "commandQueue full — command dropped");
                    }
                }
            }
        }

        // ── 7. Audit log ──────────────────────────────────────────────────────
        uint8_t logSeverity = doIrrigate
                              ? ERR_LEVEL_1_INFO
                              : (decision.status_flag == IRRIGATION_BLOCKED_SAFETY
                                 ? ERR_LEVEL_2_OPERATIONAL
                                 : ERR_LEVEL_1_INFO);
        logDecisionEvent(decision, logSeverity, "Decision finalized");

        // ── 7b. UART dispatch to Pi Pico — live soil/NPK/env + decision ────────
        dispatchSensorPacket(sensorRecord);
        dispatchDecisionPacket(decision);

        // ── 8. Push CloudPayload to cloud + SD queues ─────────────────────────
        // Archive every record regardless of outcome — analytics require the
        // full picture including BLOCKED and NOT_REQUIRED decisions.
        {
            CloudPayload payload;
            payload.sensorRecord = sensorRecord;
            payload.decision     = decision;
            payload.hasDecision  = true;   // always — decision_reason tells the story

            // Cloud: P2 — 10 ms timeout, drop if full (cloud has its own retry cycle)
            if (cloudQueue != nullptr) {
                if (xQueueSend(cloudQueue, &payload, pdMS_TO_TICKS(10)) != pdPASS) {
                    LOG_WARN("DECISION", "cloudQueue full — payload dropped");
                }
            }

            // SD: P2 — fire-and-forget (cloud copy is primary; SD is local archive)
            if (sdQueue != nullptr) {
                if (xQueueSend(sdQueue, &payload, 0) != pdPASS) {
                    LOG_WARN("DECISION", "sdQueue full — SD payload dropped");
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
