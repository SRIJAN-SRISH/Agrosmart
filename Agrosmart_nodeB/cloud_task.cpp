/**
 * @file    cloud_task.cpp
 * @brief   AgroSmart Node B — FreeRTOS Cloud Upload Task
 *
 * Critical invariants:
 *   1. FK ORDER  — sensor_records POSTed and confirmed before irrigation_decisions.
 *   2. NON-BLOCK — WiFi loss triggers reconnect + vTaskDelay, never a spin-loop.
 *   3. MEMORY    — All JSON serialised into stack char[] buffers, never heap String.
 *   4. RETRY     — Payload stays on cloudQueue until HTTP 2xx/409 confirmed.
 *   5. IDEMPOTENT— Prefer: resolution=ignore-duplicates handles network retries.
 *   6. LOG DRAIN — Capped at LOG_DRAIN_PER_CYCLE entries per loop so cloudQueue
 *                  is never starved by a burst of system events.
 */

#include "cloud_task.h"
#include "logger.h"
#include "config.h"
#include "structs.h"
#include "queues.h"
#include "config_manager.h"
#include "health_task.h"

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// ─────────────────────────────────────────────────────────────────────────────
// Timing constants (override in config.h if needed)
// ─────────────────────────────────────────────────────────────────────────────
#ifndef CLOUD_TASK_PERIOD_MS
#define CLOUD_TASK_PERIOD_MS      5000UL
#endif

#ifndef WIFI_RECONNECT_PERIOD_MS
#define WIFI_RECONNECT_PERIOD_MS  10000UL
#endif

#ifndef HTTP_TIMEOUT_MS
#define HTTP_TIMEOUT_MS           8000
#endif

// Maximum system_logs entries drained per cycle.
// Prevents logQueue from blocking cloudQueue for hundreds of seconds
// when a burst of events fills the 50-entry queue (50 × 8 s = 400 s otherwise).
#ifndef LOG_DRAIN_PER_CYCLE
#define LOG_DRAIN_PER_CYCLE       5
#endif

// ─────────────────────────────────────────────────────────────────────────────
// Supabase REST endpoints
// NOTE: SUPABASE_URL in config.h already ends with "/rest/v1/"
//       so these must NOT repeat that prefix.
// ─────────────────────────────────────────────────────────────────────────────
#define EP_SENSOR_RECORDS       "sensor_records"
#define EP_IRRIGATION_DECISIONS "irrigation_decisions"
#define EP_SYSTEM_LOGS          "system_logs"

// Shared WiFi state — health_task reads this to avoid calling WiFi.status()
// from a different context while cloud_task may be inside WiFi.begin().
volatile bool g_wifiConnected = false;

// ─────────────────────────────────────────────────────────────────────────────
// Forward declarations
// ─────────────────────────────────────────────────────────────────────────────
static bool ensureWiFi(void);
static int  postJson(const char* endpoint, const char* body);
static bool uploadSensorRecord(const MasterSensorRecord& rec);
static bool uploadIrrigationDecision(const IrrigationDecision& dec);
static bool uploadSystemLog(const SystemEventLog& log);

// ─────────────────────────────────────────────────────────────────────────────
// vCloudTask
// ─────────────────────────────────────────────────────────────────────────────
void vCloudTask(void* pvParameters) {
    (void)pvParameters;

    // ── Initial WiFi connection ───────────────────────────────────────────────
    vTaskDelay(pdMS_TO_TICKS(2000));   // let scheduler settle after boot

    LOG_INFO("CLOUD", "Starting WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 15) {
        delay(500);
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        LOG_INFO("CLOUD", "WiFi connected. IP: %s", WiFi.localIP().toString().c_str());
    } else {
        LOG_ERROR("CLOUD", "WiFi failed at boot — will retry in loop.");
    }

    // ── Main loop ─────────────────────────────────────────────────────────────
    for (;;) {
        healthPing(HTASK_CLOUD);

        // 1. WiFi health check
        if (!ensureWiFi()) {
            vTaskDelay(pdMS_TO_TICKS(WIFI_RECONNECT_PERIOD_MS));
            continue;
        }

        // 2. Drain logQueue — capped at LOG_DRAIN_PER_CYCLE per loop
        //    so cloudQueue is never starved during a burst of system events.
        {
            SystemEventLog logEntry;
            int drained = 0;
            while (drained < LOG_DRAIN_PER_CYCLE &&
                   logQueue != nullptr &&
                   xQueueReceive(logQueue, &logEntry, 0) == pdTRUE)
            {
                if (!uploadSystemLog(logEntry)) {
                    LOG_WARN("CLOUD", "system_log POST failed (discarded).");
                }
                drained++;
            }
        }

        // 3. Upload one CloudPayload per cycle — FK-ordered, peek-then-dequeue
        {
            CloudPayload payload;
            if (cloudQueue != nullptr &&
                xQueuePeek(cloudQueue, &payload, 0) == pdTRUE)
            {
                // 3a. POST sensor_records first (FK anchor row)
                bool sensorOk = uploadSensorRecord(payload.sensorRecord);

                if (!sensorOk) {
                    LOG_ERROR("CLOUD", "sensor_records FAILED (id=%s) — retrying next cycle.", payload.sensorRecord.record_id);
                } else {

                    // 3b. POST irrigation_decisions only after sensor confirmed
                    bool decisionOk = true;
                    if (payload.hasDecision) {
                        decisionOk = uploadIrrigationDecision(payload.decision);
                        if (!decisionOk) {
                            LOG_ERROR("CLOUD", "irrigation_decisions FAILED (id=%s) — retrying.", payload.decision.decision_id);
                        }
                    }

                    if (decisionOk) {
                        // Both confirmed — dequeue and verify the slot was consumed
                        if (xQueueReceive(cloudQueue, &payload, 0) != pdTRUE) {
                            LOG_ERROR("CLOUD", "cloudQueue dequeue unexpectedly failed.");
                        } else {
                            LOG_INFO("CLOUD", "Payload uploaded and dequeued.");
                        }
                    }
                }
            }
        }

        // 4. Yield
        vTaskDelay(pdMS_TO_TICKS(CLOUD_TASK_PERIOD_MS));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// ensureWiFi
// ─────────────────────────────────────────────────────────────────────────────
static bool ensureWiFi(void) {
    if (WiFi.status() == WL_CONNECTED) {
        g_wifiConnected = true;
        return true;
    }
    LOG_WARN("CLOUD", "WiFi dropped. Reconnecting...");
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 10) {
        delay(500);
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        LOG_INFO("CLOUD", "WiFi reconnected. IP: %s", WiFi.localIP().toString().c_str());
        return true;
    } else {
        LOG_ERROR("CLOUD", "WiFi still down.");
        return false;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// postJson
// All callers pass a stack char[] — no heap allocation (architecture §3).
// url[] is 192 bytes: SUPABASE_URL (≤80) + endpoint (≤25) + null = safe margin.
// ─────────────────────────────────────────────────────────────────────────────
static int postJson(const char* endpoint, const char* body) {
    HTTPClient http;

    char url[192];
    snprintf(url, sizeof(url), "%s%s", SUPABASE_URL, endpoint);

    http.begin(url);
    http.setTimeout(HTTP_TIMEOUT_MS);
    http.addHeader("Content-Type",  "application/json");
    http.addHeader("apikey",        SUPABASE_ANON_KEY);
    http.addHeader("Authorization", "Bearer " SUPABASE_ANON_KEY);
    http.addHeader("Prefer",        "return=minimal,resolution=ignore-duplicates");

    int code = http.POST((uint8_t*)body, strlen(body));

    if (code < 0) {
        LOG_ERROR("CLOUD", "HTTP error on /%s: %s", endpoint, http.errorToString(code).c_str());
    } else {
        LOG_DEBUG("CLOUD", "POST /%s → %d", endpoint, code);
    }

    http.end();
    return code;
}

// ─────────────────────────────────────────────────────────────────────────────
// uploadSensorRecord
// StaticJsonDocument<1024>: 19 fields × ~16 bytes/node + key strings +
// float precision headroom — well within the 1024-byte pool.
// char body[1024]: serializeJson return value checked for truncation.
// ─────────────────────────────────────────────────────────────────────────────
static bool uploadSensorRecord(const MasterSensorRecord& rec) {
    StaticJsonDocument<1024> doc;

    doc["record_id"]       = rec.record_id;
    doc["farm_id"]         = "FARM_001";
    doc["node_id"]         = NODE_ID;
    doc["timestamp_epoch"] = rec.timestamp_epoch;

    doc["vwc_percent"]     = rec.vwc_percent;
    doc["soil_temp"]       = rec.soil_temp;
    doc["ec"]              = rec.ec;
    doc["ph"]              = rec.ph;
    doc["nitrogen"]        = rec.nitrogen;
    doc["phosphorus"]      = rec.phosphorus;
    doc["potassium"]       = rec.potassium;

    doc["air_temp"]        = rec.air_temp;
    doc["humidity"]        = rec.humidity;
    doc["pressure"]        = rec.pressure;
    doc["rainfall"]        = rec.rainfall;

    doc["gps_valid"]       = rec.gps_valid;
    doc["latitude"]        = rec.latitude;
    doc["longitude"]       = rec.longitude;
    doc["health_flag"]     = rec.health_flag;

    char body[1024];
    size_t written = serializeJson(doc, body, sizeof(body));
    if (written >= sizeof(body) - 1) {
        LOG_WARN("CLOUD", "sensor body truncated — increase body[] size.");
    }

    int code = postJson(EP_SENSOR_RECORDS, body);
    return (code == 201 || code == 200 || code == 409);
}

// ─────────────────────────────────────────────────────────────────────────────
// uploadIrrigationDecision
// StaticJsonDocument<768>: 16 fields — comfortable within 768-byte pool.
// ─────────────────────────────────────────────────────────────────────────────
static bool uploadIrrigationDecision(const IrrigationDecision& dec) {
    StaticJsonDocument<768> doc;

    doc["decision_id"]         = dec.decision_id;
    doc["record_id"]           = dec.record_id;        // FK → sensor_records
    doc["farm_id"]             = "FARM_001";            // FK → farm_profiles
    doc["node_id"]             = NODE_ID;
    doc["timestamp_epoch"]     = dec.timestamp_epoch;
    doc["boot_time_ms"]        = dec.boot_time_ms;
    doc["record_sequence"]     = dec.record_sequence;
    doc["timestamp_valid"]     = dec.timestamp_valid;
    doc["taw_mm"]              = dec.taw_mm;
    doc["raw_mm"]              = dec.raw_mm;
    doc["deficit_mm"]          = dec.deficit_mm;
    doc["gross_required_1"]    = dec.gross_required_l;  // column is _1 (one), not _l (L)
    doc["vpd_kpa"]             = dec.vpd_kpa;
    doc["irrigation_required"] = (bool)dec.irrigation_required;
    doc["decision_reason"]     = dec.decision_reason;
    doc["status_flag"]         = dec.status_flag;

    char body[768];
    size_t written = serializeJson(doc, body, sizeof(body));
    if (written >= sizeof(body) - 1) {
        LOG_WARN("CLOUD", "decision body truncated — increase body[] size.");
    }

    int code = postJson(EP_IRRIGATION_DECISIONS, body);
    return (code == 201 || code == 200 || code == 409);
}

// ─────────────────────────────────────────────────────────────────────────────
// uploadSystemLog
// farm_id and node_id included so NOT NULL constraints are satisfied.
// 409 accepted as success: duplicate timestamp rows are harmless.
// ─────────────────────────────────────────────────────────────────────────────
static bool uploadSystemLog(const SystemEventLog& log) {
    StaticJsonDocument<384> doc;

    doc["farm_id"]         = "FARM_001";
    doc["node_id"]         = NODE_ID;
    doc["timestamp_epoch"] = log.timestamp_epoch;
    doc["boot_time_ms"]    = log.boot_time_ms;
    doc["record_sequence"] = log.record_sequence;
    doc["severity"]        = log.severity;
    doc["source"]          = log.source;
    doc["message"]         = log.message;

    char body[384];
    size_t written = serializeJson(doc, body, sizeof(body));
    if (written >= sizeof(body) - 1) {
        LOG_WARN("CLOUD", "system_log body truncated — increase body[] size.");
    }

    int code = postJson(EP_SYSTEM_LOGS, body);
    return (code == 201 || code == 200 || code == 409);
}
