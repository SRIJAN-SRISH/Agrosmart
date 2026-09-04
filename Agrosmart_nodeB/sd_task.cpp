/**
 * @file    sd_task.cpp
 * @brief   AgroSmart Node B — SD Card Logging Task
 *
 * SD card directory structure (flat — one directory deep, monthly files)
 * ───────────────────────────────────────────────────────────────────────
 *  /CONFIG/
 *      farm_profile.json         ← PRE-LOADED by operator before deployment.
 *
 *  /SENSOR/2026-06.csv           ← Fused sensor snapshot (all 19 fields).
 *  /DECISIONS/2026-06.csv        ← Decision engine outputs, paired via record_id.
 *  /IRRIGATION/2026-06.csv       ← Node C pump ACKs (commanded vs delivered).
 *  /LOGS/2026-06.csv             ← System faults and operational events.
 *  /BOOT/boot_log.csv            ← One row per power cycle (runs forever).
 *
 * Why monthly flat files?
 *  - FAT32 on low-RAM MCUs slows with deep nested directories.
 *  - Arduino SD.h struggles with dynamic multi-level mkdir at midnight.
 *  - 30-min poll → 48 rows/day → ~1440 rows/month → < 150 KB: trivially small.
 *  - One level deep: /SENSOR exists at boot; no ensureDirectoryTree() needed.
 *
 * NAN handling
 *  - NAN floats are written as an empty field (,,) not "ERR".
 *  - Pandas/MATLAB/Excel auto-cast empty CSV fields as NaN float, preserving
 *    column dtype. "ERR" casts the entire column to object/string.
 *
 * nodate fallback
 *  - When RTC is dead, files are named nodate_boot_<millis>.csv per boot.
 *  - Prevents one infinite nodate.csv from growing unbounded.
 *
 * Key rules
 *  - SPI bus shared with LoRa — every SD access holds spiMutex (sync.h).
 *  - All CSV files get a header only when first created (size == 0 guard).
 *  - open → append → close per queue item (VFS open-file limit = 5 on ESP32).
 *  - SD re-init retried every 5 s on failure; task never halts.
 *
 * Hardware
 *  SPI: MOSI=23 MISO=19 SCK=18 CS=13 (pins.h SD_CS_PIN)
 */

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <time.h>
#include <math.h>

#include "config.h"
#include "pins.h"
#include "queues.h"
#include "sync.h"
#include "structs.h"
#include "tasks.h"
#include "health_task.h"
#include "logger.h"

// ─────────────────────────────────────────────────────────────────────────────
// Internal state
// ─────────────────────────────────────────────────────────────────────────────
static bool     sdAvailable  = false;
static uint32_t bootMillis   = 0;   // captured once at vSdTask start for nodate filenames
static SPIClass spiSD(HSPI);        // Dedicated hardware SPI bus for SD Card
// ─────────────────────────────────────────────────────────────────────────────
// sanitize() — replace commas/newlines in free-text fields
// ─────────────────────────────────────────────────────────────────────────────
static void sanitize(char *str)
{
    if (!str) return;
    for (; *str; str++) {
        if (*str == ',')                *str = ';';
        if (*str == '\n' || *str == '\r') *str = ' ';
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// fmtFloat() — write decimal or EMPTY STRING for NAN
//
// Empty field (,,) is the standard CSV representation of a missing numeric
// value. Pandas, MATLAB, and Excel all parse it as NaN float automatically,
// preserving column dtype. "ERR" would cast the entire column to object/string.
// ─────────────────────────────────────────────────────────────────────────────
static void fmtFloat(char *buf, size_t bufsz, float val, uint8_t decimals = 2)
{
    if (isnan(val)) {
        buf[0] = '\0';   // empty field — produces ,, in the CSV row
    } else {
        snprintf(buf, bufsz, "%.*f", (int)decimals, (double)val);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// buildFilePath() — FLAT monthly path: /BASE/YYYY-MM.csv
//
// Only one directory deep — no recursive mkdir needed.
// Returns false when epoch == 0 (RTC offline); caller uses nodate fallback.
// ─────────────────────────────────────────────────────────────────────────────
static bool buildFilePath(uint32_t epoch, const char *base,
                          char *out, size_t outsz)
{
    if (epoch == 0) return false;
    time_t raw = (time_t)epoch;
    struct tm t;
    if (!gmtime_r(&raw, &t)) return false;
    snprintf(out, outsz, "%s/%04d-%02d.csv",
             base, t.tm_year + 1900, t.tm_mon + 1);
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// nodatePath() — per-boot fallback when RTC is offline
//
// Uses bootMillis (captured at task start) so each power cycle gets its own
// file. Prevents one unbounded nodate.csv from growing for months.
// Example: /SENSOR/nodate_boot_00045321.csv
// ─────────────────────────────────────────────────────────────────────────────
static void nodatePath(const char *base, char *out, size_t outsz)
{
    snprintf(out, outsz, "%s/nodate_boot_%08lu.csv",
             base, (unsigned long)bootMillis);
}

// ─────────────────────────────────────────────────────────────────────────────
// humanTime() — fill human-readable date/time strings from epoch
// ─────────────────────────────────────────────────────────────────────────────
static void humanTime(uint32_t epoch,
                      char *dateBuf, size_t dateSz,
                      char *timeBuf, size_t timeSz)
{
    strncpy(dateBuf, "nodate", dateSz);
    strncpy(timeBuf, "notime", timeSz);
    if (epoch == 0) return;
    time_t raw = (time_t)epoch;
    struct tm t;
    if (!gmtime_r(&raw, &t)) return;
    snprintf(dateBuf, dateSz, "%04d-%02d-%02d",
             t.tm_year + 1900, t.tm_mon + 1, t.tm_mday);
    snprintf(timeBuf, timeSz, "%02d:%02d:%02d",
             t.tm_hour, t.tm_min, t.tm_sec);
}

// ─────────────────────────────────────────────────────────────────────────────
// ensureHeader() — write CSV header only when file is new or zero bytes
// ─────────────────────────────────────────────────────────────────────────────
static void ensureHeader(const char *filename, const char *header)
{
    bool needsHeader = !SD.exists(filename);
    if (!needsHeader) {
        File probe = SD.open(filename, FILE_READ);
        if (probe) {
            needsHeader = (probe.size() == 0);
            probe.close();
        }
    }
    if (!needsHeader) return;

    File f = SD.open(filename, FILE_WRITE);
    if (f) {
        f.println(header);
        f.flush();
        f.close();
        LOG_INFO("SD", "Created: %s", filename);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// openForAppend() — ensure header then open FILE_APPEND
// Top-level directories (/SENSOR etc.) are created once at startup, so no
// recursive mkdir is needed here.
// Returns an invalid File on failure; caller checks f.operator bool().
// ─────────────────────────────────────────────────────────────────────────────
static File openForAppend(const char *filepath, const char *header)
{
    ensureHeader(filepath, header);
    File f = SD.open(filepath, FILE_APPEND);
    if (!f) {
        sdAvailable = false;
        LOG_ERROR("SD", "Cannot open %s", filepath);
    }
    return f;
}

// ─────────────────────────────────────────────────────────────────────────────
// writeSensorRow — /SENSOR/YYYY-MM.csv
//
// Complete fused snapshot: Node A soil (7 fields) + Node B weather (4 fields)
// + GPS + health_flag. One row per received sensor cycle.
//
// date and time columns are derived from timestamp_epoch for human readability.
// All float fields that are NAN (sensor offline) appear as empty fields (,,).
//
// Columns (19 data + 4 meta = 23 total):
//   record_id, timestamp_epoch, date, time,
//   vwc_pct, soil_temp_c, ec_ds_m, ph,
//   nitrogen_mg, phosphorus_mg, potassium_mg,
//   air_temp_c, humidity_pct, pressure_hpa, rainfall_mm,
//   gps_valid, latitude, longitude, health_flag
// ─────────────────────────────────────────────────────────────────────────────
static void writeSensorRow(const MasterSensorRecord &r)
{
    char filepath[64];
    if (!buildFilePath(r.timestamp_epoch, "/SENSOR", filepath, sizeof(filepath))) {
        nodatePath("/SENSOR", filepath, sizeof(filepath));
    }

    if (!lockSPI(500)) return;

    File f = openForAppend(filepath,
        "record_id,timestamp_epoch,date,time,"
        "vwc_pct,soil_temp_c,ec_ds_m,ph,"
        "nitrogen_mg,phosphorus_mg,potassium_mg,"
        "air_temp_c,humidity_pct,pressure_hpa,rainfall_mm,"
        "gps_valid,latitude,longitude,health_flag");

    if (f) {
        char date[12], time_[10];
        humanTime(r.timestamp_epoch, date, sizeof(date), time_, sizeof(time_));

        char vwc[10], st[10], ec[10], ph[10];
        char n[10],   p[10],  k[10];
        char at[10],  hum[10], pres[10], rain[10];
        char lat[14], lon[14];

        fmtFloat(vwc,  sizeof(vwc),  r.vwc_percent);
        fmtFloat(st,   sizeof(st),   r.soil_temp);
        fmtFloat(ec,   sizeof(ec),   r.ec, 3);
        fmtFloat(ph,   sizeof(ph),   r.ph, 2);
        fmtFloat(n,    sizeof(n),    r.nitrogen,   1);
        fmtFloat(p,    sizeof(p),    r.phosphorus, 1);
        fmtFloat(k,    sizeof(k),    r.potassium,  1);
        fmtFloat(at,   sizeof(at),   r.air_temp);
        fmtFloat(hum,  sizeof(hum),  r.humidity);
        fmtFloat(pres, sizeof(pres), r.pressure,   1);
        fmtFloat(rain, sizeof(rain), r.rainfall,   2);
        fmtFloat(lat,  sizeof(lat),  r.latitude,   6);
        fmtFloat(lon,  sizeof(lon),  r.longitude,  6);

        char row[384];
        snprintf(row, sizeof(row),
                 "%s,%lu,%s,%s,"
                 "%s,%s,%s,%s,"
                 "%s,%s,%s,"
                 "%s,%s,%s,%s,"
                 "%u,%s,%s,0x%02X\n",
                 r.record_id, (unsigned long)r.timestamp_epoch, date, time_,
                 vwc, st, ec, ph,
                 n, p, k,
                 at, hum, pres, rain,
                 r.gps_valid, lat, lon, r.health_flag);

        if (f.print(row) == 0) sdAvailable = false;
        f.flush();
        f.close();
    }

    unlockSPI();
}

// ─────────────────────────────────────────────────────────────────────────────
// writeDecisionRow — /DECISIONS/YYYY-MM.csv
//
// Decision engine math outputs + outcome. Paired with SENSOR via record_id FK.
// Use for deficit trend analysis, daily water budget review, gate audit.
//
// Columns:
//   decision_id, record_id, timestamp_epoch, date, time,
//   taw_mm, raw_mm, deficit_mm, vpd_kpa,
//   gross_required_l, irrigation_required,
//   decision_reason, status_flag
// ─────────────────────────────────────────────────────────────────────────────
static void writeDecisionRow(const IrrigationDecision &d)
{
    char filepath[64];
    if (!buildFilePath(d.timestamp_epoch, "/DECISIONS", filepath, sizeof(filepath))) {
        nodatePath("/DECISIONS", filepath, sizeof(filepath));
    }

    if (!lockSPI(500)) return;

    File f = openForAppend(filepath,
        "decision_id,record_id,timestamp_epoch,date,time,"
        "taw_mm,raw_mm,deficit_mm,vpd_kpa,"
        "gross_required_l,irrigation_required,"
        "decision_reason,status_flag");

    if (f) {
        char date[12], time_[10];
        humanTime(d.timestamp_epoch, date, sizeof(date), time_, sizeof(time_));

        char taw[10], raw_[10], def[10], vpd[10], grl[10];
        fmtFloat(taw,  sizeof(taw),  d.taw_mm);
        fmtFloat(raw_, sizeof(raw_), d.raw_mm);
        fmtFloat(def,  sizeof(def),  d.deficit_mm);
        fmtFloat(vpd,  sizeof(vpd),  d.vpd_kpa, 3);
        fmtFloat(grl,  sizeof(grl),  d.gross_required_l, 1);

        char row[256];
        snprintf(row, sizeof(row),
                 "%s,%s,%lu,%s,%s,"
                 "%s,%s,%s,%s,"
                 "%s,%u,"
                 "0x%02X,0x%02X\n",
                 d.decision_id, d.record_id,
                 (unsigned long)d.timestamp_epoch, date, time_,
                 taw, raw_, def, vpd,
                 grl, d.irrigation_required,
                 d.decision_reason, d.status_flag);

        if (f.print(row) == 0) sdAvailable = false;
        f.flush();
        f.close();
    }

    unlockSPI();
}

// ─────────────────────────────────────────────────────────────────────────────
// writeIrrigationRow — /IRRIGATION/YYYY-MM.csv
//
// Actual pump outcome: commanded vs delivered, runtime, flow rate, battery.
// Source of truth for efficiency calculations and flow calibration.
//
// Columns:
//   timestamp_epoch, date, time, command_id,
//   delivered_l, flow_lpm, runtime_sec, battery_v, status_flag
// ─────────────────────────────────────────────────────────────────────────────
static void writeIrrigationRow(const IrrigationFeedbackLog &log)
{
    char filepath[64];
    if (!buildFilePath(log.timestamp_epoch, "/IRRIGATION", filepath, sizeof(filepath))) {
        nodatePath("/IRRIGATION", filepath, sizeof(filepath));
    }

    if (!lockSPI(500)) return;

    File f = openForAppend(filepath,
        "timestamp_epoch,date,time,command_id,"
        "delivered_l,flow_lpm,runtime_sec,battery_v,status_flag");

    if (f) {
        char date[12], time_[10];
        humanTime(log.timestamp_epoch, date, sizeof(date), time_, sizeof(time_));

        char del[10], flw[10], bat[8];
        fmtFloat(del, sizeof(del), log.feedback.delivered_volume_l, 1);
        fmtFloat(flw, sizeof(flw), log.feedback.flow_rate_lpm, 2);
        fmtFloat(bat, sizeof(bat), log.feedback.battery_voltage, 2);

        char row[192];
        snprintf(row, sizeof(row),
                 "%lu,%s,%s,%s,"
                 "%s,%s,%u,%s,0x%02X\n",
                 (unsigned long)log.timestamp_epoch, date, time_,
                 log.feedback.command_id,
                 del, flw, log.feedback.runtime_sec, bat,
                 log.feedback.status_flag);

        if (f.print(row) == 0) sdAvailable = false;
        f.flush();
        f.close();
    }

    unlockSPI();
}

// ─────────────────────────────────────────────────────────────────────────────
// writeLogRow — /LOGS/YYYY-MM.csv
//
// System event audit trail. Severity-coded; source-tagged by subsystem.
//
// Columns:
//   timestamp_epoch, boot_time_ms, record_sequence, severity, source, message
// ─────────────────────────────────────────────────────────────────────────────
static void writeLogRow(const SystemEventLog &entry)
{
    char filepath[64];
    if (!buildFilePath(entry.timestamp_epoch, "/LOGS", filepath, sizeof(filepath))) {
        nodatePath("/LOGS", filepath, sizeof(filepath));
    }

    if (!lockSPI(500)) return;

    File f = openForAppend(filepath,
        "timestamp_epoch,boot_time_ms,record_sequence,severity,source,message");

    if (f) {
        char src[sizeof(entry.source)];
        char msg[sizeof(entry.message)];
        strncpy(src, entry.source,  sizeof(src)  - 1); src[sizeof(src)  - 1] = '\0';
        strncpy(msg, entry.message, sizeof(msg) - 1);  msg[sizeof(msg) - 1] = '\0';
        sanitize(src);
        sanitize(msg);

        char row[256];
        snprintf(row, sizeof(row),
                 "%lu,%lu,%lu,%u,%s,\"%s\"\n",
                 (unsigned long)entry.timestamp_epoch,
                 (unsigned long)entry.boot_time_ms,
                 (unsigned long)entry.record_sequence,
                 entry.severity, src, msg);

        if (f.print(row) == 0) sdAvailable = false;
        f.flush();
        f.close();
    }

    unlockSPI();
}

// ─────────────────────────────────────────────────────────────────────────────
// writeBootRecord — /BOOT/boot_log.csv
//
// One row per power cycle. File grows slowly (years of uptime = small file).
// Columns:
//   boot_millis, firmware_version, node_id, sd_capacity_mb, free_heap_bytes
// ─────────────────────────────────────────────────────────────────────────────
static void writeBootRecord(uint64_t cardMB)
{
    if (!lockSPI(500)) return;

    if (!SD.exists("/BOOT")) SD.mkdir("/BOOT");

    File f = openForAppend("/BOOT/boot_log.csv",
        "boot_millis,firmware_version,node_id,sd_capacity_mb,free_heap_bytes");

    if (f) {
        char row[128];
        snprintf(row, sizeof(row),
                 "%lu,%s,%s,%llu,%lu\n",
                 (unsigned long)millis(),
                 FIRMWARE_VERSION, NODE_ID,
                 (unsigned long long)cardMB,
                 (unsigned long)ESP.getFreeHeap());
        f.print(row);
        f.flush();
        f.close();
        LOG_INFO("SD", "Boot record written. Firmware=%s  Heap=%lu B",
                 FIRMWARE_VERSION, (unsigned long)ESP.getFreeHeap());
    }

    unlockSPI();
}

// ─────────────────────────────────────────────────────────────────────────────
// vSdTask
// ─────────────────────────────────────────────────────────────────────────────
void vSdTask(void *pvParameters)
{
    (void)pvParameters;

    bootMillis = millis();   // capture for nodate_boot_<millis> filenames

    LOG_INFO("SD", "Task starting...");

    // ── SD Initialization ─────────────────────────────────────────────────────
    spiSD.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

    if (lockSPI(3000)) {
        sdAvailable = SD.begin(SD_CS_PIN, spiSD);
        unlockSPI();
    }

    if (sdAvailable) {
        uint64_t cardMB = SD.cardSize() / (1024ULL * 1024ULL);
        LOG_INFO("SD", "Card online. Capacity: %llu MB", cardMB);

        // Pre-create all top-level directories — no midnight mkdir surprises
        if (lockSPI(1000)) {
            const char *dirs[] = { "/CONFIG", "/SENSOR", "/DECISIONS",
                                   "/IRRIGATION", "/LOGS", "/BOOT" };
            for (const char *d : dirs) {
                if (!SD.exists(d)) SD.mkdir(d);
            }
            unlockSPI();
        }

        writeBootRecord(SD.cardSize() / (1024ULL * 1024ULL));

    } else {
        LOG_WARN("SD", "Card not detected. Retrying every 5 s...");
    }

    // ── Main Loop ─────────────────────────────────────────────────────────────
    CloudPayload          payload;
    IrrigationFeedbackLog irrLog;
    SystemEventLog        logEntry;

    for (;;) {
        healthPing(HTASK_SD);

        // Re-init if card was lost
        if (!sdAvailable) {
            vTaskDelay(pdMS_TO_TICKS(5000));
            if (lockSPI(2000)) {
                sdAvailable = SD.begin(SD_CS_PIN, spiSD);
                unlockSPI();
            }
            if (sdAvailable) {
                LOG_INFO("SD", "Card recovered.");
                writeBootRecord(SD.cardSize() / (1024ULL * 1024ULL));
            }
            continue;
        }

        // Query health_task for SD storage state once per cycle
        bool storageCritical = isSdStorageCritical();

        if (storageCritical) {
            // Card nearly full — drain queues without writing sensor/decision rows.
            // LOGS and IRRIGATION are kept: they are small and safety-critical.
            // SENSOR/DECISION rows are the high-volume data; drop them to buy time.
            if (sdQueue != nullptr) {
                while (xQueueReceive(sdQueue, &payload, 0) == pdPASS) { /* discard */ }
            }
            LOG_WARN("SD", "Storage critical — SENSOR/DECISION writes suppressed");
        } else {
            // 1. SENSOR + DECISIONS (drain fully each tick — highest agronomic value)
            while (sdQueue != nullptr &&
                   xQueueReceive(sdQueue, &payload, 0) == pdPASS)
            {
                writeSensorRow(payload.sensorRecord);
                if (payload.hasDecision) writeDecisionRow(payload.decision);
            }
        }

        // 2. IRRIGATION — always write; pump ACKs are the legal/audit record
        while (irrigationLogQueue != nullptr &&
               xQueueReceive(irrigationLogQueue, &irrLog, 0) == pdPASS)
        {
            writeIrrigationRow(irrLog);
        }

        // 3. LOGS — always write, capped at 5 per tick to prevent starvation
        // Reads sdLogQueue (SD-dedicated copy) — cloud_task reads logQueue independently.
        // Every log entry is pushed to both by pushLog() in queues.h.
        uint8_t logBatch = 0;
        while (logBatch < 5 &&
               sdLogQueue != nullptr &&
               xQueueReceive(sdLogQueue, &logEntry, 0) == pdPASS)
        {
            writeLogRow(logEntry);
            logBatch++;
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
