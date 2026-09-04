/**
 * @file    config_manager.cpp
 * @brief   AgroSmart Node B — Farm Profile SD Loader / Saver
 *
 * Pre-loaded file: /CONFIG/farm_profile.json
 *
 * The operator places this file on the SD card before field deployment.
 * Node B reads it at every boot. If the file is absent or invalid, the
 * system falls back to the hardcoded defaults in Agrosmart_nodeB.ino and
 * logs a warning — it does NOT halt.
 *
 * JSON schema (all fields required):
 * {
 *   "profile_version":      1,
 *   "farm_id":              1,
 *   "field_id":             1,
 *   "field_area_m2":        1200.0,
 *   "crop_id":              0,
 *   "growth_stage":         2,
 *   "root_depth_m":         0.4,
 *   "field_capacity_vwc":   32.0,
 *   "wilting_point_vwc":    14.0,
 *   "mad_percent":          50.0,
 *   "irrigation_efficiency":82.0,
 *   "max_daily_l":          6000.0,
 *   "max_dose_l":           2000.0,
 *   "rain_lock_mm":         5.0,
 *   "stab_delay_min":       90,
 *   "default_zone_id":      1
 * }
 *
 * Caller must hold spiMutex before calling either function.
 * SD.begin() must have already succeeded.
 */

#include <Arduino.h>
#include <SD.h>
#include <ArduinoJson.h>
#include "config_manager.h"
#include "logger.h"
#include "structs.h"
#include "enums.h"

#define PROFILE_PATH "/CONFIG/farm_profile.json"

// ─────────────────────────────────────────────────────────────────────────────
// loadProfileFromSD
// ─────────────────────────────────────────────────────────────────────────────
bool loadProfileFromSD(FarmProfile* p)
{
    if (p == nullptr) return false;

    if (!SD.exists(PROFILE_PATH)) {
        LOG_WARN("CONFIG", "%s not found on SD card.", PROFILE_PATH);
        return false;
    }

    File f = SD.open(PROFILE_PATH, FILE_READ);
    if (!f) {
        LOG_ERROR("CONFIG", "Cannot open %s for reading.", PROFILE_PATH);
        return false;
    }

    // Read entire file into a stack buffer (farm_profile.json is < 512 bytes)
    char buf[512];
    size_t len = 0;
    while (f.available() && len < sizeof(buf) - 1) {
        buf[len++] = (char)f.read();
    }
    buf[len] = '\0';
    f.close();

    StaticJsonDocument<512> doc;
    DeserializationError err = deserializeJson(doc, buf);
    if (err) {
        LOG_ERROR("CONFIG", "JSON parse error: %s", err.c_str());
        return false;
    }

    // Validate required keys exist before touching *p
    const char* required[] = {
        "field_capacity_vwc", "wilting_point_vwc", "mad_percent",
        "irrigation_efficiency", "root_depth_m", "field_area_m2",
        "max_daily_l", "max_dose_l"
    };
    for (const char* key : required) {
        if (!doc.containsKey(key)) {
            LOG_ERROR("CONFIG", "Missing required key: %s", key);
            return false;
        }
    }

    memset(p, 0, sizeof(FarmProfile));

    p->profile_version      = doc["profile_version"]      | 1;
    p->farm_id              = doc["farm_id"]               | (uint16_t)1;
    p->field_id             = doc["field_id"]              | (uint16_t)1;
    p->field_area_m2        = doc["field_area_m2"]         | 1200.0f;
    p->crop_id              = doc["crop_id"]               | (uint8_t)CROP_VEGETABLE;
    p->growth_stage         = doc["growth_stage"]          | (uint8_t)STAGE_MID_SEASON;
    p->root_depth_m         = doc["root_depth_m"]          | 0.4f;
    p->field_capacity_vwc   = doc["field_capacity_vwc"]    | 32.0f;
    p->wilting_point_vwc    = doc["wilting_point_vwc"]     | 14.0f;
    p->mad_percent          = doc["mad_percent"]           | 50.0f;
    p->irrigation_efficiency= doc["irrigation_efficiency"] | 82.0f;
    p->max_daily_l          = doc["max_daily_l"]           | 6000.0f;
    p->max_dose_l           = doc["max_dose_l"]            | 2000.0f;
    p->rain_lock_mm         = doc["rain_lock_mm"]          | 5.0f;
    p->stab_delay_min       = doc["stab_delay_min"]        | (uint16_t)90;
    p->default_zone_id      = doc["default_zone_id"]       | (uint8_t)1;

    LOG_INFO("CONFIG", "farm_profile.json loaded: fc=%.1f%% wp=%.1f%% mad=%.0f%% area=%.0fm2",
             p->field_capacity_vwc, p->wilting_point_vwc,
             p->mad_percent, p->field_area_m2);
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// saveProfileToSD
// ─────────────────────────────────────────────────────────────────────────────
bool saveProfileToSD(const FarmProfile* p)
{
    if (p == nullptr) return false;

    // Ensure /CONFIG directory exists
    if (!SD.exists("/CONFIG")) {
        if (!SD.mkdir("/CONFIG")) {
            LOG_ERROR("CONFIG", "Failed to create /CONFIG directory.");
            return false;
        }
    }

    File f = SD.open(PROFILE_PATH, FILE_WRITE);
    if (!f) {
        LOG_ERROR("CONFIG", "Cannot open %s for writing.", PROFILE_PATH);
        return false;
    }

    StaticJsonDocument<512> doc;
    doc["profile_version"]       = p->profile_version;
    doc["farm_id"]               = p->farm_id;
    doc["field_id"]              = p->field_id;
    doc["field_area_m2"]         = p->field_area_m2;
    doc["crop_id"]               = p->crop_id;
    doc["growth_stage"]          = p->growth_stage;
    doc["root_depth_m"]          = p->root_depth_m;
    doc["field_capacity_vwc"]    = p->field_capacity_vwc;
    doc["wilting_point_vwc"]     = p->wilting_point_vwc;
    doc["mad_percent"]           = p->mad_percent;
    doc["irrigation_efficiency"] = p->irrigation_efficiency;
    doc["max_daily_l"]           = p->max_daily_l;
    doc["max_dose_l"]            = p->max_dose_l;
    doc["rain_lock_mm"]          = p->rain_lock_mm;
    doc["stab_delay_min"]        = p->stab_delay_min;
    doc["default_zone_id"]       = p->default_zone_id;

    serializeJsonPretty(doc, f);
    f.println();
    f.flush();
    f.close();

    LOG_INFO("CONFIG", "Profile saved to %s", PROFILE_PATH);
    return true;
}
