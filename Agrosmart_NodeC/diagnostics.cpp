#include "diagnostics.h"
#include <esp_system.h>
#include <Preferences.h>

// ============================================================
// diagnostics.cpp
// NVS namespace "nodec_diag", keys "reboot_cnt" (uint32) / "last_reason"
// (uint8) — separate namespace from anything actuators/sensors/comms
// might ever persist, so a future addition elsewhere can't collide with
// this one by accident.
// ============================================================

static Preferences diagNvs;
static uint32_t rebootCount        = 0;
static uint8_t  lastResetReasonCode = 0;

static const char *resetReasonName(esp_reset_reason_t r) {
    switch (r) {
        case ESP_RST_POWERON:   return "POWERON";
        case ESP_RST_EXT:       return "EXTERNAL_PIN";
        case ESP_RST_SW:        return "SOFTWARE";
        case ESP_RST_PANIC:     return "PANIC (exception/abort)";
        case ESP_RST_INT_WDT:   return "INTERRUPT_WATCHDOG";
        case ESP_RST_TASK_WDT:  return "TASK_WATCHDOG (our esp_task_wdt fired)";
        case ESP_RST_WDT:       return "OTHER_WATCHDOG";
        case ESP_RST_DEEPSLEEP: return "DEEPSLEEP_WAKE";
        case ESP_RST_BROWNOUT:  return "BROWNOUT (supply sag)";
        case ESP_RST_SDIO:      return "SDIO";
        default:                return "UNKNOWN";
    }
}

void initDiagnostics() {
    esp_reset_reason_t reason = esp_reset_reason();
    lastResetReasonCode = (uint8_t)reason;

    diagNvs.begin("nodec_diag", false);
    rebootCount = diagNvs.getUInt("reboot_cnt", 0) + 1;
    diagNvs.putUInt("reboot_cnt", rebootCount);
    diagNvs.putUChar("last_reason", lastResetReasonCode);
    diagNvs.end();

    Serial.printf("[DIAG] Boot #%lu | reset reason: %s (0x%02X)\n",
                  (unsigned long)rebootCount, resetReasonName(reason), lastResetReasonCode);

    // Flag the reasons that matter operationally — a healthy field unit
    // should see POWERON almost exclusively. Anything else recurring is
    // a real signal, not noise.
    if (reason == ESP_RST_TASK_WDT) {
        Serial.println("[DIAG] *** Previous boot was terminated by the hardware watchdog — "
                        "loop() froze while running. Investigate if this repeats. ***");
    } else if (reason == ESP_RST_BROWNOUT) {
        Serial.println("[DIAG] *** Previous boot was a brownout — supply sagged below the "
                        "chip's operating threshold, most likely during pump/valve inrush. "
                        "Check wire gauge and power source capacity if this repeats. ***");
    } else if (reason == ESP_RST_PANIC) {
        Serial.println("[DIAG] *** Previous boot was a firmware panic/exception. ***");
    }
}

void tickDiagnosticsHeap() {
    Serial.printf("[DIAG] Free heap: %u bytes (min ever this boot: %u bytes)\n",
                  ESP.getFreeHeap(), ESP.getMinFreeHeap());
}

uint32_t getRebootCount()         { return rebootCount; }
uint8_t  getLastResetReasonCode() { return lastResetReasonCode; }
