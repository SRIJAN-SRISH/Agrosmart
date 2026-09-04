// ============================================================
// uart_task.cpp
// AgroSmart Node B — Pi Pico HMI Co-processor Interface
//
// Role:
//   Receives single-byte operator commands from the Pi Pico
//   display over Serial2 (UART2: RX=GPIO16, TX=GPIO17).
//   Translates each command into the appropriate FreeRTOS queue
//   message for the rest of the firmware to act on.
//
// Node A sensor data arrives via LoRa → lora_task, NOT here.
//
// Wire:
//   ESP32 GPIO17 (TX2) → Pi Pico GP1 (RX)
//   ESP32 GPIO16 (RX2) ← Pi Pico GP0 (TX)
// ============================================================

#include <Arduino.h>
#include "config.h"
#include "pins.h"
#include "queues.h"
#include "uart_protocol.h"
#include "enums.h"
#include "structs.h"
#include "tasks.h"
#include "health_task.h"
#include "logger.h"

extern FarmProfile activeProfile;

// ─────────────────────────────────────────────────────────────
// Pi Pico → Node B command handler
// Called for every byte received on Serial2.
// ─────────────────────────────────────────────────────────────
static void handlePiLinkCommand(uint8_t cmd)
{
    switch (cmd) {

        case PILINK_CMD_IRRIGATE_NOW: {
            NodeCCommand manualCmd{};
            snprintf(manualCmd.command_id, sizeof(manualCmd.command_id),
                     "CMD_MANUAL_%08lX", (unsigned long)millis());
            manualCmd.target_volume_l = (activeProfile.max_dose_l > 0.0f)
                                        ? activeProfile.max_dose_l : 500.0f;
            manualCmd.max_runtime_sec = MAX_IRRIGATION_RUNTIME_SEC;
            manualCmd.zone_id         = 1;

            if (commandQueue != nullptr &&
                xQueueSend(commandQueue, &manualCmd, pdMS_TO_TICKS(100)) == pdPASS)
            {
                LOG_INFO("UART", "Pi Pico: MANUAL IRRIGATE queued");
                SystemEventLog entry{};
                entry.severity     = ERR_LEVEL_1_INFO;
                entry.boot_time_ms = (uint32_t)millis();
                strncpy(entry.source,  "UART_TASK",  sizeof(entry.source)  - 1);
                strncpy(entry.message, "Manual irrigate from display",
                        sizeof(entry.message) - 1);
                pushLog(entry);
            } else {
                LOG_WARN("UART", "Pi Pico: MANUAL IRRIGATE — commandQueue full, dropped");
            }
            break;
        }

        case PILINK_CMD_EMERGENCY_STOP: {
            NodeCCommand stopCmd{};
            snprintf(stopCmd.command_id, sizeof(stopCmd.command_id),
                     "CMD_ESTOP_%08lX", (unsigned long)millis());
            stopCmd.target_volume_l = -1.0f;   // sentinel: emergency stop
            stopCmd.max_runtime_sec = 0;
            stopCmd.zone_id         = 0xFF;    // broadcast all zones

            if (commandQueue != nullptr)
                xQueueSend(commandQueue, &stopCmd, pdMS_TO_TICKS(100));

            LOG_INFO("UART", "Pi Pico: EMERGENCY STOP queued");
            SystemEventLog entry{};
            entry.severity     = ERR_LEVEL_3_CRITICAL;
            entry.boot_time_ms = (uint32_t)millis();
            strncpy(entry.source,  "UART_TASK",  sizeof(entry.source)  - 1);
            strncpy(entry.message, "EMERGENCY STOP from display",
                    sizeof(entry.message) - 1);
            pushLog(entry);
            break;
        }

        case PILINK_CMD_STATUS_REQUEST:
            // health_task dispatches PKT_HEALTH automatically every cycle;
            // config is static, so send it immediately on request (covers
            // the Pico rebooting/reconnecting after Node B is already up).
            dispatchConfigPacket(activeProfile);
            LOG_INFO("UART", "Pi Pico: STATUS REQUEST — config packet sent");
            break;

        default:
            LOG_WARN("UART", "Pi Pico: unknown command 0x%02X — ignored", cmd);
            break;
    }
}

// ─────────────────────────────────────────────────────────────
// FreeRTOS Task
// ─────────────────────────────────────────────────────────────
void vUartTask(void *pvParameters)
{
    // Serial2 is opened once in setup() (Agrosmart_nodeB.ino) before any
    // task starts — do not re-begin() it here, this task only reads it.
    LOG_INFO("UART", "Serial2 (Pi Pico link) ready at %d baud (RX=GPIO%d TX=GPIO%d)",
             PILINK_BAUD, PILINK_RX_PIN, PILINK_TX_PIN);

    for (;;) {
        healthPing(HTASK_UART);

        while (Serial2.available()) {
            handlePiLinkCommand((uint8_t)Serial2.read());
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
