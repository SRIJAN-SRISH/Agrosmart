/**
 * @file    lora_task.cpp
 * @brief   AgroSmart Node B — FreeRTOS LoRa Transceiver Task (Production)
 *
 * Radio topology:
 *   Node A → Node B : LoRaSensorPacket (PACKET_SENSOR_DATA)
 *   Node B → Node C : LoRaCommandPacket (PACKET_IRRIGATION_COMMAND)
 *   Node C → Node B : LoRaFeedbackPacket (PACKET_IRRIGATION_COMPLETE)
 *   Node C → Node B : LoRaHeartbeatPacket (PACKET_HEARTBEAT)
 *
 * Queue FMEA policies (architecture.md):
 *   sensorQueue    P1 — drop newest if full   (10 ms timeout on send)
 *   commandQueue   P0 — never block receiver  (0 ms peek/receive)
 *   feedbackQueue  P0 — never drop            (portMAX_DELAY on send)
 *   heartbeatQueue P2 — fire-and-forget       (0 ms timeout on send)
 *
 * Environment injection:
 *   After CRC-validating a Node A packet, injectEnvironmentData() stamps
 *   local BMP280 + SEN0575 readings into the MasterSensorRecord BEFORE
 *   it enters sensorQueue. NAN readings set the appropriate Node B
 *   health_flag bits without discarding the soil telemetry.
 *
 * SPI mutex:
 *   lockSPI() / unlockSPI() (sync.h) are held around every LoRa call
 *   to prevent conflicts with the SD card task on the shared SPI bus.
 */

#include <Arduino.h>
#include <LoRa.h>
#include <string.h>
#include <math.h>

#include "config.h"
#include "pins.h"
#include "queues.h"
#include "sync.h"
#include "enums.h"
#include "structs.h"
#include "lora_protocol.h"
#include "environment_task.h"
#include "tasks.h"
#include "health_task.h"
#include "logger.h"

// ─────────────────────────────────────────────────────────────────────────────
// Forward declarations
// ─────────────────────────────────────────────────────────────────────────────
static uint16_t crc16Modbus(const uint8_t *data, size_t length);
static void     handleSensorPacket(const uint8_t *buf, int size);
static void     handleFeedbackPacket(const uint8_t *buf, int size);
static void     handleHeartbeatPacket(const uint8_t *buf, int size);
static bool     transmitCommand(const NodeCCommand &cmd, uint16_t &seqCounter);

// ─────────────────────────────────────────────────────────────────────────────
// CRC-16/Modbus
// ─────────────────────────────────────────────────────────────────────────────
static uint16_t crc16Modbus(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 0x0001) { crc >>= 1; crc ^= 0xA001; }
            else              { crc >>= 1; }
        }
    }
    return crc;
}

// ─────────────────────────────────────────────────────────────────────────────
// handleSensorPacket — Node A → Node B
// Validates CRC, injects Node B env data, sets health_flag bits, enqueues.
// ─────────────────────────────────────────────────────────────────────────────
static void handleSensorPacket(const uint8_t *buf, int size)
{
    if (size != (int)sizeof(LoRaSensorPacket)) {
        LOG_WARN("LORA", "REJECTED sensor pkt: size %d != %d",
                 size, (int)sizeof(LoRaSensorPacket));
        return;
    }

    LoRaSensorPacket pkt;
    memcpy(&pkt, buf, sizeof(pkt));

    // Magic byte check
    if (pkt.header.magic_byte != LORA_MAGIC_BYTE) {
        LOG_WARN("LORA", "REJECTED: bad magic 0x%02X", pkt.header.magic_byte);
        return;
    }

    // CRC check (over all bytes except the last 2)
    uint16_t computed = crc16Modbus(buf, sizeof(LoRaSensorPacket) - 2);
    uint16_t received;
    memcpy(&received, &pkt.crc16, sizeof(uint16_t));
    if (computed != received) {
        LOG_WARN("LORA", "REJECTED: CRC fail computed=0x%04X received=0x%04X",
                 computed, received);
        return;
    }

    MasterSensorRecord rec;
    memcpy(&rec, &pkt.payload, sizeof(rec));

    // ── Node B local environment injection ───────────────────────────────────
    // injectEnvironmentData() reads the mutex-protected cache filled by
    // vEnvironmentTask. NAN is returned if a sensor is offline.
    float envTemp, envPressure, envRainfall;
    injectEnvironmentData(envTemp, envPressure, envRainfall);

    rec.air_temp = envTemp;
    rec.pressure = envPressure;
    rec.rainfall = envRainfall;

    // ── Augment health_flag with Node B peripheral status ────────────────────
    // Node A's own fault bits are already set in rec.health_flag.
    // We OR in Node B bits — never overwrite Node A's bits.
    if (isnan(envTemp) || isnan(envPressure)) {
        rec.health_flag |= HEALTH_NODEB_BMP280_FAULT;
    }
    if (isnan(envRainfall)) {
        rec.health_flag |= HEALTH_NODEB_RAIN_FAULT;
    }
    if (!rec.gps_valid) {
        rec.health_flag |= HEALTH_NODEA_GPS_FAULT;
    }

    LOG_INFO("LORA", "Node A pkt valid | id=%s | VWC=%.1f%% | AirT=%s | Rain=%s | flags=0x%02X",
             rec.record_id, rec.vwc_percent,
             isnan(envTemp)     ? "OFFLINE" : String(envTemp, 1).c_str(),
             isnan(envRainfall) ? "OFFLINE" : String(envRainfall, 2).c_str(),
             rec.health_flag);

    // ── FMEA P1: drop newest if sensorQueue is full ──────────────────────────
    if (sensorQueue != nullptr) {
        if (xQueueSend(sensorQueue, &rec, pdMS_TO_TICKS(10)) != pdPASS) {
            LOG_WARN("LORA", "sensorQueue full — record dropped (P1 policy)");
        } else {
            reportNodeASeen((int8_t)LoRa.packetRssi());
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// handleFeedbackPacket — Node C → Node B (pump completion report)
// FMEA P0: feedbackQueue must NEVER drop — portMAX_DELAY on send.
// ─────────────────────────────────────────────────────────────────────────────
static void handleFeedbackPacket(const uint8_t *buf, int size)
{
    if (size != (int)sizeof(LoRaFeedbackPacket)) {
        LOG_WARN("LORA", "REJECTED feedback pkt: size %d != %d",
                 size, (int)sizeof(LoRaFeedbackPacket));
        return;
    }

    LoRaFeedbackPacket pkt;
    memcpy(&pkt, buf, sizeof(pkt));

    uint16_t computed = crc16Modbus(buf, sizeof(LoRaFeedbackPacket) - 2);
    uint16_t received;
    memcpy(&received, &pkt.crc16, sizeof(uint16_t));
    if (computed != received) {
        LOG_WARN("LORA", "REJECTED feedback CRC fail");
        return;
    }

    NodeCFeedback fb;
    memcpy(&fb, &pkt.payload, sizeof(fb));

    LOG_INFO("LORA", "Node C feedback | cmd=%s | delivered=%.1f L | runtime=%u s | status=0x%02X",
             fb.command_id, fb.delivered_volume_l, fb.runtime_sec, fb.status_flag);

    // P0 policy: block until feedbackQueue has space — irrigation audit MUST land
    if (feedbackQueue != nullptr) {
        xQueueSend(feedbackQueue, &fb, portMAX_DELAY);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// handleHeartbeatPacket — Node C → Node B (alive ping)
// FMEA P2: fire-and-forget, drop if full — not mission-critical
// ─────────────────────────────────────────────────────────────────────────────
static void handleHeartbeatPacket(const uint8_t *buf, int size)
{
    if (size != (int)sizeof(LoRaHeartbeatPacket)) {
        return;
    }

    LoRaHeartbeatPacket pkt;
    memcpy(&pkt, buf, sizeof(pkt));

    uint16_t computed = crc16Modbus(buf, sizeof(LoRaHeartbeatPacket) - 2);
    uint16_t received;
    memcpy(&received, &pkt.crc16, sizeof(uint16_t));
    if (computed != received) return;

    NodeCHeartbeat hb;
    memcpy(&hb, &pkt.payload, sizeof(hb));

    LOG_INFO("LORA", "Node C heartbeat | bat=%.2f V | status=0x%02X",
             hb.battery_voltage, hb.status_flag);

    if (heartbeatQueue != nullptr) {
        xQueueSend(heartbeatQueue, &hb, 0); // P2: drop if full
    }

    reportNodeCSeen(hb.battery_voltage, hb.status_flag, (int8_t)LoRa.packetRssi());
}

// ─────────────────────────────────────────────────────────────────────────────
// transmitCommand — Node B → Node C
// Wraps NodeCCommand in a LoRaCommandPacket, signs with CRC, transmits.
// Returns true if LoRa.endPacket() reported success.
// ─────────────────────────────────────────────────────────────────────────────
static bool transmitCommand(const NodeCCommand &cmd, uint16_t &seqCounter)
{
    LoRaCommandPacket txPkt;
    memset(&txPkt, 0, sizeof(txPkt));

    txPkt.header.magic_byte       = LORA_MAGIC_BYTE;
    txPkt.header.protocol_version = LORA_PROTOCOL_VERSION;
    txPkt.header.sender_id        = NODE_B_ADDRESS;
    txPkt.header.receiver_id      = NODE_C_ADDRESS;
    txPkt.header.packet_type      = PACKET_IRRIGATION_COMMAND;
    txPkt.header.payload_length   = sizeof(NodeCCommand);
    // Emergency stop sentinel: target_volume_l == -1 → FLAG_EMERGENCY, no ACK wait
    bool isEmergencyStop = (cmd.target_volume_l < 0.0f);
    txPkt.header.flags       = isEmergencyStop ? FLAG_EMERGENCY : FLAG_ACK_REQUIRED;
    txPkt.header.sequence_id = ++seqCounter;

    memcpy(&txPkt.payload, &cmd, sizeof(NodeCCommand));

    txPkt.crc16 = crc16Modbus((uint8_t *)&txPkt, sizeof(LoRaCommandPacket) - 2);

    if (!lockSPI(500)) {
        LOG_ERROR("LORA", "SPI mutex timeout — command not sent");
        return false;
    }

    LoRa.beginPacket();
    LoRa.write((uint8_t *)&txPkt, sizeof(LoRaCommandPacket));
    bool ok = (bool)LoRa.endPacket();

    unlockSPI();

    if (ok) {
        LOG_INFO("LORA", "Command sent | id=%s | vol=%.1f L | seq=%u",
                 cmd.command_id, cmd.target_volume_l, seqCounter);
    } else {
        LOG_ERROR("LORA", "LoRa.endPacket() failed");
    }

    return ok;
}

// ─────────────────────────────────────────────────────────────────────────────
// vLoraTask
// ─────────────────────────────────────────────────────────────────────────────
void vLoraTask(void *pvParameters)
{
    (void)pvParameters;

    LOG_INFO("LORA", "Booting LoRa transceiver...");
    LOG_DEBUG("LORA", "Expected LoRaSensorPacket size  : %d bytes", (int)sizeof(LoRaSensorPacket));
    LOG_DEBUG("LORA", "Expected LoRaCommandPacket size : %d bytes", (int)sizeof(LoRaCommandPacket));
    LOG_DEBUG("LORA", "Expected LoRaFeedbackPacket size: %d bytes", (int)sizeof(LoRaFeedbackPacket));

    LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_IRQ_PIN);

    if (!lockSPI(2000)) {
        LOG_ERROR("LORA", "CRITICAL: SPI mutex unavailable at init. Task halted.");
        vTaskDelete(NULL);
        return;
    }

    if (!LoRa.begin((long)LORA_FREQUENCY_HZ)) {
        unlockSPI();
        LOG_ERROR("LORA", "CRITICAL: LoRa.begin() failed. Check SPI wiring.");
        vTaskDelete(NULL);
        return;
    }

    LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
    LoRa.setSignalBandwidth((long)LORA_SIGNAL_BANDWIDTH);
    LoRa.setCodingRate4(LORA_CODING_RATE);
    LoRa.setSyncWord(LORA_SYNC_WORD);
    LoRa.setPreambleLength(LORA_PREAMBLE_LENGTH);
    LoRa.setTxPower(LORA_TX_POWER_DBM);

    unlockSPI();

    LOG_INFO("LORA", "Hardware online. Transceiver active.");

    uint16_t outboundSeq = 0;

    for (;;) {
        healthPing(HTASK_LORA);

        // ── 1. TRANSMIT PHASE: Check commandQueue for outbound irrigation command ──
        // commandQueue P0: non-blocking receive on our side — we consume and send.
        NodeCCommand cmd;
        if (commandQueue != nullptr &&
            xQueueReceive(commandQueue, &cmd, 0) == pdPASS)
        {
            transmitCommand(cmd, outboundSeq);
        }

        // ── 2. RECEIVE PHASE: Parse any incoming packet ───────────────────────────
        if (!lockSPI(50)) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        int packetSize = LoRa.parsePacket();

        if (packetSize > 0) {
            // Read raw bytes
            uint8_t rxBuf[MAX_LORA_PACKET_SIZE];
            int bytesRead = 0;
            while (LoRa.available() && bytesRead < (int)sizeof(rxBuf)) {
                rxBuf[bytesRead++] = (uint8_t)LoRa.read();
            }

            unlockSPI();

            // Peek at packet_type in the header to route correctly
            if (bytesRead >= (int)sizeof(LoRaHeader)) {
                LoRaHeader hdr;
                memcpy(&hdr, rxBuf, sizeof(hdr));

                if (hdr.magic_byte != LORA_MAGIC_BYTE) {
                    LOG_WARN("LORA", "Dropped: bad magic 0x%02X", hdr.magic_byte);
                } else {
                    switch (hdr.packet_type) {
                        case PACKET_SENSOR_DATA:
                            handleSensorPacket(rxBuf, bytesRead);
                            break;
                        case PACKET_IRRIGATION_COMPLETE:
                            handleFeedbackPacket(rxBuf, bytesRead);
                            break;
                        case PACKET_HEARTBEAT:
                            handleHeartbeatPacket(rxBuf, bytesRead);
                            break;
                        default:
                            LOG_WARN("LORA", "Unknown packet_type=0x%02X, dropped",
                                     hdr.packet_type);
                            break;
                    }
                }
            } else {
                LOG_WARN("LORA", "Dropped: too short (%d bytes)", bytesRead);
            }

        } else {
            unlockSPI();
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
