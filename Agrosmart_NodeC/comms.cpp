#include "comms.h"
#include <Arduino.h>
#include <LoRa.h>
#include <string.h>

#include "pins.h"
#include "config.h"
#include "enums.h"
#include "structs.h"
#include "lora_protocol.h"
#include "actuators.h"
#include "sensors.h"

// ============================================================
// comms.cpp
//
// Note on timing: LoRa.endPacket() is blocking (~150-200ms at SF9) and
// pauses tickActuators() for that duration whenever a feedback or
// heartbeat packet goes out mid-cycle. This is an accepted tradeoff,
// not an oversight — the flow meter's pulse count is captured by ISR
// completely independent of the main loop, so no water-volume data is
// lost during a transmission; the only effect is up to ~200ms of jitter
// on the ACS712 fault-sample cadence and the pump's exact stop instant,
// well within the safety margins the timing constants already carry.
// ============================================================

static uint16_t outboundSeq = 0;
static uint32_t lastHeartbeatMs = 0;

// ── Duplicate/replay guard — see handleIncomingPacket() ──────────────────
// Deliberately NOT a strict "must be increasing" check: Node B's own
// sequence counter resets to 0 on every Node B reboot, while Node C's
// memory of the last-seen value does not. A monotonic check would start
// rejecting every legitimate command the moment Node B restarts. Instead
// this only catches an EXACT repeat (same sequence_id AND same
// command_id) — the actual shape of a LoRa retransmit, without the
// cross-reboot false-positive risk.
static uint16_t lastAcceptedSeq = 0;
static char     lastAcceptedCommandId[24] = {0};
static bool     haveAcceptedAny = false;

// ── CRC-16/Modbus — MUST be byte-identical to Node B's implementation ───
static uint16_t crc16Modbus(const uint8_t *data, size_t length) {
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

// ── Transmit a LoRaFeedbackPacket (PACKET_IRRIGATION_COMPLETE) ──────────
static void sendFeedback(const NodeCFeedback &fb) {
    LoRaFeedbackPacket pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.header.magic_byte       = LORA_MAGIC_BYTE;
    pkt.header.protocol_version = LORA_PROTOCOL_VERSION;
    pkt.header.sender_id        = NODE_C_ADDRESS;
    pkt.header.receiver_id      = NODE_B_ADDRESS;
    pkt.header.packet_type      = PACKET_IRRIGATION_COMPLETE;
    pkt.header.payload_length   = sizeof(NodeCFeedback);
    pkt.header.flags            = FLAG_NONE;
    pkt.header.sequence_id      = ++outboundSeq;
    pkt.payload                 = fb;
    pkt.crc16 = crc16Modbus((uint8_t *)&pkt, sizeof(LoRaFeedbackPacket) - 2);

    LoRa.beginPacket();
    LoRa.write((uint8_t *)&pkt, sizeof(LoRaFeedbackPacket));
    bool ok = (bool)LoRa.endPacket();

    if (ok) {
        Serial.printf("[COMMS] TX feedback | cmd=%s status=0x%02X vol=%.2fL\n",
                      fb.command_id, fb.status_flag, fb.delivered_volume_l);
    } else {
        Serial.printf("[COMMS] TX FAILED — feedback for cmd=%s did not transmit. "
                      "Node B will not learn this cycle's result.\n", fb.command_id);
    }
}

// ── Immediate rejection — command never became a cycle (busy/invalid zone) ─
static void sendRejection(const char *commandId, uint8_t statusFlag) {
    NodeCFeedback fb;
    memset(&fb, 0, sizeof(fb));
    strncpy(fb.command_id, commandId, sizeof(fb.command_id) - 1);
    fb.status_flag = statusFlag;
    fb.battery_voltage = readBatteryVoltage();
    sendFeedback(fb);
}

// ── Priority-ordered status for the heartbeat — worst condition wins ─────
// enums.h's NodeCStatus has more values than the old isActuatorIdle() ?
// NODEC_OK : NODEC_PUMP_RUNNING check ever used. Node C already has the
// data for the rest; reporting it costs nothing extra on the wire since
// status_flag is a single byte either way.
static uint8_t determineHeartbeatStatus(float batteryV) {
    if (!isBatteryMonitorOk())            return NODEC_HARDWARE_FAULT;
    if (batteryV < NODEC_BATTERY_LOW_THRESHOLD) return NODEC_LOW_BATTERY;
    if (!isActuatorIdle())                return NODEC_PUMP_RUNNING;
    if (lastCycleFaulted())               return NODEC_FLOW_FAULT;
    return NODEC_OK;
}

// ── Transmit a LoRaHeartbeatPacket (PACKET_HEARTBEAT) ────────────────────
static void sendHeartbeat() {
    NodeCHeartbeat hb;
    hb.battery_voltage = readBatteryVoltage();
    hb.status_flag      = determineHeartbeatStatus(hb.battery_voltage);

    LoRaHeartbeatPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.header.magic_byte       = LORA_MAGIC_BYTE;
    pkt.header.protocol_version = LORA_PROTOCOL_VERSION;
    pkt.header.sender_id        = NODE_C_ADDRESS;
    pkt.header.receiver_id      = NODE_B_ADDRESS;
    pkt.header.packet_type      = PACKET_HEARTBEAT;
    pkt.header.payload_length   = sizeof(NodeCHeartbeat);
    pkt.header.flags            = FLAG_NONE;
    pkt.header.sequence_id      = ++outboundSeq;
    pkt.payload                 = hb;
    pkt.crc16 = crc16Modbus((uint8_t *)&pkt, sizeof(LoRaHeartbeatPacket) - 2);

    LoRa.beginPacket();
    LoRa.write((uint8_t *)&pkt, sizeof(LoRaHeartbeatPacket));
    bool ok = (bool)LoRa.endPacket();

    if (ok) {
        Serial.printf("[COMMS] TX heartbeat | batt=%.2fV status=0x%02X\n",
                      hb.battery_voltage, hb.status_flag);
    } else {
        Serial.println("[COMMS] TX FAILED — heartbeat did not transmit.");
    }
}

// ── Validate + dispatch an inbound LoRaCommandPacket ─────────────────────
static void handleIncomingPacket(int packetSize) {
    if (packetSize != (int)sizeof(LoRaCommandPacket)) {
        Serial.printf("[COMMS] REJECT — size %d != expected %d\n",
                      packetSize, (int)sizeof(LoRaCommandPacket));
        while (LoRa.available()) LoRa.read();   // drain
        return;
    }

    uint8_t rxBuf[sizeof(LoRaCommandPacket)];
    int bytesRead = 0;
    while (LoRa.available() && bytesRead < (int)sizeof(rxBuf)) {
        rxBuf[bytesRead++] = (uint8_t)LoRa.read();
    }

    LoRaCommandPacket pkt;
    memcpy(&pkt, rxBuf, sizeof(pkt));

    if (pkt.header.magic_byte != LORA_MAGIC_BYTE) {
        Serial.printf("[COMMS] REJECT — bad magic 0x%02X\n", pkt.header.magic_byte);
        return;
    }
    if (pkt.header.sender_id != NODE_B_ADDRESS) {
        Serial.printf("[COMMS] REJECT — sender 0x%02X != Node B\n", pkt.header.sender_id);
        return;
    }
    if (pkt.header.receiver_id != NODE_C_ADDRESS) {
        Serial.printf("[COMMS] IGNORE — not addressed to us (0x%02X)\n", pkt.header.receiver_id);
        return;
    }
    if (pkt.header.packet_type != PACKET_IRRIGATION_COMMAND) {
        Serial.printf("[COMMS] REJECT — packet_type 0x%02X != COMMAND\n", pkt.header.packet_type);
        return;
    }

    uint16_t computedCRC = crc16Modbus(rxBuf, sizeof(LoRaCommandPacket) - 2);
    uint16_t receivedCRC;
    memcpy(&receivedCRC, &pkt.crc16, sizeof(uint16_t));
    if (computedCRC != receivedCRC) {
        Serial.printf("[COMMS] REJECT — CRC mismatch computed=0x%04X received=0x%04X\n",
                      computedCRC, receivedCRC);
        return;
    }

    digitalWrite(LED_RX_PIN, HIGH);
    Serial.printf("[COMMS] RX valid | seq=%u cmd=%s vol=%.2fL zone=%u flags=0x%02X\n",
                  pkt.header.sequence_id, pkt.payload.command_id,
                  pkt.payload.target_volume_l, pkt.payload.zone_id, pkt.header.flags);
    digitalWrite(LED_RX_PIN, LOW);

    // ── Duplicate/replay guard — exact repeat of the last accepted packet ─
    if (haveAcceptedAny &&
        pkt.header.sequence_id == lastAcceptedSeq &&
        strncmp(pkt.payload.command_id, lastAcceptedCommandId,
                sizeof(lastAcceptedCommandId)) == 0) {
        Serial.printf("[COMMS] IGNORE — duplicate of last accepted packet "
                      "(seq=%u cmd=%s), likely a LoRa retransmit.\n",
                      pkt.header.sequence_id, pkt.payload.command_id);
        return;
    }
    lastAcceptedSeq = pkt.header.sequence_id;
    strncpy(lastAcceptedCommandId, pkt.payload.command_id, sizeof(lastAcceptedCommandId) - 1);
    lastAcceptedCommandId[sizeof(lastAcceptedCommandId) - 1] = '\0';
    haveAcceptedAny = true;

    // ── Emergency stop takes priority over everything, from any state ───
    if (pkt.header.flags & FLAG_EMERGENCY) {
        emergencyStop();
        return;   // feedback (IRRIGATION_ABORTED) is sent by tickComms()
                  // once actuators.cpp finishes its bleed-and-close sequence
    }

    // ── Normal command — actuators.cpp owns zone/state validation.
    // Rejection reasons (busy vs invalid zone) are distinguished in the
    // Serial log actuators.cpp already prints; enums.h has no separate
    // "busy" IrrigationStatus code, so both map to IRRIGATION_FAILED here.
    if (!startPumpCycle(pkt.payload)) {
        sendRejection(pkt.payload.command_id, IRRIGATION_FAILED);
    }
}

// ============================================================
// Public API
// ============================================================

void initComms() {
    pinMode(LED_RX_PIN, OUTPUT);
    digitalWrite(LED_RX_PIN, LOW);

    LoRa.setPins(LORA_SS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);

    if (!LoRa.begin((long)LORA_FREQUENCY_HZ)) {
        Serial.println("[COMMS] CRITICAL: LoRa.begin() failed — check wiring/antenna. Halting.");
        while (true) {
            digitalWrite(LED_RX_PIN, !digitalRead(LED_RX_PIN));
            delay(300);
        }
    }

    LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
    LoRa.setSignalBandwidth((long)LORA_SIGNAL_BANDWIDTH);
    LoRa.setCodingRate4(LORA_CODING_RATE);
    LoRa.setSyncWord(LORA_SYNC_WORD);
    LoRa.setPreambleLength(LORA_PREAMBLE_LENGTH);
    LoRa.setTxPower(LORA_TX_POWER_DBM);

    Serial.println("[COMMS] LoRa online — SF9 | BW125 | CR4/5 | SyncWord=0x12");
    Serial.printf("[COMMS] LoRaCommandPacket=%dB  LoRaFeedbackPacket=%dB  LoRaHeartbeatPacket=%dB\n",
                  (int)sizeof(LoRaCommandPacket), (int)sizeof(LoRaFeedbackPacket),
                  (int)sizeof(LoRaHeartbeatPacket));

    lastHeartbeatMs = millis();
}

void tickComms() {
    // ── RX phase ──────────────────────────────────────────────────────
    int packetSize = LoRa.parsePacket();
    if (packetSize > 0) {
        handleIncomingPacket(packetSize);
    }

    // ── Drain any completed pump cycle and report it ─────────────────
    NodeCFeedback fb;
    if (popCompletedFeedback(fb)) {
        sendFeedback(fb);
    }

    // ── Periodic heartbeat ────────────────────────────────────────────
    uint32_t now = millis();
    if (now - lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS) {
        lastHeartbeatMs = now;
        sendHeartbeat();
    }
}
