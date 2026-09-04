#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include "lora_comms.h"
#include "lora_protocol.h"
#include "pins.h"
#include "config.h"
#include "crc16.h"

static bool g_loraReady = false;
static uint16_t g_sequenceId = 0;

bool initLora() {
    SPI.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_SS_PIN);
    LoRa.setPins(LORA_SS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);

    if (!LoRa.begin(LORA_FREQ_HZ)) {
        Serial.println("[INIT]  CRITICAL: LoRa init FAILED — system halted");
        g_loraReady = false;
        return false;
    }

    LoRa.setSpreadingFactor(LORA_SF);
    LoRa.setSignalBandwidth((long)LORA_SIGNAL_BANDWIDTH);
    LoRa.setCodingRate4(LORA_CODING_RATE);
    LoRa.setSyncWord(LORA_SYNC_WORD);
    LoRa.setPreambleLength(LORA_PREAMBLE_LENGTH);
    LoRa.setTxPower(LORA_TX_POWER_DBM);
    g_loraReady = true;

    Serial.printf("[INIT]  LoRa OK  433MHz  SF%d  sync=0x%02X  pwr=%ddBm\n",
                  LORA_SF, LORA_SYNC_WORD, LORA_TX_POWER_DBM);
    return true;
}

bool loraIsReady() {
    return g_loraReady;
}

bool transmitPacket(const MasterSensorRecord &record) {
    if (!g_loraReady) {
        Serial.println("[LORA]  Not ready — skipping TX");
        return false;
    }

    LoRaSensorPacket pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.payload = record;

    pkt.header.magic_byte       = LORA_MAGIC_BYTE;
    pkt.header.protocol_version = LORA_PROTOCOL_VERSION;
    pkt.header.sender_id        = NODE_A_ADDRESS;
    pkt.header.receiver_id      = NODE_B_ADDRESS;
    pkt.header.packet_type      = PACKET_SENSOR_DATA;
    pkt.header.payload_length   = sizeof(MasterSensorRecord);
    pkt.header.flags            = FLAG_ACK_REQUIRED;
    pkt.header.sequence_id      = ++g_sequenceId;

    pkt.crc16 = crc16Modbus((const uint8_t *)&pkt,
                             sizeof(LoRaHeader) + sizeof(MasterSensorRecord));

    LoRa.beginPacket();
    LoRa.write((const uint8_t *)&pkt, sizeof(pkt));
    bool ok = LoRa.endPacket();

    Serial.printf("[LORA]  TX seq=%u  size=%d bytes  CRC=0x%04X  %s\n",
                  g_sequenceId, (int)sizeof(pkt), pkt.crc16,
                  ok ? "OK" : "FAILED");
    return ok;
}
