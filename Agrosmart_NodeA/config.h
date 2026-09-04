#pragma once

#include <stdint.h>

// ═══════════════════════════════════════════════════════════════════════════
// PROTOCOL CONSTANTS
// LORA_MAGIC_BYTE / LORA_PROTOCOL_VERSION / NODE_A_ADDRESS / NODE_B_ADDRESS /
// PACKET_SENSOR_DATA now come from lora_protocol.h / enums.h (shared,
// byte-identical copies of Node B's files) instead of being redefined here —
// a local copy of these values could silently drift from Node B's.
// ═══════════════════════════════════════════════════════════════════════════

// Referenced by lora_protocol.h's LoRaHeader.flags field — kept here (not
// in lora_protocol.h) to match Node B's exact layout, so both sides define
// it in the same place.
#define FLAG_ACK_REQUIRED      0x01

// ═══════════════════════════════════════════════════════════════════════════
// LoRa RF Configuration
// MUST match Node B's config.h exactly — different sync word or radio
// params means the two radios never see each other's packets at all,
// with no error on either side to tell you why.
// ═══════════════════════════════════════════════════════════════════════════
#define LORA_FREQ_HZ           433E6
#define LORA_SF                9
#define LORA_SIGNAL_BANDWIDTH  125000UL
#define LORA_CODING_RATE       5
#define LORA_SYNC_WORD         0x12
#define LORA_PREAMBLE_LENGTH   8
#define LORA_TX_POWER_DBM      17

// RS-485 Modbus
#define SENSOR_MODBUS_ADDR     0x01
#define RS485_BAUD             4800
#define MODBUS_TIMEOUT_MS      500

// Timing
#define TX_INTERVAL_MS         60000UL   // 60 s production cycle
#define HEARTBEAT_INTERVAL_MS  30000UL   // white pulse every 30 s

// Health flag bits — HEALTH_OK / HEALTH_NODEA_SOIL_FAULT / HEALTH_NODEA_COMM_FAULT
// now come from enums.h (shared with Node B) instead of a locally-named copy.
