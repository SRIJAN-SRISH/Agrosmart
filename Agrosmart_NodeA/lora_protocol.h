#ifndef LORA_PROTOCOL_H
#define LORA_PROTOCOL_H

#include <stdint.h>
#include "enums.h"
#include "structs.h"

// ==========================================
// Protocol Identification
// ==========================================
#define LORA_MAGIC_BYTE 0xAB          // Changed to AB
#define LORA_PROTOCOL_VERSION 0x01

// ==========================================
// Node Addresses
// ==========================================
#define NODE_A_ADDRESS 0x0A           // Added Node A for completeness
#define NODE_B_ADDRESS 0x02           // Changed to 02
#define NODE_C_ADDRESS 0x0C

// ==========================================
// Packet Flags
// ==========================================
#define FLAG_NONE 0x00
// FLAG_ACK_REQUIRED defined in config.h — do not redefine here

// Retransmitted packet
#define FLAG_IS_RETRY 0x02

// Emergency stop / critical message
#define FLAG_EMERGENCY 0x04

// Reserved for future use
#define FLAG_RESERVED_1 0x08
#define FLAG_RESERVED_2 0x10
#define FLAG_RESERVED_3 0x20
#define FLAG_RESERVED_4 0x40
#define FLAG_RESERVED_5 0x80

// ==========================================
// Common Packet Header
// ==========================================
struct LoRaHeader
{
    uint8_t magic_byte;
    uint8_t protocol_version;

    uint8_t sender_id;
    uint8_t receiver_id;

    uint8_t packet_type;
    uint8_t payload_length;

    uint8_t flags;

    uint16_t sequence_id;
} __attribute__((packed));

// ==========================================
// Lightweight ACK/NACK Payload
// ==========================================
struct AckPayload
{
    uint16_t acknowledged_sequence;
    uint8_t status;
} __attribute__((packed));

// ==========================================
// Complete Transmission Wrappers
// ==========================================

// Node A -> Node B  (soil sensor telemetry)
struct LoRaSensorPacket
{
    LoRaHeader header;
    MasterSensorRecord payload;
    uint16_t crc16;
} __attribute__((packed));

// Node B -> Node C  (irrigation command)
struct LoRaCommandPacket
{
    LoRaHeader header;
    NodeCCommand payload;
    uint16_t crc16;
} __attribute__((packed));

// Node C -> Node B
struct LoRaFeedbackPacket
{
    LoRaHeader header;
    NodeCFeedback payload;
    uint16_t crc16;
} __attribute__((packed));

// Node C -> Node B (Alive Status)
struct LoRaHeartbeatPacket
{
    LoRaHeader header;
    NodeCHeartbeat payload;
    uint16_t crc16;
} __attribute__((packed));

// Immediate RF Confirmation (Node C <-> Node B)
struct LoRaAckPacket
{
    LoRaHeader header;
    AckPayload payload;
    uint16_t crc16;
} __attribute__((packed));

#endif // LORA_PROTOCOL_H
