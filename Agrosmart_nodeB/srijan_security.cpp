#include "srijan_security.h"
#include <Arduino.h>
#include <string.h>

// ESP32 Hardware Cryptography (mbedtls)
#include "mbedtls/gcm.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "esp_system.h" // for esp_random()

// ==========================================
// Global State & Cryptographic Context
// ==========================================

// Master Key Definition (16 bytes = 128 bit)
// This is identical across Node A, B, and C.
const uint8_t SRIJAN_MASTER_KEY[16] = {
    0x4A, 0x9F, 0x22, 0x51, 0xC3, 0x08, 0x7E, 0x1B,
    0x88, 0x34, 0xDD, 0x67, 0x55, 0xAA, 0x19, 0xF0
};

static SrijanState_t g_srijan_state = SRIJAN_STATE_RESYNC;
static uint32_t      g_outbound_seq = 0;

// Replay Attack Memory (Store highest seen sequence ID per node)
static uint32_t g_highest_seen_seq_A = 0;
static uint32_t g_highest_seen_seq_C = 0;
static uint32_t g_highest_seen_seq_Pico = 0; // For UART

// Boot Sync Challenge Memory
static uint32_t g_pending_challenge = 0;

// The time tolerance window (5 minutes)
#define SRIJAN_TIME_TOLERANCE_SEC 300 

// To handle Nodes A/C without hardware RTCs, we store an offset
// between local millis() and the true epoch time provided by Node B.
static uint32_t g_local_epoch_offset = 0; 

// ==========================================
// Helper Functions
// ==========================================

// Get current Unix Epoch. If we are Node B, this should ideally pull from DS3231.
// For now, we rely on the system time or the offset calculation.
static uint32_t get_current_epoch() {
    // If we have an offset (Node A/C), calculate true time
    if (g_local_epoch_offset > 0) {
        return g_local_epoch_offset + (millis() / 1000);
    }
    // Fallback/Node B: return standard time()
    time_t now;
    time(&now);
    return (uint32_t)now;
}

// Construct the 12-byte GCM Nonce deterministically from the Header
static void construct_nonce(const SrijanHeader* hdr, unsigned char* nonce) {
    memset(nonce, 0, SRIJAN_GCM_NONCE_LEN);
    nonce[0] = hdr->sender_id;
    nonce[1] = hdr->receiver_id;
    memcpy(&nonce[2], &hdr->sequence_id, 4);
    memcpy(&nonce[6], &hdr->timestamp_epoch, 4);
    // bytes 10 and 11 are zero padding
}

// ==========================================
// Core API Implementation
// ==========================================

void srijan_init() {
    // Assuming we boot in RESYNC state unless we are Node B.
    // Node B has a hardware RTC, so it can jump straight to SECURE once NTP syncs.
    // For this unified library, we'll let the application logic transition it.
    g_srijan_state = SRIJAN_STATE_RESYNC;
    g_outbound_seq = 0;
    
    // Seed random number generator just in case
    srand(esp_random());
    
    Serial.println("[SRIJAN] Security Engine Initialized (mbedtls AES-128-GCM)");
}

SrijanState_t srijan_get_state() {
    return g_srijan_state;
}

bool srijan_seal(uint8_t packet_type, uint8_t sender_id, uint8_t receiver_id,
                 const uint8_t* payload, size_t payload_len, 
                 SrijanSecureFrame* out_frame) 
{
    if (payload_len > SRIJAN_MAX_PAYLOAD_SIZE) {
        Serial.println("[SRIJAN] SEAL ERROR: Payload too large!");
        return false;
    }

    // 1. Construct Header
    out_frame->header.magic_byte = SRIJAN_MAGIC_BYTE;
    out_frame->header.packet_type_ver = (SRIJAN_VERSION << 4) | (packet_type & 0x0F);
    out_frame->header.sender_id = sender_id;
    out_frame->header.receiver_id = receiver_id;
    out_frame->header.sequence_id = ++g_outbound_seq;
    out_frame->header.timestamp_epoch = get_current_epoch();

    // 2. Construct deterministic Nonce
    unsigned char nonce[SRIJAN_GCM_NONCE_LEN];
    construct_nonce(&out_frame->header, nonce);

    // 3. Encrypt and Authenticate using mbedtls
    mbedtls_gcm_context gcm;
    mbedtls_gcm_init(&gcm);
    
    int ret = mbedtls_gcm_setkey(&gcm, MBEDTLS_CIPHER_ID_AES, SRIJAN_MASTER_KEY, 128);
    if (ret != 0) {
        Serial.printf("[SRIJAN] SEAL ERROR: mbedtls_gcm_setkey failed: %d\n", ret);
        mbedtls_gcm_free(&gcm);
        return false;
    }

    // The header is the Additional Authenticated Data (AAD)
    ret = mbedtls_gcm_crypt_and_tag(&gcm, MBEDTLS_GCM_ENCRYPT, payload_len,
                                    nonce, SRIJAN_GCM_NONCE_LEN,
                                    (const unsigned char*)&out_frame->header, sizeof(SrijanHeader),
                                    payload, out_frame->ciphertext,
                                    SRIJAN_GCM_TAG_LEN, out_frame->gcm_tag);
                                    
    mbedtls_gcm_free(&gcm);

    if (ret != 0) {
        Serial.printf("[SRIJAN] SEAL ERROR: mbedtls_gcm_crypt_and_tag failed: %d\n", ret);
        return false;
    }

    return true;
}

bool srijan_open(const SrijanSecureFrame* in_frame, size_t payload_len, 
                 uint8_t expected_recv, uint8_t* out_payload) 
{
    // 1. Basic sanity checks
    if (in_frame->header.magic_byte != SRIJAN_MAGIC_BYTE) {
        Serial.println("[SRIJAN] OPEN ERROR: Invalid Magic Byte");
        return false;
    }
    
    if (in_frame->header.receiver_id != expected_recv) {
        Serial.println("[SRIJAN] OPEN ERROR: Not addressed to this node");
        return false;
    }

    // 2. Time Window Check (The Drift Defense)
    uint32_t current_time = get_current_epoch();
    uint32_t packet_time = in_frame->header.timestamp_epoch;
    
    // Allow if local clock is somehow 0 (e.g., we haven't fully synced yet but are trying to)
    if (current_time > 1000000) { 
        if (packet_time < (current_time - SRIJAN_TIME_TOLERANCE_SEC) || 
            packet_time > (current_time + SRIJAN_TIME_TOLERANCE_SEC)) {
            Serial.printf("[SRIJAN] OPEN ERROR: Timestamp out of bounds! pkt=%lu, cur=%lu\n", packet_time, current_time);
            return false;
        }
    }

    // 3. Sequence ID Check (The Absolute Replay Defense)
    uint32_t* highest_seen = nullptr;
    if (in_frame->header.sender_id == 0x0A) highest_seen = &g_highest_seen_seq_A; // Node A
    else if (in_frame->header.sender_id == 0x0C) highest_seen = &g_highest_seen_seq_C; // Node C
    else highest_seen = &g_highest_seen_seq_Pico; // UART Pico
    
    if (highest_seen != nullptr) {
        if (in_frame->header.sequence_id <= *highest_seen) {
            Serial.printf("[SRIJAN] OPEN ERROR: REPLAY ATTACK DETECTED! Seq %lu <= %lu\n", 
                          in_frame->header.sequence_id, *highest_seen);
            return false;
        }
    }

    // 4. Construct deterministic Nonce
    unsigned char nonce[SRIJAN_GCM_NONCE_LEN];
    construct_nonce(&in_frame->header, nonce);

    // 5. Decrypt and Verify using mbedtls
    mbedtls_gcm_context gcm;
    mbedtls_gcm_init(&gcm);
    
    int ret = mbedtls_gcm_setkey(&gcm, MBEDTLS_CIPHER_ID_AES, SRIJAN_MASTER_KEY, 128);
    if (ret == 0) {
        ret = mbedtls_gcm_auth_decrypt(&gcm, payload_len,
                                       nonce, SRIJAN_GCM_NONCE_LEN,
                                       (const unsigned char*)&in_frame->header, sizeof(SrijanHeader),
                                       in_frame->gcm_tag, SRIJAN_GCM_TAG_LEN,
                                       in_frame->ciphertext, out_payload);
    }
    
    mbedtls_gcm_free(&gcm);

    if (ret != 0) {
        Serial.printf("[SRIJAN] OPEN ERROR: AUTHENTICATION FAILED! Tag mismatch or data corrupted (err: %d)\n", ret);
        return false;
    }

    // 6. Update Sequence ID memory ONLY if cryptographic verification passed!
    if (highest_seen != nullptr) {
        *highest_seen = in_frame->header.sequence_id;
    }

    return true;
}

// ==========================================
// Boot Sync (Power Loss Catch-22) Implementation
// ==========================================

void srijan_generate_time_request(uint8_t sender_id, uint8_t receiver_id, 
                                  uint8_t* out_buffer, size_t* out_len)
{
    g_srijan_state = SRIJAN_STATE_RESYNC;
    
    // Generate a 32-bit random challenge
    g_pending_challenge = esp_random();
    
    // Format: [Magic][Type=0xFF][Sender][Receiver][4-byte Challenge]
    out_buffer[0] = SRIJAN_MAGIC_BYTE;
    out_buffer[1] = 0xFF; // Special packet type for TIME_REQUEST
    out_buffer[2] = sender_id;
    out_buffer[3] = receiver_id;
    memcpy(&out_buffer[4], &g_pending_challenge, 4);
    
    *out_len = 8;
    Serial.printf("[SRIJAN] Generated TIME_REQUEST with Challenge: %08lX\n", g_pending_challenge);
}

bool srijan_handle_time_request(const uint8_t* in_buffer, size_t in_len,
                                uint8_t my_id,
                                SrijanSecureFrame* out_frame)
{
    if (in_len != 8 || in_buffer[0] != SRIJAN_MAGIC_BYTE || in_buffer[1] != 0xFF) {
        return false; // Not a valid time request
    }
    
    if (in_buffer[3] != my_id) {
        return false; // Not addressed to me
    }
    
    uint8_t sender_id = in_buffer[2];
    uint32_t challenge;
    memcpy(&challenge, &in_buffer[4], 4);
    
    Serial.printf("[SRIJAN] Received TIME_REQUEST from %02X. Challenge: %08lX\n", sender_id, challenge);
    
    // Construct Payload: [4-byte Challenge][4-byte Current Epoch]
    uint8_t payload[8];
    uint32_t current_epoch = get_current_epoch();
    
    memcpy(&payload[0], &challenge, 4);
    memcpy(&payload[4], &current_epoch, 4);
    
    // Seal it using a special packet type (0xFE = TIME_RESPONSE)
    return srijan_seal(0xFE, my_id, sender_id, payload, 8, out_frame);
}

bool srijan_process_time_response(const SrijanSecureFrame* in_frame)
{
    if (g_srijan_state != SRIJAN_STATE_RESYNC) {
        return false; // Not expecting a time response
    }
    
    // Expected Payload is 8 bytes
    uint8_t payload[8];
    if (!srijan_open(in_frame, 8, in_frame->header.receiver_id, payload)) {
        return false; // Verification failed
    }
    
    // Extract Challenge and Time
    uint32_t received_challenge;
    uint32_t received_time;
    memcpy(&received_challenge, &payload[0], 4);
    memcpy(&received_time, &payload[4], 4);
    
    if (received_challenge != g_pending_challenge) {
        Serial.printf("[SRIJAN] TIME RESPONSE ERROR: Challenge mismatch! Expected %08lX, got %08lX\n", 
                      g_pending_challenge, received_challenge);
        return false;
    }
    
    // Success! Calculate the offset so that millis() + offset = true epoch
    g_local_epoch_offset = received_time - (millis() / 1000);
    g_srijan_state = SRIJAN_STATE_SECURE;
    
    Serial.printf("[SRIJAN] BOOT SYNC SUCCESS! Clock synchronized. Offset: %lu. Entering SECURE state.\n", 
                  g_local_epoch_offset);
    return true;
}
