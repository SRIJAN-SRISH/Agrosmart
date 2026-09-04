#ifndef SRIJAN_SECURITY_H
#define SRIJAN_SECURITY_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// ==========================================
// SRIJAN Cryptographic Constants
// ==========================================
#define SRIJAN_MAGIC_BYTE       0x53 // 'S'
#define SRIJAN_VERSION          0x01
#define SRIJAN_GCM_TAG_LEN      16   // 128-bit authentication tag
#define SRIJAN_GCM_NONCE_LEN    12   // 96-bit IV
#define SRIJAN_CHALLENGE_LEN    4    // 32-bit random nonce for boot sync

// The Master AES-128 Key (Shared securely across all nodes)
// WARNING: In production, this should not be hardcoded here, but loaded from NVS/eFuse.
// For the breadboard prototype, we define it here.
extern const uint8_t SRIJAN_MASTER_KEY[16];

// ==========================================
// SRIJAN State Machine
// ==========================================
typedef enum {
    SRIJAN_STATE_RESYNC = 0, // Node has lost power, clock is invalid, must ask Node B for time
    SRIJAN_STATE_SECURE = 1  // Clock is synced, Sequence ID initialized, ready for secure traffic
} SrijanState_t;

// ==========================================
// SRIJAN Frame Structures
// ==========================================

// 12-Byte Unencrypted (but authenticated) Header
typedef struct {
    uint8_t  magic_byte;         // Always 0x53
    uint8_t  packet_type_ver;    // Top 4 bits: Version, Bottom 4 bits: Packet Type
    uint8_t  sender_id;
    uint8_t  receiver_id;
    uint32_t sequence_id;        // Strictly increasing counter to prevent replays
    uint32_t timestamp_epoch;    // RTC epoch for +/- 5 min window check
} __attribute__((packed)) SrijanHeader;

// Maximum expected ciphertext payload size
#define SRIJAN_MAX_PAYLOAD_SIZE 128 

// The total physical frame that goes over the air
typedef struct {
    SrijanHeader header;
    uint8_t      ciphertext[SRIJAN_MAX_PAYLOAD_SIZE];
    uint8_t      gcm_tag[SRIJAN_GCM_TAG_LEN];
} __attribute__((packed)) SrijanSecureFrame;

// ==========================================
// SRIJAN Core API
// ==========================================

/**
 * @brief Initialize the SRIJAN security engine.
 *        Sets up mbedtls contexts and random number generators.
 */
void srijan_init();

/**
 * @brief Get the current SRIJAN state (RESYNC or SECURE).
 */
SrijanState_t srijan_get_state();

/**
 * @brief Encrypt and Authenticate a payload.
 * 
 * @param packet_type  The type of packet (from lora_protocol.h or enums.h)
 * @param sender_id    Who is sending this
 * @param receiver_id  Who should receive this
 * @param payload      Pointer to the raw struct (e.g., MasterSensorRecord)
 * @param payload_len  Size of the payload
 * @param out_frame    Pointer to the destination secure frame
 * @return true if encryption succeeded
 */
bool srijan_seal(uint8_t packet_type, uint8_t sender_id, uint8_t receiver_id,
                 const uint8_t* payload, size_t payload_len, 
                 SrijanSecureFrame* out_frame);

/**
 * @brief Verify and Decrypt an incoming frame.
 * 
 * @param in_frame       The received secure frame
 * @param payload_len    The expected size of the inner payload (must be known by packet_type)
 * @param expected_recv  The address of THIS node (to prevent cross-talk)
 * @param out_payload    Buffer to store the decrypted payload
 * @return true if mathematically authenticated and within anti-replay windows
 */
bool srijan_open(const SrijanSecureFrame* in_frame, size_t payload_len, 
                 uint8_t expected_recv, uint8_t* out_payload);


// ==========================================
// SRIJAN Boot Sync API (The Power Loss Catch-22)
// ==========================================

/**
 * @brief (Node A/C) Generate a plaintext TIME_REQUEST packet containing a 
 *        cryptographically secure random 32-bit challenge nonce.
 *        Also transitions state to SRIJAN_STATE_RESYNC.
 */
void srijan_generate_time_request(uint8_t sender_id, uint8_t receiver_id, 
                                  uint8_t* out_buffer, size_t* out_len);

/**
 * @brief (Node B) Receive a TIME_REQUEST, sign the Challenge Nonce with the current RTC time,
 *        and output a securely sealed TIME_RESPONSE.
 */
bool srijan_handle_time_request(const uint8_t* in_buffer, size_t in_len,
                                uint8_t sender_id,
                                SrijanSecureFrame* out_frame);

/**
 * @brief (Node A/C) Receive a TIME_RESPONSE. Verify the GCM tag. Verify the 
 *        Challenge Nonce exactly matches the one we just sent. 
 *        If valid, update the internal clock offset and transition to SRIJAN_STATE_SECURE.
 */
bool srijan_process_time_response(const SrijanSecureFrame* in_frame);

#endif // SRIJAN_SECURITY_H
