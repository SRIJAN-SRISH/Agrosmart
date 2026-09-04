#pragma once
#ifndef CONFIG_H
#define CONFIG_H

// ==========================================
// Firmware Meta & Environment
// ==========================================
#define PROJECT_NAME "AgroSmart_NodeB"
#define NODE_ROLE "DECISION_ENGINE"
#define FIRMWARE_VERSION "1.0.0"
#define NODE_ID "NODE_B_01"

// ⚠️ UNCOMMENT THE LINE BELOW FOR BENCH/LAB TESTING ⚠️
//#define DEVELOPMENT_MODE

// ==========================================
// System Timings & Limits
// ==========================================
#ifdef DEVELOPMENT_MODE
#define SENSOR_POLL_INTERVAL_MS 60000   // 1 min  (Fast testing)
#define HEALTH_CHECK_INTERVAL_MS 10000  // 10 sec (Fast testing)
#define WEATHER_STALE_TIMEOUT_MS 120000 // 2 min  (Fast testing)
#else
#define SENSOR_POLL_INTERVAL_MS 1800000  // 30 min (Production)
#define HEALTH_CHECK_INTERVAL_MS 60000   // 1 min  (Production)
#define WEATHER_STALE_TIMEOUT_MS 3600000 // 1 hr   (Production)
#endif

#define LORA_ACK_TIMEOUT_MS 5000
#define MAX_LORA_RETRIES 3

// ==========================================
// WiFi & Cloud
// ==========================================
#define WIFI_SSID "srijan"
#define WIFI_PASS "srijan1234"

#define SUPABASE_URL "https://hrubrokdmvnpzejqjuqa.supabase.co/rest/v1/"
#define SUPABASE_ANON_KEY "sb_publishable_ki003MnbTVLvNppgRrZq1A_IKZJQlc9"

// ==========================================
// Hardware Safety & Watchdog
// ==========================================
#define WATCHDOG_TIMEOUT_MS 10000
#define BATTERY_LOW_VOLTAGE 11.2
#define BATTERY_RECOVERY_VOLTAGE 11.8
#define HEAP_WARNING_THRESHOLD 32000
#define HEAP_CRITICAL_THRESHOLD 16000
#define SD_STORAGE_CRITICAL_PCT 5.0

// ==========================================
// UART Protocol Parameters
// ==========================================
#define UART_BAUD_RATE 4800
#define UART_MAGIC_BYTE 0xBB
#define UART_PROTOCOL_VERSION 0x01

// ==========================================
// LoRa RF Configuration
// ==========================================
#define LORA_FREQUENCY_HZ 433000000UL
#define LORA_TX_POWER_DBM 17
#define LORA_SPREADING_FACTOR 9
#define LORA_SIGNAL_BANDWIDTH 125000UL
#define LORA_CODING_RATE 5
#define LORA_SYNC_WORD 0x12
#define LORA_PREAMBLE_LENGTH 8
#define MAX_LORA_PACKET_SIZE 255

// ==========================================
// Protocol Flags
// ==========================================
#define FLAG_ACK_REQUIRED 0x01

// ==========================================
// Decision Engine Limits
// ==========================================
#define VWC_MEDIAN_WINDOW 5
#define ACTIVATION_MARGIN_MM 2.0f
#define MIN_VALID_VWC_PERCENT 0.0f
#define MAX_VALID_VWC_PERCENT 100.0f
#define MIN_VALID_HUMIDITY_PERCENT 0.0f
#define MAX_VALID_HUMIDITY_PERCENT 100.0f
#define MIN_VALID_TEMP_C -40.0f
#define MAX_VALID_TEMP_C 85.0f
#define MIN_VALID_MAD_PERCENT 0.0f
#define MAX_VALID_MAD_PERCENT 100.0f
#define MIN_VALID_EFFICIENCY_PERCENT 0.0f
#define MAX_VALID_EFFICIENCY_PERCENT 100.0f
#define SECONDS_PER_DAY 86400
#define MAX_IRRIGATION_RUNTIME_SEC 2700

#endif // CONFIG_H
