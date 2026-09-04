#ifndef PINS_H
#define PINS_H

#define LORA_SCK_PIN 18
#define LORA_MISO_PIN 19
#define LORA_MOSI_PIN 23
#define LORA_CS_PIN 5
#define LORA_RST_PIN 4
#define LORA_IRQ_PIN 2

// ── HSPI — SD Card (Dedicated Bus) ──────────────────────────────────────────
#define SD_SCK_PIN 14
#define SD_MISO_PIN 12
#define SD_MOSI_PIN 13
#define SD_CS_PIN 27

// ── I2C-1 (Wire1) — PCA9685 LEDs (Rain Gauge moved to UART) ─────────────────
// Values match what environment_task.cpp/led_task.cpp actually run with —
// these constants were previously defined but unreferenced anywhere, and
// swapped relative to the real, working pin order. Wire1.begin() is now
// called once, centrally, in Agrosmart_nodeB.ino's setup() using these.
#define I2C1_SDA_PIN 33
#define I2C1_SCL_PIN 32

// ── SEN0575 Rain Gauge UART (Serial1) ───────────────────────────────────────
#define RAIN_RX_PIN 34  // ESP32 RX1 ← Rain Gauge TX
#define RAIN_TX_PIN 25  // ESP32 TX1 → Rain Gauge RX

// ── Pi Pico HMI co-processor link — UART2 (Serial2) ─────────────────────────
// ESP32 GPIO17 (TX2) → Pi Pico GP1 (RX)
// ESP32 GPIO16 (RX2) ← Pi Pico GP0 (TX)
#define PILINK_TX_PIN  17   // ESP32 TX2 → Pi Pico RX
#define PILINK_RX_PIN  16   // ESP32 RX2 ← Pi Pico TX
#define PILINK_BAUD    115200

// ── PCA9685 16-channel PWM LED driver (I2C-1 Wire1: SDA=33 SCL=32) ──────────
// I2C address: 0x40 (all address pins grounded — factory default)
// PWM frequency: 1000 Hz (no visible flicker; fast enough for all LED types)
// 12-bit resolution: 0 = full off, 4095 = full on
#define LED_PCA9685_ADDR 0x40

#define LED_CH_SYSTEM_OK 0  // Green  — all buses OK, no faults, all tasks alive
#define LED_CH_FAULT 1      // Red    — global_health_flag != 0 (any active fault)
#define LED_CH_LORA 2       // Blue   — Node A or Node C seen within timeout window
#define LED_CH_WIFI 3       // Cyan   — WiFi station connected to AP
#define LED_CH_NODE_A 4     // White  — Node A packet received within 10 min
#define LED_CH_NODE_C 5     // Yellow — Node C heartbeat received within 5 min
#define LED_CH_IRRIGATION 6 // Teal   — irrigation command pending (pump running)
#define LED_CH_WARNING 7    // Amber  — battery low, heap warn, or storage warn

// PWM brightness levels
#define LED_PWM_FULL 4095 // 100% — solid on
#define LED_PWM_DIM 512   // 12%  — standby glow
#define LED_PWM_OFF 0     // 0%   — off

#endif