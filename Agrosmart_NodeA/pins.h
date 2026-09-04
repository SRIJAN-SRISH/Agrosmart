#pragma once

// ═══════════════════════════════════════════════════════════════════════════
// PIN DEFINITIONS (AgroSmart Node A)
// ═══════════════════════════════════════════════════════════════════════════

// LoRa SPI
#define LORA_SCK_PIN    18
#define LORA_MISO_PIN   19
#define LORA_MOSI_PIN   23
#define LORA_SS_PIN      5
#define LORA_RST_PIN    14
#define LORA_DIO0_PIN    2

// RS-485 Modbus
#define RE_DE_PIN        4
#define RX2_PIN         16
#define TX2_PIN         17

// Discrete LEDs (direct GPIO, no driver)
#define PIN_LED_WHITE   25
#define PIN_LED_GREEN   26
#define PIN_LED_YELLOW  27
#define PIN_LED_RED     33
