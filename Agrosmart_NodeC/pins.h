#ifndef PINS_H
#define PINS_H

// ============================================================
// AgroSmart Node C — Pin Map
// ============================================================

// ── LoRa SX1278 (VSPI, hardware-fixed pins) ─────────────────────────────
// SCK/MISO/MOSI are fixed by the ESP32 silicon on the VSPI bus — do not
// reuse them for anything else. Sync word, SF, BW, CR must match Node B
// exactly (see config.h) or the two radios will never see each other.
#define LORA_SCK_PIN 18  // hardware VSPI clock — do not reuse
#define LORA_MISO_PIN 19 // hardware VSPI data in — do not reuse
#define LORA_MOSI_PIN 23 // hardware VSPI data out — do not reuse
#define LORA_SS_PIN 5
#define LORA_RST_PIN 14
#define LORA_DIO0_PIN 4

// ── Relay board — pump + valve (active-LOW, 2-channel isolated module) ──
// JD-VCC jumper must be physically removed for opto-isolation.
#define PUMP_RELAY_PIN 25   // Relay1 IN — main pump motor
#define VALVE_ZONE_1_PIN 26 // Relay2 IN — the only valve currently wired
#define VALVE_ZONE_2_PIN 27 // reserved — protocol-ready, no 3rd relay
                            // channel installed yet

// ── Flow meter — YF-S201, pulse output through 10k/15k divider ──────────
#define FLOW_PULSE_PIN 34 // input-only, ADC1-capable, interrupt-capable

// ── ACS712-30A current sensor — analog output through 10k/15k divider ───
#define ACS712_PIN 35 // input-only, ADC1-capable — stays usable even if
                      // WiFi is ever added later (ADC2 does not)

// ── INA219 battery/system monitor (I2C) ──────────────────────────────────
// Wired across the main 12V supply rail (battery/solar side), NOT the pump
// line — this is system-level telemetry for the heartbeat, not pump
// current (that's ACS712's job, in series with the pump only).
#define INA219_SDA_PIN 21
#define INA219_SCL_PIN 22
#define INA219_ADDR 0x40

// ── Status LEDs ───────────────────────────────────────────────────────
#define LED_RX_PIN 17   // blinks on valid LoRa packet received
#define LED_PUMP_PIN 16 // ON while pump relay is energized

#endif // PINS_H
