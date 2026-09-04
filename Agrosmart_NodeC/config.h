#pragma once
#ifndef CONFIG_H
#define CONFIG_H

// ==========================================
// Firmware Meta
// ==========================================
#define PROJECT_NAME "AgroSmart_NodeC"
#define NODE_ROLE "ACTUATOR"
#define FIRMWARE_VERSION "1.0.0"
#define NODE_ID "NODE_C_01"

// ==========================================
// LoRa RF Configuration
// MUST match Node B's config.h exactly — different sync word or radio
// params means the two radios never see each other's packets at all,
// with no error on either side to tell you why.
// ==========================================
#define LORA_FREQUENCY_HZ 433000000UL
#define LORA_TX_POWER_DBM 17
#define LORA_SPREADING_FACTOR 9
#define LORA_SIGNAL_BANDWIDTH 125000UL
#define LORA_CODING_RATE 5
#define LORA_SYNC_WORD 0x12
#define LORA_PREAMBLE_LENGTH 8
#define MAX_LORA_PACKET_SIZE 255

// Referenced by lora_protocol.h's LoRaHeader.flags field — kept here (not
// in lora_protocol.h) to match Node B's exact layout, so both sides define
// it in the same place.
#define FLAG_ACK_REQUIRED 0x01

// ==========================================
// Node C Own Safety Ceilings (independent of Node B)
// Enforced locally regardless of what Node B's command says — Node C
// must never trust Node B blindly. cmd.max_runtime_sec from Node B is
// clamped against these, never the other way around.
// ==========================================
#define ABSOLUTE_MAX_RUNTIME_SEC 3600UL   // 1 hour — Node C will NEVER run longer
#define ABSOLUTE_MAX_VOLUME_L    3000.0f  // Node C will NEVER pump more than this

// Conservative flow-rate estimate used ONLY for the time-based runtime
// ceiling (one of three clamp tiers). Deliberately set below the pump's
// 5.5L/min nameplate max so the time ceiling stays generous — the flow
// meter's live pulse count is the accurate, primary "stop at target
// volume" mechanism; this constant is just a backstop for a dead/failed
// flow meter, not the main control loop.
#define PUMP_FLOW_RATE_ESTIMATE_LPM 4.0f

// ==========================================
// Actuation Sequencing Timing (bench-validated)
// ==========================================
#define VALVE_PRECHARGE_MS  1500   // valve open -> pump start delay (let the
                                    // solenoid fully seat before the 120PSI
                                    // pump pressurizes the line)
#define PRESSURE_BLEED_MS   2000   // pump stop -> valve close delay (let
                                    // residual pressure relax first)
#define NO_FLOW_GRACE_MS    2000   // pump-on time allowed before zero flow
                                    // pulses is treated as suspicious

// ==========================================
// Flow Meter — YF-S201
// ==========================================
// Datasheet K-factor: F(Hz) = 7.5 * Q(L/min) -> 450 pulses/liter, constant
// across the linear range. STILL bucket-calibrate at this pump's actual
// ~2-5.5L/min operating point before trusting this for anything that
// matters (Node B's daily-volume-limit enforcement reads this value).
#define PULSES_PER_LITER 450

// ==========================================
// ACS712 Current Sensor + Resistor Divider
// ==========================================
// DIVIDER_SCALE = R2 / (R1 + R2), from YOUR OWN multimeter measurement of
// the actual resistors in circuit — do not trust a nominal value blindly,
// 5% tolerance resistors can land anywhere in a real range.
// (No ADC_VREF/ADC_MAX_COUNTS constants needed — readAcsVoltsSensorDomain()
// uses analogReadMilliVolts(), which is self-calibrated per-chip.)
#define DIVIDER_SCALE   0.604743f    // <-- REPLACE with your measured R2/(R1+R2)

// ACS712-30A variant (per pins.h) — 66mV/A sensitivity. If the physical
// module ever changes, update this to match: 185.0f=5A, 100.0f=20A, 66.0f=30A.
#define ACS712_MV_PER_A 66.0f

#define DRY_RUN_CURRENT_A    0.3f   // near-zero current while relay ON —
                                     // valve didn't open, blocked line, or
                                     // this pump's own 120PSI cutoff engaged
#define JAM_CURRENT_A         5.0f  // sustained high current — genuine
                                     // mechanical jam/locked rotor. Re-check
                                     // this threshold once real running-current
                                     // data is logged — a 30A sensor's coarser
                                     // resolution than the 20A bench-test math
                                     // assumed means it's worth confirming this
                                     // still sits well clear of normal inrush.
#define JAM_STARTUP_GRACE_MS 500    // ignore the HIGH threshold for this long
                                     // after pump-on — startup inrush is normal
#define FAULT_SAMPLE_MS      100
#define FAULT_CONSEC_COUNT   3
#define ZERO_CAL_SAMPLES     64     // averaged at boot, pump confirmed OFF,
                                     // for the true zero-current baseline

// ==========================================
// Heartbeat (NodeCHeartbeat -> Node B, PACKET_HEARTBEAT)
// ==========================================
// Node B's NODE_C_TIMEOUT_MS is 5 minutes (health_task.cpp) — this interval
// needs at least 2-3x margin below that or a single missed/collided packet
// falsely declares Node C dead and closes the battery-gate on Node B.
#define HEARTBEAT_INTERVAL_MS 60000UL   // 60s

// ==========================================
// INA219 Battery Monitor
// ==========================================
// Used only as a fault-safe fallback if the INA219 fails to initialize or
// returns an implausible reading (sensors.cpp validates against this
// range) — NOT the primary battery-voltage path. sensors::readBatteryVoltage()
// reads the real sensor every time it's called; this constant only fires
// when that read can't be trusted, so a hardware fault degrades gracefully
// instead of transmitting garbage to Node B's isNodeCBatteryOk() gate.
#define BATTERY_VOLTAGE_FALLBACK 12.0f
#define BATTERY_VOLTAGE_MIN_PLAUSIBLE 8.0f
#define BATTERY_VOLTAGE_MAX_PLAUSIBLE 16.0f

// Operational low-battery warning threshold, reported via
// NodeCHeartbeat.status_flag (NODEC_LOW_BATTERY) — distinct from the
// plausibility bounds above, which catch sensor faults, not real low
// charge. Matches Node B's own BATTERY_LOW_VOLTAGE (config.h) since both
// nodes should agree on what "low" means for the same battery pack.
#define NODEC_BATTERY_LOW_THRESHOLD 11.2f

#endif // CONFIG_H
