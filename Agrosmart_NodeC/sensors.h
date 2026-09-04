#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>

// ============================================================
// sensors.h — Node C battery/system monitor (INA219)
//
// ACS712 (pump current, dual-threshold fault detection) and the flow
// meter live in actuators.cpp/.h — they're tightly coupled to the
// pump-cycle state machine's per-100ms tick. The INA219 is different:
// it's system-level telemetry (battery health), read once per heartbeat
// interval, independent of whether a pump cycle is running at all. That
// split is why it gets its own module instead of living in actuators.*.
// ============================================================

// Call once from setup(), after Wire is available. Initializes the
// INA219 on pins.h's INA219_SDA_PIN/INA219_SCL_PIN. Logs a clear failure
// message if the sensor doesn't ACK — does not halt the sketch, since
// Node C should still be able to run the pump/valve/LoRa link even with
// a dead battery monitor; it just falls back to BATTERY_VOLTAGE_FALLBACK
// until the sensor is fixed.
void initSensors();

// Reads the INA219's actual bus voltage. Returns BATTERY_VOLTAGE_FALLBACK
// (config.h) instead if the sensor failed to initialize, or if the live
// reading falls outside the plausible range (config.h) — a real hardware
// fault degrades to a safe default rather than transmitting a garbage
// value into Node B's isNodeCBatteryOk() gate.
float readBatteryVoltage();

// True once initSensors() has confirmed the INA219 is present and
// responding. False means readBatteryVoltage() is currently returning
// the fallback constant, not a live reading — useful if this needs to
// be surfaced in a future NodeCStatus/health field.
bool isBatteryMonitorOk();

#endif // SENSORS_H
