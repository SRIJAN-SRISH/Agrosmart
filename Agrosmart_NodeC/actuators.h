#ifndef ACTUATORS_H
#define ACTUATORS_H

#include <Arduino.h>
#include "structs.h"

// ============================================================
// actuators.h — Node C pump/valve/flow/current control
//
// Owns the non-blocking managePumpCycle() state machine that was
// validated on the bench (relay + pump + valve + flow meter + ACS712
// dual-threshold fault detection). comms.cpp is the only other module
// that talks to this one — it hands over a validated NodeCCommand and
// polls for a completed NodeCFeedback to transmit back to Node B.
// ============================================================

// Call once from setup(). Forces both relays to their inactive level
// BEFORE pinMode(OUTPUT) (boot-glitch guard), attaches the flow meter
// ISR, and runs the ACS712 zero-current baseline calibration — pump is
// guaranteed off at this point, so the baseline reflects true zero.
void initActuators();

// Call every loop() iteration, unconditionally. Advances the state
// machine by comparing millis() against stored timestamps — contains
// no delay() calls anywhere, by design (see nodeC_architecture.md §6).
void tickActuators();

// comms.cpp calls this after validating an inbound LoRaCommandPacket.
// Returns false (and does not start a cycle) if the state machine is
// not IDLE, or if cmd.zone_id doesn't map to a physically wired valve —
// currently only zone 1 is wired (see pins.h). The caller is
// responsible for sending an IRRIGATION_FAILED feedback packet in that
// case; this function does not transmit anything itself.
bool startPumpCycle(const NodeCCommand &cmd);

// comms.cpp calls this immediately on receiving FLAG_EMERGENCY, from
// any state. Stops the pump immediately; if the valve is open, it still
// closes through the normal PRESSURE_BLEED_MS delay rather than
// slamming shut under system pressure — an instant valve slam under a
// pressurized line is exactly the water-hammer risk the bleed delay
// exists to prevent, so "emergency" means "stop pumping now", not
// "skip every safety timing that protects the hardware".
void emergencyStop();

// Returns true exactly once per completed cycle (success, fault, or
// abort), with `feedback` populated and ready to wrap in a
// LoRaFeedbackPacket. Returns false otherwise. comms.cpp should poll
// this every loop() iteration, after tickActuators().
bool popCompletedFeedback(NodeCFeedback &feedback);

// True only when the state machine is IDLE — comms.cpp should check
// this before calling startPumpCycle() so it can reject an inbound
// command as "busy" instead of silently ignoring it.
bool isActuatorIdle();

// True if the most recently COMPLETED cycle ended in IRRIGATION_FAILED
// (dry-run/jam fault). Stays true until the next cycle actually starts —
// not cleared by popCompletedFeedback() — so comms.cpp can fold this
// into the heartbeat's status_flag (NODEC_FLOW_FAULT) for as long as
// it's relevant, not just in the single tick the fault occurred.
bool lastCycleFaulted();

#endif // ACTUATORS_H
