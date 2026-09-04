#ifndef COMMS_H
#define COMMS_H

// ============================================================
// comms.h — Node C LoRa link to Node B
//
// Owns everything radio-related: init, the size->magic->sender->
// receiver->type->CRC16 validation chain, dispatching a validated
// command into actuators.h, and transmitting feedback/heartbeat
// packets. actuators.h and sensors.h know nothing about LoRa — this
// is the only module that does.
// ============================================================

// Call once from setup(), after initActuators()/initSensors(). Brings
// up the SX1278 with radio params matching Node B exactly (config.h).
// Halts the sketch on failure (blinks LED_RX_PIN and loops forever) —
// a Node C that can't receive commands at all has no useful degraded
// mode to fall back to.
void initComms();

// Call every loop() iteration, unconditionally. Polls for an inbound
// LoRaCommandPacket, validates and dispatches it; drains
// actuators::popCompletedFeedback() and transmits it if a cycle just
// finished; fires the periodic heartbeat on its own timer.
void tickComms();

#endif // COMMS_H
