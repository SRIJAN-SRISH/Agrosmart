#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

#include <Arduino.h>

// ============================================================
// diagnostics.h — boot-reason logging, NVS-persisted reboot count,
// periodic heap health.
//
// Node B already has this pattern (reboot_cnt/reboot_rsn in NVS,
// health_task.cpp). Node C had zero persistence across reboots until
// now — every reset started from a completely blank slate, with no way
// to tell a normal power cycle apart from a watchdog save or a
// brownout, and no way to know if the device is rebooting far more
// often than it should be in the field.
// ============================================================

// Call once from setup(), as early as possible — before actuators/
// sensors/comms init — so this boot's diagnostics are captured even if
// something later in setup() halts (e.g. comms.cpp's LoRa failure halt).
void initDiagnostics();

// Call periodically (piggybacked on the heartbeat/bench print cadence)
// to log free heap. Cheap, and gives an early warning of slow heap
// fragmentation over long uptime before it becomes a crash.
void tickDiagnosticsHeap();

// Accessors — not used yet, but exposed now so a future richer
// heartbeat/health payload can fold these in without re-plumbing NVS
// access from comms.cpp.
uint32_t getRebootCount();
uint8_t  getLastResetReasonCode();   // raw esp_reset_reason_t, cast to uint8_t

#endif // DIAGNOSTICS_H
