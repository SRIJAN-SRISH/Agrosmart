#ifndef QUEUES_H
#define QUEUES_H

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "structs.h"

// ==========================================
// Global FreeRTOS Queue Handles
//
// These handles are initialized in the
// setup() block of AgroSmart_NodeB.ino
// and accessed by tasks via 'extern'.
// ==========================================

// Ingests raw sensor data from Node A (via UART / LoRa)
extern QueueHandle_t sensorQueue;

// Sends irrigation commands to Node C (via LoRa)
extern QueueHandle_t commandQueue;

// Receives pump feedback/status from Node C (via LoRa)
extern QueueHandle_t feedbackQueue;

// Node C periodic health/alive pings
extern QueueHandle_t heartbeatQueue;

// System event logs → WiFi cloud upload (vCloudTask)
extern QueueHandle_t logQueue;

// System event logs → SD card archive (vSdTask)
// Separate from logQueue so EVERY log entry reaches BOTH cloud AND SD.
extern QueueHandle_t sdLogQueue;

// Paired sensor+decision payload → WiFi cloud upload (vCloudTask)
extern QueueHandle_t cloudQueue;

// Paired sensor+decision payload → SD card archive (vSdTask)
// Parallel to cloudQueue so SD and cloud write independently.
// FMEA P2: if SD queue full, payload is dropped (cloud copy is the primary record).
extern QueueHandle_t sdQueue;

// Node C pump completion ACKs stamped with epoch → /IRRIGATION CSV on SD
// Populated by decision_task after processing feedbackQueue.
extern QueueHandle_t irrigationLogQueue;

// ==========================================
// pushLog() — send one SystemEventLog to BOTH cloud and SD queues.
// Use this everywhere instead of xQueueSend(logQueue, ...) directly.
// Fire-and-forget on both: drop if full (never block a task for a log entry).
// ==========================================
static inline void pushLog(const SystemEventLog &entry)
{
    if (logQueue   != nullptr) xQueueSend(logQueue,   &entry, 0);
    if (sdLogQueue != nullptr) xQueueSend(sdLogQueue, &entry, 0);
}

#endif // QUEUES_H