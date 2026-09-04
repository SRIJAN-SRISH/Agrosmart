/**
 * @file  cloud_task.h
 * @brief AgroSmart Node B — Cloud Upload Task public interface
 */

#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/**
 * FreeRTOS task function.
 * Register with:
 *   xTaskCreatePinnedToCore(vCloudTask, "CloudTask", 8192, NULL, 1,
 *                           &xCloudTaskHandle, 0);
 *
 * Stack note: 8192 bytes recommended — HTTPClient + ArduinoJson + WiFi TLS
 * stack frames are large. Reduce only after watermark profiling with
 * uxTaskGetStackHighWaterMark().
 */
void vCloudTask(void* pvParameters);

// WiFi connection state — updated by vCloudTask after each connect/disconnect.
// health_task reads this instead of calling WiFi.status() directly to avoid
// potential driver contention between Core 0 tasks.
extern volatile bool g_wifiConnected;