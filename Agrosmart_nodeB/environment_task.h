#ifndef ENVIRONMENT_TASK_H
#define ENVIRONMENT_TASK_H

// ============================================================
// environment_task.h
// AgroSmart Node B — Local Environment Sensor Task
//
// This task reads BMP280 (temp + pressure) and SEN0575 (rainfall)
// via dual I2C buses and injects the values directly into the
// MasterSensorRecord before it is dispatched to the decision engine.
//
// Architecture note:
//   Environment data does NOT use a separate queue or volatile globals.
//   It is written into the sensorRecord struct received from UART/LoRa
//   before that record enters sensorQueue. This keeps MasterSensorRecord
//   as the single source of truth for the entire pipeline.
// ============================================================

// FreeRTOS task entry point — launched from setup()
void vEnvironmentTask(void *pvParameters);

// Called by uart_task.cpp / lora_task.cpp to inject live local readings
// into a MasterSensorRecord before it is pushed to sensorQueue.
// Thread-safe: protected internally by envMutex.
void injectEnvironmentData(float &air_temp, float &pressure, float &rainfall);

// Returns current Unix epoch from the DS3231 RTC.
// Returns 0 if RTC is offline or has lost power.
// Called by decision_task and sd_task when sensorRecord.timestamp_valid == 0
// (Node A's timestamp absent) to keep Node B records time-stamped correctly.
uint32_t getNodeBEpoch();

#endif // ENVIRONMENT_TASK_H
