#ifndef TASKS_H
#define TASKS_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Forward declarations
void vHealthTask(void *pvParameters);
void vSdTask(void *pvParameters);
void vLoraTask(void *pvParameters);
void vUartTask(void *pvParameters);
void vDecisionTask(void *pvParameters);
void vEnvironmentTask(void *pvParameters);
void vCloudTask(void *pvParameters);
void vLedTask(void *pvParameters);

#endif