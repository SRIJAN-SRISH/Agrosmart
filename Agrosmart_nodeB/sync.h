#ifndef SYNC_H
#define SYNC_H
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// SPI bus mutex — shared by lora_task and sd_task
extern SemaphoreHandle_t spiMutex;

// I2C-1 bus mutex — shared by environment_task (SEN0575) and led_task (PCA9685)
// Both run on Core 1; without this a SEN0575 read and a PCA9685 write could
// overlap and corrupt the Wire1 transaction.
extern SemaphoreHandle_t i2c1Mutex;

// ==========================================

// SPI Lock Helper

// ==========================================

inline bool lockSPI(uint32_t timeoutMs)

{

    if (spiMutex == nullptr)

    {

        return false;
    }

    return xSemaphoreTake(

               spiMutex,

               pdMS_TO_TICKS(timeoutMs)

                   ) == pdPASS;
}

// ==========================================

// SPI Unlock Helper

// ==========================================

inline void unlockSPI()

{

    if (spiMutex != nullptr)

    {

        xSemaphoreGive(spiMutex);
    }
}

inline bool lockI2C1(uint32_t timeoutMs)
{
    if (i2c1Mutex == nullptr) return true;  // not yet created — allow through
    return xSemaphoreTake(i2c1Mutex, pdMS_TO_TICKS(timeoutMs)) == pdPASS;
}

inline void unlockI2C1()
{
    if (i2c1Mutex != nullptr) xSemaphoreGive(i2c1Mutex);
}

#endif // SYNC_H