// ============================================================
// environment_task.cpp
// AgroSmart Node B — Local Environment Sensor Task
//
// Hardware:
//   I2C Bus 0 (Wire)  : GPIO 21 (SDA), 22 (SCL) → BMP280 + DS3231 RTC
//   I2C Bus 1 (Wire1) : GPIO 33 (SDA), 32 (SCL) → PCA9685 LEDs + SEN0575 Rain Gauge
//
// Both Wire.begin() and Wire1.begin() are called ONCE, centrally, in
// Agrosmart_nodeB.ino's setup() — before any task starts, so both buses
// are guaranteed initialized regardless of which tasks actually run
// (e.g. this task doesn't start at all in DEVELOPMENT_MODE, but
// led_task/health_task still need Wire1 either way). This task used to
// own a second, separate TwoWire(1) instance ("I2C_Rain") distinct from
// the global Wire1 that led_task.cpp/health_task.cpp use — two C++
// objects both claiming the same physical I2C1 peripheral, with only one
// of them ever actually begun. Fixed by sharing the single global Wire1.
//
// Architecture:
//   - Reads BMP280 and SEN0575 every SENSOR_POLL_INTERVAL_MS
//   - Stores readings in a mutex-protected cache (envCache)
//   - injectEnvironmentData() is called by uart_task / lora_task
//     to stamp live local readings into MasterSensorRecord BEFORE
//     that record enters sensorQueue → decision engine → SD log
//   - Faults are pushed to logQueue for SD audit trail
//   - NO volatile globals. NO separate environment queue.
//   - SPI mutex NOT needed — I2C buses are independent of SPI.
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <Adafruit_BMP280.h>
#include "DFRobot_RainfallSensor.h"
#include "RTClib.h"
#include "config.h"
#include "queues.h"
#include "enums.h"
#include "structs.h"
#include "environment_task.h"
#include "health_task.h"
#include "sync.h"
#include "logger.h"
#include "pins.h"
#include <HardwareSerial.h>

// ============================================================
// Hardware Objects (file-scoped — not exposed externally)
// ============================================================
static Adafruit_BMP280            bmp;
static DFRobot_RainfallSensor_UART rainSensor(&Serial1);
static RTC_DS3231                 rtc;

// ============================================================
// Sensor Health Flags
// ============================================================
static bool bmpReady  = false;
static bool rainReady = false;
static bool rtcReady  = false;

// ============================================================
// Environment Cache
// Holds the most recent valid readings.
// Protected by envMutex so injectEnvironmentData() is safe
// to call from uart_task or lora_task on any core.
// ============================================================
static struct {
    float air_temp;   // °C  — NAN if BMP280 offline
    float pressure;   // hPa — NAN if BMP280 offline
    float rainfall;   // mm  — NAN if SEN0575 offline
} envCache = { NAN, NAN, NAN };

static SemaphoreHandle_t envMutex = nullptr;

// ============================================================
// Internal: Push fault to logQueue (non-blocking)
// ============================================================
static void logEnvFault(uint8_t severity, const char *message)
{
    SystemEventLog entry;
    memset(&entry, 0, sizeof(entry));

    entry.severity        = severity;
    entry.boot_time_ms    = (uint32_t)millis();
    entry.timestamp_valid = 0; // RTC may not be available yet

    strncpy(entry.source,  "ENV_TASK", sizeof(entry.source) - 1);
    strncpy(entry.message, message,    sizeof(entry.message) - 1);

    pushLog(entry); // → both cloud logQueue and SD sdLogQueue
}

// ============================================================
// Public API: getNodeBEpoch()
// Returns current Unix epoch from DS3231.
// Returns 0 if RTC never initialised or has lost power.
// I2C read is fast (<1 ms) — no mutex needed; Wire is not SPI-shared.
// ============================================================
uint32_t getNodeBEpoch()
{
    if (!rtcReady) return 0;
    DateTime now = rtc.now();
    // DS3231 that lost power returns 2000-01-01 00:00:00 (epoch 946684800).
    // Treat any epoch before 2024-01-01 as invalid (RTC not set).
    uint32_t ep = (uint32_t)now.unixtime();
    return (ep > 1704067200UL) ? ep : 0;
}

// ============================================================
// Public API: injectEnvironmentData()
// Called by uart_task.cpp and lora_task.cpp immediately before
// pushing a MasterSensorRecord into sensorQueue.
// Copies the cached readings into the caller's variables.
// If the cache holds NAN (sensor offline), NAN is injected —
// decision_task.cpp validates ranges and flags HEALTH_SENSOR_FAULT.
// ============================================================
void injectEnvironmentData(float &air_temp, float &pressure, float &rainfall)
{
    if (envMutex == nullptr) {
        air_temp = NAN;
        pressure = NAN;
        rainfall = NAN;
        return;
    }

    if (xSemaphoreTake(envMutex, pdMS_TO_TICKS(50)) == pdPASS) {
        air_temp = envCache.air_temp;
        pressure = envCache.pressure;
        rainfall = envCache.rainfall;
        xSemaphoreGive(envMutex);
    } else {
        // Mutex timeout — return NAN rather than stale unlocked data
        air_temp = NAN;
        pressure = NAN;
        rainfall = NAN;
        logEnvFault(ERR_LEVEL_2_OPERATIONAL, "ENV cache mutex timeout");
    }
}

// ============================================================
// FreeRTOS Task
// ============================================================
void vEnvironmentTask(void *pvParameters)
{
    LOG_INFO("ENV", "Booting environment sensor task...");

    // -- Create cache mutex --
    envMutex = xSemaphoreCreateMutex();
    if (envMutex == nullptr) {
        LOG_ERROR("ENV", "FATAL: Could not allocate envMutex. Task halted.");
        logEnvFault(ERR_LEVEL_4_CATASTROPHIC, "envMutex alloc failed");
        vTaskDelete(nullptr);
        return;
    }

    // I2C-0/I2C-1 are already initialized centrally in setup() (Agrosmart_nodeB.ino)
    // before any task starts — do not re-begin() either bus here.

    // --------------------------------------------------------
    // 1. DS3231 RTC
    // Used to timestamp logEnvFault entries accurately.
    // --------------------------------------------------------
    if (!rtc.begin(&Wire)) {
        LOG_WARN("ENV", "DS3231 not found on GPIO 21/22.");
        logEnvFault(ERR_LEVEL_2_OPERATIONAL, "DS3231 init failed");
    } else {
        rtcReady = true;
        if (rtc.lostPower()) {
            rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
            LOG_WARN("ENV", "RTC lost power — reset to compile time.");
            logEnvFault(ERR_LEVEL_1_INFO, "RTC power loss: reset to compile time");
        }
        DateTime now = rtc.now();
        LOG_INFO("ENV", "DS3231 online. Time: %04d-%02d-%02d %02d:%02d:%02d",
                 now.year(), now.month(), now.day(),
                 now.hour(), now.minute(), now.second());
    }

    // --------------------------------------------------------
    // 2. BMP280 — Temperature + Pressure
    // --------------------------------------------------------
    if (!bmp.begin(0x76)) {
        LOG_WARN("ENV", "BMP280 not found at 0x76 on GPIO 21/22. Try 0x77 if SDO is pulled high.");
        logEnvFault(ERR_LEVEL_2_OPERATIONAL, "BMP280 init failed at 0x76");
    } else {
        bmpReady = true;
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                        Adafruit_BMP280::SAMPLING_X2,
                        Adafruit_BMP280::SAMPLING_X16,
                        Adafruit_BMP280::FILTER_X16,
                        Adafruit_BMP280::STANDBY_MS_500);
        LOG_INFO("ENV", "BMP280 online. Temp: %.2f°C  Pressure: %.2f hPa",
                 bmp.readTemperature(), bmp.readPressure() / 100.0F);
    }

    // --------------------------------------------------------
    // 3. SEN0575 Rain Gauge — UART1
    // --------------------------------------------------------
    Serial1.begin(9600, SERIAL_8N1, RAIN_RX_PIN, RAIN_TX_PIN);
    if (!rainSensor.begin()) {
        LOG_WARN("ENV", "SEN0575 not found on UART1 (TX=%d, RX=%d).", RAIN_TX_PIN, RAIN_RX_PIN);
        logEnvFault(ERR_LEVEL_2_OPERATIONAL, "SEN0575 init failed on UART");
    } else {
        rainReady = true;
        LOG_INFO("ENV", "SEN0575 online. Rainfall: %.2f mm", rainSensor.getRainfall());
    }

    // Boot summary
    LOG_INFO("ENV", "Sensor Boot Summary: RTC:%s BMP280:%s Rain:%s",
             rtcReady ? "ONLINE" : "OFFLINE",
             bmpReady ? "ONLINE" : "OFFLINE",
             rainReady ? "ONLINE" : "OFFLINE");

    // --------------------------------------------------------
    // Sensor Read Loop
    // Runs at SENSOR_POLL_INTERVAL_MS (config.h).
    // Writes fresh readings into envCache under mutex.
    // uart_task / lora_task call injectEnvironmentData()
    // independently — they are never blocked by this loop.
    // --------------------------------------------------------
    for (;;) {
        healthPing(HTASK_ENV);

        float newTemp     = NAN;
        float newPressure = NAN;
        float newRainfall = NAN;

        // -- BMP280 read --
        if (bmpReady) {
            float t = bmp.readTemperature();
            float p = bmp.readPressure() / 100.0F;

            if (t == 0.0F && p == 0.0F) {
                // Both zero simultaneously = I2C fault mid-run
                logEnvFault(ERR_LEVEL_2_OPERATIONAL, "BMP280 read failed (0/0)");
            } else {
                newTemp     = t;
                newPressure = p;
            }
        }

        // -- SEN0575 read (UART is thread-safe here since no other task uses Serial1) --
        if (rainReady) {
            newRainfall = rainSensor.getRainfall();
        }

        // -- Update cache under mutex --
        if (xSemaphoreTake(envMutex, pdMS_TO_TICKS(100)) == pdPASS) {
            envCache.air_temp = newTemp;
            envCache.pressure = newPressure;
            envCache.rainfall = newRainfall;
            xSemaphoreGive(envMutex);
        }

        // -- Serial diagnostic --
        LOG_DEBUG("ENV", "Local Env | Temp: %s | Press: %s | Rain: %s",
                  isnan(newTemp) ? "OFFLINE" : (String(newTemp, 2) + " C").c_str(),
                  isnan(newPressure) ? "OFFLINE" : (String(newPressure, 2) + " hPa").c_str(),
                  isnan(newRainfall) ? "OFFLINE" : (String(newRainfall, 2) + " mm").c_str());

        vTaskDelay(pdMS_TO_TICKS(SENSOR_POLL_INTERVAL_MS));
    }
}
