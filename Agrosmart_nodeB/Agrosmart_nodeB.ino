#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "config.h"
#include "pins.h"
#include "queues.h"
#include "sync.h"
#include "config_manager.h"
#include "structs.h"
#include "tasks.h"
#include "cloud_task.h"
#include "environment_task.h"
#include "health_task.h"
#include "led_task.h"
#include "logger.h"

#ifdef DEVELOPMENT_MODE
#include "mock_task.h"
#endif

// ==========================================
// Global Object Definitions
// ==========================================
SemaphoreHandle_t spiMutex   = nullptr;
SemaphoreHandle_t i2c1Mutex  = nullptr;

QueueHandle_t sensorQueue       = nullptr;
QueueHandle_t commandQueue      = nullptr;
QueueHandle_t feedbackQueue     = nullptr;
QueueHandle_t logQueue          = nullptr;   // system logs → cloud
QueueHandle_t sdLogQueue        = nullptr;   // system logs → SD  (parallel copy)
QueueHandle_t heartbeatQueue    = nullptr;
QueueHandle_t cloudQueue        = nullptr;
QueueHandle_t sdQueue           = nullptr;   // SD archive — parallel to cloudQueue
QueueHandle_t irrigationLogQueue = nullptr;  // Node C ACKs → /IRRIGATION CSV

TaskHandle_t g_taskHandles[HTASK_COUNT] = { nullptr };

FarmProfile activeProfile;

// ==========================================
// Default farm profile (fallback when SD is absent or config file missing)
// ==========================================
static void loadDefaultProfile(FarmProfile* p)
{
    memset(p, 0, sizeof(FarmProfile));
    p->profile_version      = 1;
    p->farm_id              = 1;
    p->field_id             = 1;
    p->field_area_m2        = 1200.0f;
    p->crop_id              = CROP_VEGETABLE;
    p->growth_stage         = STAGE_MID_SEASON;
    p->root_depth_m         = 0.4f;
    p->field_capacity_vwc   = 32.0f;
    p->wilting_point_vwc    = 14.0f;
    p->mad_percent          = 50.0f;
    p->irrigation_efficiency = 82.0f;
    p->max_daily_l          = 6000.0f;
    p->max_dose_l           = 2000.0f;
    p->rain_lock_mm         = 5.0f;
#ifdef DEVELOPMENT_MODE
    p->stab_delay_min       = 0;    // no stabilization wait in simulation
#else
    p->stab_delay_min       = 90;   // 90-minute post-irrigation wait (production)
#endif
    p->default_zone_id      = 1;
}

// ==========================================
// Setup
// ==========================================
void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000);
    LOG_INFO("SYSTEM", "AgroSmart Node B Booting...");

    // ── 1. SPI Mutex ─────────────────────────────────────────────────────────
    spiMutex  = xSemaphoreCreateMutex();
    i2c1Mutex = xSemaphoreCreateMutex();
    if (spiMutex == nullptr || i2c1Mutex == nullptr) {
        LOG_ERROR("SYSTEM", "CRITICAL: Mutex Init Failed");
        while (1) { delay(1000); }
    }

    // ── 1b. I2C buses — initialized ONCE, centrally, before any task starts.
    // Both health_task (bus probes) and led_task (PCA9685) depend on Wire1
    // being up regardless of whether vEnvironmentTask itself ever runs (it
    // doesn't in DEVELOPMENT_MODE) — this used to be environment_task's job
    // alone, which left Wire1 uninitialized whenever that task was skipped.
    Wire.begin(21, 22);                          // I2C-0: BMP280 + DS3231 RTC
    Wire1.begin(I2C1_SDA_PIN, I2C1_SCL_PIN);      // I2C-1: PCA9685 LEDs + SEN0575 rain gauge
    LOG_INFO("SETUP", "I2C-0 ready (SDA=21 SCL=22) | I2C-1 ready (SDA=%d SCL=%d)",
             I2C1_SDA_PIN, I2C1_SCL_PIN);

    // ── 2. Farm Profile — load from SD; fall back to built-in defaults ────────
    // SD.begin() runs here (before tasks start) so the profile is ready for
    // vDecisionTask without any task-ordering dependency.
    bool profileFromSD = false;
    if (lockSPI(2000)) {
        if (SD.begin(SD_CS_PIN)) {
            profileFromSD = loadProfileFromSD(&activeProfile);
        }
        unlockSPI();
    }

    if (!profileFromSD) {
        LOG_WARN("SETUP", "SD profile not available — using built-in defaults.");
        loadDefaultProfile(&activeProfile);
    }

    // ── 3. Queues ─────────────────────────────────────────────────────────────
    sensorQueue        = xQueueCreate(10,  sizeof(MasterSensorRecord));
    commandQueue       = xQueueCreate(10,  sizeof(NodeCCommand));
    feedbackQueue      = xQueueCreate(10,  sizeof(NodeCFeedback));
    logQueue           = xQueueCreate(50,  sizeof(SystemEventLog));
    sdLogQueue         = xQueueCreate(50,  sizeof(SystemEventLog));
    heartbeatQueue     = xQueueCreate(5,   sizeof(NodeCHeartbeat));
    cloudQueue         = xQueueCreate(50,  sizeof(CloudPayload));  // 50 × ~200 B = 10 KB; covers ~25 h WiFi outage at 30-min poll
    sdQueue            = xQueueCreate(10,  sizeof(CloudPayload));
    irrigationLogQueue = xQueueCreate(10,  sizeof(IrrigationFeedbackLog));

    if (!sensorQueue || !commandQueue  || !feedbackQueue ||
        !logQueue    || !sdLogQueue   || !heartbeatQueue ||
        !cloudQueue  || !sdQueue      || !irrigationLogQueue) {
        LOG_ERROR("SETUP", "CRITICAL: Queue Init Failed — insufficient heap");
        while (1) { delay(1000); }
    }

    // ── 3b. Pi Pico HMI link — UART2, opened before any task can dispatch a frame ──
    Serial2.begin(PILINK_BAUD, SERIAL_8N1, PILINK_RX_PIN, PILINK_TX_PIN);
    LOG_INFO("SETUP", "UART2 Pi Pico link ready — TX=GPIO%d RX=GPIO%d @ %d baud",
             PILINK_TX_PIN, PILINK_RX_PIN, PILINK_BAUD);
    dispatchConfigPacket(activeProfile);   // static farm profile — send once now

    // ── 4. Tasks ──────────────────────────────────────────────────────────────

    // Core 0 — non-deterministic networking tasks
    xTaskCreatePinnedToCore(vCloudTask,   "CloudTask",   8192, NULL, 1, &g_taskHandles[HTASK_CLOUD], 0);
    xTaskCreatePinnedToCore(vHealthTask,  "HealthTask",  6144, NULL, 1, NULL, 0);

    // Core 1 — deterministic sensor & control tasks
    xTaskCreatePinnedToCore(vDecisionTask, "DecisionTask", 4096, NULL, 3, &g_taskHandles[HTASK_DEC],   1);
    xTaskCreatePinnedToCore(vSdTask,       "SdTask",       4096, NULL, 1, &g_taskHandles[HTASK_SD],    1);
    xTaskCreatePinnedToCore(vLedTask,      "LedTask",      3072, NULL, 1, &g_taskHandles[HTASK_LED],   1);

    // vUartTask only handles the Pi Pico's Serial2 command bytes (IRRIGATE_NOW/
    // EMERGENCY_STOP/STATUS_REQUEST) — it has no dependency on real Node A
    // hardware, so unlike vEnvironmentTask/vLoraTask it's safe and useful to
    // run in BOTH DEVELOPMENT_MODE and production. Without this, Pi Pico
    // telemetry still works (health_task/decision_task dispatch outbound
    // regardless) but its buttons silently do nothing during bench testing.
    xTaskCreatePinnedToCore(vUartTask, "UartTask", 4096, NULL, 4, &g_taskHandles[HTASK_UART], 1);

#ifdef DEVELOPMENT_MODE
    // ── Hardware Simulation (replaces vLoraTask + vEnvironmentTask) ─────────
    xTaskCreatePinnedToCore(vMockTask, "MockTask", 4096, NULL, 2, NULL, 1);
    LOG_WARN("SETUP", "*** DEVELOPMENT_MODE: MockTask running, LoRa/ENV disabled ***");
#else
    // ── Production Hardware Tasks ─────────────────────────────────────────────
    // vEnvironmentTask starts first — populates envMutex+envCache before
    // vLoraTask calls injectEnvironmentData().
    xTaskCreatePinnedToCore(vEnvironmentTask, "EnvTask",  4096, NULL, 2, &g_taskHandles[HTASK_ENV],  1);
    xTaskCreatePinnedToCore(vLoraTask,        "LoRaTask", 4096, NULL, 3, &g_taskHandles[HTASK_LORA], 1);
#endif

    LOG_INFO("SETUP", "System initialized. FreeRTOS scheduler running.");
}

void loop()
{
    // All work is done in FreeRTOS tasks.
    vTaskDelete(NULL);
}
