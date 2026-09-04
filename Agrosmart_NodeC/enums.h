#ifndef ENUMS_H
#define ENUMS_H

#include <stdint.h>

// ==========================================
// 1. Core System State Machine
// ==========================================
enum SystemState : uint8_t
{
    STATE_BOOT = 0x00,
    STATE_LOAD_PROFILE = 0x01,
    STATE_WAIT_DATA = 0x02,
    STATE_VALIDATE = 0x03,
    STATE_HEALTH_CHECK = 0x04,
    STATE_STATE_ESTIMATION = 0x05,
    STATE_ENV_CORRECTION = 0x06,
    STATE_DECISION = 0x07,
    STATE_SAFETY_CHECK = 0x08,
    STATE_DOSE_CALCULATION = 0x09,
    STATE_SEND_COMMAND = 0x0A,
    STATE_WAIT_FEEDBACK = 0x0B,
    STATE_STABILIZATION = 0x0C,
    STATE_REMEASURE = 0x0D,
    STATE_LOG_RECORD = 0x0E,
    STATE_SYNC_CLOUD = 0x0F,
    STATE_SAFE_IDLE = 0x10,
    STATE_FAULT = 0x11,
    STATE_DEGRADED_MODE = 0x12
};

// ==========================================
// 2. Error Handling Severity Levels
// ==========================================
enum ErrorSeverity : uint8_t
{
    ERR_LEVEL_1_INFO = 0x01,
    ERR_LEVEL_2_OPERATIONAL = 0x02,
    ERR_LEVEL_3_CRITICAL = 0x03,
    ERR_LEVEL_4_CATASTROPHIC = 0x04
};

// ==========================================
// 3. Hardware & Sensor Health Flags (bitmask)
//
// health_flag in MasterSensorRecord is an OR-able bitmask.
// Multiple faults can be set simultaneously.
//
// Bits 0-3 : Node A source faults (RS-485 sensor cluster, GPS, LoRa/UART link)
// Bits 4-7 : Node B local faults  (I2C peripherals, SD card)
//
// Node B sets its own bits in lora_task / uart_task BEFORE pushing
// the record into sensorQueue. Node A's bits arrive pre-set in the
// received packet. decision_task bails on ANY non-zero flag.
// ==========================================
#define HEALTH_OK               0x00  // All systems nominal

// Node A fault bits
#define HEALTH_NODEA_SOIL_FAULT 0x01  // 7-in-1 RS-485: OOB / stale / static reading
#define HEALTH_NODEA_GPS_FAULT  0x02  // NEO-M8N GPS fix lost (gps_valid == 0)
#define HEALTH_NODEA_COMM_FAULT 0x04  // LoRa/UART link: CRC fail or packet timeout

// Node B fault bits
#define HEALTH_NODEB_BMP280_FAULT 0x10  // BMP280 offline or read returned NAN
#define HEALTH_NODEB_RAIN_FAULT   0x20  // SEN0575 offline or read returned NAN
#define HEALTH_NODEB_RTC_FAULT    0x40  // DS3231 lost power or not found on bus
#define HEALTH_NODEB_SD_FAULT     0x80  // SD card mount or write failure

// ==========================================
// 4. LoRa & UART Packet Types
// ==========================================
enum PacketType : uint8_t
{
    PACKET_UNKNOWN = 0x00,
    PACKET_SENSOR_DATA = 0x01,
    PACKET_IRRIGATION_COMMAND = 0x02,
    PACKET_COMMAND_ACK = 0x03,
    PACKET_IRRIGATION_COMPLETE = 0x04,
    PACKET_HEARTBEAT = 0x05,
    PACKET_ERROR = 0x06,
    PACKET_NACK = 0x07,
    PACKET_DIAGNOSTIC = 0x08
};

// ==========================================
// 5. Irrigation Execution Status
// ==========================================
enum IrrigationStatus : uint8_t
{
    IRRIGATION_NOT_REQUIRED = 0x00,
    IRRIGATION_REQUIRED = 0x01,
    IRRIGATION_IN_PROGRESS = 0x02,
    IRRIGATION_COMPLETED = 0x03,
    IRRIGATION_BLOCKED_SAFETY = 0x04,
    IRRIGATION_ABORTED = 0x05,
    IRRIGATION_FAILED = 0x06
};

// ==========================================
// 6. Node C (Pump Controller) Hardware Status
// ==========================================
enum NodeCStatus : uint8_t
{
    NODEC_OK = 0x00,
    NODEC_PUMP_RUNNING = 0x01,
    NODEC_PUMP_STOPPED = 0x02,
    NODEC_LOW_BATTERY = 0x03,
    NODEC_FLOW_FAULT = 0x04,
    NODEC_TIMEOUT = 0x05,
    NODEC_HARDWARE_FAULT = 0x06
};

// ==========================================
// 7. Farm Crop Categories
// ==========================================
enum CropCategory : uint8_t
{
    CROP_VEGETABLE = 0x00,
    CROP_GRAIN = 0x01,
    CROP_FRUIT = 0x02,
    CROP_FIBER = 0x03,
    CROP_OTHER = 0x04
};

// ==========================================
// 8. Growth Stage
// ==========================================
enum GrowthStage : uint8_t
{
    STAGE_INITIAL = 0x00,
    STAGE_DEVELOPMENT = 0x01,
    STAGE_MID_SEASON = 0x02,
    STAGE_LATE_SEASON = 0x03,
    STAGE_HARVEST = 0x04
};

// ==========================================
// 9. Decision Reason (Safety Locks vs Triggers)
// ==========================================
enum DecisionReason : uint8_t
{
    REASON_NONE = 0x00,

    // Positive trigger
    REASON_DEFICIT_EXCEEDED = 0x01,

    // Blocking conditions
    REASON_RAIN_LOCK = 0x10,
    REASON_LOW_BATTERY = 0x11,
    REASON_SENSOR_FAULT = 0x12,
    REASON_LORA_FAULT = 0x13,
    REASON_SD_FAULT = 0x14,
    REASON_MANUAL_OVERRIDE = 0x15,
    REASON_CONFIG_FAULT = 0x16,

    // Non-fault operational blocks
    REASON_DAILY_LIMIT = 0x17,
    REASON_STABILIZATION_DELAY = 0x18,
    REASON_BELOW_THRESHOLD = 0x19,
    REASON_INTERNAL_FAULT = 0x1A // Added for RTOS queue/memory limits
};
#endif // ENUMS_H
