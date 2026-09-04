#ifndef STRUCTS_H
#define STRUCTS_H

#include <stdint.h>
#include "enums.h"

// 1. Static Configuration (Loaded from SD)
struct FarmProfile
{
    uint8_t profile_version;
    uint8_t reserved;
    uint16_t farm_id;
    uint16_t field_id;
    float field_area_m2;
    uint8_t crop_id;
    uint8_t growth_stage;
    float root_depth_m;
    float field_capacity_vwc;
    float wilting_point_vwc;
    float mad_percent;
    float irrigation_efficiency;
    float max_daily_l;
    float max_dose_l;
    float rain_lock_mm;
    uint16_t stab_delay_min;
    uint8_t default_zone_id;
};

// 2. The Fused Input (Ingested via UART + Local I2C)
// CRITICAL FIX: Added __attribute__((packed)) to ensure radio memory matches Node A exactly.
struct MasterSensorRecord
{
    char record_id[24];
    uint32_t timestamp_epoch;
    uint32_t boot_time_ms;
    uint32_t record_sequence;
    uint8_t timestamp_valid;

    // Node A Soil Parameters (7-in-1 RS485)
    float vwc_percent;
    float soil_temp;
    float ec;
    float ph;
    float nitrogen;
    float phosphorus;
    float potassium;

    // Node A GPS (NEO-M8N)
    uint8_t gps_valid;
    float latitude;
    float longitude;

    // Node B Local Environment
    float air_temp;
    float humidity;
    float pressure;
    float rainfall;

    uint8_t health_flag;
} __attribute__((packed));

// 3. The Mathematical Output
struct IrrigationDecision
{
    char decision_id[24];
    char record_id[24];
    uint32_t timestamp_epoch;
    uint32_t boot_time_ms;
    uint32_t record_sequence;
    uint8_t timestamp_valid;

    float taw_mm;
    float raw_mm;

    float deficit_mm;
    float net_required_mm;
    float gross_required_l;
    float vpd_kpa;
    uint8_t irrigation_required;
    uint8_t status_flag;
    uint8_t decision_reason;
};

// 4. The Action Command (Sent via LoRa)
struct NodeCCommand
{
    char command_id[24];
    float target_volume_l;
    uint16_t max_runtime_sec;
    uint8_t zone_id;
} __attribute__((packed));

// 5. The Physical Result (Received via LoRa)
struct NodeCFeedback
{
    char command_id[24];
    float delivered_volume_l;
    float flow_rate_lpm;
    float battery_voltage;
    uint16_t runtime_sec;
    uint8_t status_flag;
} __attribute__((packed));

// 6. Node C Health (Received via LoRa)
struct NodeCHeartbeat
{
    float battery_voltage;
    uint8_t status_flag;
} __attribute__((packed));

// 7. The Learning Layer Record (Logged to SD)
struct IrrigationEvent
{
    char decision_id[24];
    char record_id[24];
    uint32_t timestamp_epoch;
    uint32_t boot_time_ms;
    uint32_t record_sequence;
    uint8_t timestamp_valid;
    float target_volume_l;
    float delivered_volume_l;
    uint16_t runtime_sec;
    float moisture_before;
    float moisture_after;
    uint8_t zone_id;
    uint8_t status_flag;
};

// 8. System Health & Fault Tracking (Logged to SD)
struct SystemEventLog
{
    char record_id[24];
    uint32_t timestamp_epoch;
    uint32_t boot_time_ms;
    uint32_t record_sequence;
    uint8_t timestamp_valid;

    uint8_t severity;
    char source[32];
    char message[64];
};

// 9. Cloud + SD Delivery Payload (shared by cloudQueue and sdQueue)
struct CloudPayload {
    MasterSensorRecord  sensorRecord;
    IrrigationDecision  decision;
    bool                hasDecision;
};

// 10. Irrigation Feedback Log (SD /IRRIGATION CSV)
// Built by decision_task when NodeCFeedback is processed; pushed to irrigationLogQueue.
// Carries the feedback plus the epoch at which the pump completed, for time-alignment.
struct IrrigationFeedbackLog {
    uint32_t    timestamp_epoch;    // epoch when feedback was received by Node B
    NodeCFeedback feedback;         // the full Node C ACK
};

#endif // STRUCTS_H
