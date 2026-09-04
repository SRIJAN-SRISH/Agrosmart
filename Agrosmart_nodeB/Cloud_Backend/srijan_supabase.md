-- ============================================================
-- AgroSmart V1 — Master Supabase Schema
-- PostgreSQL / Supabase DDL
--
-- Architecture principle:
--   Every major firmware struct maps to exactly one table.
--   farm_id (VARCHAR) is the universal anchor across all tables.
--   record_id links sensor → decision → event (full water audit trail).
--   uploaded_at vs timestamp_epoch lets you detect network dropouts.
--   recorded_at is auto-generated from epoch — never set manually.
--
-- Table creation order matters (FK dependencies):
--   1. farm_profiles
--   2. sensor_records
--   3. irrigation_decisions
--   4. irrigation_events
--   5. system_logs
--   6. node_health
-- ============================================================


-- ============================================================
-- EXTENSION
-- Required for UUID fallback support (system_logs)
-- ============================================================
CREATE EXTENSION IF NOT EXISTS "uuid-ossp";


-- ============================================================
-- TABLE 1: farm_profiles
-- Source struct : FarmProfile
-- Written by   : Node B on boot (when WiFi task implemented)
--                Or manually via Supabase dashboard for now
-- Purpose      : Agronomic constants anchor — every sensor
--                reading and decision references this table.
-- ============================================================
CREATE TABLE farm_profiles (

    -- Identity
    farm_id         VARCHAR(32)     PRIMARY KEY,        -- e.g. "FARM_001"
    field_id        VARCHAR(32)     NOT NULL,           -- e.g. "FIELD_A"
    node_id         VARCHAR(32)     NOT NULL,           -- e.g. "NODE_B_01"

    -- Crop metadata (maps to enums.h CropCategory / GrowthStage)
    crop_id         SMALLINT        NOT NULL DEFAULT 0,
    growth_stage    SMALLINT        NOT NULL DEFAULT 0,
    profile_version SMALLINT        NOT NULL DEFAULT 1,

    -- Physical field parameters
    field_area_m2           REAL    NOT NULL,
    root_depth_m            REAL    NOT NULL,

    -- Soil water constants
    field_capacity_vwc      REAL    NOT NULL,
    wilting_point_vwc       REAL    NOT NULL,

    -- Irrigation logic parameters
    mad_percent             REAL    NOT NULL,
    irrigation_efficiency   REAL    NOT NULL,
    max_daily_l             REAL    NOT NULL,
    max_dose_l              REAL    NOT NULL,
    rain_lock_mm            REAL    NOT NULL,
    stab_delay_min          INTEGER NOT NULL,           -- From FarmProfile.stab_delay_min
    default_zone_id         SMALLINT NOT NULL DEFAULT 1,

    -- Housekeeping
    created_at      TIMESTAMPTZ     DEFAULT NOW(),
    updated_at      TIMESTAMPTZ     DEFAULT NOW()       -- Update on profile change
);

COMMENT ON TABLE farm_profiles IS
    'Static agronomic configuration. Maps to FarmProfile struct in firmware.';


-- ============================================================
-- TABLE 2: sensor_records
-- Source struct : MasterSensorRecord
-- Written by   : Node B cloud_task after every UART ingestion
-- Purpose      : Complete raw telemetry archive.
--                Every downstream table (decisions, events)
--                traces back to a row here via record_id.
-- ============================================================
CREATE TABLE sensor_records (

    -- Surrogate PK (fast joins) + business key
    id              BIGINT          GENERATED ALWAYS AS IDENTITY PRIMARY KEY,
    record_id       VARCHAR(24)     UNIQUE NOT NULL,    -- From MasterSensorRecord.record_id

    -- Farm anchor
    farm_id         VARCHAR(32)     REFERENCES farm_profiles(farm_id),
    node_id         VARCHAR(32)     NOT NULL DEFAULT 'NODE_B_01',

    -- Timing
    timestamp_epoch BIGINT          NOT NULL,
    recorded_at     TIMESTAMPTZ     GENERATED ALWAYS AS
                    (to_timestamp(timestamp_epoch)) STORED,  -- Auto-derived, never set manually
    boot_time_ms    BIGINT,
    record_sequence BIGINT,
    timestamp_valid SMALLINT        DEFAULT 0,          -- 0=fallback, 1=RTC synced

    -- Node A: Soil parameters (7-in-1 RS485 sensor)
    vwc_percent     REAL,
    soil_temp       REAL,
    ec              REAL,
    ph              REAL,
    nitrogen        REAL,
    phosphorus      REAL,
    potassium       REAL,

    -- Node A: GPS (NEO-M8N)
    gps_valid       SMALLINT        DEFAULT 0,
    latitude        DOUBLE PRECISION,
    longitude       DOUBLE PRECISION,

    -- Node B: Local environment (BMP280 + SEN0575)
    -- Injected by injectEnvironmentData() in environment_task.cpp
    air_temp        REAL,
    humidity        REAL,
    pressure        REAL,
    rainfall        REAL,

    -- Health
    health_flag     SMALLINT        DEFAULT 0,          -- Maps to HealthFlag enum

    -- Cloud housekeeping
    uploaded_at     TIMESTAMPTZ     DEFAULT NOW()       -- When it hit Supabase
);

COMMENT ON TABLE sensor_records IS
    'Complete fused telemetry. Maps to MasterSensorRecord. '
    'air_temp/pressure/rainfall come from Node B environment_task via injectEnvironmentData().';


-- ============================================================
-- TABLE 3: irrigation_decisions
-- Source struct : IrrigationDecision
-- Written by   : Node B cloud_task after every decision cycle
-- Purpose      : Full audit trail of the decision engine.
--                Answers: "Why did the system irrigate or not?"
-- ============================================================
CREATE TABLE irrigation_decisions (

    -- Surrogate PK + business key
    id              BIGINT          GENERATED ALWAYS AS IDENTITY PRIMARY KEY,
    decision_id     VARCHAR(24)     UNIQUE NOT NULL,    -- Derived from record_id in firmware

    -- Relational anchors
    record_id       VARCHAR(24)     REFERENCES sensor_records(record_id),
    farm_id         VARCHAR(32)     REFERENCES farm_profiles(farm_id),
    node_id         VARCHAR(32)     NOT NULL DEFAULT 'NODE_B_01',

    -- Timing
    timestamp_epoch BIGINT          NOT NULL,
    recorded_at     TIMESTAMPTZ     GENERATED ALWAYS AS
                    (to_timestamp(timestamp_epoch)) STORED,
    boot_time_ms    BIGINT,
    record_sequence BIGINT,
    timestamp_valid SMALLINT        DEFAULT 0,

    -- Decision engine outputs (from IrrigationDecision struct)
    taw_mm          REAL,           -- Total Available Water
    raw_mm          REAL,           -- Readily Available Water (MAD threshold)
    deficit_mm      REAL,           -- Current soil moisture deficit
    net_required_mm REAL,           -- Ideal agronomic water depth needed
    gross_required_l REAL,          -- Actual liters after efficiency correction
    vpd_kpa         REAL,           -- Vapor Pressure Deficit

    -- Outcome
    irrigation_required BOOLEAN     NOT NULL DEFAULT FALSE,
    decision_reason SMALLINT        NOT NULL DEFAULT 0, -- Maps to DecisionReason enum
    status_flag     SMALLINT        DEFAULT 0,

    -- Cloud housekeeping
    uploaded_at     TIMESTAMPTZ     DEFAULT NOW()
);

COMMENT ON TABLE irrigation_decisions IS
    'Decision engine output per sensor cycle. Maps to IrrigationDecision struct. '
    'decision_reason maps to DecisionReason enum in enums.h.';


-- ============================================================
-- TABLE 4: irrigation_events
-- Source structs: NodeCCommand + NodeCFeedback + IrrigationEvent
-- Written by    : Node B cloud_task after receiving NodeCFeedback
-- Purpose       : Command vs delivered volume audit.
--                 Answers: "What was ordered vs what actually happened?"
--                 Critical for pump efficiency and leak/clog detection.
-- ============================================================
CREATE TABLE irrigation_events (

    -- Surrogate PK + business key
    id              BIGINT          GENERATED ALWAYS AS IDENTITY PRIMARY KEY,
    event_id        VARCHAR(24)     UNIQUE NOT NULL,

    -- Relational anchors
    decision_id     VARCHAR(24)     REFERENCES irrigation_decisions(decision_id),
    command_id      VARCHAR(24)     NOT NULL,           -- From NodeCCommand.command_id
    farm_id         VARCHAR(32)     REFERENCES farm_profiles(farm_id),
    node_id         VARCHAR(32)     NOT NULL DEFAULT 'NODE_C_01',

    -- Timing
    timestamp_epoch BIGINT          NOT NULL,
    executed_at     TIMESTAMPTZ     GENERATED ALWAYS AS
                    (to_timestamp(timestamp_epoch)) STORED,

    -- Command (from NodeCCommand)
    target_volume_l REAL            NOT NULL,
    zone_id         SMALLINT        NOT NULL DEFAULT 1,
    max_runtime_sec INTEGER,

    -- Feedback (from NodeCFeedback)
    delivered_volume_l  REAL,       -- Actual volume pumped
    flow_rate_lpm       REAL,       -- For clog/leak detection
    runtime_sec         INTEGER,    -- Actual time run
    battery_voltage     REAL,       -- Node C power state

    -- Learning layer (from IrrigationEvent)
    moisture_before REAL,           -- VWC% just before irrigation
    moisture_after  REAL,           -- VWC% after stabilization remeasure

    -- Status
    status_flag     SMALLINT        DEFAULT 0,          -- Maps to IrrigationStatus enum

    -- Cloud housekeeping
    uploaded_at     TIMESTAMPTZ     DEFAULT NOW()
);

COMMENT ON TABLE irrigation_events IS
    'Physical pump execution record. Merges NodeCCommand, NodeCFeedback, IrrigationEvent. '
    'flow_rate_lpm enables clog and leak detection analytics.';


-- ============================================================
-- TABLE 5: system_logs
-- Source struct : SystemEventLog
-- Written by   : All tasks via logQueue → cloud_task
-- Purpose      : Remote debugging and fault forensics.
--                Flat table — no FK to data tables because
--                faults are often asynchronous and independent
--                of any specific sensor reading.
-- ============================================================
CREATE TABLE system_logs (

    -- Auto-generated PK (no business key needed for logs)
    id              BIGINT          GENERATED ALWAYS AS IDENTITY PRIMARY KEY,

    -- Context
    farm_id         VARCHAR(32),    -- No FK — must log even if farm_profiles missing
    node_id         VARCHAR(32)     NOT NULL DEFAULT 'NODE_B_01',
    record_sequence BIGINT,

    -- Timing
    timestamp_epoch BIGINT,
    logged_at       TIMESTAMPTZ     GENERATED ALWAYS AS
                    (to_timestamp(timestamp_epoch)) STORED,
    boot_time_ms    BIGINT,

    -- Event data (maps directly to SystemEventLog struct)
    severity        SMALLINT        NOT NULL,           -- 1=INFO 2=OPERATIONAL 3=CRITICAL 4=CATASTROPHIC
    source          VARCHAR(32)     NOT NULL,           -- e.g. "UART_TASK", "SD_TASK", "ENV_TASK"
    message         TEXT            NOT NULL,           -- Full error string

    -- Cloud housekeeping
    uploaded_at     TIMESTAMPTZ     DEFAULT NOW()
);

COMMENT ON TABLE system_logs IS
    'Flat audit trail. Maps to SystemEventLog. No FK on purpose — '
    'must accept logs even when other tables are empty or broken.';


-- ============================================================
-- TABLE 6: node_health
-- Source      : health_task.cpp (currently empty — to implement)
-- Written by  : Each node periodically via WiFi
-- Purpose     : Fleet monitoring, battery trending,
--               predictive maintenance, uptime tracking.
-- ============================================================
CREATE TABLE node_health (

    id              BIGINT          GENERATED ALWAYS AS IDENTITY PRIMARY KEY,

    -- Node identity
    node_id         VARCHAR(32)     NOT NULL,           -- "NODE_A_01", "NODE_B_01", "NODE_C_01"
    farm_id         VARCHAR(32),
    role            VARCHAR(16)     NOT NULL,           -- "SENSOR", "DECISION", "ACTUATOR"
    mac_address     VARCHAR(17),                        -- For device fingerprinting
    firmware_version VARCHAR(16),

    -- Health metrics (from ESP.getFreeHeap(), WiFi.RSSI() etc.)
    battery_voltage REAL,
    free_heap_bytes INTEGER,
    uptime_ms       BIGINT,
    wifi_rssi       INTEGER,
    lora_rssi       INTEGER,        -- Last received signal strength from peer node

    -- Housekeeping
    last_seen       TIMESTAMPTZ     DEFAULT NOW()       -- Updated on every health ping
);

COMMENT ON TABLE node_health IS
    'Fleet health monitoring. One row per node per health ping. '
    'Maps to health_task.cpp (stub). Use for battery trends and predictive maintenance.';


-- ============================================================
-- INDEXES
-- Created on all columns used in WHERE, JOIN, and ORDER BY.
-- Dramatically speeds up dashboard queries on large datasets.
-- ============================================================

-- Time-series queries (most common dashboard pattern)
CREATE INDEX idx_sensor_recorded_at
    ON sensor_records(recorded_at DESC);

CREATE INDEX idx_decision_recorded_at
    ON irrigation_decisions(recorded_at DESC);

CREATE INDEX idx_event_executed_at
    ON irrigation_events(executed_at DESC);

CREATE INDEX idx_logs_logged_at
    ON system_logs(logged_at DESC);

-- Farm filtering (multi-farm queries)
CREATE INDEX idx_sensor_farm_id
    ON sensor_records(farm_id);

CREATE INDEX idx_decision_farm_id
    ON irrigation_decisions(farm_id);

CREATE INDEX idx_event_farm_id
    ON irrigation_events(farm_id);

-- Log severity filtering (most common debug query)
CREATE INDEX idx_logs_severity
    ON system_logs(severity);

CREATE INDEX idx_logs_source
    ON system_logs(source);

-- Join performance
CREATE INDEX idx_decision_record_id
    ON irrigation_decisions(record_id);

CREATE INDEX idx_event_decision_id
    ON irrigation_events(decision_id);

-- Node health lookups
CREATE INDEX idx_health_node_id
    ON node_health(node_id);

CREATE INDEX idx_health_last_seen
    ON node_health(last_seen DESC);


-- ============================================================
-- ROW LEVEL SECURITY (RLS)
-- Enable on all tables before building a web dashboard.
-- Without RLS, any client with your anon key can read
-- ALL farm data — a critical privacy and security risk
-- if AgroSmart is ever deployed across multiple farms.
-- ============================================================
ALTER TABLE farm_profiles      ENABLE ROW LEVEL SECURITY;
ALTER TABLE sensor_records     ENABLE ROW LEVEL SECURITY;
ALTER TABLE irrigation_decisions ENABLE ROW LEVEL SECURITY;
ALTER TABLE irrigation_events  ENABLE ROW LEVEL SECURITY;
ALTER TABLE system_logs        ENABLE ROW LEVEL SECURITY;
ALTER TABLE node_health        ENABLE ROW LEVEL SECURITY;

-- Development policy: allow all access (replace with farm-scoped policy in production)
-- To activate: uncomment these and run in Supabase SQL editor
-- CREATE POLICY "dev_allow_all" ON farm_profiles      FOR ALL USING (true);
-- CREATE POLICY "dev_allow_all" ON sensor_records     FOR ALL USING (true);
-- CREATE POLICY "dev_allow_all" ON irrigation_decisions FOR ALL USING (true);
-- CREATE POLICY "dev_allow_all" ON irrigation_events  FOR ALL USING (true);
-- CREATE POLICY "dev_allow_all" ON system_logs        FOR ALL USING (true);
-- CREATE POLICY "dev_allow_all" ON node_health        FOR ALL USING (true);