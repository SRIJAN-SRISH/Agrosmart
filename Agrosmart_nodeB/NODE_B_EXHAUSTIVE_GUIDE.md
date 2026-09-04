# NODE B: The Central Gateway
**Exhaustive Technical Specification & "Every Single Datum" Analysis**

---

## 1. System Topology & Hardware Role
**Hardware:** ESP32-WROOM-32U
**Operating System:** FreeRTOS (Asymmetric Multiprocessing)
**Directory Path:** `C:\Users\abhishek kumar thaku\OneDrive\Documents\Arduino\Agrosmart_nodeB`

Node B acts as the definitive "brain" of the AgroSmart system. It assumes that Node A is a "dumb" sensor and Node C is a "dumb" actuator. Node B is responsible for mathematical decision-making, historical tracking (via SD Card and NVS), and synchronization with the Supabase cloud infrastructure via Wi-Fi.

---

## 2. FreeRTOS Task Matrix & Concurrency

Node B strictly partitions its tasks across the two ESP32 processor cores to prevent network latency (Wi-Fi/HTTP) from blocking real-time control (LoRa/UART/SPI).

### Core 0 (Network & Diagnostics - Non-Deterministic)
- **`vCloudTask` (Priority 1):** Suspends and waits on data from `cloudQueue` and `logQueue`. Formats payloads into JSON dynamically (strictly using stack allocation like `StaticJsonDocument` to prevent Heap fragmentation). Dispatches `POST` requests to the REST API utilizing the `Prefer: return=minimal,resolution=ignore-duplicates` header to handle network idempotency.
- **`vHealthTask` (Priority 1):** The internal watchdog. Monitors `heap_caps_get_largest_free_block()`. If the largest free block drops below a critical threshold (e.g., 16KB), JSON parsing could trigger a `LoadProhibited` panic, so this task triggers a `controlledReboot()`.

### Core 1 (Control Loop - Deterministic)
- **`vDecisionTask` (Priority 3):** The FAO-56 Mathematical Brain. Processes soil deficit, assesses safety gates, tracks daily limits via Flash NVS, and routes commands.
- **`vLoraTask` (Priority 3):** Asynchronous state machine governing the SX1278 transceiver. Blocks on `commandQueue` and handles `TX_WAIT_ACK` sequencing.
- **`vUartTask` (Priority 4):** Ingests serial bytes from Node A. Implements COBS decoding and CRC-8 validation before injecting payloads into the `sensorQueue`.
- **`vSdTask` (Priority 2):** Mounts the FAT32 filesystem over the HSPI bus. Pulls from `sdQueue` and `sdLogQueue` to append `.csv` matrices asynchronously.

---

## 3. Queue Topology & FMEA Discard Policies
Inter-task communication happens purely through FreeRTOS Queues (`queues.h`). Based on Failure Mode and Effects Analysis (FMEA), if a queue fills up, explicit rules apply:

- **`commandQueue` & `feedbackQueue` (Priority 0):** NEVER discard. Senders block (`portMAX_DELAY`) until space frees. Dropping a command or an ACK compromises irrigation safety.
- **`sensorQueue` (Priority 1):** Drop the OLDEST item. The system only cares about the most recent environmental reality.
- **`logQueue` & `heartbeatQueue` (Priority 2/3):** Discard immediately. Logging must never block or crash the system.
- **`cloudQueue` (Priority 2):** Retains data on HTTP failure. The payload remains at the front of the queue for the next loop.

---

## 4. Exact Data Structures & Memory Models (`structs.h`)
All structs utilize `__attribute__((packed))` to ensure memory alignment matches exactly across different compilers and architectures (ESP32 vs. Node A/C hardware).

### 4.1 The Configuration Layer
```cpp
struct FarmProfile {
    uint8_t profile_version;
    uint16_t farm_id;
    uint16_t field_id;
    float field_area_m2;
    uint8_t crop_id;
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
```
*Loaded from SD card JSON/INI. Dictates all limits for `vDecisionTask`.*

### 4.2 The Fused Sensory Ingest
```cpp
struct MasterSensorRecord {
    char record_id[24];
    uint32_t timestamp_epoch;
    float vwc_percent; // From Node A RS485
    float soil_temp;
    float ec;
    float ph;
    float nitrogen;
    float phosphorus;
    float potassium;
    float latitude; // From Node A GPS
    float longitude;
    float air_temp; // From Node B Local I2C (BMP280)
    float humidity;
    float pressure;
    float rainfall; // From Node B Local I2C (SEN0575)
    uint8_t health_flag; // Bitmask of faults
} __attribute__((packed));
```

### 4.3 The Mathematical Output & Action
```cpp
struct IrrigationDecision {
    char decision_id[24];
    float taw_mm;       // Total Available Water
    float raw_mm;       // Readily Available Water
    float deficit_mm;   // Calculated Deficit
    float net_required_mm;
    float gross_required_l;
    float vpd_kpa;      // Vapor Pressure Deficit
    uint8_t irrigation_required;
    uint8_t status_flag;
    uint8_t decision_reason; // Gate flag (e.g. REASON_RAIN_LOCK)
};

struct NodeCCommand {
    char command_id[24];
    float target_volume_l;
    uint16_t max_runtime_sec;
    uint8_t zone_id;
} __attribute__((packed));
```

---

## 5. Agronomic Math & Execution Gates (`decision_task.cpp`)

### 5.1 Deficit & VPD Formulas
Node B utilizes a derivative of the **FAO-56 Penman-Monteith** model.
1. **Total Available Water (TAW):** $(FieldCapacity - WiltingPoint) \times RootDepth \times 1000$
2. **Readily Available Water (RAW):** $TAW \times MAD$ (Management Allowed Depletion)
3. **Deficit:** $(FieldCapacity - VWC) \times RootDepth \times 1000$
4. **Vapor Pressure Deficit (VPD):** Calculated via standard exponential saturation functions using `air_temp` and `humidity`.

### 5.2 The Safety Gate Cascade
Before `vDecisionTask` issues a `NodeCCommand`, it passes the data through a strict cascade of negative gates. If *any* gate fails, execution aborts, and the `DecisionReason` is logged to the SD card.

1. **Gate: Sensor Fault (`sensorRecord.health_flag != HEALTH_OK`)**
   - Aborts if Node A's GPS is lost, if the RS485 probe returns static, or if the I2C bus failed on Node B.
2. **Gate: Node C Battery (`isNodeCBatteryOk()`)**
   - Aborts if Node C's most recent `LoRaHeartbeatPacket` shows voltage `< 11.2V`.
3. **Gate: Rain Lock**
   - Aborts if local rainfall `> rain_lock_mm` AND field is at capacity.
4. **Gate: Stabilization Delay**
   - Aborts if the time since the last pump stop is less than `stab_delay_min`. Allows water to seep into the root zone before re-evaluating.
5. **Gate: Daily Volume Limit**
   - Aborts if `daily_total_l + pending_volume_l` exceeds `max_daily_l`.

### 5.3 Non-Volatile Storage (NVS) Safety
If Node B loses power *while* Node C is irrigating, Node B wakes up with its RAM erased. To prevent over-irrigating on reboot, Node B saves `pending_volume_l` to the ESP32's Flash NVS (`nvsSavePending()`) *before* transmitting a command. When Node B boots, it calls `nvsLoadPending()` to resume tracking the outstanding volume.

---

## 6. Security Layer (`srijan_security.cpp`)
All wireless payloads transmitted to Node C traverse the **SRIJAN Security Architecture**.
- Uses AES-128-GCM to prevent replay attacks and spoofing (e.g., a bad actor broadcasting a fake "Turn On Pump" command).
- The protocol relies on a 12-Byte header mapping the version and packet type, followed by a dynamically generated nonce synced with Node B's internal GPS epoch clock.
