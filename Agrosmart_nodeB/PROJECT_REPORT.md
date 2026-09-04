# AgroSmart Project Report
**Comprehensive System Methodology & Execution Summary**

## 1. Executive Summary
AgroSmart is an enterprise-grade precision agriculture IoT system comprising three highly specialized, decoupled nodes. The system captures environmental data, applies rigorous agronomic mathematics to determine water deficits, and executes physical irrigation with safety-critical fallbacks. 

The architecture is built around **Node B** (the Central Gateway), which acts as the deterministic brain of the system, orchestrating data from **Node A** (Edge Sensor) and commanding **Node C** (Actuator) over long-range RF.

---

## 2. Methodology & System Roles

### 2.1 Node A (The Edge Sensor)
- **Role:** Pure sensory ingest.
- **Methodology:** Node A operates on a strict, time-slotted loop to poll environmental data. It reads soil metrics (Volumetric Water Content, Temperature, EC, pH, NPK) via an RS-485 Modbus interface and gathers precise timestamp/location data via a NEO-M8N GPS module. 
- **Execution:** Node A holds no operational state or decision-making power. It formats the raw data into a strict binary structure (`MasterSensorRecord`), calculates a CRC-16 checksum, and broadcasts it over LoRa (SX1278) or a hardwired UART link.

### 2.2 Node B (The Central Gateway - Study Center)
- **Role:** The Agronomic Brain and System Coordinator.
- **Methodology (Asymmetric Multiprocessing):** Node B operates on an ESP32 utilizing a FreeRTOS environment to separate deterministic and non-deterministic workloads. 
  - **Core 0:** Manages Wi-Fi connections and REST API POSTs to the Supabase Cloud. This isolates unpredictable network latency from critical operations.
  - **Core 1:** Manages hardware ingest (LoRa/UART), localized SD Card logging, and the FSM Decision Engine.
- **Execution (The Agronomic FSM):** Node B's core logic (`vDecisionTask`) calculates the Water Deficit using the FAO-56 Penman-Monteith methodology. It evaluates local variables (Soil VWC, Rain, Temperature) against a configured Farm Profile (Crop Root Depth, Field Capacity, Wilting Point). 
- **Safety Gates:** Before commanding Node C, Node B must pass multiple safety gates:
  1. **Sensor Integrity:** Are the sensors reporting healthy values?
  2. **Rain Lock:** Is the field already saturated by local rainfall?
  3. **Battery Check:** Does Node C have enough battery to safely run the pump?
  4. **Daily Limits:** Would this irrigation exceed the maximum allowed daily volume?

### 2.3 Node C (The Actuator)
- **Role:** Physical Execution and Local Safety.
- **Methodology:** Node C receives commands via LoRa but does not blindly trust Node B. It acts as a safety enforcer, managing high-current relay outputs for the irrigation pump and solenoid valves.
- **Execution:** When commanded to irrigate a specific volume, Node C activates the pump and uses a hardware-interrupt flow meter (YF-S201) to measure the exact amount of water delivered. It simultaneously monitors an analog current sensor (ACS712) to detect pump stalls or dry-running. Once the target volume is hit, or a safety limit is breached, it shuts down and sends an exact feedback report back to Node B.

---

## 3. Communication Protocol

The system relies on a unified, transport-agnostic binary protocol shared across all nodes (`structs.h` and `lora_protocol.h`).

### 3.1 Data Encapsulation
Rather than parsing heavy text formats like JSON over the air, nodes transmit fixed-width binary structs wrapped in a `LoRaHeader`. The header dictates the packet type (e.g., `PACKET_SENSOR_DATA`, `PACKET_IRRIGATION_COMMAND`) and is protected by a 16-bit Modbus CRC to detect corrupted bytes over the radio.

### 3.2 Security (SRIJAN)
A custom AES-128-GCM cryptographic layer (`srijan_security.cpp`) ensures that commands sent over the air cannot be spoofed or replayed. The system utilizes synchronized time epochs to generate secure nonces, authenticating every packet before the payload is unpacked.

---

## 4. End-to-End System State Machine

The FSM traces the life-cycle of a single irrigation event across the three nodes:

1. **Ingest (Node A):** Node A polls sensors -> Broadcasts `MasterSensorRecord` -> Returns to Sleep.
2. **Evaluation (Node B):** Node B receives data via `vUartTask`/`vLoraTask` -> Pushes to `sensorQueue` -> `vDecisionTask` evaluates the deficit against safety gates.
3. **Dispatch (Node B -> C):** If irrigation is required, Node B constructs a `NodeCCommand` -> Transmits via LoRa -> Transitions to `STATE_WAIT_FEEDBACK`.
4. **Execution (Node C):** Node C receives the command -> Validates CRC/Security -> Enters `PUMP_RUNNING` state -> Stops pump when volume is reached -> Transmits `LoRaFeedbackPacket`.
5. **Reconciliation (Node B):** Node B receives feedback -> Updates persistent non-volatile memory (NVS) to track the daily volume -> Pushes the final `IrrigationDecision` and `NodeCFeedback` to the SD card and the Cloud.

---

## 5. Conclusion
AgroSmart's methodology prioritizes **deterministic execution and fail-safe safety**. By centralizing the heavy agronomic mathematics and cloud-syncing on Node B, Node A remains extremely power-efficient, and Node C can remain focused purely on the immediate electrical safety of the irrigation hardware. 
