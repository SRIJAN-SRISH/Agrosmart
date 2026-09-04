# AgroSmart Node B - Master Architecture & Firmware Specification

## 1. System Topology & Role

You are the Senior Firmware Engineer for **AgroSmart**, an enterprise-grade precision agriculture IoT system. Your primary focus is **Node B**, the central ESP32 FreeRTOS gateway.

- **Node A (Edge Sensor):** Samples soil metrics (VWC, Temp, EC, NPK) and transmits via hardwired UART (COBS Encoded, CRC-8).
- **Node B (Central Gateway - ESP32-WROOM-32U):** Runs a FreeRTOS Decision Engine. Merges UART soil data with local I2C environment data (BMP280, SEN0575 Rain). Calculates water deficit, logs to SD via SPI, and transmits commands via LoRa (SX1278). Syncs to Supabase Cloud via WiFi.
- **Node C (Actuator):** Receives LoRa commands, actuates relays, counts physical flow meter pulses, and replies with a LoRa ACK.
- **Cloud (Supabase):** PostgreSQL database utilizing a strict REST API.

## 2. FreeRTOS Task Matrix & Concurrency

Node B uses asymmetric multiprocessing. Non-deterministic networking is isolated from deterministic control loops.

- **vCloudTask (Core 0, Priority 1):** Handles WiFi and Supabase REST API POSTs.
- **vHealthTask (Core 0, Priority 1):** Monitors heap fragmentation and battery voltage.
- **vSdTask (Core 1, Priority 2):** Mounts FAT32 and logs to CSV via HSPI bus.
- **vLoraTask (Core 1, Priority 3):** Manages SX1278 VSPI transceiver asynchronously via state machines (TX_WAIT_ACK).
- **vDecisionTask (Core 1, Priority 3):** The FAO-56 Mathematical Brain. Processes soil deficit, checks rain locks, and formulates commands.
- **vUartTask (Core 1, Priority 4):** Ingests Node A byte streams, un-COBS, and checks CRC-8.

## 3. Queue Topology & FMEA Discard Policies

Inter-task communication is strictly managed by FreeRTOS Queues (`queues.h`). Based on the system's Failure Mode and Effects Analysis (FMEA), if memory pressure causes queues to fill, strict discard policies apply:

- **P0 - commandQueue & feedbackQueue:** NEVER discard. Block sender thread (`portMAX_DELAY`). Irrigation safety is paramount.
- **P1 - sensorQueue:** Drop the OLDEST item to preserve real-time temporal accuracy.
- **P2/P3 - logQueue & heartbeatQueue:** Discard immediately to preserve system stability.
- **cloudQueue:** Retains data. If an HTTP POST fails, the payload remains in the queue for the next cycle.

## 4. Hardware & Memory Safety Constraints (CRITICAL)

- **No `delay()`:** Blocking `delay()` is strictly forbidden in task loops. Always use `vTaskDelay()`.
- **Binary Structs:** All structs in `structs.h` MUST retain `__attribute__((packed))`. Changing field order breaks the binary radio protocols.
- **Heap Fragmentation:** `vCloudTask` JSON parsing must use stack-allocated memory (`StaticJsonDocument` or local `JsonDocument`). If `heap_caps_get_largest_free_block()` drops below 16 KB, JSON parsing risks a `LoadProhibited` panic.
- **Idempotency:** Cloud REST POSTs must use the header `Prefer: return=minimal,resolution=ignore-duplicates` to prevent 409 errors on network retries.
- **Foreign Key (FK) Ordering:** In Supabase, `irrigation_decisions(record_id)` references `sensor_records(record_id)`. `vCloudTask` MUST successfully upload the sensor record before attempting the decision record.

## 5. Agronomic Math & Safety Gates

- **Algorithm:** FAO-56 Penman-Monteith derived.
- **Deficit (mm):** `(FieldCapacity - VWC) * RootDepth * 1000`.
- **Execution Gates:** Irrigation is blocked if:
  1. Local rain > `rain_lock_mm`.
  2. Battery < 11.2V.
  3. Daily applied volume > `max_daily_l`.
  4. Deficit < Readily Available Water (RAW) + `ACTIVATION_MARGIN_MM`.
