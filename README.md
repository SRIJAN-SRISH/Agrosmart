# AgroSmart - Precision Agriculture System 🌱

A closed-loop, distributed precision-agriculture system for automated soil monitoring and irrigation, built around three networked embedded nodes communicating over LoRa. AgroSmart continuously samples soil and environmental conditions, computes exact irrigation needs using field-specific agronomic physics, and drives pump/valve hardware safely and automatically — logging every decision to SD card and the cloud.

Originally developed as a Smart India Hackathon (SIH) 2025 submission and continued as a department-level Third Year student project, supervised by three faculty members with a team of six students.

---

## 🏗️ System Overview

```
┌────────────────────┐        LoRa 433 MHz        ┌──────────────────────────┐        LoRa 433 MHz        ┌────────────────────┐
│   Node A           │ ─────────────────────────► │   Node B                 │ ─────────────────────────► │   Node C           │
│   Field Telemetry  │   MasterSensorRecord       │   Gateway & Decision     │   NodeCCommand             │   Physical Actuator│
│                    │ ◄────────────────────────  │   Engine                 │ ◄───────────────────────── │                    │
└────────────────────┘                            └──────────────────────────┘        NodeCFeedback       └────────────────────┘
      ESP32                                          ESP32 (dual-core, FreeRTOS)                                    ESP32
Soil + GPS sensing                                Agronomic physics engine                                    Pump/valve control
                                                      Cloud sync + SD logging                                Flow + current sensing
                                                              │
                                                              ▼
                                                     ┌──────────────────┐
                                                     │  NodeB Display   │
                                                     │  (Pi Pico,       |
                                                     |   ili9341)       │
                                                     └──────────────────┘
```

All inter-node communication uses packed binary C++ structs (`__attribute__((packed))`) wrapped in a common frame:

```
LoRaHeader (8 B: magic 0xAB, protocol version, sender ID, receiver ID, packet type)
  + payload struct (MasterSensorRecord / NodeCCommand / NodeCFeedback / NodeCHeartbeat)
  + CRC16 (2 B, Modbus polynomial)
```

Validation on receipt: size check → magic byte check → receiver-address check → CRC16 check. An `FLAG_EMERGENCY` bit can trigger an immediate `emergencyStop()` on Node C with a safe pressure-bleed sequence.

---

## 📡 Node A — Field Telemetry

Deployed in the crop root zone for continuous environmental sampling and spatial tracking.

- **7-in-1 soil parameter polling** over RS-485 Modbus RTU: Volumetric Water Content (VWC), soil temperature, Electrical Conductivity (EC), pH, Nitrogen, Phosphorus, Potassium.
- **Geospatial tracking** via NEO-M8N GPS module (NMEA parsing) — latitude, longitude, fix validity.
- **Diagnostic bitmask health flagging**: soil fault, GPS fault, comm fault (bits `0x01`, `0x02`, `0x04`).
- Packs data into a `MasterSensorRecord`, wraps it in a `LoRaSensorPacket` (magic byte `0xAB`, sender `0x0A`, receiver `0x02`), appends CRC16, and transmits.

**Hardware:** ESP32 · MAX485 TTL-to-RS485 converter · 7-in-1 RS-485 soil probe · NEO-M8N GPS · SX1278 LoRa transceiver (433 MHz, SF9, BW 125 kHz)

**Files:** `Agrosmart_NodeA.ino`, `config.h`, `pins.h`, `enums.h`, `structs.h`, `crc16.h`, `lora_comms.cpp/.h`, `lora_protocol.h`, `modbus_core.cpp/.h`, `leds.cpp/.h`

---

## 🧠 Node B — Master Gateway & Agronomic Decision Engine

Multi-threaded, FreeRTOS-driven gateway fusing Node A telemetry, local environmental sensors, and Node C status into irrigation decisions.

- **Six concurrent FreeRTOS tasks** communicating via thread-safe queues: `lora_task`, `sensor`/`environment_task`, `decision_task`, `sd_task`, `cloud_task`, `health_task` (plus `led_task`, `uart_task`, `mock_task` for testing/display).
- **19-state master state machine** governing the full irrigation decision lifecycle, from `STATE_BOOT` through validation, health checks, environmental correction, decision, safety checks, dose calculation, command dispatch, feedback wait, stabilization, remeasure, logging, cloud sync, down to `STATE_SAFE_IDLE`, `STATE_FAULT`, and `STATE_DEGRADED_MODE`.
- **Agronomic physics engine** (`decision_task.cpp`) computing VPD, Total Available Water (TAW), Readily Available Water (RAW), soil moisture deficit, and gross irrigation volume required (in liters), based on a `FarmProfile` (field capacity, wilting point, MAD, root depth, field area).
- **Multi-tiered safety lock array** (`DecisionReason`): rain lock, daily volume limit, low battery, sensor/LoRa fault, stabilization delay.
- **Cloud sync** (`cloud_task.cpp`) to a Supabase backend, plus **SD card logging** (`sd_task.cpp`) for offline-durable records.
- **Security layer** (`srijan_security.cpp/.h`) for request/data integrity.
- **Secondary display unit** (`NodeB_Display_Pi_Pico/`) — a Raspberry Pi Pico driving an ILI9341 TFT for live status readout.

**Hardware:** ESP32 (dual-core) · SX1278 LoRa transceiver (node address `0x02`) · BMP280 (pressure/temp/humidity) · DS3231 RTC · SEN0575 rain gauge · MicroSD module · 4G/WiFi modem

**Files:** `Agrosmart_nodeB.ino`, `config.h`, `pins.h`, `enums.h`, `structs.h`, `queues.h`, `tasks.h`, `lora_task.cpp`, `lora_protocol.h`, `decision_task.cpp`, `environment_task.cpp/.h`, `health_task.cpp/.h`, `cloud_task.cpp/.h`, `sd_task.cpp`, `led_task.cpp/.h`, `uart_task.cpp`, `uart_protocol.h`, `config_manager.cpp/.h`, `srijan_security.cpp/.h`, `mock_task.cpp/.h`, `logger.h`, `sync.h`, plus architecture docs (`architecture.md`, `AgroTerm_Architecture.md`, `SYSTEM_ARCHITECTURE_DEEP_DIVE.md`, `NODE_B_EXHAUSTIVE_GUIDE.md`, `nodeC_architecture.md`, `PROJECT_REPORT.md`) and `Cloud_Backend/srijan_supabase.md`.

> ⚠️ **Before publishing:** `config.h` must have real WiFi credentials and Supabase URL/API key replaced with placeholders — see setup notes below.

---

## ⚙️ Node C — Physical Actuator

Non-blocking `millis()`-based state machine (no RTOS) for deterministic hardware control of the irrigation pump and valve.

- **Water-hammer prevention sequencing**: valve opens 1500 ms before pump start (pre-charge); pump stops 2000 ms before valve closes (bleed).
- **Dual-threshold fault engine** via ACS712-30A current sensor: dry-run/blockage detection (<0.3 A), mechanical jam detection (>5.0 A), 500 ms startup inrush grace window, 3-consecutive-reading trip persistence.
- **Three-tier safety clamping** on runtime/volume, with absolute ceilings (3600 s / 3000 L).
- **Volumetric integration** via YF-S201 flow meter (interrupt-driven pulse counting).
- **Telemetry**: INA219 voltage monitor, 60-second `NodeCHeartbeat` packets.

**Hardware:** ESP32 · SX1278 LoRa transceiver (node address `0x0C`) · 2-channel opto-isolated relay board (active-LOW, JD-VCC jumper removed) · ACS712-30A current sensor · YF-S201 flow meter · INA219 voltage monitor · 1N4007 flyback diodes · LM2596 buck converter · resistor-divider logic-level shifting · 0.1 µF ceramic capacitor low-pass filter on the flow sensor line

**Files:** `AgroSmart_NodeC.ino`, `config.h`, `pins.h`, `enums.h`, `structs.h`, `lora_protocol.h`, `actuators.cpp/.h`, `sensors.cpp/.h`, `comms.cpp/.h`, `diagnostics.cpp/.h`

---

## 🔄 Operational Lifecycle (example walkthrough)

1. Node A samples soil at 14% VWC, gets a GPS fix, transmits a `MasterSensorRecord`.
2. Node B validates and queues the packet.
3. `decision_task` computes TAW/RAW/deficit, arriving at a 250.0 L gross dose requirement.
4. Safety gates checked (rain = 0.0 mm, daily volume under limit, battery = 12.4 V, health flag OK) — decision passes.
5. Node B dispatches a `NodeCCommand` (target 250.0 L, max runtime 3000 s, zone 1).
6. Node C clamps values and calls `startPumpCycle()`.
7. Hydraulic sequencing: valve opens → 1500 ms pre-charge delay → pump starts.
8. Live monitoring: current sampled every 100 ms (nominal 2.1 A), flow meter accumulates volume via GPIO 34 interrupts, watchdog reset each loop.
9. Cycle termination at 250.0 L: pump off → 2000 ms bleed delay → valve closes.
10. Node C sends `NodeCFeedback` (delivered volume, status) back to Node B, which updates its daily ledger, logs to SD, and queues the record for cloud sync.

---

## 🛠️ Tech Stack

- **Firmware** — C++ (Arduino framework), FreeRTOS (Node B), bare `millis()` state machine (Node C)
- **MCU** — ESP32 (all three nodes), Raspberry Pi Pico (Node B display, MicroPython)
- **RF** — SX1278 LoRa (433 MHz)
- **Sensing** — RS-485 Modbus RTU (soil), NEO-M8N GPS (NMEA), ACS712/INA219 (current/voltage), YF-S201 (flow), BMP280 (env), SEN0575 (rain)
- **Cloud** — Supabase (REST API)
- **Storage** — MicroSD (local durable logging)

---

## ⚙️ Setup Notes

Each node folder contains its own `config.h` with hardware pins, timing constants, and (for Node B) network/cloud credentials.

**Before building or publishing Node B**, replace the placeholders in `Agrosmart_nodeB/config.h`:

```cpp
#define WIFI_SSID "your_wifi_ssid_here"
#define WIFI_PASS "your_wifi_password_here"

#define SUPABASE_URL "your_supabase_url_here"
#define SUPABASE_ANON_KEY "your_supabase_anon_key_here"
```

Flash each node's `.ino` via the Arduino IDE / PlatformIO with the ESP32 board package selected. Node B's optional display runs separately on a Pi Pico via MicroPython (`NodeB_Display_Pi_Pico/main.py`).

---

## 📜 Project Status

- **Origin:** Smart India Hackathon (SIH) 2025 entry (did not qualify)
- **Continuation:** Department-level student project, Third Year classification
- **Team:** 6 students
- **Supervision:** 3 faculty supervisors

## 📜 License

Shared for portfolio and academic review. Not licensed for redistribution or commercial use.
