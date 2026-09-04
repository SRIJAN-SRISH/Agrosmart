# AgroSmart Node C — Actuator Architecture Specification

## 1. Role & Topology

Node C is the field actuator: an ESP32 that receives irrigation commands over LoRa
from Node B, drives a pump + solenoid valve, measures what it actually delivered
with a physical flow meter, and reports both completion and periodic health back to
Node B. It has no decision-making authority — every gate (rain lock, daily limit,
deficit math) already happened on Node B before a command is ever transmitted.
Node C's own intelligence is limited to **local safety enforcement**: it must never
trust Node B blindly and must never run longer/more than its own hard ceilings allow,
even if Node B is compromised, buggy, or sends a corrupt packet that somehow passes CRC.

**Deployment scope (current phase): 1 pump + 1 valve zone.** The relay hardware on
hand is a 2-channel isolated 5V module, which covers exactly pump + Zone 1. Zone 2
support stays defined in the protocol/struct layer (`NodeCCommand.zone_id` still
exists, `zoneToPin()` still has a case for it) but is not physically wired — adding a
second valve later is a wiring-plus-one-relay-channel change, not a protocol change.

## 2. Protocol Conformance — Non-Negotiable

Node C shares three files' worth of binary contract with Node B and must stay
byte-identical to them. Do not hand-roll local copies of these structs — copy them
verbatim from Node B's repo on every change:

- `structs.h` → `NodeCCommand`, `NodeCFeedback`, `NodeCHeartbeat`
- `lora_protocol.h` → `LoRaHeader`, `LoRaCommandPacket`, `LoRaFeedbackPacket`, `LoRaHeartbeatPacket`
- `enums.h` → `PacketType` (`PACKET_IRRIGATION_COMMAND=0x02`, `PACKET_IRRIGATION_COMPLETE=0x04`, `PACKET_HEARTBEAT=0x05`), `FLAG_EMERGENCY`

**Existing bug to fix before any new development:** the current
`nodeC_irrigation_controller.ino` sends a locally-defined `AckPayload`/`LoRaAckPacket`
tagged `PACKET_ACK=0x03`. Node B's `lora_task.cpp` switch statement has no case for
0x03 — it silently drops it as "unknown packet_type". Node C must instead send
`LoRaFeedbackPacket` (`PACKET_IRRIGATION_COMPLETE`) and `LoRaHeartbeatPacket`
(`PACKET_HEARTBEAT`), which Node B already parses into `feedbackQueue` (P0, never
dropped) and `heartbeatQueue` (P2) respectively.

**Existing bug to fix — LoRa sync word mismatch:** Node B's `config.h` sets
`LORA_SYNC_WORD 0x12`. The current Node C sketch sets `LoRa.setSyncWord(0xF3)`. Two
radios with different sync words do not see each other's packets at the LoRa layer at
all — this would look like total radio silence with no error message on either side.
Node C must use `0x12`, matching Node B exactly, along with the rest of the radio
config (SF9, BW125kHz, CR4/5, preamble 8, freq 433 MHz).

## 3. Electrical Protection — Non-Negotiable, Before First Power-On

Three heavy inductive loads (pump motor + 2 solenoid coils) are being switched by
relays right next to a 3.3V microcontroller sharing the same ground plane. This
section is not optional polish — skipping it risks silent ESP32 resets in the field
or a destroyed LoRa front-end on day one.

- **Flyback diodes.** Every inductive load being switched (currently: the pump
  motor and the one wired solenoid valve) needs a flyback diode (e.g. 1N4007) across
  its own terminals, cathode to the +12V side, installed directly at the load — not
  relying on whatever protection the relay module claims to have on its logic side.
  Placed at the pump/valve terminals rather than across the relay contacts, the
  flyback current recirculates in a tight local loop and never has to pass back
  through the ACS712 or the open relay contact — the sensor never sees the spike.
  When a relay opens, the collapsing magnetic field in the coil generates a reverse
  voltage spike; on a shared ground plane that spike can couple back into the 3.3V
  rail and brown out or reset the ESP32 mid-cycle. Many solenoid valves have a diode
  built into their connector housing already — verify with a multimeter (diode mode)
  rather than assuming either way.
- **Antenna — never skip.** The LoRa module (SX1278 on this build, matching Node B)
  must have its 433 MHz antenna attached before it is ever powered and transmitting.
  Transmitting into an unterminated port reflects RF energy back into the power
  amplifier and can permanently damage the chip. Treat "antenna connected" as a
  pre-power-on checklist item, the same tier as "relay board polarity confirmed".
- **Bulk capacitance / TVS on the 12V rail** as defense-in-depth alongside the
  flyback diodes (e.g. a bulk electrolytic cap near the relay board's 12V input, and
  a TVS diode if the supply is exposed to a solar charge controller with its own
  switching transients) — belt-and-suspenders against the same class of surge.

### 3.1 Actuation Safety Wiring Map (Relay + Pump + Valve)

Everything electrical between the 12V source and the pump/valve, laid out as
protection layers rather than a flat wire list — each layer catches a different
failure mode, and skipping any one of them leaves a real gap, not redundant
caution.

**Layer 1 — Overcurrent protection (missing from every earlier table; add this
first).** Flyback diodes protect against inductive spikes, not against a wiring
fault or a shorted winding pulling far more current than the wire can handle. Size
a fuse against actual combined load: pump nominal 3.5A + valve holding current
(~0.3-0.8A for a typical 12V irrigation solenoid) ≈ 4-4.5A continuous when both are
on together (valve stays open while pump runs). A **7.5A slow-blow automotive
blade fuse**, inline on the main 12V feed *before* it reaches either relay's `COM`,
gives headroom above normal running current (avoids nuisance trips on the pump's
brief start-up inrush) while still tripping well below what the wire gauge below
can sustain without heating.

```
12V Supply (+) → [7.5A slow-blow fuse] → [Physical E-stop switch, Layer 2] →
   → 12V Distribution Point → Relay 1 COM (pump) + Relay 2 COM (valve)
```

**Layer 2 — Physical kill switch, independent of firmware.** A simple inline
DPST/SPST switch or circuit breaker between the fuse and the 12V distribution
point, mounted somewhere accessible without opening the enclosure. This is not the
same thing as the LoRa emergency-stop command — that depends on the ESP32 still
being alive and its firmware still executing. If the ESP32 hangs, browns out, or
the firmware has a bug, this switch is the only thing that reliably cuts pump/valve
power. Treat it as mandatory for any unattended field deployment, not a nice-to-have.

**Layer 3 — Wire gauge (voltage-drop and heating margin).** At 3.5-4.5A on a
nominal 12V system, thin wire both wastes pump performance (voltage sag under load,
narrowing the pump's already-tight 9-14.4V operating window) and heats up under
fault conditions before the fuse necessarily reacts. For runs under ~3m (control
box to pump, typical bench/small-plot setup): **18AWG minimum**. For anything
longer — realistic for a field deployment — step up to **16AWG**, or **14AWG**
past ~5m, to keep voltage drop under roughly 3% of the 12V rail at full pump
current.

**Layer 4 — Relay switching (opto-isolated logic, as already specified in §3/§4):**

```
ESP32 3.3V  → Relay VCC          ESP32 GND → Relay GND
ESP32 GPIO16 → Relay1 IN (pump)  ESP32 GPIO17 → Relay2 IN (valve)
5V Rail     → Relay JD-VCC       (jumper cap physically removed — Layer 4a)
```

**Layer 4a — JD-VCC isolation is itself a safety layer, not just a wiring
preference.** With the jumper removed, a coil-side fault (shorted relay coil,
miswired 12V feed) cannot backfeed into the ESP32's 3.3V logic rail — the two power
domains only communicate through the opto-isolator's light path. Leaving the jumper
in place defeats this and is the single easiest mistake to make with this exact
relay module.

**Layer 5 — Switched high-current path, current sensor in series (pump only):**

```
12V Distribution → Relay1 NO → ACS712 IP+ → ACS712 IP− → Pump(+) → Pump(−) → GND
12V Distribution → Relay2 NO → Valve(+) → Valve(−) → GND
```

**Layer 6 — Flyback diodes at the load** (1N4007, cathode to `+`, across pump
terminals downstream of the ACS712, and across the valve terminals) — as detailed
in §3. Placement at the load, not the relay contact, is what keeps the flyback
current out of the ACS712's measurement path.

**Layer 7 — Grounding discipline.** All grounds — 12V return, the 5V rail's LM2596
return, ESP32 `GND`, relay logic `GND` — should tie back to a single star point
rather than daisy-chaining through the breadboard rail in series. This matters more
here than in a typical low-power project because switching 3.5A+ inductive loads
right next to an ADC that's trying to resolve ~0.013A steps: any ground-loop
current from the high-power side riding on the same return path as the ACS712/flow
meter signal returns shows up as noise on exactly the readings the stall-detection
logic depends on. Where practical, route the 12V pump/valve wiring as a physically
separate bundle from the ACS712 signal wire and flow meter signal wire, rather than
running them side-by-side in the same loom.

## 4. Hardware & Pin Map

Confirmed BOM, from nameplate/datasheet photos: 12V DC diaphragm pump (nameplate:
12V nominal, 9-14.4V actual operating range, **3.5A rated current**, 5.5L/min max
flow, **120 PSI / 7.0 Bar internal pressure-cutoff switch** — this pump auto-stops
itself on deadhead pressure independent of Node C, see note below), genuine YF-S201
flow sensor (1-30L/min working range, ≤1.75MPa, standard 450 pulses/liter K-factor —
`F(Hz) = 7.5 × Q(L/min)`), 12V DC solenoid valve ×1 (wired now; a 2nd is
protocol-ready but not physically present), ACS712-20A (analog current sensor, in
series with the pump — sized against this pump's nameplate 3.5A nominal / ~5-7A
estimated stall current, see §5), INA219
(I2C voltage/current sensor, deferred — see §5), 2-channel isolated 5V relay module
(JD-VCC jumper removed for opto-isolation; relay-side logic powered from ESP32 3.3V,
coil-side JD-VCC powered from the 5V rail). Two current sensors is not redundancy —
they do different jobs (see §5).

The current sketch has two undetected GPIO conflicts with the LoRa hardware SPI bus
(VSPI's SCK=18 and MOSI=23 are fixed by the ESP32 silicon, not by `LoRa.setPins()`).
`VALVE_ZONE_2` was assigned GPIO18 and `LED_PUMP` was assigned GPIO23 — both collide
with the SPI clock/data lines the LoRa radio depends on. Revised map, conflict-free,
with GPIO room reserved for the flow meter + both current/voltage sensors:

```
LoRa SX1278 (VSPI, hardware-fixed pins)
  SCK   → GPIO 18   (hardware VSPI clock — do not reuse)
  MISO  → GPIO 19   (hardware VSPI data in — do not reuse)
  MOSI  → GPIO 23   (hardware VSPI data out — do not reuse)
  SS    → GPIO  5
  RST   → GPIO 14
  DIO0  → GPIO  2

INA219 battery/system monitor (I2C, default Wire pins) — DEFERRED, not installed yet
  SDA   → GPIO 21   (reserved — do not repurpose even while unpopulated)
  SCL   → GPIO 22   (reserved)
  Addr  → 0x40 (default, A0/A1 pins grounded)
  Wired across the main 12V supply rail (battery/solar side), NOT the pump line, when
  installed. Until then, firmware sends a hardcoded NodeCHeartbeat.battery_voltage
  (e.g. 12.0f) so Node B's isNodeCBatteryOk() gate stays open for LoRa/pump testing.

ACS712-20A pump current sensor (analog, ADC1, via resistor divider)
  OUT → [10kΩ series] → GPIO 35 node → [15kΩ to GND]   (see divider math below)
  Wired in series with the pump's switched 12V+ line, between the relay's NO
  contact and the pump's own + terminal — this is what detects a stalled/dry-running
  pump DURING a run (see §5). GPIO35 is input-only, ADC1-capable (stays usable even
  if WiFi is ever added later; ADC2 does not, under active WiFi use).
  Divider: R1=10kΩ (sensor OUT → node), R2=15kΩ (node → GND).
    Vout = 5.0V × 15k/(10k+15k) = 3.00V at nominal 5V rail — ~300mV headroom below
    the ESP32's 3.3V ADC ceiling even if the rail drifts to 5.1-5.15V. ACS712's
    zero-current point (2.5V on a 5V module) shifts to ~1.50V post-divider — read
    this live at boot (zero-load baseline, see §6), don't assume the nominal value.

Flow meter (genuine YF-S201, 1-30L/min rated, K-factor 450 pulses/liter per
  F(Hz)=7.5×Q(L/min) — bucket-calibrate near this pump's actual ~2-5.5L/min
  operating point rather than trusting the nominal K-factor outright, since it
  sits in the bottom ~18% of the sensor's rated range where linearity is weakest)
  PULSE → [10kΩ series] → GPIO 34 node → [15kΩ to GND]   (same divider as ACS712)
  Same 10k/15k divider brings the 5V pulse down to 3.00V — safe swing for a clean
  digital HIGH/LOW read. GPIO34 is input-only, ADC1-capable, interrupt-capable.
  Add a 0.1µF ceramic capacitor from the GPIO34 node to GND, right at the sensor's
  connector. Hall-effect flow sensors chatter under water turbulence — the internal
  magnet can trigger 3-4 rapid false edges per actual rotation. This RC filter
  absorbs that chatter in hardware before it reaches the GPIO; software debouncing
  alone (a minimum-interval check in the ISR) is not reliable enough on its own
  against this failure mode and should be kept as a second layer, not the only one.

Pump / valve outputs (2-channel isolated relay board, active-LOW, JD-VCC jumper
  removed — relay logic side from ESP32 3.3V, coil side JD-VCC from 5V rail)
  PUMP_RELAY_PIN  → GPIO 16   (Relay 1 IN — main pump motor)
  VALVE_ZONE_1    → GPIO 17   (Relay 2 IN — the only valve currently wired)
  VALVE_ZONE_2    → GPIO 27   (reserved in firmware/protocol; no 3rd relay channel
                                installed yet — see §1 deployment scope)

Status LEDs
  LED_RX          → GPIO 26   (moved off GPIO22 — now reserved for I2C SCL)
  LED_PUMP        → GPIO 25   (moved off GPIO23 — was colliding with LoRa MOSI)
```

## 5. Data Model Additions

`NodeCFeedback` (already defined in Node B's `structs.h`, unchanged) carries
`delivered_volume_l` and `flow_rate_lpm` — with a physical flow meter, both become
**measured** values instead of time-based estimates:

```
delivered_volume_l = pulse_count / PULSES_PER_LITER      // calibrated constant
flow_rate_lpm       = (pulses_in_last_window / PULSES_PER_LITER) / (window_sec / 60.0)
```

`PULSES_PER_LITER` = 450 for this confirmed genuine YF-S201 (datasheet K-factor,
`F(Hz) = 7.5 × Q(L/min)`), but still needs a bucket-and-stopwatch calibration pass
at this pump's actual ~2-5.5L/min operating point before trusting it for
daily-volume-limit enforcement on Node B — see §4 flow meter note on why the
datasheet constant alone isn't sufficient at this end of the sensor's range.

**Two current/voltage sensors, two distinct jobs — do not conflate them:**

- **INA219** (I2C, on the battery/supply rail) → `NodeCHeartbeat.battery_voltage`
  via `getBusVoltage_V()`, sampled once per heartbeat interval (~60s). This is
  system-level telemetry: "is the battery healthy enough to irrigate" — exactly
  what Node B's `isNodeCBatteryOk()` gate needs. Not fast enough, and not wired to
  the right rail, to catch a stalled pump mid-run.
- **ACS712** (analog, in series with the pump motor) → sampled every ~100ms *during
  an active pump cycle only*. This is what actually detects a stall/dry-run/seized
  motor: pump relay commanded ON but ACS712 reads near-zero current for >3
  consecutive samples means the pump itself isn't drawing load, regardless of what
  the relay output pin says. Because it's a plain `analogRead()`, it costs nothing
  in I2C bus time and won't contend with the INA219 or introduce timing jitter into
  the pump-cycle tick loop.

**This specific pump has its own internal 120 PSI pressure-cutoff switch**
(nameplate spec) — it stops drawing current on its own if downstream pressure
deadheads, independent of anything Node C does. That means an ACS712 "near-baseline
current while relay is ON" reading can be triggered by two different root causes:
(a) an actual pump fault (seized/dry-run), or (b) the valve failing to open /
downstream blockage causing the pump to hit its own cutoff almost immediately. Node
C cannot electrically distinguish these two cases from current alone, and doesn't
need to — both are genuine faults and both should produce the same
`IRRIGATION_FAILED` feedback so Node B logs it and a human investigates. Don't spend
effort trying to disambiguate them in firmware.

## 6. Runtime Workflow

**Non-blocking is a hard requirement, not a suggestion.** The current sketch's
`executePumpCycle()` uses a `while() { ...; delay(100); }` loop that, despite
polling LoRa internally, still blocks everything else in `loop()` for the full run
duration — heartbeat timing drifts, and any logic added later has no chance to run
during a pump cycle. The rewrite must structure this as an explicit
`void managePumpCycle()` function, called unconditionally on every `loop()`
iteration, that advances a state machine (`IDLE` → `VALVE_OPENING` → `PUMP_RUNNING`
→ `VALVE_CLOSING` → `IDLE`) purely by comparing `millis()` against stored
timestamps — no `delay()` anywhere inside it. `loop()` itself becomes: service LoRa
RX, service heartbeat timer, call `managePumpCycle()`, repeat. This is what makes an
emergency-stop command received mid-run actually take effect immediately instead of
waiting for the current polling loop to unblock.

```
BOOT
 ├─ GPIO init — all relay outputs OFF (write inactive level BEFORE pinMode(OUTPUT))
 ├─ I2C init — Wire.begin(21,22), INA219.begin(), verify ACK
 ├─ ACS712 — analogReadResolution(12), record a zero-load baseline reading
 │  (pump confirmed OFF at boot) to calibrate the module's actual zero-current
 │  offset rather than trusting the datasheet's nominal VCC/2
 ├─ Flow meter — attachInterrupt(FLOW_PULSE_PIN, isr, FALLING), reset pulse counter
 ├─ LoRa init — matching Node B exactly (sync word 0x12!) — halt on failure
 └─ Arm heartbeat timer (non-blocking, millis()-based, e.g. every 60s)

LOOP (single-core, non-blocking — no delay() in the hot path)
 ├─ Heartbeat due? → read INA219 → build NodeCHeartbeat → send LoRaHeartbeatPacket
 ├─ LoRa.parsePacket() → bytes available?
 │    ├─ size/magic/sender/receiver/type/CRC16 validation chain (unchanged from
 │    │  current sketch — this part is already correct)
 │    ├─ FLAG_EMERGENCY set? → immediate relayOff(pump), relayOff(both valves),
 │    │  skip normal command handling, still send a feedback packet with
 │    │  status_flag = IRRIGATION_ABORTED
 │    └─ Normal command → executePumpCycle()
 └─ (state-machine tick if a pump cycle is in progress — see §6)

managePumpCycle() STATE MACHINE (ticked every loop() iteration, zero delay() calls)
 ├─ IDLE → new command accepted:
 │    ├─ Validate zone_id → reject (feedback status=IRRIGATION_FAILED) if out of range
 │    ├─ Reset pulse counter to 0
 │    ├─ Open valve, record valveOpenMs → state = VALVE_OPENING
 ├─ VALVE_OPENING → once millis() - valveOpenMs >= 1500ms (solenoid pre-charge —
 │    500ms is too short for a 110 PSI diaphragm pump; too fast a pump-on before the
 │    valve is fully seated risks water hammer or a blown fitting):
 │    start pump relay, record pumpStartMs → state = PUMP_RUNNING
 ├─ PUMP_RUNNING → checked every tick:
 │    ├─ stop if elapsed >= min(volume-derived runtime, cmd.max_runtime_sec,
 │    │  ABSOLUTE_MAX_RUNTIME_SEC)          [existing 3-tier clamp — keep as-is]
 │    ├─ stop early if delivered_volume_l (from live pulse count) >= target volume
 │    │  — the flow meter makes this possible; the old time-only version couldn't
 │    │  actually detect "target reached", it could only detect "time's up"
 │    ├─ stall detection: pump relay ON but ACS712 current ~baseline for >3
 │    │  consecutive 100ms samples → fault stop (dry-run/seized/blown-fuse)
 │    └─ emergency-stop command received (checked in the same loop() iteration,
 │       independent of this state machine) → immediate transition to VALVE_CLOSING
 │    On any stop condition: pump relay OFF, record pumpStopMs → state = VALVE_CLOSING
 └─ VALVE_CLOSING → once millis() - pumpStopMs >= 2000ms (let pressure bleed off
      before de-energizing the solenoid): close valve → state = IDLE → compute final
      delivered_volume_l/flow_rate_lpm from pulse count → send LoRaFeedbackPacket
      (PACKET_IRRIGATION_COMPLETE)
```

## 7. Status Flags on Feedback

Reuse `IrrigationStatus` from `enums.h` for `NodeCFeedback.status_flag` rather than
inventing new codes — Node B's SD/cloud logging already expects these values:

- `IRRIGATION_COMPLETED` — ran to target volume or clean time-based stop
- `IRRIGATION_ABORTED` — emergency stop received mid-run
- `IRRIGATION_FAILED` — invalid zone, pump stall (ACS712 current-sense fault), or
  any rejected command

## 8. Open Items Before Implementation

**Resolved this pass, from the actual pump spec sheet (45W/12V, 0.75MPa, 4L/min max):**
- ACS712 rating → **20A variant.** Nominal draw `45W/12V = 3.75A`, stall/inrush
  estimated 6-8A. A 5A module would saturate at exactly the moment stall detection
  matters; 20A covers both nominal and stall with headroom while keeping useful
  resolution (~0.013A/ADC-step through the 10k/15k divider).
- Flow meter divider → **10kΩ/15kΩ**, giving 3.00V nominal (see §4) — more headroom
  than a 10k/20k split (3.33V, too close to the ADC's accurate ceiling) and lower
  source impedance than either, which matters for ADC sample accuracy.
- Relay channel count → **2-channel board matches current 1 pump + 1 valve scope**
  exactly (§1). No 3rd channel needed until Zone 2 is added.

**Still open:**
- Confirm the flow meter model/K-factor and calibrate it specifically near this
  pump's actual ~2-4L/min operating point (§4) — not the sensor's rated max.
- Confirm relay board is active-LOW (matches current sketch's `ACTIVE_LOW true`) —
  worth a multimeter check before first power-on, wrong polarity means the valve is
  open by default at boot.
- Decide heartbeat interval (60s suggested above) against Node B's
  `NODE_C_TIMEOUT_MS` (5 min in `health_task.cpp`) — needs at least 2-3 missed
  heartbeats of margin before Node B declares Node C dead and closes the battery gate.
- When the INA219 is later added: confirm it lands on the reserved GPIO21/22 I2C
  pins without conflicting with anything added to this build in the meantime.

**Pre-power-on checklist (physical, not code):**
- [ ] 433 MHz antenna attached to the LoRa module — never power it up transmitting without one (§3)
- [ ] Flyback diode present (built-in or added) across the pump motor and the solenoid valve (§3)
- [ ] Relay board `JD-VCC` jumper physically removed (opto-isolation) — logic side to 3.3V, JD-VCC to 5V rail
- [ ] Relay board polarity/active-LOW confirmed with a multimeter
- [ ] ACS712-20A + 10k/15k divider verified reading ~1.50V at true zero pump current
- [ ] 7.5A slow-blow fuse installed inline on the main 12V feed, before the relay `COM` rails (§3.1)
- [ ] Physical kill switch installed and reachable without opening the enclosure (§3.1) — verify it actually cuts pump/valve power with the ESP32 still running, so you know it works independent of firmware state
- [ ] Wire gauge matches actual cable run length (§3.1) — 18AWG under ~3m, 16AWG/14AWG for longer runs
