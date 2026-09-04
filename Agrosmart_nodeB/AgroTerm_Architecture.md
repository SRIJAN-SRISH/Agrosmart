# AgroTerm — AgroSmart Engineering Console Architecture

## 1. Role & Scope

AgroTerm is the host-side engineering terminal for the AgroSmart system —
starting with Node C, extending later to Node B and eventually the full
three-node system. It replaces raw use of the Arduino Serial Monitor with a
structured, real-protocol console for bench testing, calibration, diagnostics,
and (later) field engineering access.

**Phase 1 (this document's primary scope): Node C terminal.**
**Phase 2 (future): extend the same transport pattern to Node B.**
**Phase 3 (future): a unified AgroSmart console addressing any node.**

This is infrastructure, not a bench script — treat it with the same rigor as
the firmware it talks to.

## 2. The Core Architectural Principle — Non-Negotiable

**AgroTerm speaks the same protocol the nodes already speak to each other.
It does not invent a second one.**

`structs.h` (`NodeCCommand`, `NodeCFeedback`, `NodeCHeartbeat`) and
`lora_protocol.h` (`LoRaHeader`, CRC16 framing, packet types) are Node C's
real, working, byte-identical wire contract with Node B. AgroTerm treats
Serial/USB as a **second transport carrying the same structs**, not a reason
to design a new one.

This is not a style preference — it's the same lesson `pilink.py` already
proved on Node B's side (the Pi Pico parses the exact same packed
`SystemHealthState` struct over UART that could equally travel another way),
and the same lesson `BENCH_NO_LORA` proved on Node C's side (bench testing is
only trustworthy when it exercises the *real* `startPumpCycle()` code path,
not a hand-rolled stand-in).

**Rejected alternative:** an ad-hoc CSV/text protocol (e.g.
`$TELE,STATE,VOLTAGE,...*`) was proposed and reviewed for this project. It was
rejected because:
- It requires new firmware serialization code with zero purpose in the field.
- It cannot be validated against the real dispatch/CRC logic — a terminal
  built against it would be testing an imitation of Node C, not Node C.
- It is a second protocol to keep in sync with `structs.h` forever, by hand,
  with no compiler to catch drift.
- It does not generalize to Node B or a unified console — every future node
  would need its own bespoke text format instead of reusing one binary
  contract across transports.

## 3. Node C Firmware Changes Required

Node C's `comms.cpp` currently has LoRa-specific transport code and
packet-handling logic fused together. This needs to split into two layers:

```
┌─────────────────────────────────────────────────────────┐
│  Packet Dispatcher (transport-agnostic)                 │
│  size → magic → sender → receiver → type → CRC16 checks │
│  routes PACKET_IRRIGATION_COMMAND -> startPumpCycle()    │
│  routes FLAG_EMERGENCY -> emergencyStop()                │
│  takes: raw byte buffer + a "send reply bytes" callback  │
└───────────────────────▲───────────────────────────────────┘
            ┌────────────┴────────────┐
            │                         │
   ┌────────┴────────┐      ┌─────────┴─────────┐
   │  LoRaTransport   │      │  SerialTransport   │
   │  (existing)      │      │  (new)             │
   │  LoRa.beginPacket│      │  Serial.write /     │
   │  /parsePacket    │      │  Serial.available   │
   └──────────────────┘      └────────────────────┘
```

- **Packet Dispatcher** — extracted from the current `handleIncomingPacket()`
  body in `comms.cpp`. Everything from magic-byte check through
  `startPumpCycle()`/`emergencyStop()` dispatch is identical regardless of
  which transport delivered the bytes — this code does not change, it just
  moves to a transport-agnostic function.
- **LoRaTransport** — the existing `LoRa.beginPacket()`/`parsePacket()` code,
  now just responsible for getting bytes in and out over the radio.
- **SerialTransport** (new) — reads/writes the identical framed packets
  (`LoRaHeader` + payload struct + CRC16) over `Serial` instead of the radio.
  Reuses the exact same `crc16Modbus()` and struct layouts already in
  `lora_protocol.h`/`structs.h` — no new framing format to design.
- **`BENCH_NO_LORA` is retired, not extended.** Once `SerialTransport`
  exists, sending a real framed `NodeCCommand` over USB *is* the bench test —
  there's no longer a need for the `.ino`'s special-case keystroke harness
  that hand-builds a `NodeCCommand` in place of a real Node B.
- Both transports can be active simultaneously if useful (e.g. Serial for
  bench/engineering access, LoRa for production Node B traffic) — the
  dispatcher doesn't care which one delivered a given packet.

## 4. Host-Side AgroTerm Architecture

```
┌──────────────────────────────────────────────┐
│  UI Layer (TUI or GUI — stack TBD, §6)        │
├──────────────────────────────────────────────┤
│  Session/State Layer                          │
│  - current NodeCHeartbeat/NodeCFeedback       │
│  - command history, connection state          │
├──────────────────────────────────────────────┤
│  Protocol Layer                               │
│  - struct pack/unpack (mirrors structs.h      │
│    field-for-field, matching padding rules —  │
│    same discipline as pilink.py's _HEALTH_FMT)│
│  - CRC16 (identical algorithm to Node C's)    │
├──────────────────────────────────────────────┤
│  Persistence Layer                            │
│  - append-only log (CSV or SQLite — team call)│
│  - every parsed packet, timestamped           │
├──────────────────────────────────────────────┤
│  Transport Layer                              │
│  - serial port I/O, VID/PID auto-discovery    │
│  - CP210x / CH340 USB-UART bridge detection    │
└──────────────────────────────────────────────┘
```

Each layer only talks to the one below it. The UI layer never touches raw
bytes; the Transport layer never knows what a `NodeCCommand` is.

## 5. Functional Requirements

### 5.1 Telemetry Display (read path)
Parses inbound `LoRaFeedbackPacket`/`LoRaHeartbeatPacket` frames identically
to how Node B would. Displays: current state, battery voltage, ACS712
current, flow rate, delivered volume, fault status. No new fields — this is
exactly what `NodeCFeedback`/`NodeCHeartbeat` already carry.

### 5.2 Command Dispatch (write path)
Builds a real `LoRaCommandPacket` wrapping a `NodeCCommand` — zone, target
volume, max runtime — computes the CRC16, and sends it framed over
`SerialTransport`. Node C cannot distinguish this from a command that arrived
over LoRa from Node B, by design.

### 5.3 Emergency Stop
Sends a `NodeCCommand` with `header.flags |= FLAG_EMERGENCY` set — the
existing, already-implemented emergency path. No new byte code (`0xFF` or
otherwise) needed; the protocol already has this covered.

### 5.4 Logging / Forensics
Every parsed packet, timestamped, appended to a persistent log. Purpose:
if a fault occurs in the field, the telemetry leading up to it is on disk,
not just scrolled off a terminal.

### 5.5 Automated Stress Test
Scripted loop: send N `NodeCCommand`s in sequence with realistic delays
between them, log every resulting `NodeCFeedback`, summarize pass/fail and
timing statistics at the end. This is the fastest way to catch intermittent
hardware issues (relay contact wear, loose wiring) — build this early, it
does not depend on anything else in this document being finished first.

### 5.6 Calibration Wizard — Blocked on a Firmware Prerequisite
**Not buildable yet.** `DIVIDER_SCALE`, `PULSES_PER_LITER`, and the fault
thresholds are currently compile-time `#define`s in Node C's `config.h` —
there is nothing on the device side for a wizard to write to. This feature
requires Node C to first grow:
- A new packet type (or repurposed existing one) for "set calibration
  parameter X to value Y."
- NVS-backed storage for those parameters, read at boot in place of the
  `#define` defaults.

Sequence this after §3's transport split, not before — building a wizard UI
against a device that can't yet accept calibration writes is wasted work.

## 6. Open Decisions for the Team

- **Language/framework.** Python (`pyserial` + `Textual`) is fast to build
  and has strong serial/TUI ecosystem support. C++ (`FTXUI` or similar)
  matches the firmware's own language and avoids a Python runtime dependency
  for whoever runs the console. Both are viable — pick based on team
  familiarity and whether this ever needs to ship to a non-developer's
  machine.
- **Persistence format.** SQLite (queryable, handles concurrent
  write-while-read cleanly) vs. flat CSV (trivially inspectable, no
  dependency, easy to hand off to someone doing analysis in a spreadsheet).
- **Serial framing details.** Reusing `LoRaHeader`'s CRC16 framing exactly
  as proposed in §3 is the recommendation — but confirm there's no reason to
  diverge (e.g. if Serial ever needs a framing byte the radio doesn't, for
  binary-transparency reasons over USB-CDC).
- **Single-node vs multi-node from day one.** §1 phases this as Node C
  first, but if the team wants the transport abstraction (§3's dispatcher
  split) to be written generically enough to add Node B as a second target
  immediately, that's worth deciding before writing the dispatcher's
  interface, not after.

## 7. What NOT to Build Yet

- The ASCII waveform/live-plotting view — genuinely nice, zero dependency
  for anything else, safe to defer to a later pass once the core read/write
  path is proven.
- The calibration wizard (§5.6) — blocked on firmware work that doesn't
  exist yet.
- Multi-node addressing / a unified console — Phase 3. Don't design for it
  prematurely; the Phase 1 transport split (§3) is deliberately structured
  so it *can* generalize later without being over-built now.
