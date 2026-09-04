#include "actuators.h"
#include "pins.h"
#include "config.h"
#include "enums.h"
#include "sensors.h"
#include <string.h>

// ============================================================
// actuators.cpp — graduated from the bench-tested sketch. Same
// sequencing, same dual-threshold fault detection, same non-blocking
// state machine shape — now driven by a real NodeCCommand instead of
// a Serial 'r' keypress, and producing a real NodeCFeedback instead of
// a Serial printout.
// ============================================================

enum CycleState { IDLE, VALVE_OPENING, PUMP_RUNNING, VALVE_CLOSING };

static CycleState state = IDLE;
static uint32_t stateEnteredMs = 0;
static uint32_t pumpStartedMs  = 0;

// ── Current in-flight command (copied at startPumpCycle time) ──────────
static char     activeCommandId[24];
static float    targetVolumeL   = 0.0f;
static uint32_t effectiveRuntimeMs = 0;

// ── Fault/result tracking for the in-flight cycle ───────────────────────
static bool emergencyRequested = false;
static bool stallFault         = false;
static uint8_t finalStatusFlag = IRRIGATION_COMPLETED;

// ── Flow meter — ISR increments, everything else only ever reads ───────
static volatile uint32_t pulseCount = 0;
static void IRAM_ATTR onFlowPulse() { pulseCount++; }

// ── ACS712 sampling state ───────────────────────────────────────────────
static float    zeroVoltsSensorDomain = 0.0f;
static uint32_t lastStallSampleMs = 0;
static uint8_t  stallSampleStreak = 0;
static float    lastCurrentA = 0.0f;

// ── Completed-cycle feedback mailbox (single slot — comms.cpp drains it) ─
static bool          feedbackReady = false;
static NodeCFeedback pendingFeedback;

// ── Sticky fault flag — persists across the feedback mailbox handoff,
//    only cleared when the NEXT cycle actually starts ─────────────────
static bool lastCycleFaultFlag = false;

// ── Relay helpers — active-LOW ──────────────────────────────────────────
static void pumpOn()   { digitalWrite(PUMP_RELAY_PIN,     LOW);  Serial.println("[PUMP]  ON"); }
static void pumpOff()  { digitalWrite(PUMP_RELAY_PIN,     HIGH); Serial.println("[PUMP]  OFF"); }
static void valveOn()  { digitalWrite(VALVE_ZONE_1_PIN,   LOW);  Serial.println("[VALVE] OPEN"); }
static void valveOff() { digitalWrite(VALVE_ZONE_1_PIN,   HIGH); Serial.println("[VALVE] CLOSED"); }

static void setState(CycleState s) {
    state = s;
    stateEnteredMs = millis();
}

static float litersFromPulses(uint32_t pulses) {
    return (float)pulses / (float)PULSES_PER_LITER;
}

// ── Direct hard-stop, bypassing the state machine entirely ─────────────
// Used only by VALVE_OPENING's absolute-timeout guard in tickActuators().
// Deliberately NOT applied to VALVE_CLOSING too, despite the surface
// symmetry: VALVE_CLOSING's existing single check (elapsed >=
// PRESSURE_BLEED_MS -> close valve, publish, IDLE) already performs the
// correct terminal action no matter how delayed tickActuators() was
// re-entered — closing the valve is always safe, sooner or later, and
// there's no "distrust this and abort instead" policy that makes sense
// for it. VALVE_OPENING is different: an excessively stale pre-charge
// decision is a real reason to refuse to start the pump rather than
// blindly proceed on a decision that's been sitting for 2x as long as
// intended (during which an emergency stop could have gone unprocessed).
// This forces both relays off immediately and resets straight to IDLE.
static void forceHardStopToIdle(const char *reason) {
    Serial.printf("[SAFETY RESET] %s — forcing both relays off, state -> IDLE.\n", reason);
    pumpOff();
    valveOff();
    finalStatusFlag = IRRIGATION_FAILED;
    publishFeedback(IRRIGATION_FAILED);
    state = IDLE;
    stateEnteredMs = millis();
}

// ── ACS712 read — calibrated mV -> post-divider volts -> sensor-domain volts ──
// analogReadMilliVolts() uses the ESP32's built-in ADC calibration curve
// (characterized per-chip by the core, not a fixed linear ADC_VREF/ADC_MAX_COUNTS
// assumption) — a real accuracy upgrade over a raw analogRead() count.
// Still oversampled (8x, near-simultaneous reads) to fight per-conversion
// noise. Deliberately NOT a recursive/cross-tick filter (e.g. an EMA):
// this reading feeds jam/dry-run fault detection, where the whole point
// is reacting fast to a genuine step-change in current — a filter that
// blends in reading history would delay exactly the transient (a locked
// rotor) this function exists to catch quickly.
static float readAcsVoltsSensorDomain() {
    uint32_t sumMv = 0;
    const int N = 8;
    for (int i = 0; i < N; i++) sumMv += analogReadMilliVolts(ACS712_PIN);
    float voltsPostDivider = (sumMv / (float)N) / 1000.0f;
    return voltsPostDivider / DIVIDER_SCALE;
}

static float readCurrentA() {
    float v = readAcsVoltsSensorDomain();
    return (v - zeroVoltsSensorDomain) / (ACS712_MV_PER_A / 1000.0f);
}

// ── Publish a completed cycle into the feedback mailbox ─────────────────
static void publishFeedback(uint8_t statusFlag) {
    uint32_t finalPulses;
    noInterrupts();
    finalPulses = pulseCount;
    interrupts();

    float totalLiters = litersFromPulses(finalPulses);
    float runSec = (pumpStartedMs > 0)
                    ? (stateEnteredMs - pumpStartedMs) / 1000.0f
                    : 0.0f;
    float avgLpm = (runSec > 0) ? (totalLiters / (runSec / 60.0f)) : 0.0f;

    memset(&pendingFeedback, 0, sizeof(pendingFeedback));
    strncpy(pendingFeedback.command_id, activeCommandId, sizeof(pendingFeedback.command_id) - 1);
    pendingFeedback.delivered_volume_l = totalLiters;
    pendingFeedback.flow_rate_lpm      = avgLpm;
    pendingFeedback.battery_voltage    = readBatteryVoltage();   // sensors.cpp — INA219
    pendingFeedback.runtime_sec        = (uint16_t)runSec;
    pendingFeedback.status_flag        = statusFlag;
    feedbackReady = true;
    lastCycleFaultFlag = (statusFlag == IRRIGATION_FAILED);

    Serial.println("=== CYCLE COMPLETE ===");
    Serial.printf("  cmd=%s  status=0x%02X  volume=%.3fL  runtime=%us  avgLpm=%.2f\n",
                  activeCommandId, statusFlag, totalLiters,
                  pendingFeedback.runtime_sec, avgLpm);
}

// ============================================================
// Public API
// ============================================================

void initActuators() {
    // Force both relays inactive BEFORE pinMode(OUTPUT) — and again after,
    // some cores glitch on the pinMode() call itself. This is what
    // prevents a boot-time floating-GPIO window from energizing anything.
    digitalWrite(PUMP_RELAY_PIN,   HIGH);
    digitalWrite(VALVE_ZONE_1_PIN, HIGH);
    pinMode(PUMP_RELAY_PIN,   OUTPUT);
    pinMode(VALVE_ZONE_1_PIN, OUTPUT);
    digitalWrite(PUMP_RELAY_PIN,   HIGH);
    digitalWrite(VALVE_ZONE_1_PIN, HIGH);

    pinMode(FLOW_PULSE_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(FLOW_PULSE_PIN), onFlowPulse, FALLING);

    pinMode(ACS712_PIN, INPUT);
    analogReadResolution(12);

    // ── Zero-current baseline — pump confirmed OFF at this point ────────
    Serial.println("[ACTUATORS] Calibrating ACS712 zero-current baseline...");
    delay(200);   // let the sensor settle after boot before sampling
    float sum = 0.0f;
    for (int i = 0; i < ZERO_CAL_SAMPLES; i++) {
        sum += readAcsVoltsSensorDomain();
        delay(5);
    }
    zeroVoltsSensorDomain = sum / ZERO_CAL_SAMPLES;
    Serial.printf("[ACTUATORS] Zero-current baseline: %.3fV (expected ~1.50V nominal — "
                  "if far off, check DIVIDER_SCALE and wiring before trusting readings)\n",
                  zeroVoltsSensorDomain);
    // This baseline calibration is also what keeps the ESP32's known ADC
    // non-linearity (worst near 0V and above ~3.1V) out of the picture:
    // by centering on the sensor's real zero-current point (~1.5V) rather
    // than trusting a hardcoded VCC/2 assumption, every fault-detection
    // read stays in the middle of the ADC's range where it's most linear.

    Serial.println("[ACTUATORS] Ready. Both relays confirmed OFF.");
}

bool isActuatorIdle() {
    return state == IDLE;
}

bool lastCycleFaulted() {
    return lastCycleFaultFlag;
}

bool startPumpCycle(const NodeCCommand &cmd) {
    if (state != IDLE) {
        Serial.println("[ACTUATORS] REJECTED — cycle already in progress.");
        return false;
    }

    // Only zone 1 is physically wired (see pins.h — VALVE_ZONE_2 is
    // protocol-reserved but has no relay channel installed).
    if (cmd.zone_id != 1) {
        Serial.printf("[ACTUATORS] REJECTED — zone_id=%u not physically wired.\n", cmd.zone_id);
        return false;
    }

    strncpy(activeCommandId, cmd.command_id, sizeof(activeCommandId) - 1);
    activeCommandId[sizeof(activeCommandId) - 1] = '\0';

    // ── Three-tier runtime clamp (never trust Node B's numbers alone) ──
    targetVolumeL = min(cmd.target_volume_l, ABSOLUTE_MAX_VOLUME_L);
    if (targetVolumeL != cmd.target_volume_l) {
        Serial.printf("[ACTUATORS] WARN — volume clamped: %.2fL -> %.2fL (absolute max)\n",
                      cmd.target_volume_l, targetVolumeL);
    }

    uint32_t volumeRuntimeSec = (uint32_t)((targetVolumeL / PUMP_FLOW_RATE_ESTIMATE_LPM) * 60.0f);
    uint32_t runtimeSec = volumeRuntimeSec;
    runtimeSec = min(runtimeSec, (uint32_t)cmd.max_runtime_sec);
    runtimeSec = min(runtimeSec, (uint32_t)ABSOLUTE_MAX_RUNTIME_SEC);
    effectiveRuntimeMs = runtimeSec * 1000UL;

    Serial.printf("[ACTUATORS] Starting cycle: cmd=%s target=%.2fL runtime_ceiling=%lus\n",
                  activeCommandId, targetVolumeL, (unsigned long)runtimeSec);

    noInterrupts();
    pulseCount = 0;
    interrupts();
    stallFault = false;
    stallSampleStreak = 0;
    emergencyRequested = false;
    finalStatusFlag = IRRIGATION_COMPLETED;
    pumpStartedMs = 0;
    lastCycleFaultFlag = false;   // this new attempt gets a clean slate

    valveOn();
    setState(VALVE_OPENING);
    return true;
}

void emergencyStop() {
    if (state == IDLE) return;   // nothing running — no-op

    Serial.println("[ACTUATORS] EMERGENCY STOP requested.");
    emergencyRequested = true;

    if (state == PUMP_RUNNING || state == VALVE_OPENING) {
        pumpOff();
        finalStatusFlag = IRRIGATION_ABORTED;
        setState(VALVE_CLOSING);   // still bleed pressure before closing —
                                    // an instant valve slam under a
                                    // pressurized line is its own hazard
    }
    // If already in VALVE_CLOSING, let it finish the bleed timer normally.
}

bool popCompletedFeedback(NodeCFeedback &feedback) {
    if (!feedbackReady) return false;
    feedback = pendingFeedback;
    feedbackReady = false;
    return true;
}

void tickActuators() {
    uint32_t now = millis();
    uint32_t elapsed = now - stateEnteredMs;

    switch (state) {

        case IDLE:
            break;   // nothing to do — waiting for startPumpCycle()

        case VALVE_OPENING:
            // Checked independently, not chained off the transition below —
            // this only fires if tickActuators() itself gets delayed between
            // calls (e.g. a blocking LoRa TX elsewhere in loop()) long enough
            // that elapsed jumps past 2x in a single observation. Under
            // normal per-tick execution the transition below always fires
            // first, so this is a real backstop, not dead code.
            if (elapsed >= (uint32_t)(VALVE_PRECHARGE_MS * 2)) {
                forceHardStopToIdle("VALVE_OPENING exceeded 2x expected duration");
                break;
            }
            if (elapsed >= VALVE_PRECHARGE_MS) {
                pumpOn();
                pumpStartedMs = now;
                lastStallSampleMs = now;
                setState(PUMP_RUNNING);
            }
            break;

        case PUMP_RUNNING: {
            // ── Dual-threshold current fault detection ──────────────────
            if (now - lastStallSampleMs >= FAULT_SAMPLE_MS) {
                lastStallSampleMs = now;
                lastCurrentA = readCurrentA();

                bool pastInrushWindow = (now - pumpStartedMs) >= JAM_STARTUP_GRACE_MS;
                bool dryRun = (lastCurrentA < DRY_RUN_CURRENT_A);
                bool jammed = pastInrushWindow && (lastCurrentA > JAM_CURRENT_A);

                if (dryRun || jammed) {
                    stallSampleStreak++;
                    if (stallSampleStreak >= FAULT_CONSEC_COUNT) {
                        Serial.printf("[FAULT] %s detected — current=%.2fA. Stopping pump.\n",
                                      jammed ? "JAM" : "DRY-RUN/BLOCKED", lastCurrentA);
                        stallFault = true;
                        finalStatusFlag = IRRIGATION_FAILED;
                        pumpOff();
                        setState(VALVE_CLOSING);
                        break;
                    }
                } else {
                    stallSampleStreak = 0;
                }
            }

            // ── Early stop: target volume reached (flow meter is the
            //    accurate signal; the time ceiling below is a backstop) ──
            noInterrupts();
            uint32_t pulses = pulseCount;
            interrupts();
            if (litersFromPulses(pulses) >= targetVolumeL) {
                Serial.println("[ACTUATORS] Target volume reached.");
                pumpOff();
                setState(VALVE_CLOSING);
                break;
            }

            // ── No-flow lockout: pump drawing normal current (so the
            //    ACS712 checks above see nothing wrong) but zero pulses
            //    after NO_FLOW_GRACE_MS means a real plumbing failure —
            //    detached coupling, blocked intake, lost prime — that
            //    current sensing alone cannot see. Without this, the
            //    pump would otherwise run dry until the time ceiling. ──
            if ((now - pumpStartedMs) >= NO_FLOW_GRACE_MS && pulses == 0) {
                Serial.println("[FAULT] Pump running, normal current, but ZERO flow "
                                "pulses — pipeline failure (blocked/detached/lost prime).");
                stallFault = true;
                finalStatusFlag = IRRIGATION_FAILED;
                pumpOff();
                setState(VALVE_CLOSING);
                break;
            }

            // ── Time-based ceiling (3-tier clamp backstop) ───────────────
            if (elapsed >= effectiveRuntimeMs) {
                Serial.println("[SAFETY] Runtime ceiling reached — stopping pump.");
                pumpOff();
                setState(VALVE_CLOSING);
            }
            break;
        }

        case VALVE_CLOSING:
            if (elapsed >= PRESSURE_BLEED_MS) {
                valveOff();
                uint8_t status = emergencyRequested ? IRRIGATION_ABORTED : finalStatusFlag;
                publishFeedback(status);
                setState(IDLE);
            }
            break;
    }
}
