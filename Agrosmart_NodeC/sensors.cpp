#include "sensors.h"
#include "pins.h"
#include "config.h"
#include <Wire.h>
#include <Adafruit_INA219.h>   // Arduino Library Manager: "Adafruit INA219"

// ============================================================
// sensors.cpp — INA219 battery/system monitor
// ============================================================

static Adafruit_INA219 ina219(INA219_ADDR);
static bool inaOk = false;

void initSensors() {
    Wire.begin(INA219_SDA_PIN, INA219_SCL_PIN);

    // A 12V DC pump is a serious EMI source. If a noise spike corrupts
    // an I2C clock pulse mid-transaction, the Wire library can otherwise
    // block indefinitely waiting for a response that will never come —
    // freezing the entire loop(), pump relay included. This timeout
    // turns that failure mode into a normal, handled one: the read
    // aborts, ina219.begin()/getBusVoltage_V() report failure, and
    // readBatteryVoltage() falls back to BATTERY_VOLTAGE_FALLBACK exactly
    // like a missing sensor would.
    Wire.setTimeOut(50);   // ms

    if (!ina219.begin(&Wire)) {
        inaOk = false;
        Serial.println("[SENSORS] WARN: INA219 not found on I2C bus — "
                        "check wiring/address. Falling back to "
                        "BATTERY_VOLTAGE_FALLBACK until resolved.");
        return;
    }

    inaOk = true;
    Serial.printf("[SENSORS] INA219 online at 0x%02X (SDA=%d SCL=%d)\n",
                  INA219_ADDR, INA219_SDA_PIN, INA219_SCL_PIN);

    // First live read at boot — sanity check the sensor is actually
    // reporting plausible numbers, not just ACKing on the bus.
    float v = readBatteryVoltage();
    Serial.printf("[SENSORS] Initial battery reading: %.2fV\n", v);
}

float readBatteryVoltage() {
    if (!inaOk) {
        return BATTERY_VOLTAGE_FALLBACK;
    }

    float v = ina219.getBusVoltage_V();

    if (isnan(v) || v < BATTERY_VOLTAGE_MIN_PLAUSIBLE || v > BATTERY_VOLTAGE_MAX_PLAUSIBLE) {
        Serial.printf("[SENSORS] WARN: INA219 reading %.2fV outside plausible "
                      "range [%.1f, %.1f] — using fallback instead.\n",
                      v, BATTERY_VOLTAGE_MIN_PLAUSIBLE, BATTERY_VOLTAGE_MAX_PLAUSIBLE);
        return BATTERY_VOLTAGE_FALLBACK;
    }

    return v;
}

bool isBatteryMonitorOk() {
    return inaOk;
}
