#include <Arduino.h>
#include "modbus_core.h"
#include "pins.h"
#include "config.h"
#include "crc16.h"

// ═══════════════════════════════════════════════════════════════════════════
// RS-485 / Modbus RTU
// ═══════════════════════════════════════════════════════════════════════════

static inline void rs485Tx() { digitalWrite(RE_DE_PIN, HIGH); delayMicroseconds(100); }
static inline void rs485Rx() { Serial2.flush(); delayMicroseconds(100); digitalWrite(RE_DE_PIN, LOW); }

void initModbus() {
    pinMode(RE_DE_PIN, OUTPUT);
    rs485Rx();
    Serial2.begin(RS485_BAUD, SERIAL_8N1, RX2_PIN, TX2_PIN);
    Serial.printf("[INIT]  RS-485 UART2  RX=%d TX=%d DE=%d  baud=%d\n",
                  RX2_PIN, TX2_PIN, RE_DE_PIN, RS485_BAUD);
}

static bool modbusRead(uint8_t addr, uint16_t startReg,
                        uint8_t count, uint16_t *out)
{
    uint8_t req[8];
    req[0] = addr;
    req[1] = 0x03;
    req[2] = (startReg >> 8) & 0xFF;
    req[3] =  startReg       & 0xFF;
    req[4] = 0x00;
    req[5] = count;
    uint16_t reqCrc = crc16Modbus(req, 6);
    req[6] = reqCrc & 0xFF;
    req[7] = (reqCrc >> 8) & 0xFF;

    while (Serial2.available()) Serial2.read();

    rs485Tx();
    Serial2.write(req, 8);
    rs485Rx();

    uint8_t  expectedLen = 5 + count * 2;
    uint8_t  resp[32];
    uint8_t  idx = 0;
    uint32_t t0  = millis();

    while (millis() - t0 < MODBUS_TIMEOUT_MS && idx < expectedLen) {
        if (Serial2.available()) resp[idx++] = Serial2.read();
    }

    if (idx < expectedLen) return false;

    uint16_t rxCrc   = resp[idx-2] | ((uint16_t)resp[idx-1] << 8);
    uint16_t calcCrc = crc16Modbus(resp, idx - 2);
    if (rxCrc != calcCrc) return false;

    for (uint8_t i = 0; i < count; i++)
        out[i] = ((uint16_t)resp[3 + i*2] << 8) | resp[4 + i*2];

    return true;
}

bool readSoilSensor(MasterSensorRecord &r) {
    uint16_t regs[7] = {0};
    bool ok = modbusRead(SENSOR_MODBUS_ADDR, 0x0000, 7, regs);

    if (!ok) {
        r.health_flag |= HEALTH_NODEA_SOIL_FAULT;
        r.vwc_percent  = NAN;
        r.soil_temp = r.ec = r.ph = r.nitrogen = r.phosphorus = r.potassium = NAN;
        Serial.println("[SOIL]  Modbus read FAILED — no response or CRC error");
        return false;
    }

    r.vwc_percent = regs[0] / 10.0f;
    r.soil_temp   = (int16_t)regs[1] / 10.0f;
    r.ec          = regs[2] / 10.0f;
    r.ph          = regs[3] / 10.0f;
    r.nitrogen    = (float)regs[4];
    r.phosphorus  = (float)regs[5];
    r.potassium   = (float)regs[6];

    if (r.vwc_percent < 0.0f   || r.vwc_percent > 100.0f ||
        r.soil_temp   < -40.0f || r.soil_temp   >  85.0f ||
        r.ph          <  3.0f  || r.ph          >  10.0f) {
        r.health_flag |= HEALTH_NODEA_SOIL_FAULT;
        Serial.printf("[SOIL]  Sanity fail — VWC=%.1f Temp=%.1f pH=%.1f\n",
                      r.vwc_percent, r.soil_temp, r.ph);
        return false;
    }

    Serial.printf("[SOIL]  VWC=%.1f%%  Temp=%.1fC  EC=%.2f  pH=%.1f\n",
                  r.vwc_percent, r.soil_temp, r.ec, r.ph);
    Serial.printf("        N=%.0f  P=%.0f  K=%.0f  mg/kg\n",
                  r.nitrogen, r.phosphorus, r.potassium);
    return true;
}
