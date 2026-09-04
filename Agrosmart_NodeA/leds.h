#pragma once

#include <stdint.h>

void ledInit();
void ledAllOff();
void ledSet(uint8_t pin, bool on);
void ledPulse(uint8_t pin, uint16_t ms);
void ledBootRipple();
void ledShowFault(bool sensorFault, bool txFault);
