#include <Arduino.h>
#include "leds.h"
#include "pins.h"

void ledInit() {
    pinMode(PIN_LED_WHITE,  OUTPUT);
    pinMode(PIN_LED_GREEN,  OUTPUT);
    pinMode(PIN_LED_YELLOW, OUTPUT);
    pinMode(PIN_LED_RED,    OUTPUT);
    ledAllOff();
}

void ledAllOff() {
    digitalWrite(PIN_LED_WHITE,  LOW);
    digitalWrite(PIN_LED_GREEN,  LOW);
    digitalWrite(PIN_LED_YELLOW, LOW);
    digitalWrite(PIN_LED_RED,    LOW);
}

void ledSet(uint8_t pin, bool on) {
    digitalWrite(pin, on ? HIGH : LOW);
}

void ledPulse(uint8_t pin, uint16_t ms) {
    digitalWrite(pin, HIGH);
    delay(ms);
    digitalWrite(pin, LOW);
}

// Boot ripple: WHITE → GREEN → YELLOW → RED → all off
void ledBootRipple() {
    const uint8_t pins[] = {PIN_LED_WHITE, PIN_LED_GREEN,
                             PIN_LED_YELLOW, PIN_LED_RED};
    for (uint8_t i = 0; i < 4; i++) {
        digitalWrite(pins[i], HIGH); delay(150);
        digitalWrite(pins[i], LOW);  delay(50);
    }
    delay(300);
}

// Set fault LEDs: sensor fault = RED only; TX fault = RED + GREEN
void ledShowFault(bool sensorFault, bool txFault) {
    ledAllOff();
    ledSet(PIN_LED_RED, true);
    if (!sensorFault && txFault) ledSet(PIN_LED_GREEN, true);
}
