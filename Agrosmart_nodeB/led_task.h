#pragma once
#ifndef LED_TASK_H
#define LED_TASK_H

// ─────────────────────────────────────────────────────────────────────────────
// led_task.h — AgroSmart Node B LED Status Driver (PCA9685)
//
// Hardware : PCA9685 16-channel PWM driver
//            I2C-1  Wire1  SDA=33  SCL=32  Address=0x40
//            Driven at 1000 Hz — 12-bit resolution (0–4095)
//
// Channels : see pins.h LED_CH_* defines
//
// Patterns :
//   LED_OFF          — 0 % duty cycle
//   LED_SOLID        — 100 % duty cycle
//   LED_DIM          — 12 % duty cycle (standby glow)
//   LED_BLINK_SLOW   — 1 Hz square wave (600 ms on / 400 ms off)
//   LED_BLINK_FAST   — 4 Hz square wave (100 ms on / 150 ms off)
//   LED_BREATHE      — sinusoidal ramp 0→4095→0, period 3 s
//
// The task reads SystemHealthState every LED_HEALTH_POLL_MS and maps it to
// the 8 physical channels. Mapping priority (high → low):
//   FAULT    > WARNING > HEALTHY for CH_SYSTEM_OK / CH_FAULT
//   liveness > timeout > never-seen for node LEDs
//   pending  > not-pending for CH_IRRIGATION
// ─────────────────────────────────────────────────────────────────────────────

// FreeRTOS task update period
#define LED_UPDATE_PERIOD_MS    50     // 20 Hz — smooth breathing ramp
#define LED_HEALTH_POLL_MS      500    // re-read SystemHealthState every 500 ms

void vLedTask(void *pvParameters);

#endif // LED_TASK_H
