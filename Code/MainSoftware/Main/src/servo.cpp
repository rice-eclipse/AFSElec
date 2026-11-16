/**
 * servo.cpp - 50 Hz servo PWM on PWM1-PWM8 and the startup sweep task.
 */
#include "servo.h"

#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "rtos.h"

#include "board_pins.h"
#include "shared.h"

static constexpr uint SERVO_PINS[SERVO_COUNT] = {PWM1, PWM2, PWM3, PWM4, PWM5, PWM6, PWM7, PWM8};

// The PWM counter ticks once per microsecond, so the compare level is the pulse width in us.
static constexpr uint32_t SERVO_PERIOD_US = 1'000'000 / SERVO_PWM_HZ;

static uint32_t s_pulseUs[SERVO_COUNT];   // last commanded pulse width of each servo

uint32_t servoPulseUs(int index)
{
    return index >= 0 && index < SERVO_COUNT ? s_pulseUs[index] : 0;
}

void servoRelease(int index)
{
    if (index < 0 || index >= SERVO_COUNT) return;
    s_pulseUs[index] = 0;
    pwm_set_gpio_level(SERVO_PINS[index], 0);   // no pulses: the servo stops driving
}

void servoReleaseAll()
{
    for (int i = 0; i < SERVO_COUNT; ++i) servoRelease(i);
}

void servoCenterAll()
{
    for (int i = 0; i < SERVO_COUNT; ++i) servoWriteUs(i, SERVO_CENTER_US[i]);
}

void servoInit()
{
    const float div = (float)clock_get_hz(clk_sys) / 1'000'000.0f;   // 1 MHz counter
    for (int i = 0; i < SERVO_COUNT; ++i) {
        const uint pin = SERVO_PINS[i];
        gpio_set_function(pin, GPIO_FUNC_PWM);
        const uint slice = pwm_gpio_to_slice_num(pin);
        pwm_set_clkdiv(slice, div);
        pwm_set_wrap(slice, SERVO_PERIOD_US - 1);
        servoWriteUs(i, SERVO_CENTER_US[i]);
        pwm_set_enabled(slice, true);
    }
}

void servoWriteUs(int index, uint32_t pulseUs)
{
    if (index < 0 || index >= SERVO_COUNT) return;
    if (pulseUs < servoMinUs(index)) pulseUs = servoMinUs(index);
    if (pulseUs > servoMaxUs(index)) pulseUs = servoMaxUs(index);
    s_pulseUs[index] = pulseUs;
    pwm_set_gpio_level(SERVO_PINS[index], (uint16_t)pulseUs);
}

void servoWriteDeg(int index, float deg)
{
    if (index < 0 || index >= SERVO_COUNT) return;
    const float us = (float)SERVO_CENTER_US[index] + deg * SERVO_US_PER_DEG;
    servoWriteUs(index, us < 0.0f ? 0u : (uint32_t)lroundf(us));
}

void servoTask(void*)
{
    servoInit();
    printf("Servo sweep: +-%.1f deg around center for %lu ms\n", (double)SERVO_SWEEP_DEG,
           (unsigned long)SERVO_SWEEP_MS);

    // Triangle wave: -SERVO_SWEEP_DEG -> +SERVO_SWEEP_DEG in SERVO_SWEEP_HALF_MS, then back, repeated
    const uint32_t start = rtos::millis();
    rtos::Periodic rate(SERVO_UPDATE_MS);
    uint32_t elapsed;
    while ((elapsed = rtos::millis() - start) < SERVO_SWEEP_MS) {
        const uint32_t phase = elapsed % (2 * SERVO_SWEEP_HALF_MS);
        const uint32_t ramp  = phase < SERVO_SWEEP_HALF_MS ? phase : 2 * SERVO_SWEEP_HALF_MS - phase;
        const float    deg   = SERVO_SWEEP_DEG * (2.0f * (float)ramp / (float)SERVO_SWEEP_HALF_MS - 1.0f);
        for (int i = 0; i < SERVO_COUNT; ++i) servoWriteDeg(i, deg);
        rate.wait();
    }

    for (int i = 0; i < SERVO_COUNT; ++i) servoWriteDeg(i, 0.0f);
    printf("Servo sweep done\n");
    g_events.set(EVT_SERVO_DONE);
    vTaskDelete(nullptr);
}
