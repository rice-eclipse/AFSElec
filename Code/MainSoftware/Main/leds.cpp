/**
 * leds.cpp - Indicator LEDs on GPIO 0-3 (blue, red, yellow, green).
 */
#include "leds.h"

#include "pico/stdlib.h"
#include "board_pins.h"

// Indexed by Led (leds.h)
static constexpr uint LED_PINS[LED_COUNT] = {LED_BLUE_PIN, LED_RED_PIN, LED_YELLOW_PIN, LED_GREEN_PIN};
static bool s_on[LED_COUNT];

void ledsInit()
{
    for (int i = 0; i < LED_COUNT; ++i) {
        gpio_init(LED_PINS[i]);
        gpio_set_dir(LED_PINS[i], GPIO_OUT);
        ledSet(i, false);
    }
}

void ledSet(int index, bool on)
{
    if (index < 0 || index >= LED_COUNT) return;
    s_on[index] = on;
    gpio_put(LED_PINS[index], on == LED_ACTIVE_HIGH);
}

void ledsSetAll(bool on)
{
    for (int i = 0; i < LED_COUNT; ++i) ledSet(i, on);
}

void ledsToggleAll()
{
    for (int i = 0; i < LED_COUNT; ++i) ledSet(i, !s_on[i]);
}
