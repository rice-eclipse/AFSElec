/**
 * leds.h - Indicator LEDs on GPIO 0-3:
 *   GPIO 0 = blue, GPIO 1 = red, GPIO 2 = yellow, GPIO 3 = green
 *
 * LED_ACTIVE_HIGH in config.h sets the polarity (true = GPIO high turns the LED on).
 */
#pragma once

#include "config.h"   // LED_COUNT

/** LED indices for ledSet(). */
enum Led : int {
    LED_BLUE   = 0,   // GPIO 0
    LED_RED    = 1,   // GPIO 1
    LED_YELLOW = 2,   // GPIO 2
    LED_GREEN  = 3,   // GPIO 3
};

/** Sets the LED pins as outputs, all off. */
void ledsInit();

/** Turns one LED (LED_BLUE, LED_RED, LED_YELLOW, LED_GREEN) on or off. */
void ledSet(int index, bool on);

/** Turns every LED on or off. */
void ledsSetAll(bool on);

/** Flips every LED. */
void ledsToggleAll();
