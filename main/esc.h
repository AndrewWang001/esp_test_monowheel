#pragma once

// Servo-style PWM for the Repeat Robotics AM32 bi-directional ESC: 50 Hz, pulse width sets
// throttle. 1000 us = full reverse, 1500 us = stop, 2000 us = full forward.
//
// Wiring: ESC signal -> GPIO18, ESC signal GND -> ESP GND (battery powers the ESC, not the ESP)

#include <stdint.h>

#define ESC_US_MIN     1000
#define ESC_US_NEUTRAL 1500
#define ESC_US_MAX     2000

void esc_init(void);                  // starts outputting neutral (1500 us) right away
void esc_write_us(uint32_t pulse_us); // clamped to ESC_US_MIN..ESC_US_MAX
