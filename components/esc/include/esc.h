#pragma once

// Servo-style PWM for the Repeat Robotics AM32 bi-directional ESC: 50 Hz, pulse width sets
// throttle. 1000 us = full reverse, 1500 us = stop, 2000 us = full forward.
//
// Two ESCs (2-wheel drive) get the exact same signal: every call drives both.
// Wiring: ESC 1 signal -> PIN_ESC, ESC 2 signal -> PIN_ESC2 (pins.h), both signal GNDs -> ESP GND
// (the battery powers the ESCs, not the ESP). If a wheel turns the wrong way, swap two of its
// three motor wires.

#include <stdint.h>

#define ESC_US_MIN     1000
#define ESC_US_NEUTRAL 1500
#define ESC_US_MAX     2000

void esc_init(void);                  // starts outputting neutral (1500 us) right away
void esc_write_us(uint32_t pulse_us); // clamped to ESC_US_MIN..ESC_US_MAX
