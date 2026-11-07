#pragma once

// NOT STARTED YET (kept for later). Use motor_test for the scripted bench test first.
//
// Drive motor from the T8L: right stick up/down (CH2) sets wheel speed and direction through the ESC.
// Only runs while the arm switch (CH5) is on. Stops if the switch is off or the radio drops out.
// Speed is capped at DRIVE_LIMIT_PCT of full throttle for testing.

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t pulse_us;   // what is being sent to the ESC right now
    bool armed;
} rc_drive_state_t;

void rc_drive_start(void);
void rc_drive_get(rc_drive_state_t *out);
