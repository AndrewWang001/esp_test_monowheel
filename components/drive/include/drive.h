#pragma once

// Drive motor from the T8L: right stick up/down (CH2) sets wheel speed and direction through
// the ESC. CH5 is the kill switch: the motor only runs while it is in the RUN position, and
// only becomes enabled while the stick is centred. Stops if the switch goes to KILL or the
// radio drops out. Speed is capped at DRIVE_LIMIT_PCT of full throttle.
//
// Needs rc_start() to be running. Owns the ESC: don't run alongside another ESC user.

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t pulse_us;      // what is being sent to the ESC right now
    bool motor_enabled;     // kill switch in RUN and enabled (stick was centred)
} drive_state_t;

void drive_start(void);
void drive_get(drive_state_t *out);
