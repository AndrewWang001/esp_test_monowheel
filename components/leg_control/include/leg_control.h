#pragma once

// Legs from the T8L: left stick up/down (CH3) sets the angle of every leg joint (servo IDs 1-6),
// -45 deg (stick down) .. 0 (centre) .. +45 deg (stick up), around each joint's zero in legs.c.
// Same motion as the leg sweep test, but following the stick.
//
// Kill switch (CH5, see rc.h): on KILL or radio loss the legs freeze where they are. On RUN they
// move slowly to the stick position, then follow it. Servos stay limp until the first RUN.
// Missing servos are skipped and join in when they show up.
//
// Needs rc_start() to be running. Owns the servo bus: don't run alongside another servo user.

#include <stdbool.h>

typedef struct {
    float angle_deg;    // angle the legs were last sent to
    bool following;     // kill switch in RUN and legs following the stick
    int servos;         // leg servos currently answering (of 6)
} leg_control_state_t;

void leg_control_start(void);
void leg_control_get(leg_control_state_t *out);
