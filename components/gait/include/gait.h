#pragma once

// Leg gaits for moving along +y, ported from sim/gait.py. Plain C + <math.h>, no ESP-IDF deps.
//
// Two legs stick out sideways: the front leg to +x, the back leg to -x. The back leg is a
// mirror image of the front leg, so both legs get the same targets in their own frames and
// the same angles give mirrored poses. If a servo on the mirrored leg turns the opposite
// way, flip its sign when converting to servo position.
//
// Each foot cycles between stance (on the ground, sliding back so the body moves forward)
// and swing (lifted and brought forward).

#include <stdbool.h>
#include <stdint.h>
#include "leg_kinematics.h"   // also defines leg_id_t / NUM_LEGS

typedef enum {
    GAIT_PUSH = 0,              // both legs push together, then both lift and glide
    GAIT_WALK,                  // legs alternate, one foot always down
} gait_type_t;

typedef struct {
    leg_angles_t q[NUM_LEGS];      // joint angles to command, deg
    leg_point_t foot[NUM_LEGS];    // foot target in that leg's own frame, mm
    bool on_ground[NUM_LEGS];      // true = stance, false = swing
} gait_pose_t;

// Joint angles for both legs at t_ms into the gait. Repeats every gait_period_ms().
// Returns the first leg's error if a pose is unreachable (only if the gait constants are changed).
leg_err_t gait_pose(gait_type_t gait, uint32_t t_ms, gait_pose_t *out);

uint32_t gait_period_ms(void);

// Forward speed the gait produces, mm/s (planted feet don't slide at this speed)
float gait_body_speed(gait_type_t gait);
