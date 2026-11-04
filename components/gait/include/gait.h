#pragma once

// Leg gaits for moving along +y, ported from sim/gait.py. Plain C + <math.h>, no ESP-IDF deps.
//
// Two legs stick out sideways: the front leg to +x, the back leg to -x. The back arm is a
// mirror image of the front arm, so both legs get the same targets in their own frames and
// the same angles give mirrored poses. If a servo on the mirrored arm turns the opposite
// way, flip its sign when converting to servo position.
//
// Each foot cycles between stance (on the ground, sliding back so the body moves forward)
// and swing (lifted and brought forward).

#include <stdbool.h>
#include <stdint.h>
#include "arm_kinematics.h"

#define GAIT_NUM_LEGS 2

typedef enum {
    LEG_BACK = 0,               // shoulder at y = 0, foot out to -x
    LEG_FRONT = 1,              // shoulder SPACING mm ahead, foot out to +x
} gait_leg_t;

typedef enum {
    GAIT_PUSH = 0,              // both legs push together, then both lift and glide
    GAIT_WALK,                  // legs alternate, one foot always down
} gait_type_t;

typedef struct {
    arm_angles_t q[GAIT_NUM_LEGS];      // joint angles to command, deg
    arm_point_t foot[GAIT_NUM_LEGS];    // foot target in that arm's own frame, mm
    bool on_ground[GAIT_NUM_LEGS];      // true = stance, false = swing
} gait_pose_t;

// Joint angles for both legs at t_ms into the gait. Repeats every gait_period_ms().
// Returns the first leg's error if a pose is unreachable (only if the gait constants are changed).
arm_err_t gait_pose(gait_type_t gait, uint32_t t_ms, gait_pose_t *out);

uint32_t gait_period_ms(void);

// Forward speed the gait produces, mm/s (planted feet don't slide at this speed)
float gait_body_speed(gait_type_t gait);
