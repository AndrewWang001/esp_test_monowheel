#pragma once

// 3-joint arm kinematics, ported from sim/arm.py. Plain C + <math.h>, no ESP-IDF deps.
//
// Angle convention (all zero = arm pointing straight up, base facing +y), degrees:
//   q1  base yaw about z, positive turns from +y toward -x (counterclockwise seen from above)
//   q2  shoulder tilt from vertical, positive leans forward
//   q3  elbow bend relative to link 1, positive bends further forward
// Positions are in mm, in the arm's own frame: origin at the shoulder, z up.

#include <stdbool.h>

#define ARM_L1_MM      170.0f   // shoulder -> elbow
#define ARM_L2_MM      170.0f   // elbow -> foot
#define ARM_Q_MIN_DEG  -100.0f  // joint limits, same for all three joints
#define ARM_Q_MAX_DEG  100.0f

typedef struct {
    float q1, q2, q3;           // deg
} arm_angles_t;

typedef struct {
    float x, y, z;              // mm
} arm_point_t;

typedef enum {
    ARM_OK = 0,
    ARM_ERR_OUT_OF_REACH,       // too far from (or too close to) the shoulder for any angles
    ARM_ERR_JOINT_LIMIT,        // physically reachable, but only past ARM_Q_MIN/MAX_DEG
} arm_err_t;

// Foot position for the given joint angles
void arm_fk(const arm_angles_t *q, arm_point_t *out);

// Joint angles that put the foot at p. Tries both elbows (elbow_up first; elbow up = q3 > 0)
// and both base directions, and returns the first solution inside the joint limits.
// out is only written on ARM_OK.
arm_err_t arm_ik(const arm_point_t *p, bool elbow_up, arm_angles_t *out);

bool arm_within_limits(const arm_angles_t *q);

const char *arm_err_str(arm_err_t err);
