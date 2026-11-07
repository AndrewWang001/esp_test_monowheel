#pragma once

// 3-joint leg kinematics, ported from sim/leg.py. Plain C + <math.h>, no ESP-IDF deps.
//
// Angle convention (all zero = leg pointing straight up, base facing +y), degrees:
//   q1  base yaw about z, positive turns from +y toward -x (counterclockwise seen from above)
//   q2  shoulder tilt from vertical, positive leans forward
//   q3  elbow bend relative to link 1, positive bends further forward
// Positions are in mm, in the leg's own frame: origin at the shoulder, z up.

#include <stdbool.h>

#define LEG_L1_MM      170.0f   // shoulder -> elbow
#define LEG_L2_MM      170.0f   // elbow -> foot
#define LEG_Q_MIN_DEG  -100.0f  // joint limits, same for all three joints
#define LEG_Q_MAX_DEG  100.0f

// The robot's two legs. The back leg is a mirror image of the front leg (see gait.h).
#define NUM_LEGS 2
typedef enum {
    LEG_BACK = 0,               // shoulder at y = 0, foot out to -x
    LEG_FRONT = 1,              // shoulder SPACING mm ahead, foot out to +x
} leg_id_t;

typedef struct {
    float q1, q2, q3;           // deg
} leg_angles_t;

typedef struct {
    float x, y, z;              // mm
} leg_point_t;

typedef enum {
    LEG_OK = 0,
    LEG_ERR_OUT_OF_REACH,       // too far from (or too close to) the shoulder for any angles
    LEG_ERR_JOINT_LIMIT,        // physically reachable, but only past LEG_Q_MIN/MAX_DEG
} leg_err_t;

// Foot position for the given joint angles
void leg_fk(const leg_angles_t *q, leg_point_t *out);

// Joint angles that put the foot at p. Tries both elbows (elbow_up first; elbow up = q3 > 0)
// and both base directions, and returns the first solution inside the joint limits.
// out is only written on LEG_OK.
leg_err_t leg_ik(const leg_point_t *p, bool elbow_up, leg_angles_t *out);

bool leg_within_limits(const leg_angles_t *q);

const char *leg_err_str(leg_err_t err);
