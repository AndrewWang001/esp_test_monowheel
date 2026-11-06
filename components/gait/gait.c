#include "gait.h"

#include <math.h>

#define PI_F          3.14159265f

#define HIP_HEIGHT_MM 164.0f    // shoulder joints above the ground
#define STRIDE_MM     100.0f    // foot travel along y per cycle
#define LIFT_MM       30.0f     // foot clearance at the top of a swing
#define REACH_OUT_MM  215.0f    // foot sits this far out to the side of its shoulder
#define FOOT_Y_MM     40.0f     // middle of the stride sits this far ahead of the shoulder;
                                // keeps q1 inside +/-100 deg (straight sideways is already -90)
#define PERIOD_MS     1500      // one gait cycle

typedef struct {
    float duty;                         // fraction of the cycle a foot is on the ground
    float offset[GAIT_NUM_LEGS];        // where in the cycle each leg starts
} gait_params_t;

static const gait_params_t GAITS[] = {
    [GAIT_PUSH] = {.duty = 0.5f, .offset = {[LEG_BACK] = 0.0f, [LEG_FRONT] = 0.0f}},
    [GAIT_WALK] = {.duty = 0.6f, .offset = {[LEG_BACK] = 0.0f, [LEG_FRONT] = 0.5f}},
};

// Foot position relative to the middle of the stride at phase (0..1). Returns true in stance.
static bool foot_offset(float phase, float duty, float *dy, float *dz)
{
    if (phase < duty) {                 // stance: slide back in a straight line
        float s = phase / duty;
        *dy = STRIDE_MM * (0.5f - s);
        *dz = 0.0f;
        return true;
    }
    float s = (phase - duty) / (1.0f - duty);  // swing: lift and bring forward
    *dy = STRIDE_MM * (-0.5f + (1.0f - cosf(PI_F * s)) / 2.0f);   // eases in and out
    *dz = LIFT_MM * sinf(PI_F * s);
    return false;
}

arm_err_t gait_pose(gait_type_t gait, uint32_t t_ms, gait_pose_t *out)
{
    const gait_params_t *g = &GAITS[gait];
    float cycle = (float)(t_ms % PERIOD_MS) / PERIOD_MS;   // wrap first so t_ms never loses precision

    for (int leg = 0; leg < GAIT_NUM_LEGS; leg++) {
        float phase = fmodf(cycle + g->offset[leg], 1.0f);
        float dy, dz;
        out->on_ground[leg] = foot_offset(phase, g->duty, &dy, &dz);

        // In each arm's own frame the foot is always out to +x: the mirror that puts the
        // back foot at -x in the body frame cancels when going back into the arm's frame.
        arm_point_t p = {REACH_OUT_MM, FOOT_Y_MM + dy, -HIP_HEIGHT_MM + dz};
        out->foot[leg] = p;
        arm_err_t err = arm_ik(&p, true, &out->q[leg]);
        if (err != ARM_OK) return err;
    }
    return ARM_OK;
}

uint32_t gait_period_ms(void)
{
    return PERIOD_MS;
}

float gait_body_speed(gait_type_t gait)
{
    // during stance the foot slides back STRIDE_MM in duty * period; moving the body
    // forward at that speed keeps the planted foot still on the ground
    return STRIDE_MM / (GAITS[gait].duty * PERIOD_MS / 1000.0f);
}
