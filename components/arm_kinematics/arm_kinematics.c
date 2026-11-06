#include "arm_kinematics.h"

#include <math.h>

#define PI_F       3.14159265f
#define RAD(deg)   ((deg) * (PI_F / 180.0f))
#define DEG(rad)   ((rad) * (180.0f / PI_F))

#define L1 ARM_L1_MM
#define L2 ARM_L2_MM

// Keep angle in [-180, 180)
static float wrap180(float a)
{
    a = fmodf(a + 180.0f, 360.0f);
    if (a < 0.0f) a += 360.0f;
    return a - 180.0f;
}

// Two-link IK in the arm's plane (r = forward, z = up). False if out of reach.
static bool ik_planar(float r, float z, bool elbow_up, float *q2_deg, float *q3_deg)
{
    float c3 = (r * r + z * z - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
    if (fabsf(c3) > 1.0f + 1e-6f) return false;
    c3 = fmaxf(-1.0f, fminf(1.0f, c3));     // clean up tiny rounding errors
    float q3 = acosf(c3);
    if (!elbow_up) q3 = -q3;
    float beta = atan2f(L2 * sinf(q3), L1 + L2 * cosf(q3));
    *q2_deg = DEG(atan2f(r, z) - beta);     // measured from vertical
    *q3_deg = DEG(q3);
    return true;
}

void arm_fk(const arm_angles_t *q, arm_point_t *out)
{
    float q1 = RAD(q->q1), q2 = RAD(q->q2), q3 = RAD(q->q3);
    float r = L1 * sinf(q2) + L2 * sinf(q2 + q3);
    out->x = -r * sinf(q1);
    out->y = r * cosf(q1);
    out->z = L1 * cosf(q2) + L2 * cosf(q2 + q3);
}

bool arm_within_limits(const arm_angles_t *q)
{
    return q->q1 >= ARM_Q_MIN_DEG && q->q1 <= ARM_Q_MAX_DEG &&
           q->q2 >= ARM_Q_MIN_DEG && q->q2 <= ARM_Q_MAX_DEG &&
           q->q3 >= ARM_Q_MIN_DEG && q->q3 <= ARM_Q_MAX_DEG;
}

arm_err_t arm_ik(const arm_point_t *p, bool elbow_up, arm_angles_t *out)
{
    float q1 = DEG(atan2f(-p->x, p->y));
    float r = hypotf(p->x, p->y);

    // up to 4 solutions: elbow up/down, and base facing the target or
    // turned 180 deg with the arm reaching back over the top
    const bool elbows[2] = {elbow_up, !elbow_up};    // preferred elbow first
    const float bases[2] = {q1, wrap180(q1 + 180.0f)};
    const float reaches[2] = {r, -r};

    bool reachable = false;
    for (int e = 0; e < 2; e++) {
        for (int b = 0; b < 2; b++) {
            float q2, q3;
            if (!ik_planar(reaches[b], p->z, elbows[e], &q2, &q3)) continue;
            reachable = true;
            arm_angles_t sol = {bases[b], wrap180(q2), q3};
            if (arm_within_limits(&sol)) {
                *out = sol;
                return ARM_OK;
            }
        }
    }
    return reachable ? ARM_ERR_JOINT_LIMIT : ARM_ERR_OUT_OF_REACH;
}

const char *arm_err_str(arm_err_t err)
{
    switch (err) {
    case ARM_OK:               return "ok";
    case ARM_ERR_OUT_OF_REACH: return "target out of reach";
    case ARM_ERR_JOINT_LIMIT:  return "target needs angles outside joint limits";
    }
    return "unknown";
}
