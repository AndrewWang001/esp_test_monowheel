#pragma once

// Leg servos: joint angles (deg, leg_kinematics convention) <-> servo positions, using each
// servo's calibration in joint_cal (legs.c). Measure the calibration with the servo
// calibration tool mode (main/modes/tool_servo_cal.c).
//
// Calls the servo driver, so use it only from the task that owns the servo bus.

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "leg_kinematics.h"

#define JOINTS_PER_LEG    3                       // q1 base, q2 shoulder, q3 elbow
#define JOINT_POS_PER_DEG (1000.0f / 240.0f)      // HTD-45H: 0..1000 = 0..240 deg

typedef struct {
    uint8_t id;      // bus ID
    int8_t sign;     // +1 if a higher servo position means a higher joint angle, else -1
    int16_t zero;    // servo position when the joint is at 0 deg
} joint_cal_t;

extern const joint_cal_t joint_cal[NUM_LEGS][JOINTS_PER_LEG];

const char *joint_name(leg_id_t leg, int joint);   // e.g. "back q2"

// Servo position for a joint angle, clamped to 0..SERVO_POS_MAX
uint16_t joint_deg_to_pos(leg_id_t leg, int joint, float deg);
float joint_pos_to_deg(leg_id_t leg, int joint, int16_t pos);

// Waits for every servo, warns about any joint whose +/-100 deg range doesn't fit the servo's
// travel, and sets each servo's own angle limit to the joint limits. Torque stays off.
esp_err_t legs_init(void);

esp_err_t legs_move(leg_id_t leg, const leg_angles_t *q, uint16_t time_ms);
esp_err_t legs_read(leg_id_t leg, leg_angles_t *q);
