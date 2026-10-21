#pragma once

// MPU-6050 attitude estimator running in its own FreeRTOS task at 100 Hz.
// Other tasks read the latest result with imu_get_state().

#include <stdbool.h>

typedef struct {
    float roll, pitch, yaw; // deg (yaw is gyro-only and drifts)
    float rate[3];          // deg/s x, y, z, Kalman-smoothed and bias-corrected
    bool still;             // board has been motionless for >= 0.5 s
    bool valid;             // false until calibration has finished
} imu_state_t;

void imu_start(void);
void imu_get_state(imu_state_t *out);
