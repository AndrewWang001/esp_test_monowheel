#pragma once

// BNO085 IMU over I2C, using the rinku404/bno085 library (same setup as Victor's working test).
// Runs in its own task; any task can read the latest values with imu_get().
//
// Wiring (I2C mode): VCC->3V3, GND->GND, PS0+PS1->GND, ADO->GND (0x4A), CS->3V3,
//                    SDA, SCL, INT, RST: see pins.h

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float roll, pitch, yaw;  // deg, game rotation vector (accel + gyro, no magnetometer; yaw drifts)
    float gyro[3];           // deg/s, calibrated
    uint8_t accuracy;        // orientation accuracy: 0 unreliable .. 3 high
    bool valid;              // false until the first orientation report arrives
} imu_state_t;

void imu_start(void);
void imu_get(imu_state_t *out);
