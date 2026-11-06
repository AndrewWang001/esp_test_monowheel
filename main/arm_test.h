#pragma once

// Arm test: sweeps the back-leg servos (IDs 1, 2, 3) +/-45 deg around their calibrated zero
// (joint_cal in joints.c) so you can see they're alive. Sole owner of the servo UART.

void arm_test_start(void);
