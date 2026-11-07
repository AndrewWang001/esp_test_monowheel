#pragma once

// Arm test: sweeps every servo in joint_cal (back leg IDs 1 2 3, front leg IDs 4 5 6) +/-45 deg
// around its zero in joints.c so you can see they're alive. Servos that don't answer are skipped
// and join in when they show up. Sole owner of the servo UART.

void arm_test_start(void);
