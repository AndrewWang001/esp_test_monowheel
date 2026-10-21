#pragma once

// Servo control task: sole owner of the servo UART. Later it will read imu_get_state()
// and command the servo from it; for now it runs a spin + LED blink test.

void servo_task_start(void);
