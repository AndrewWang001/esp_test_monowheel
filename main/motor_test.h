#pragma once

// Scripted bench test for the drive motor (ESC on GPIO18). Runs once after power-up, no radio:
//   1. neutral while the ESC arms
//   2. ramp up slowly forward, hold, ramp down to stop
//   3. ramp up slowly in reverse, hold, ramp down to stop
//   4. ramp up forward to 80%, hold, ramp down to stop
// then stays stopped. Press EN (reset) to run it again.

#include <stdint.h>

void motor_test_start(void);
uint32_t motor_test_get_us(void);   // pulse being sent to the ESC right now
int motor_test_get_pct(void);       // same as -100..100 % throttle
