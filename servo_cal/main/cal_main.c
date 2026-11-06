#include "servo_calibrate.h"

// Pick the step, then Build, Flash and Monitor. Full instructions in ../../main/servo_calibrate.h
//   CAL_CENTER  horns off: servos go to mid travel (500) and hold, fit horns in the zero pose
//   CAL_READ    torque off: prints raw position + angle, copy zeros/signs into ../../main/joints.c
//   CAL_CHECK   torque on: each joint goes to +20 deg and back so you can confirm directions
//   CAL_SLIDER  torque on: drive each servo live from sliders on your computer. Flash, close
//               the monitor, then run  sim/.venv/bin/python sim/servo_sliders.py
#define CAL_STEP CAL_SLIDER

void app_main(void)
{
    servo_calibrate_run(CAL_STEP);
}
