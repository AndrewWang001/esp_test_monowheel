#pragma once

// One-time tool for installing the leg servos so the angles in arm_kinematics.h line up with
// the real joints. Run it from app_main in place of the normal tasks. Works with however many
// of the servos in joints.c are connected, so you can do one leg (or one joint) at a time.
//
// Joint directions (arm_kinematics.h), zero pose = leg pointing straight up, base facing +y:
//   q1 base      positive turns counterclockwise seen from above (FRONT leg).
//                The back leg is mirrored, so for it positive is CLOCKWISE seen from above.
//   q2 shoulder  positive leans the leg forward (toward +y, the direction of travel)
//   q3 elbow     positive bends the lower link further forward
//
// Steps:
//   1. CAL_CENTER  before fitting horns/links. Drives every servo to position 500 (mid travel)
//                  and holds it. Fit each horn so its joint is as close to the zero pose as the
//                  spline allows (the spline only gives you steps of ~14 deg; that's fine).
//   2. CAL_READ    torque off. Prints raw position and calibrated angle of every joint.
//                  Hold the leg in the zero pose, copy the raw positions into joint_cal[].zero,
//                  then turn each joint positive and flip joint_cal[].sign where the angle drops.
//                  Reflash and repeat until the zero pose reads ~0 and directions are right.
//   3. CAL_CHECK   torque on. Puts the legs in the zero pose, then moves one joint at a time to
//                  +CHECK_DEG and back, logging which joint should move which way.
//   CAL_SLIDER     torque on, servos hold where they are. Move them live from your computer:
//                  close idf.py monitor, then run  sim/.venv/bin/python sim/servo_sliders.py
//                  Serial protocol (USB console, 115200, one command per line):
//                    L                 -> one "J <id> <pos> <zero> <name>" line per connected
//                                         servo (<zero> = its 0 deg position from joints.c)
//                    M <id> <pos> [ms] -> move servo <id> to <pos> (0..1000) over ms (default 100)
//                  and it prints "P <id>=<pos> ..." with the read-back positions 4x a second.

typedef enum {
    CAL_CENTER,
    CAL_READ,
    CAL_CHECK,
    CAL_SLIDER,
} cal_mode_t;

void servo_calibrate_run(cal_mode_t mode);
