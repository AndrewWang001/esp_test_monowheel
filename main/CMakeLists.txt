idf_component_register(SRCS "main.c" "imu.c" "rc.c" "esc.c" "motor_test.c"
                            # Kept for later (compiled, but not started in main.c right now):
                            "rc_drive.c" "arm_test.c"
                            "joints.c" "servo.c" "servo_calibrate.c" "servo_set_id.c"
                    INCLUDE_DIRS ".")
# The BNO085 library (rinku404/bno085) comes in through idf_component.yml
