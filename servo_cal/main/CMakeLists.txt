# Shares the driver, joint table and calibration tool with the main project, so a
# calibration saved in ../../main/joints.c is the one the main project uses
set(APP_MAIN ../../main)

idf_component_register(SRCS "cal_main.c"
                            "${APP_MAIN}/servo.c"
                            "${APP_MAIN}/joints.c"
                            "${APP_MAIN}/servo_calibrate.c"
                       INCLUDE_DIRS "." "${APP_MAIN}"
                       REQUIRES driver esp_timer gait arm_kinematics)
