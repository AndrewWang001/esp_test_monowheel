# Standalone servo calibration firmware. Flash this to calibrate the leg servos, then flash
# the main project again. Uses the servo code and calibration table from ../main.
cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS ../components)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(servo_cal)
