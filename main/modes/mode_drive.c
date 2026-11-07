// MODE_DRIVE: the normal firmware. IMU + controller, with the CH5 kill switch over everything:
//   CH2 (right stick up/down) -> both drive motors (components/drive)
//   CH3 (left stick up/down)  -> all leg joints, IDs 1-6, +/-45 deg (components/leg_control)

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "modes.h"
#include "imu.h"
#include "rc.h"
#include "drive.h"
#include "leg_control.h"

#define PRINT_PERIOD_MS 200
#define PRINT_CHANNELS  8       // T8L has 8 channels

void mode_drive_run(void)
{
    drive_start();      // first, so the ESCs see neutral as early as possible
    imu_start();
    rc_start();
    leg_control_start();

    while (1) {
        print_imu();
        print_drive();
        print_legs();
        print_rc(PRINT_CHANNELS);
        printf("\n");
        vTaskDelay(pdMS_TO_TICKS(PRINT_PERIOD_MS));
    }
}
