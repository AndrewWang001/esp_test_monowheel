// MODE_DRIVE: the normal firmware. IMU + controller + drive motor from the T8L (kill switch).
// Leg servos are not used yet.

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "modes.h"
#include "imu.h"
#include "rc.h"
#include "drive.h"

#define PRINT_PERIOD_MS 200
#define PRINT_CHANNELS  8       // T8L has 8 channels

void mode_drive_run(void)
{
    drive_start();      // first, so the ESC sees neutral as early as possible
    imu_start();
    rc_start();

    while (1) {
        print_imu();
        print_drive();
        print_rc(PRINT_CHANNELS);
        printf("\n");
        vTaskDelay(pdMS_TO_TICKS(PRINT_PERIOD_MS));
    }
}
