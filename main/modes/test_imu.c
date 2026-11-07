// MODE_TEST_IMU: BNO085 only. Prints roll / pitch / yaw and gyro 5x a second.

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "modes.h"
#include "imu.h"

#define PRINT_PERIOD_MS 200

void test_imu_run(void)
{
    imu_start();
    while (1) {
        imu_state_t s;
        imu_get(&s);
        if (s.valid) {
            printf("roll %7.2f  pitch %7.2f  yaw %7.2f | gyro %7.1f %7.1f %7.1f dps | acc %u\n",
                   s.roll, s.pitch, s.yaw, s.gyro[0], s.gyro[1], s.gyro[2], s.accuracy);
        } else {
            printf("IMU waiting for first reading\n");
        }
        vTaskDelay(pdMS_TO_TICKS(PRINT_PERIOD_MS));
    }
}
