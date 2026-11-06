#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu.h"
#include "rc.h"
#include "arm_test.h"

// Runs on its own at power-up: no laptop or serial commands needed. If USB is plugged in,
// idf.py monitor shows the IMU, the controller channels and the servo logs.

#define PRINT_PERIOD_MS 200
#define PRINT_CHANNELS  8       // T8L has 8 channels

void app_main(void)
{
    imu_start();        // BNO085 on I2C (SDA 21, SCL 22, INT 26, RST 32), see imu.h
    rc_start();         // ELRS receiver on UART1 (RX 27, TX 25), see rc.h
    arm_test_start();   // servos ID 1, 2, 3 on UART2 (TX 17, RX 16) via BusLinker

    while (1) {
        imu_state_t imu;
        rc_state_t rc;
        imu_get(&imu);
        rc_get(&rc);

        if (imu.valid) {
            printf("R%7.1f P%7.1f Y%7.1f | ", imu.roll, imu.pitch, imu.yaw);
        } else {
            printf("IMU waiting           | ");
        }
        if (rc.connected) {
            for (int i = 0; i < PRINT_CHANNELS; i++) printf("CH%d %4u ", i + 1, rc.ch[i]);
            printf("\n");
        } else {
            printf("RC no signal\n");
        }
        vTaskDelay(pdMS_TO_TICKS(PRINT_PERIOD_MS));
    }
}
