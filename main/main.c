#include "imu.h"
#include "servo_task.h"

void app_main(void)
{
    imu_start();        // MPU-6050 on I2C (SDA 21, SCL 22), 100 Hz
    servo_task_start(); // HTD-45H via BusLinker on UART2 (TX 17, RX 16)
}
