#include "imu.h"
#include "servo_task.h"
#include "servo_set_id.h"

void app_main(void)
{
    // Normal run (disabled while renumbering a servo):
     //imu_start();        // MPU-6050 on I2C (SDA 21, SCL 22), 100 Hz
     servo_task_start(); // HTD-45H via BusLinker on UART2 (TX 17, RX 16)

    // One-time: connect ONLY the second servo, flash, wait for "Done", then restore the lines above
    //servo_set_id_run(1, 2);
}
