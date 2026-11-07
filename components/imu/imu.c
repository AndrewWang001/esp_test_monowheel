#include "imu.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "bno085.h"   // rinku404/bno085 library (managed component)
#include "pins.h"


// Our scan found the chip at 0x4A. (The library's docs say 0x28/0x29,
// but those are BNO055 addresses; the BNO085 uses 0x4A/0x4B.)
#define BNO085_ADDR     0x4A
// The BNO085 holds SCL low ("clock stretching") while it prepares data, sometimes
// for several ms. ESP-IDF's default stretch timeout on the ESP32 is only 2 ms, which
// aborts transfers mid-way and leaves the bus stuck. Use the hardware maximum
// (~13 ms on the ESP32; the driver rounds down) and a gentler 100 kHz clock.
#define I2C_FREQ_HZ     100000
#define I2C_STRETCH_US  13000

#define RV_REPORT_US    10000   // orientation every 10 ms (100 Hz)
#define GYRO_REPORT_US  20000   // gyro every 20 ms (50 Hz)
#define RAD_TO_DEG      57.29578f

#define IMU_TASK_STACK  4096
#define IMU_TASK_PRIO   6       // above the arm task
#define IMU_TASK_CORE   1

static const char *TAG = "IMU";

static imu_state_t s_state;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

// Called by bno085_service() each time a sensor report arrives
static void on_sensor(bno085_handle_t handle, const bno085_sensor_value_t *v, void *ctx)
{
    if (v->sensor_id == BNO085_SENSOR_GAME_ROTATION_VECTOR) {
        float roll, pitch, yaw;
        bno085_quaternion_to_euler(v->data.game_rotation_vector.i, v->data.game_rotation_vector.j,
                                   v->data.game_rotation_vector.k, v->data.game_rotation_vector.real,
                                   &roll, &pitch, &yaw);
        portENTER_CRITICAL(&s_lock);
        s_state.roll = roll * RAD_TO_DEG;
        s_state.pitch = pitch * RAD_TO_DEG;
        s_state.yaw = yaw * RAD_TO_DEG;
        s_state.accuracy = v->status;
        s_state.valid = true;
        portEXIT_CRITICAL(&s_lock);
    } else if (v->sensor_id == BNO085_SENSOR_GYROSCOPE_CALIBRATED) {
        portENTER_CRITICAL(&s_lock);
        s_state.gyro[0] = v->data.gyroscope_calibrated.x * RAD_TO_DEG;
        s_state.gyro[1] = v->data.gyroscope_calibrated.y * RAD_TO_DEG;
        s_state.gyro[2] = v->data.gyroscope_calibrated.z * RAD_TO_DEG;
        portEXIT_CRITICAL(&s_lock);
    }
}

static void imu_task(void *arg)
{
    // I2C bus and device
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PIN_IMU_SDA,
        .scl_io_num = PIN_IMU_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BNO085_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
        .scl_wait_us = I2C_STRETCH_US,
    };
    i2c_master_dev_handle_t dev;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &dev));

    // Start the BNO085 (the library resets it via RST and sets up INT itself).
    // Keep retrying instead of crashing, so it still comes up if the IMU powers on late.
    bno085_config_t cfg;
    bno085_config_default(&cfg);
    bno085_handle_t bno;
    while (bno085_init(&cfg, dev, PIN_IMU_INT, PIN_IMU_RST, &bno) != ESP_OK) {
        ESP_LOGW(TAG, "BNO085 init failed, retrying. Check wiring (see imu.h)");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_ERROR_CHECK(bno085_register_sensor_callback(bno, on_sensor, NULL));
    ESP_ERROR_CHECK(bno085_enable_sensor(bno, BNO085_SENSOR_GAME_ROTATION_VECTOR, RV_REPORT_US));
    ESP_ERROR_CHECK(bno085_enable_sensor(bno, BNO085_SENSOR_GYROSCOPE_CALIBRATED, GYRO_REPORT_US));
    ESP_LOGI(TAG, "BNO085 running");

    // Keep pumping the library: it reads pending reports and calls on_sensor()
    while (1) {
        bno085_service(bno);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void imu_start(void)
{
    xTaskCreatePinnedToCore(imu_task, "imu", IMU_TASK_STACK, NULL, IMU_TASK_PRIO, NULL, IMU_TASK_CORE);
}

void imu_get(imu_state_t *out)
{
    portENTER_CRITICAL(&s_lock);
    *out = s_state;
    portEXIT_CRITICAL(&s_lock);
}
