#include "servo_task.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "servo.h"

#define SERVO_TASK_STACK  4096
#define SERVO_TASK_PRIO   4           // below the IMU task
#define SERVO_TASK_CORE   0

#define SERVO_1_ID        1           // spins continuously
#define SERVO_2_ID        2           // alternates spin / stop

#define LOOP_PERIOD_MS    100
#define TEST_SPEED        500         // half of SERVO_SPEED_MAX
#define TOGGLE_MS         3000        // servo 2 switches between spinning and stopped
#define STATUS_EVERY_MS   1000        // log + re-send motor mode (it is lost if a servo browns out)

#define VIN_MIN_MV        9000        // HTD-45H runs 9.0-12.6 V
#define VIN_MAX_MV        12600       // full 3S pack

static const char *TAG = "SERVO";

// Block until every servo answers on its own ID
static void wait_for_servos(const uint8_t *ids, int n)
{
    while (1) {
        bool all = true;
        for (int i = 0; i < n; i++) {
            uint8_t got;
            if (servo_read_id(ids[i], &got) != ESP_OK) {
                ESP_LOGW(TAG, "Servo ID %u not replying. Check: power (PW LED on), jumpers on TTL+Servo, "
                              "GPIO%d->RX, GPIO%d<-TX, GND shared, IDs set",
                         ids[i], SERVO_TX_GPIO, SERVO_RX_GPIO);
                all = false;
            }
        }
        if (all) break;
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    for (int i = 0; i < n; i++) ESP_LOGI(TAG, "Found servo ID %u", ids[i]);
}

// The protocol doc's default max is 12.0 V; above it the servo cuts torque. A full 3S is 12.6 V.
static void check_vin_limit(uint8_t id)
{
    uint16_t lo, hi;
    if (servo_read_vin_limit(id, &lo, &hi) != ESP_OK) {
        ESP_LOGW(TAG, "Could not read voltage limit");
        return;
    }
    ESP_LOGI(TAG, "Voltage limit: %.2f - %.2f V", lo / 1000.0f, hi / 1000.0f);
    if (hi < VIN_MAX_MV) {
        ESP_LOGW(TAG, "Max limit is below a full 3S pack (12.6 V), raising it");
        servo_write_vin_limit(id, lo, VIN_MAX_MV);
    }
}

static void log_status(uint8_t id)
{
    int16_t pos;
    uint16_t mv;
    uint8_t temp;
    if (servo_read_pos(id, &pos) == ESP_OK && servo_read_vin(id, &mv) == ESP_OK &&
        servo_read_temp(id, &temp) == ESP_OK) {
        ESP_LOGI(TAG, "ID %u: pos=%4d  vin=%5.2f V  temp=%u C", id, pos, mv / 1000.0f, temp);
        if (mv < VIN_MIN_MV) ESP_LOGW(TAG, "ID %u: supply below 9 V, servo may not turn", id);
    } else {
        ESP_LOGW(TAG, "ID %u not replying", id);
    }
}

static void servo_task(void *arg)
{
    static const uint8_t ids[] = {SERVO_1_ID, SERVO_2_ID};

    ESP_ERROR_CHECK(servo_bus_init(SERVO_UART, SERVO_TX_GPIO, SERVO_RX_GPIO));
    wait_for_servos(ids, 2);
    for (int i = 0; i < 2; i++) {
        check_vin_limit(ids[i]);
        servo_set_torque(ids[i], true);
    }

    ESP_LOGI(TAG, "ID %u spins at %d; ID %u toggles spin/stop every %d ms",
             SERVO_1_ID, TEST_SPEED, SERVO_2_ID, TOGGLE_MS);

    bool s2_spinning = true;
    servo_set_motor_mode(SERVO_1_ID, TEST_SPEED);
    servo_set_motor_mode(SERVO_2_ID, TEST_SPEED);

    int t_ms = 0;
    TickType_t wake = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(LOOP_PERIOD_MS));
        t_ms += LOOP_PERIOD_MS;

        bool toggle = (t_ms % TOGGLE_MS == 0);
        if (toggle) {
            s2_spinning = !s2_spinning;
            ESP_LOGI(TAG, "ID %u %s", SERVO_2_ID, s2_spinning ? "spin" : "stop");
        }
        if (toggle || t_ms % STATUS_EVERY_MS == 0) {
            servo_set_motor_mode(SERVO_1_ID, TEST_SPEED);
            servo_set_motor_mode(SERVO_2_ID, s2_spinning ? TEST_SPEED : 0);
        }
        if (t_ms % STATUS_EVERY_MS == 0) {
            log_status(SERVO_1_ID);
            log_status(SERVO_2_ID);
        }
    }
}

void servo_task_start(void)
{
    xTaskCreatePinnedToCore(servo_task, "servo", SERVO_TASK_STACK, NULL, SERVO_TASK_PRIO, NULL, SERVO_TASK_CORE);
}
