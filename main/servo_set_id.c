#include "servo_set_id.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "servo.h"

static const char *TAG = "SERVO_SET_ID";

void servo_set_id_run(uint8_t old_id, uint8_t new_id)
{
    ESP_ERROR_CHECK(servo_bus_init(SERVO_UART, SERVO_TX_GPIO, SERVO_RX_GPIO));
    ESP_LOGW(TAG, "Renumbering ID %u -> %u. Only the servo being renumbered may be connected!", old_id, new_id);

    uint8_t id;
    if (servo_read_id(new_id, &id) == ESP_OK) {
        ESP_LOGI(TAG, "A servo already answers as ID %u, nothing to do", new_id);
        goto done;
    }

    while (servo_read_id(old_id, &id) != ESP_OK) {
        ESP_LOGW(TAG, "No servo answering as ID %u. Check power, wiring and that only one servo is plugged in", old_id);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    ESP_ERROR_CHECK(servo_write_id(old_id, new_id));
    vTaskDelay(pdMS_TO_TICKS(100)); // let it save to its non-volatile memory

    if (servo_read_id(new_id, &id) == ESP_OK && id == new_id) {
        ESP_LOGI(TAG, "Done: servo now answers as ID %u", new_id);
    } else {
        ESP_LOGE(TAG, "Servo did not answer as ID %u after the write. Power-cycle it and run again", new_id);
    }

done:
    ESP_LOGI(TAG, "Now restore app_main in main.c and connect both servos");
    while (1) vTaskDelay(portMAX_DELAY);
}
