// ============================================================================================
//  MODE_TOOL_SERVO_SET_ID  --  DO NOT USE
//  The leg servos are already numbered 1-6. Renumbering one by mistake breaks the leg mapping
//  in components/legs/legs.c. Only use this to set up a NEW / replacement servo.
// ============================================================================================
//
// Gives the single servo on the bus a new ID (saved permanently in the servo).
// It is not in tools/pick_mode.py on purpose. To run it you must write main/mode_select.h by hand:
//
//     #define APP_MODE MODE_TOOL_SERVO_SET_ID
//     #define SERVO_SET_ID_TARGET 4                      // the ID it should get, 1..6
//     #define I_UNDERSTAND_THIS_RENUMBERS_SERVOS 1
//
// Safety: it scans every ID first and refuses unless EXACTLY ONE servo answers, so it can never
// renumber a servo that is still connected to the others. Unplug every other servo first.

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "modes.h"
#include "servo.h"
#if __has_include("mode_select.h")
#include "mode_select.h"
#endif

static const char *TAG = "SET_ID";

static void idle_forever(void)
{
    while (1) vTaskDelay(portMAX_DELAY);
}

void tool_servo_set_id_run(void)
{
#if !defined(I_UNDERSTAND_THIS_RENUMBERS_SERVOS) || !defined(SERVO_SET_ID_TARGET)
    ESP_LOGE(TAG, "Refusing: define SERVO_SET_ID_TARGET and I_UNDERSTAND_THIS_RENUMBERS_SERVOS in "
                  "main/mode_select.h (see tool_servo_set_id.c). Nothing was changed.");
    idle_forever();
#else
    const int target = SERVO_SET_ID_TARGET;
    if (target < 1 || target > 6) {
        ESP_LOGE(TAG, "Refusing: SERVO_SET_ID_TARGET must be 1..6 (got %d). Nothing was changed.", target);
        idle_forever();
    }

    ESP_ERROR_CHECK(servo_bus_init(SERVO_UART, SERVO_TX_GPIO, SERVO_RX_GPIO));
    ESP_LOGW(TAG, "Scanning all IDs (~13 s). Only the servo being renumbered may be connected!");

    int found = 0, found_id = -1;
    for (int id = 0; id < SERVO_BROADCAST_ID; id++) {
        uint8_t got;
        if (servo_read_id(id, &got) == ESP_OK) {
            ESP_LOGI(TAG, "  servo answered at ID %d", id);
            found++;
            found_id = id;
        }
    }

    if (found != 1) {
        ESP_LOGE(TAG, "Refusing: %d servos answered, need exactly 1. Unplug the others. Nothing was changed.", found);
        idle_forever();
    }
    if (found_id == target) {
        ESP_LOGI(TAG, "Servo is already ID %d. Nothing to do.", target);
        idle_forever();
    }

    ESP_LOGW(TAG, "Renumbering ID %d -> %d", found_id, target);
    ESP_ERROR_CHECK(servo_write_id(found_id, target));
    vTaskDelay(pdMS_TO_TICKS(100));   // let it save to its non-volatile memory

    uint8_t got;
    if (servo_read_id(target, &got) == ESP_OK && got == target) {
        ESP_LOGI(TAG, "Done: servo now answers as ID %d. Delete main/mode_select.h now.", target);
    } else {
        ESP_LOGE(TAG, "Servo did not answer as ID %d after the write. Power-cycle it and check.", target);
    }
    idle_forever();
#endif
}
