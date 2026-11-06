#include "arm_test.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "servo.h"
#include "joints.h"

#define ARM_TASK_STACK  4096
#define ARM_TASK_PRIO   4             // below the BNO085 task
#define ARM_TASK_CORE   0

#define SWEEP_DEG       45.0f         // each side of zero, so 90 deg total
#define SWEEP_MS        2000          // time for one side-to-side move
#define HOME_MS         1500          // first move to zero after power-up

static const char *TAG = "ARM";

static const gait_leg_t LEG = LEG_BACK;   // joint_cal row holding servo IDs 1, 2, 3

// Asks every possible ID once and logs who answers (~13 s). Tells apart "servos have
// different IDs" from "no servo can hear us / answer at all" (power or wiring).
static void scan_ids(void)
{
    ESP_LOGW(TAG, "Scanning all servo IDs 0..253 ...");
    int found = 0;
    for (int id = 0; id < SERVO_BROADCAST_ID; id++) {
        uint8_t got;
        if (servo_read_id(id, &got) == ESP_OK) {
            ESP_LOGW(TAG, "  servo answered at ID %d", id);
            found++;
        }
    }
    if (found == 0) {
        ESP_LOGW(TAG, "No servo answered at any ID. Check: servo supply (9-12.6 V) on the "
                      "BusLinker and switched on, GND shared with ESP, GPIO%d -> BusLinker RX, "
                      "GPIO%d <- BusLinker TX, BusLinker jumpers on TTL",
                 SERVO_TX_GPIO, SERVO_RX_GPIO);
    }
}

// Waits until every servo on the leg answers. Runs forever if one never shows up, so the
// ESP can be powered before the servos and still start once they come up.
static void wait_for_servos(void)
{
    int rounds = 0;
    while (1) {
        if (++rounds == 4) scan_ids();
        bool all = true;
        for (int j = 0; j < JOINTS_PER_LEG; j++) {
            uint8_t got;
            if (servo_read_id(joint_cal[LEG][j].id, &got) != ESP_OK) {
                ESP_LOGW(TAG, "Servo ID %u (%s) not replying", joint_cal[LEG][j].id, joint_name(LEG, j));
                all = false;
            }
        }
        if (all) return;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void move_all(float deg, uint16_t time_ms)
{
    for (int j = 0; j < JOINTS_PER_LEG; j++) {
        servo_move(joint_cal[LEG][j].id, joint_deg_to_pos(LEG, j, deg), time_ms);
    }
}

static void arm_task(void *arg)
{
    ESP_ERROR_CHECK(servo_bus_init(SERVO_UART, SERVO_TX_GPIO, SERVO_RX_GPIO));
    wait_for_servos();

    for (int j = 0; j < JOINTS_PER_LEG; j++) {
        uint8_t id = joint_cal[LEG][j].id;
        servo_set_servo_mode(id);   // in case it was left in motor (spin) mode
        servo_set_torque(id, true);
        ESP_LOGI(TAG, "Servo ID %u (%s) ready, zero = %d", id, joint_name(LEG, j), joint_cal[LEG][j].zero);
    }

    move_all(0, HOME_MS);
    vTaskDelay(pdMS_TO_TICKS(HOME_MS + 500));
    ESP_LOGI(TAG, "Sweeping +/-%.0f deg, %d ms per side", SWEEP_DEG, SWEEP_MS);

    float target = SWEEP_DEG;
    while (1) {
        move_all(target, SWEEP_MS);
        vTaskDelay(pdMS_TO_TICKS(SWEEP_MS));
        target = -target;
    }
}

void arm_test_start(void)
{
    xTaskCreatePinnedToCore(arm_task, "arm", ARM_TASK_STACK, NULL, ARM_TASK_PRIO, NULL, ARM_TASK_CORE);
}
