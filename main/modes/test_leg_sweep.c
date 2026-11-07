// MODE_TEST_LEG_SWEEP: every leg servo in joint_cal (back leg IDs 1 2 3, front leg IDs 4 5 6) swings
// +/-45 deg around its zero in legs.c so you can see they are alive. Servos that don't answer are
// skipped and join in when they show up. Front leg zeros are still the placeholder 500.

#include "modes.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "servo.h"
#include "legs.h"

#define LEG_TASK_STACK  4096
#define LEG_TASK_PRIO   4             // below the IMU task
#define LEG_TASK_CORE   0

#define SWEEP_DEG       45.0f         // each side of zero, so 90 deg total
#define SWEEP_MS        2000          // time for one side-to-side move
#define HOME_MS         1500          // first move to zero after power-up
#define SCAN_AFTER      4             // half-swings with no servo at all before scanning every ID

static const char *TAG = "LEGS";

// Which servos in joint_cal (back leg IDs 1 2 3, front leg IDs 4 5 6) have answered
static bool s_active[NUM_LEGS][JOINTS_PER_LEG];
static bool s_warned[NUM_LEGS][JOINTS_PER_LEG];

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

// Pings every servo that hasn't answered yet and switches on the ones that do. Missing servos
// don't hold up the others: they join the sweep whenever they show up. Returns how many are active.
static int find_servos(void)
{
    int active = 0;
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        for (int j = 0; j < JOINTS_PER_LEG; j++) {
            uint8_t id = joint_cal[leg][j].id;
            if (!s_active[leg][j]) {
                uint8_t got;
                if (servo_read_id(id, &got) == ESP_OK) {
                    servo_set_servo_mode(id);   // in case it was left in motor (spin) mode
                    servo_set_torque(id, true);
                    s_active[leg][j] = true;
                    ESP_LOGI(TAG, "Servo ID %u (%s) found, zero = %d", id, joint_name(leg, j),
                             joint_cal[leg][j].zero);
                } else if (!s_warned[leg][j]) {
                    ESP_LOGW(TAG, "Servo ID %u (%s) not replying, will keep checking", id, joint_name(leg, j));
                    s_warned[leg][j] = true;
                }
            }
            if (s_active[leg][j]) active++;
        }
    }
    return active;
}

static void move_all(float deg, uint16_t time_ms)
{
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        for (int j = 0; j < JOINTS_PER_LEG; j++) {
            if (s_active[leg][j]) {
                servo_move(joint_cal[leg][j].id, joint_deg_to_pos(leg, j, deg), time_ms);
            }
        }
    }
}

static void leg_task(void *arg)
{
    ESP_ERROR_CHECK(servo_bus_init(SERVO_UART, SERVO_TX_GPIO, SERVO_RX_GPIO));

    // Wait for at least one servo, then home whatever answered
    int rounds = 0;
    while (find_servos() == 0) {
        if (++rounds == SCAN_AFTER) scan_ids();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    move_all(0, HOME_MS);
    vTaskDelay(pdMS_TO_TICKS(HOME_MS + 500));
    ESP_LOGI(TAG, "Sweeping +/-%.0f deg, %d ms per side", SWEEP_DEG, SWEEP_MS);

    float target = SWEEP_DEG;
    while (1) {
        find_servos();              // late servos join here (they move from wherever they are)
        move_all(target, SWEEP_MS);
        vTaskDelay(pdMS_TO_TICKS(SWEEP_MS));
        target = -target;
    }
}

void test_leg_sweep_run(void)
{
    xTaskCreatePinnedToCore(leg_task, "legs", LEG_TASK_STACK, NULL, LEG_TASK_PRIO, NULL, LEG_TASK_CORE);
}
