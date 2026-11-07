#include "motor_test.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esc.h"

#define LOOP_MS          50      // ramp step. Keep >= 2 PWM periods (40 ms), see esc_write_us()

#define ARM_WAIT_MS      5000    // neutral at start so the ESC arms (plug the battery in by then)
#define SLOW_PCT         15      // "slow" speed for the forward / reverse steps
#define FAST_PCT         80      // top speed for the last step

#define SLOW_RAMP_MS     3000    // 0 -> SLOW_PCT and back
#define SLOW_HOLD_MS     3000
#define FAST_RAMP_MS     8000    // 0 -> FAST_PCT and back, gentle on the 35 A ESC
#define FAST_HOLD_MS     2000
#define PAUSE_MS         2000    // stopped between steps

#define MOTOR_TASK_STACK 3072
#define MOTOR_TASK_PRIO  5
#define MOTOR_TASK_CORE  0

static const char *TAG = "MOTOR";

static volatile int s_pct;   // current throttle, -100..100

static uint32_t pct_to_us(int pct)
{
    return ESC_US_NEUTRAL + pct * (ESC_US_MAX - ESC_US_NEUTRAL) / 100;
}

static void set_pct(int pct)
{
    s_pct = pct;
    esc_write_us(pct_to_us(pct));
}

// Move throttle linearly from 'from' to 'to' percent over ms
static void ramp(int from, int to, int ms)
{
    int steps = ms / LOOP_MS;
    TickType_t wake = xTaskGetTickCount();
    for (int i = 1; i <= steps; i++) {
        set_pct(from + (to - from) * i / steps);
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(LOOP_MS));
    }
    set_pct(to);
}

static void hold(int ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

static void motor_task(void *arg)
{
    esc_init();   // outputs neutral from the start
    set_pct(0);
    ESP_LOGI(TAG, "Neutral for %d s so the ESC can arm. Plug in the motor battery now.", ARM_WAIT_MS / 1000);
    hold(ARM_WAIT_MS);

    ESP_LOGI(TAG, "Step 1: forward, slowly up to %d%%", SLOW_PCT);
    ramp(0, SLOW_PCT, SLOW_RAMP_MS);
    hold(SLOW_HOLD_MS);
    ESP_LOGI(TAG, "Step 1: slowing to stop");
    ramp(SLOW_PCT, 0, SLOW_RAMP_MS);
    hold(PAUSE_MS);

    ESP_LOGI(TAG, "Step 2: reverse, slowly up to %d%%", SLOW_PCT);
    ramp(0, -SLOW_PCT, SLOW_RAMP_MS);
    hold(SLOW_HOLD_MS);
    ESP_LOGI(TAG, "Step 2: slowing to stop");
    ramp(-SLOW_PCT, 0, SLOW_RAMP_MS);
    hold(PAUSE_MS);

    ESP_LOGI(TAG, "Step 3: forward, slowly up to %d%%", FAST_PCT);
    ramp(0, FAST_PCT, FAST_RAMP_MS);
    hold(FAST_HOLD_MS);
    ESP_LOGI(TAG, "Step 3: slowing to stop");
    ramp(FAST_PCT, 0, FAST_RAMP_MS);

    ESP_LOGI(TAG, "Done. Motor stopped. Press EN to run again.");
    while (1) {
        set_pct(0);   // keep sending neutral
        hold(1000);
    }
}

void motor_test_start(void)
{
    xTaskCreatePinnedToCore(motor_task, "motor", MOTOR_TASK_STACK, NULL, MOTOR_TASK_PRIO, NULL, MOTOR_TASK_CORE);
}

uint32_t motor_test_get_us(void)
{
    return pct_to_us(s_pct);
}

int motor_test_get_pct(void)
{
    return s_pct;
}
