#include "rc_drive.h"

#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esc.h"
#include "rc.h"

#define THROTTLE_CH       1       // CH2 (index 1): right stick up/down, springs back to centre
#define ARM_CH            4       // CH5 (index 4): arm switch, on = high
#define ARM_ON_ABOVE      1500    // CH5 above this = armed

#define DRIVE_LIMIT_PCT   25      // cap at 25% of full throttle while testing. Raise later.
#define STICK_DEADBAND    30      // +/- around RC_MID that counts as centred (stick jitter)

#define LOOP_MS           40      // 25 Hz. Keep >= 2 ESC PWM periods (40 ms), see esc_write_us()

#define DRIVE_TASK_STACK  3072
#define DRIVE_TASK_PRIO   5
#define DRIVE_TASK_CORE   0

static const char *TAG = "DRIVE";

static rc_drive_state_t s_state = {.pulse_us = ESC_US_NEUTRAL};
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

// Stick position -> ESC pulse. Centre = 1500, full stick = 1500 +/- 500 * DRIVE_LIMIT_PCT%.
static uint32_t stick_to_us(uint16_t ch)
{
    int off = (int)ch - RC_MID;
    if (abs(off) <= STICK_DEADBAND) return ESC_US_NEUTRAL;
    int span = RC_MAX - RC_MID;                       // 819
    if (off > span) off = span;
    if (off < -span) off = -span;
    int us_off = off * 500 / span * DRIVE_LIMIT_PCT / 100;
    return ESC_US_NEUTRAL + us_off;
}

static void drive_task(void *arg)
{
    esc_init();   // outputs neutral from the start so the ESC arms
    ESP_LOGI(TAG, "ESC on GPIO18. Arm with CH5, drive with CH2 (right stick up/down), limit %d%%",
             DRIVE_LIMIT_PCT);

    bool armed = false;
    TickType_t wake = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(LOOP_MS));

        rc_state_t rc;
        rc_get(&rc);
        bool switch_on = rc.connected && rc.ch[ARM_CH] > ARM_ON_ABOVE;
        uint32_t stick_us = rc.connected ? stick_to_us(rc.ch[THROTTLE_CH]) : ESC_US_NEUTRAL;

        if (!switch_on) {
            if (armed) ESP_LOGW(TAG, "%s", rc.connected ? "Disarmed" : "Radio lost, disarmed");
            armed = false;
        } else if (!armed) {
            // Only arm with the stick centred, so flipping the switch never makes it jump
            if (stick_us == ESC_US_NEUTRAL) {
                armed = true;
                ESP_LOGI(TAG, "Armed");
            }
        }

        uint32_t us = armed ? stick_us : ESC_US_NEUTRAL;
        esc_write_us(us);

        portENTER_CRITICAL(&s_lock);
        s_state.pulse_us = us;
        s_state.armed = armed;
        portEXIT_CRITICAL(&s_lock);
    }
}

void rc_drive_start(void)
{
    xTaskCreatePinnedToCore(drive_task, "drive", DRIVE_TASK_STACK, NULL, DRIVE_TASK_PRIO, NULL, DRIVE_TASK_CORE);
}

void rc_drive_get(rc_drive_state_t *out)
{
    portENTER_CRITICAL(&s_lock);
    *out = s_state;
    portEXIT_CRITICAL(&s_lock);
}
