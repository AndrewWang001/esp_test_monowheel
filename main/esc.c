#include "esc.h"

#include "driver/ledc.h"
#include "esp_err.h"

#define ESC_GPIO      18
#define ESC_FREQ_HZ   50
#define ESC_PERIOD_US (1000000 / ESC_FREQ_HZ)   // 20000 us
#define ESC_BITS      LEDC_TIMER_14_BIT
#define ESC_DUTY_MAX  (1 << 14)

#define ESC_MODE      LEDC_LOW_SPEED_MODE
#define ESC_TIMER     LEDC_TIMER_0
#define ESC_CHANNEL   LEDC_CHANNEL_0

static uint32_t us_to_duty(uint32_t us)
{
    return us * ESC_DUTY_MAX / ESC_PERIOD_US;
}

void esc_init(void)
{
    const ledc_timer_config_t timer = {
        .speed_mode = ESC_MODE,
        .timer_num = ESC_TIMER,
        .duty_resolution = ESC_BITS,
        .freq_hz = ESC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    const ledc_channel_config_t ch = {
        .gpio_num = ESC_GPIO,
        .speed_mode = ESC_MODE,
        .channel = ESC_CHANNEL,
        .timer_sel = ESC_TIMER,
        .duty = us_to_duty(ESC_US_NEUTRAL),   // ESC arms on neutral
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch));
}

// Note: ledc_update_duty() busy-waits (interrupts off) until the previous change has taken
// effect, which happens at the next PWM period (20 ms at 50 Hz). So don't call this faster
// than every ~2 periods, and skip it when nothing changed, or it hogs the CPU.
void esc_write_us(uint32_t us)
{
    static uint32_t last_duty = UINT32_MAX;
    if (us < ESC_US_MIN) us = ESC_US_MIN;
    if (us > ESC_US_MAX) us = ESC_US_MAX;
    uint32_t duty = us_to_duty(us);
    if (duty == last_duty) return;
    last_duty = duty;
    ledc_set_duty(ESC_MODE, ESC_CHANNEL, duty);
    ledc_update_duty(ESC_MODE, ESC_CHANNEL);
}
