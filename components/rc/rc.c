#include "rc.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "pins.h"

#define RC_UART         UART_NUM_1
#define RC_TX_GPIO      PIN_RC_TX   // -> receiver RX
#define RC_RX_GPIO      PIN_RC_RX   // <- receiver TX
#define RC_BAUD         420000  // ExpressLRS default for CRSF

#define RC_TIMEOUT_MS   250     // no packet for this long = disconnected

#define RC_TASK_STACK   4096
#define RC_TASK_PRIO    5
#define RC_TASK_CORE    1

// CRSF frame: [addr] [len] [type] [payload ...] [crc], len counts type + payload + crc
#define CRSF_ADDR_FC        0xC8
#define CRSF_ADDR_ALT       0xEE
#define CRSF_TYPE_CHANNELS  0x16    // 16 channels x 11 bits = 22 payload bytes
#define CRSF_MAX_FRAME      64

static const char *TAG = "RC";

static rc_state_t s_state;
static TickType_t s_last_packet;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

// CRC-8, poly 0xD5, over type + payload
static uint8_t crsf_crc(const uint8_t *p, int n)
{
    uint8_t crc = 0;
    for (int i = 0; i < n; i++) {
        crc ^= p[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static void handle_frame(const uint8_t *f)
{
    uint8_t len = f[1];
    if (crsf_crc(&f[2], len - 1) != f[len + 1]) return;
    if (f[2] != CRSF_TYPE_CHANNELS || len != 24) return;

    // Unpack 16 x 11-bit channels, little-endian bit order
    uint16_t ch[RC_NUM_CHANNELS];
    const uint8_t *p = &f[3];
    uint32_t bits = 0;
    int nbits = 0, c = 0;
    for (int i = 0; i < 22 && c < RC_NUM_CHANNELS; i++) {
        bits |= (uint32_t)p[i] << nbits;
        nbits += 8;
        while (nbits >= 11 && c < RC_NUM_CHANNELS) {
            ch[c++] = bits & 0x7FF;
            bits >>= 11;
            nbits -= 11;
        }
    }

    portENTER_CRITICAL(&s_lock);
    for (int i = 0; i < RC_NUM_CHANNELS; i++) s_state.ch[i] = ch[i];
    s_state.packets++;
    s_last_packet = xTaskGetTickCount();
    portEXIT_CRITICAL(&s_lock);
}

static void rc_task(void *arg)
{
    const uart_config_t cfg = {
        .baud_rate = RC_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(RC_UART, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(RC_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(RC_UART, RC_TX_GPIO, RC_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "Listening for CRSF on GPIO%d @ %d baud", RC_RX_GPIO, RC_BAUD);

    uint8_t buf[128];
    uint8_t frame[CRSF_MAX_FRAME];
    int pos = 0, total = 0;
    while (1) {
        int n = uart_read_bytes(RC_UART, buf, sizeof(buf), pdMS_TO_TICKS(20));
        for (int i = 0; i < n; i++) {
            uint8_t b = buf[i];
            if (pos == 0) {                     // wait for a frame start
                if (b == CRSF_ADDR_FC || b == CRSF_ADDR_ALT) frame[pos++] = b;
            } else if (pos == 1) {              // length
                if (b < 2 || b > CRSF_MAX_FRAME - 2) { pos = 0; continue; }
                frame[pos++] = b;
                total = b + 2;
            } else {
                frame[pos++] = b;
                if (pos == total) {
                    handle_frame(frame);
                    pos = 0;
                }
            }
        }
    }
}

void rc_start(void)
{
    xTaskCreatePinnedToCore(rc_task, "rc", RC_TASK_STACK, NULL, RC_TASK_PRIO, NULL, RC_TASK_CORE);
}

void rc_get(rc_state_t *out)
{
    portENTER_CRITICAL(&s_lock);
    *out = s_state;
    TickType_t last = s_last_packet;
    portEXIT_CRITICAL(&s_lock);
    out->connected = out->packets > 0 && (xTaskGetTickCount() - last) < pdMS_TO_TICKS(RC_TIMEOUT_MS);
}
