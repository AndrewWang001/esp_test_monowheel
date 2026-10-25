#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

// BNO085 in I2C mode. Wiring:
//   VCC -> 3V3, GND -> GND, PS0 + PS1 -> GND (I2C mode),
//   ADO -> GND (address 0x4A), CS -> 3V3
#define PIN_SDA     GPIO_NUM_21
#define PIN_SCL     GPIO_NUM_22
#define PIN_INT     GPIO_NUM_26
#define PIN_RST     GPIO_NUM_32

void app_main(void)
{
    gpio_set_direction(PIN_INT, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_INT, GPIO_PULLUP_ONLY);
    gpio_set_direction(PIN_RST, GPIO_MODE_OUTPUT);

    // Reset so the chip re-reads PS0/PS1 and ADO
    gpio_set_level(PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(PIN_RST, 1);

    // Give the chip time to boot before it will answer on I2C
    vTaskDelay(pdMS_TO_TICKS(200));

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PIN_SDA,
        .scl_io_num = PIN_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    // Scan every I2C address and report who answers
    int found = 0;
    for (int addr = 0x08; addr < 0x78; addr++) {
        if (i2c_master_probe(bus, addr, 50) == ESP_OK) {
            printf("Device answered at 0x%02X%s\n", addr,
                   (addr == 0x4A || addr == 0x4B) ? "  <- BNO085" : "");
            found++;
        }
    }
    printf("Scan done: %d device(s) found\n", found);
}
