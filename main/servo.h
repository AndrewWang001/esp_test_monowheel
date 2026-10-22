#pragma once

// Hiwonder serial bus servo driver (HTD-45H), per "Hiwonder Bus Servo Communication Protocol".
// 115200 8N1. Frame: 0x55 0x55 ID LEN CMD PARAM... CHK, LEN = params + 3,
// CHK = ~(ID + LEN + CMD + params). The BusLinker board turns our TX/RX into the servo's
// single-wire half-duplex bus. Not thread-safe: call from one task only.

#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"

#define SERVO_UART    UART_NUM_2
#define SERVO_TX_GPIO GPIO_NUM_17 // -> BusLinker RX
#define SERVO_RX_GPIO GPIO_NUM_16 // <- BusLinker TX

#define SERVO_BROADCAST_ID 0xFE // all servos obey; only ID read replies, and only with ONE servo on the bus

#define SERVO_POS_MAX   1000    // 0..1000 = 0..240 deg (0.24 deg per unit)
#define SERVO_SPEED_MAX 1000    // motor mode, sign = direction

esp_err_t servo_bus_init(uart_port_t port, int tx_gpio, int rx_gpio);

// Position mode: move to pos (0..1000) at uniform speed over time_ms (0..30000)
esp_err_t servo_move(uint8_t id, uint16_t pos, uint16_t time_ms);
esp_err_t servo_stop(uint8_t id);

// Motor mode: continuous rotation at speed (-1000..1000). Not saved across power-off.
esp_err_t servo_set_motor_mode(uint8_t id, int16_t speed);
esp_err_t servo_set_servo_mode(uint8_t id);

// Torque on (holds / drives) or off (free to turn by hand)
esp_err_t servo_set_torque(uint8_t id, bool on);

// Note: the servo saves this setting to its non-volatile memory on every write
esp_err_t servo_set_led(uint8_t id, bool on);

// Saved across power-off. Every servo currently at id takes new_id, so connect only one.
esp_err_t servo_write_id(uint8_t id, uint8_t new_id);
esp_err_t servo_read_id(uint8_t id, uint8_t *out);
esp_err_t servo_read_pos(uint8_t id, int16_t *pos);
esp_err_t servo_read_vin(uint8_t id, uint16_t *mv);
esp_err_t servo_read_temp(uint8_t id, uint8_t *deg_c);
esp_err_t servo_read_vin_limit(uint8_t id, uint16_t *min_mv, uint16_t *max_mv);
esp_err_t servo_write_vin_limit(uint8_t id, uint16_t min_mv, uint16_t max_mv);
