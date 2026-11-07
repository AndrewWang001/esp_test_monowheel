#pragma once

// Every GPIO the robot uses, in one place (ESP32-WROOM-32 DevKit V1).
// Before using a new pin, check it isn't here. Don't use these:
//   GPIO 1, 3   TX0/RX0: USB serial (flashing + idf.py monitor)
//   GPIO 6-11   on-module flash
//   GPIO 0, 2, 5, 12, 15  boot strapping pins (avoid unless you know the boot-level rules)

// BNO085 IMU, I2C (components/imu)
#define PIN_IMU_SDA     21
#define PIN_IMU_SCL     22
#define PIN_IMU_INT     26
#define PIN_IMU_RST     32

// Leg servos: Hiwonder bus via BusLinker, UART2 (components/servo_bus)
#define PIN_SERVO_TX    17      // -> BusLinker RX
#define PIN_SERVO_RX    16      // <- BusLinker TX

// ELRS receiver (RadioMaster RP2), CRSF on UART1 (components/rc)
#define PIN_RC_RX       27      // <- receiver TX
#define PIN_RC_TX       25      // -> receiver RX

// Drive motor ESCs (Repeat AM32), servo-style PWM (components/esc). Both get the same signal.
#define PIN_ESC         18      // drive motor 1
#define PIN_ESC2        19      // drive motor 2
