#pragma once

// RadioMaster RP2 (ExpressLRS) receiver over CRSF on UART1. Runs in its own task; any task can
// read the latest stick/switch positions with rc_get().
//
// Wiring: receiver TX -> PIN_RC_RX, receiver RX -> PIN_RC_TX (pins.h), 5V -> VIN, GND -> GND

#include <stdbool.h>
#include <stdint.h>

#define RC_NUM_CHANNELS 16
#define RC_MIN          172     // stick at one end
#define RC_MID          992     // centred
#define RC_MAX          1811    // other end

typedef struct {
    uint16_t ch[RC_NUM_CHANNELS];   // ch[0] = CH1 ... raw CRSF values, ~172..1811
    bool connected;                 // a channel packet arrived in the last RC_TIMEOUT_MS
    uint32_t packets;               // channel packets received since boot
} rc_state_t;

// Kill switch, shared by everything that moves (drive motors, legs)
#define RC_KILL_SWITCH_CH   4       // CH5 (index 4)
#define RC_KILL_SWITCH_RUN  1500    // above this = RUN (moving allowed), below = KILL

// True when the radio is connected and the kill switch is in RUN
static inline bool rc_run_allowed(const rc_state_t *rc)
{
    return rc->connected && rc->ch[RC_KILL_SWITCH_CH] > RC_KILL_SWITCH_RUN;
}

void rc_start(void);
void rc_get(rc_state_t *out);
