// MODE_TEST_RC: ELRS receiver only. Prints every channel 5x a second plus the packet count.
// Move one stick / switch at a time to see which channel it is (~172 .. 992 centre .. ~1811).

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "modes.h"
#include "rc.h"

#define PRINT_PERIOD_MS 200

void test_rc_run(void)
{
    rc_start();
    while (1) {
        rc_state_t rc;
        rc_get(&rc);
        if (rc.connected) {
            for (int i = 0; i < RC_NUM_CHANNELS; i++) printf("CH%-2d %4u ", i + 1, rc.ch[i]);
            printf("| packets %lu\n", (unsigned long)rc.packets);
        } else {
            printf("RC no signal (packets so far: %lu). Receiver LED solid? TX -> PIN_RC_RX?\n",
                   (unsigned long)rc.packets);
        }
        vTaskDelay(pdMS_TO_TICKS(PRINT_PERIOD_MS));
    }
}
