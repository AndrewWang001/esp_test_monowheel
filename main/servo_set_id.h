#pragma once

#include <stdint.h>

// One-time tool: renumber the servo at old_id to new_id (saved permanently), verify, then idle.
// Connect ONLY the servo being renumbered: every servo at old_id would take the new ID.
void servo_set_id_run(uint8_t old_id, uint8_t new_id);
