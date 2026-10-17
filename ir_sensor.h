#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "motor_controller.h"

// Sets up the IR sensor pins and the ADC. Call once at start-up.
void ir_init(void);

// Line following + barcode scanning. Call as often as possible from the main loop.
void ir_update(MotorController *mc);

// Barcode scanning only (no line following, motors untouched). For the barcode test.
void ir_barcode_update(MotorController *mc);

// Returns true once for each newly decoded barcode and copies its text
// (without the * start/stop characters) into out, e.g. "A".
bool ir_get_barcode(char *out, size_t size);

// Latest raw reading of the barcode sensor (0..4095), for calibration.
uint16_t ir_barcode_raw(void);

#endif