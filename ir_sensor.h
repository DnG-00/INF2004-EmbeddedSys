#pragma once

#include "motor_controller.h"

// IR sensors: line following (left + centre sensors) and Code 39 barcode
// scanning (right sensor). Drives the wheels through a MotorController.
// Usage:
//   ir_init();
//   while (true) ir_update(&motors);

// Configures the IR sensor pins and the ADC. Call once at startup.
void ir_init(void);

// Samples the barcode sensor and, every few milliseconds, steers the motors
// to follow the line. Decoded barcodes are printed with printf.
// Non-blocking -- call it as often as possible from the main loop.
void ir_update(MotorController *mc);
