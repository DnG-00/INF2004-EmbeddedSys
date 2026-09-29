#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdbool.h>

void ultrasonic_init(uint trig_pin, uint echo_pin);

float ultrasonic_read_cm(void);

#endif