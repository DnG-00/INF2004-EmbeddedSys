/** @file ir_sensor.h
 *
 * @brief Interface to the IR sensors: line following and Code 39 barcode
 *        scanning.
 */

#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "motor.h"

// Wheel speeds the line follower is asking for. This module never drives the
// motors itself: the motor control code decides whether to apply the request.
typedef struct
{
    int16_t left_pct;     // M1, -100 to 100.
    int16_t right_pct;    // M2, -100 to 100.
} ir_drive_request_t;

void     ir_sensor_init (motor_t const * p_motors);
bool     ir_sensor_update (ir_drive_request_t * p_request);
void     ir_sensor_update_barcode (void);
bool     ir_sensor_get_barcode (char * p_out, size_t size);
uint16_t ir_sensor_get_barcode_raw (void);

#endif /* IR_SENSOR_H */

/*** end of file ***/
