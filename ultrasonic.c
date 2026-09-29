#include "ultrasonic.h"

#include <stdio.h>

#include "pico/stdlib.h"

static uint trig_pin;
static uint echo_pin;

void ultrasonic_init(uint trig, uint echo)
{
    trig_pin = trig;
    echo_pin = echo;

    gpio_init(trig_pin);
    gpio_set_dir(trig_pin, GPIO_OUT);
    gpio_put(trig_pin, 0);

    gpio_init(echo_pin);
    gpio_set_dir(echo_pin, GPIO_IN);
}

float ultrasonic_read_cm(void)
{
    // Make sure TRIG starts LOW
    gpio_put(trig_pin, 0);
    sleep_us(2);

    // Send a 10 us trigger pulse
    gpio_put(trig_pin, 1);
    sleep_us(10);
    gpio_put(trig_pin, 0);

    // Wait for ECHO to go HIGH
    uint32_t timeout = 30000;
    while (!gpio_get(echo_pin) && timeout > 0) {
        sleep_us(1);
        timeout--;
    }

    if (timeout == 0) {
        return -1.0f;
    }

    // Measure how long ECHO stays HIGH
    absolute_time_t start = get_absolute_time();

    timeout = 30000;

    while (gpio_get(echo_pin) && timeout > 0) {
        sleep_us(1);
        timeout--;
    }

    if (timeout == 0) {
        return -1.0f;
    }

    int64_t pulse_us = absolute_time_diff_us(start, get_absolute_time());

    // Speed of sound ≈ 0.0343 cm/us
    // Divide by 2 because the sound travels TO the object and BACK.
    float distance_cm = (pulse_us * 0.0343f) / 2.0f;

    return distance_cm;
}