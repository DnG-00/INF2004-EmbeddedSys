// Standalone encoder diagnostic tool -- flash this separately from MotorC
// when you just want to check whether the encoder wiring is actually good,
// without the rest of the motor-demo firmware in the way.
//
// Tracks all 4 encoder pins (GP0/GP1 for Motor 1's A/B phase, GP4/GP5 for
// Motor 2's A/B phase) as INDEPENDENT counters -- not fused into per-motor
// pairs like motor_controller.c does -- so wiring faults and electrical
// crosstalk between motors show up directly instead of being hidden.
//
// Phase 1 (passive): motors are never driven by PWM. Spin each wheel BY
// HAND and watch which pin(s) respond -- this is the cleanest possible test
// since there's no PWM switching noise at all to confuse the reading.
// Phase 2 (driven): briefly drives Motor 1 alone, then Motor 2 alone, and
// reports all 4 raw counts each time so you can compare against phase 1 and
// see exactly which pins pick up noise from which motor.

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "motor_controller.h"

#define M1_A_PIN 0
#define M1_B_PIN 1
#define M2_A_PIN 4
#define M2_B_PIN 5

#define MOTOR1_A_PIN 8
#define MOTOR1_B_PIN 9
#define MOTOR2_A_PIN 10
#define MOTOR2_B_PIN 11

static volatile uint32_t s_m1a_count;
static volatile uint32_t s_m1b_count;
static volatile uint32_t s_m2a_count;
static volatile uint32_t s_m2b_count;

static void raw_isr(uint gpio, uint32_t events) {
    (void)events;
    if (gpio == M1_A_PIN) {
        s_m1a_count++;
    } else if (gpio == M1_B_PIN) {
        s_m1b_count++;
    } else if (gpio == M2_A_PIN) {
        s_m2a_count++;
    } else if (gpio == M2_B_PIN) {
        s_m2b_count++;
    }
}

static void init_encoder_pin(uint pin, bool install_callback) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
    if (install_callback) {
        gpio_set_irq_enabled_with_callback(pin, GPIO_IRQ_EDGE_FALL, true, &raw_isr);
    } else {
        gpio_set_irq_enabled(pin, GPIO_IRQ_EDGE_FALL, true);
    }
}

static void print_counts(void) {
    printf("GP0(M1-A): %u  GP1(M1-B): %u  GP4(M2-A): %u  GP5(M2-B): %u\n",
           s_m1a_count, s_m1b_count, s_m2a_count, s_m2b_count);
}

int main() {
    stdio_init_all();

    init_encoder_pin(M1_A_PIN, true);
    init_encoder_pin(M1_B_PIN, false);
    init_encoder_pin(M2_A_PIN, false);
    init_encoder_pin(M2_B_PIN, false);

    sleep_ms(2000);

    printf("--- Phase 1: passive -- spin each wheel BY HAND, motors are not driven ---\n");
    printf("Spin Motor 1's wheel now. Only GP0/GP1 should move.\n");
    for (int i = 0; i < 20; i++) {
        print_counts();
        sleep_ms(250);
    }

    printf("Now spin Motor 2's wheel. Only GP4/GP5 should move.\n");
    for (int i = 0; i < 20; i++) {
        print_counts();
        sleep_ms(250);
    }

    printf("--- Phase 2: driven -- motors will be briefly driven by PWM ---\n");
    MotorController motors;
    mc_init(&motors, MOTOR1_A_PIN, MOTOR1_B_PIN, MOTOR2_A_PIN, MOTOR2_B_PIN);
    mc_begin(&motors);

    printf("Driving Motor 1 only...\n");
    mc_set_wheel_speeds(&motors, 50, 0);
    for (int i = 0; i < 6; i++) {
        print_counts();
        sleep_ms(250);
    }
    mc_set_wheel_speeds(&motors, 0, 0);
    sleep_ms(500);

    printf("Driving Motor 2 only...\n");
    mc_set_wheel_speeds(&motors, 0, 50);
    for (int i = 0; i < 6; i++) {
        print_counts();
        sleep_ms(250);
    }
    mc_set_wheel_speeds(&motors, 0, 0);

    printf("--- Test done. Counts below are left free-running if you want to keep probing. ---\n");
    while (true) {
        print_counts();
        sleep_ms(500);
    }
}
