/** @file encoder_test_main.c
 *
 * @brief Standalone encoder diagnostic tool.
 *
 * Flash this separately from the main program when you just want to check
 * whether the encoder wiring is good, without the rest of the firmware in the
 * way.
 *
 * Tracks all 4 encoder pins (GP0/GP1 for motor 1's A/B phase, GP4/GP5 for
 * motor 2's A/B phase) as INDEPENDENT counters -- not fused into per-motor
 * pairs like motor.c does -- so wiring faults and electrical crosstalk between
 * motors show up directly instead of being hidden.
 *
 * Phase 1 (passive): the motors are never driven by PWM. Spin each wheel BY
 * HAND and watch which pin(s) respond -- this is the cleanest possible test,
 * since there's no PWM switching noise at all to confuse the reading.
 * Phase 2 (driven): briefly drives motor 1 alone, then motor 2 alone, and
 * reports all 4 raw counts each time so you can compare against phase 1 and
 * see exactly which pins pick up noise from which motor.
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "motor.h"

#define M1_A_PIN              0U
#define M1_B_PIN              1U
#define M2_A_PIN              4U
#define M2_B_PIN              5U

#define MOTOR1_A_PIN          8U
#define MOTOR1_B_PIN          9U
#define MOTOR2_A_PIN          10U
#define MOTOR2_B_PIN          11U

#define STARTUP_DELAY_MS      2000U
#define PASSIVE_PRINTS        20U     // prints per hand-spun wheel
#define DRIVEN_PRINTS         6U      // prints per driven motor
#define PRINT_INTERVAL_MS     250U
#define MOTOR_SETTLE_MS       500U
#define FREE_RUN_INTERVAL_MS  500U
#define TEST_SPEED_PCT        50

static volatile uint32_t g_m1a_count = 0U;
static volatile uint32_t g_m1b_count = 0U;
static volatile uint32_t g_m2a_count = 0U;
static volatile uint32_t g_m2b_count = 0U;

static void count_edge_isr (uint gpio, uint32_t events);
static void init_encoder_pin (uint32_t pin, bool b_install_callback);
static void print_counts (void);
static void print_counts_repeatedly (uint32_t count);

/*!
 * @brief Runs the passive and driven encoder tests, then keeps printing.
 *
 * @return Never returns.
 */
int
main (void)
{
    stdio_init_all();

    init_encoder_pin(M1_A_PIN, true);
    init_encoder_pin(M1_B_PIN, false);
    init_encoder_pin(M2_A_PIN, false);
    init_encoder_pin(M2_B_PIN, false);

    sleep_ms(STARTUP_DELAY_MS);

    printf("--- Phase 1: passive -- spin each wheel BY HAND, motors are not "
           "driven ---\n");
    printf("Spin Motor 1's wheel now. Only GP0/GP1 should move.\n");
    print_counts_repeatedly(PASSIVE_PRINTS);

    printf("Now spin Motor 2's wheel. Only GP4/GP5 should move.\n");
    print_counts_repeatedly(PASSIVE_PRINTS);

    printf("--- Phase 2: driven -- motors will be briefly driven by PWM "
           "---\n");

    motor_t motors;

    motor_init(&motors, MOTOR1_A_PIN, MOTOR1_B_PIN, MOTOR2_A_PIN,
               MOTOR2_B_PIN);
    motor_begin(&motors);

    printf("Driving Motor 1 only...\n");
    motor_set_wheel_speeds(&motors, TEST_SPEED_PCT, 0);
    print_counts_repeatedly(DRIVEN_PRINTS);
    motor_set_wheel_speeds(&motors, 0, 0);
    sleep_ms(MOTOR_SETTLE_MS);

    printf("Driving Motor 2 only...\n");
    motor_set_wheel_speeds(&motors, 0, TEST_SPEED_PCT);
    print_counts_repeatedly(DRIVEN_PRINTS);
    motor_set_wheel_speeds(&motors, 0, 0);

    printf("--- Test done. Counts below are left free-running if you want "
           "to keep probing. ---\n");

    for (;;)
    {
        print_counts();
        sleep_ms(FREE_RUN_INTERVAL_MS);
    }

    return 0;
}

/*!
 * @brief Encoder interrupt: counts each falling edge on its own pin.
 *
 * The gpio parameter uses the SDK's uint type because the callback signature
 * (gpio_irq_callback_t) requires it.
 */
static void
count_edge_isr (uint gpio, uint32_t events)
{
    // Cast: events is required by the callback signature but not needed.
    (void)events;

    if (M1_A_PIN == gpio)
    {
        g_m1a_count++;
    }
    else if (M1_B_PIN == gpio)
    {
        g_m1b_count++;
    }
    else if (M2_A_PIN == gpio)
    {
        g_m2a_count++;
    }
    else if (M2_B_PIN == gpio)
    {
        g_m2b_count++;
    }
    else
    {
        // Not one of the encoder pins.
    }
}

/*!
 * @brief Sets up an encoder pin as a pulled-up input with a falling-edge
 *        interrupt, installing the callback on the first pin.
 */
static void
init_encoder_pin (uint32_t pin, bool b_install_callback)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);

    if (b_install_callback)
    {
        gpio_set_irq_enabled_with_callback(pin, GPIO_IRQ_EDGE_FALL, true,
                                           &count_edge_isr);
    }
    else
    {
        gpio_set_irq_enabled(pin, GPIO_IRQ_EDGE_FALL, true);
    }
}

/*!
 * @brief Prints all 4 raw edge counts on one line.
 */
static void
print_counts (void)
{
    printf("GP0(M1-A): %" PRIu32 "  GP1(M1-B): %" PRIu32
           "  GP4(M2-A): %" PRIu32 "  GP5(M2-B): %" PRIu32 "\n",
           g_m1a_count, g_m1b_count, g_m2a_count, g_m2b_count);
}

/*!
 * @brief Prints the counts the given number of times, PRINT_INTERVAL_MS
 *        apart.
 */
static void
print_counts_repeatedly (uint32_t count)
{
    for (uint32_t idx = 0U; idx < count; idx++)
    {
        print_counts();
        sleep_ms(PRINT_INTERVAL_MS);
    }
}

/*** end of file ***/

