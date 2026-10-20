/** @file main.c
 *
 * @brief Robo Pico car: sets up the motors, encoders, IR sensors and Wi-Fi
 *        logging, then runs line following or one of the test modes.
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "motor.h"
#include "ir_sensor.h"
#include "wifi_log.h"
#include "pico/cyw43_arch.h"// C's library that supports PICO W's in-built WIFI
#include "lwipopts.h"
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "mqtt_client.h" 
#include "mqtt_topics.h" // Header file that contains topics where devices can publish & subscribe

// Motor pin assignments for the Robo Pico's built-in motor driver terminals.
// Change these if the motor wiring on the board changes.
#define MOTOR1_A_PIN            8U
#define MOTOR1_B_PIN            9U
#define MOTOR2_A_PIN            10U
#define MOTOR2_B_PIN            11U

// Encoder A/B pin assignments. Change these if the encoder wiring changes.
// Red wire = A pin, green wire = B pin.
// Motor 2's encoder pins are swapped to match the physical orientation of the
// motors on the Robo Pico board; otherwise motor 2's direction would read
// reversed compared to motor 1.
#define MOTOR1_ENCODER_A_PIN    0U
#define MOTOR1_ENCODER_B_PIN    1U
#define MOTOR2_ENCODER_A_PIN    5U
#define MOTOR2_ENCODER_B_PIN    4U

// Wheel diameter in millimetres. Change this if different wheels are used.
#define WHEEL_DIAMETER_MM       62.0f

// MQTT Broker's credentials
#ifndef MQTT_BROKER_IP
#define MQTT_BROKER_IP ""
#endif
#define MQTT_BROKER_PORT 1883

// WIFI's credentials
#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#endif

// Set to 1 to run the motor/encoder demo instead of line following.
#define RUN_MOTOR_DEMO          0

// Set to 1 to test the barcode sensor on its own: motors stay off, push the
// car over a barcode by hand. Prints the raw sensor value with its min/max
// (use these to set BARCODE_EDGE_DELTA in ir_sensor.c) and every decoded
// barcode.
#define RUN_BARCODE_TEST        0

// Set to 1 to test barcode reading with the car really driving, but WITHOUT
// line following: after the start-up delay the car drives straight at a
// steady speed for DRIVE_TEST_MS, stops, and prints what it read. Press RESET
// to repeat. Line the car up so the right sensor will pass over the bars.
// Set all RUN_ options to 0 for normal line following.
#define RUN_BARCODE_DRIVE_TEST  1

// Percent; too low (e.g. 15) makes the motors stall and lurch.
#define DRIVE_TEST_SPEED        30

// How long to drive; shorten it if the car runs off the mat.
#define DRIVE_TEST_MS           3000U
#define DRIVE_TEST_US           (DRIVE_TEST_MS * 1000ULL)

// Keep reading this long after stopping, in case the barcode ended right as
// the car stopped.
#define DRIVE_TEST_SETTLE_MS    1500U
#define DRIVE_TEST_SETTLE_US    (DRIVE_TEST_SETTLE_MS * 1000ULL)

// Time to open the Serial Monitor and place the robot on the line.
#define STARTUP_DELAY_MS        3000U

// The barcode test prints 5 lines a second, which is enough to read.
#define BARCODE_TEST_PRINT_MS   200U
#define BARCODE_TEST_PRINT_US   (BARCODE_TEST_PRINT_MS * 1000U)

#define ADC_MAX_READING         4095U
#define BARCODE_TEXT_SIZE       24U
#define IDLE_SLEEP_MS           1000U

// Motor demo speeds (percent), turn sharpness (percent) and timings (ms).
#define DEMO_SPEED              50U
#define DEMO_GENTLE_TURN        30U
#define DEMO_SHARP_TURN         70U
#define DEMO_STRAIGHT_MS        2000U
#define DEMO_TURN_MS            1000U
#define DEMO_PAUSE_MS           1000U
#define DEMO_REPEAT_MS          3000U

static void run_line_following (motor_t * p_motors);
static void run_barcode_test (motor_t * p_motors);
static void run_barcode_drive_test (motor_t * p_motors);
static void run_motor_demo (motor_t * p_motors);
static void print_motor_readings (motor_t const * p_motors);
static void finish_demo_move (motor_t * p_motors, uint32_t move_ms);

/*!
 * @brief Sets everything up, then runs the selected mode forever.
 *
 * @return Never returns.
 */
int
main (void)
{
    stdio_init_all();

    while (!stdio_usb_connected()) {
        printf("The USB is not connected to the PICO W\n");
        sleep_ms(100);
    }

    printf("The USB is connected to the PICO W\n");
    sleep_ms(3000); 

    bool wifi_connect_result = wifi_init();

    while (true) {

        if (wifi_connect_result) {
            printf("Hello from PICO W via WIFI!\n");
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
            sleep_ms(1000);
        } else {
            printf("Goodbye!\n");
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
            sleep_ms(1000);
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
            sleep_ms(1000);
        }
    }

    // Set up the motors with their pins, encoders and wheel size.
    motor_t motors;

    motor_init(&motors, MOTOR1_A_PIN, MOTOR1_B_PIN, MOTOR2_A_PIN,
               MOTOR2_B_PIN);
    motor_begin(&motors);
    motor_attach_encoders(&motors,
                          MOTOR1_ENCODER_A_PIN, MOTOR1_ENCODER_B_PIN,
                          MOTOR2_ENCODER_A_PIN, MOTOR2_ENCODER_B_PIN);
    motor_set_wheel_geometry(&motors, WHEEL_DIAMETER_MM,
                             MOTOR_DEFAULT_PULSES_PER_WHEEL_REV);

    // Starts the barcode timer interrupt, which reads the encoders from now
    // on (motors lives for the whole program, as main() never returns).
    ir_sensor_init(&motors);

    sleep_ms(STARTUP_DELAY_MS);

#if RUN_MOTOR_DEMO
    run_motor_demo(&motors);
#elif RUN_BARCODE_TEST
    run_barcode_test(&motors);
#elif RUN_BARCODE_DRIVE_TEST
    run_barcode_drive_test(&motors);
#else
    run_line_following(&motors);
#endif

    return 0;
}

/*!
 * @brief Normal mode: follows the line and prints each barcode it reads.
 */
static void
run_line_following (motor_t * p_motors)
{
    char               code[BARCODE_TEXT_SIZE];
    ir_drive_request_t request;

    printf("Starting line following + barcode scanning\n");

    for (;;)
    {
        // The IR module only asks for wheel speeds; this loop is the one
        // place that drives the motors. It stands in for the Motor control
        // task, which will take over applying the request.
        if (ir_sensor_update(&request))
        {
            motor_set_wheel_speeds(p_motors, request.left_pct,
                                   request.right_pct);
        }

        if (ir_sensor_get_barcode(code, sizeof(code)))
        {
            printf("BARCODE: %s\n", code);
            // TODO (Vehicle controller): turn the barcode into a navigation
            // command here once the team agrees what each letter means.
        }
    }
}

/*!
 * @brief Barcode sensor test with the motors off.
 *
 * Push the car over a barcode by hand.
 */
static void
run_barcode_test (motor_t * p_motors)
{
    uint32_t lowest        = ADC_MAX_READING;
    uint32_t highest       = 0U;
    uint64_t next_print_us = 0U;
    char     code[BARCODE_TEXT_SIZE];

    motor_stop(p_motors);
    printf("Barcode test: push the car over a barcode by hand\n");

    for (;;)
    {
        ir_sensor_update_barcode();

        uint32_t reading = ir_sensor_get_barcode_raw();

        if (reading < lowest)
        {
            lowest = reading;
        }

        if (reading > highest)
        {
            highest = reading;
        }

        uint64_t now_us = time_us_64();

        if (now_us >= next_print_us)
        {
            printf("right sensor raw=%4" PRIu32 "   min=%4" PRIu32
                   "   max=%4" PRIu32 "\n", reading, lowest, highest);
            next_print_us = now_us + BARCODE_TEST_PRINT_US;
        }

        if (ir_sensor_get_barcode(code, sizeof(code)))
        {
            printf(">>> BARCODE: %s\n", code);
        }
    }
}

/*!
 * @brief Barcode test driving straight at a steady speed, no line following.
 */
static void
run_barcode_drive_test (motor_t * p_motors)
{
    char code[BARCODE_TEXT_SIZE];
    bool b_got = false;

    printf("Barcode drive test: driving straight for %u ms\n", DRIVE_TEST_MS);
    motor_set_wheel_speeds(p_motors, DRIVE_TEST_SPEED, DRIVE_TEST_SPEED);

    // Drive and read. The sensor is sampled by the timer interrupt; this loop
    // just decodes and collects the result.
    uint64_t stop_at_us = time_us_64() + DRIVE_TEST_US;

    while (time_us_64() < stop_at_us)
    {
        ir_sensor_update_barcode();

        if (!b_got)
        {
            b_got = ir_sensor_get_barcode(code, sizeof(code));
        }
    }

    motor_stop(p_motors);

    // Keep reading briefly in case the barcode ended right as the car stopped.
    uint64_t wait_until_us = time_us_64() + DRIVE_TEST_SETTLE_US;

    while (time_us_64() < wait_until_us)
    {
        ir_sensor_update_barcode();

        if (!b_got)
        {
            b_got = ir_sensor_get_barcode(code, sizeof(code));
        }
    }

    if (b_got)
    {
        printf(">>> BARCODE: %s\n", code);
    }
    else
    {
        printf(">>> no barcode decoded (see the barcode: line above, "
               "if any)\n");
    }

    printf("Done. Press RESET to run again.\n");

    for (;;)
    {
        sleep_ms(IDLE_SLEEP_MS);
    }
}

/*!
 * @brief Drives through forward/backward/turn/spin moves forever, printing
 *        encoder readings.
 */
static void
run_motor_demo (motor_t * p_motors)
{
    for (;;)
    {
        printf("Forward\n");
        motor_drive_forward(p_motors, DEMO_SPEED);
        sleep_ms(DEMO_STRAIGHT_MS);
        motor_stop(p_motors);
        print_motor_readings(p_motors);
        sleep_ms(DEMO_PAUSE_MS);

        printf("Backward\n");
        motor_drive_backward(p_motors, DEMO_SPEED);
        sleep_ms(DEMO_STRAIGHT_MS);
        motor_stop(p_motors);
        print_motor_readings(p_motors);
        printf("Wi-Fi log dropped lines so far: %" PRIu32 "\n",
               wifi_log_get_dropped_count());
        sleep_ms(DEMO_PAUSE_MS);

        printf("Pivot gentle turn left\n");
        motor_turn_left(p_motors, DEMO_SPEED, DEMO_GENTLE_TURN);
        finish_demo_move(p_motors, DEMO_TURN_MS);

        printf("Pivot gentle turn right\n");
        motor_turn_right(p_motors, DEMO_SPEED, DEMO_GENTLE_TURN);
        finish_demo_move(p_motors, DEMO_TURN_MS);

        printf("Spin left\n");
        motor_spin_left(p_motors, DEMO_SPEED);
        finish_demo_move(p_motors, DEMO_TURN_MS);

        printf("Spin right\n");
        motor_spin_right(p_motors, DEMO_SPEED);
        finish_demo_move(p_motors, DEMO_TURN_MS);

        printf("Pivot sharp turn left\n");
        motor_turn_left(p_motors, DEMO_SPEED, DEMO_SHARP_TURN);
        finish_demo_move(p_motors, DEMO_TURN_MS);

        printf("Pivot sharp turn right\n");
        motor_turn_right(p_motors, DEMO_SPEED, DEMO_SHARP_TURN);
        finish_demo_move(p_motors, DEMO_TURN_MS);

        sleep_ms(DEMO_REPEAT_MS);
    }
}

/*!
 * @brief Prints both motors' pulse counts, distances and speeds.
 */
static void
print_motor_readings (motor_t const * p_motors)
{
    printf("Pulses -> M1: %" PRId32 "  M2: %" PRId32
           "  |  Distance(mm) -> M1: %.1f  M2: %.1f"
           "  |  Speed(mm/s) -> M1: %.1f  M2: %.1f\n",
           motor_get_m1_pulses(p_motors), motor_get_m2_pulses(p_motors),
           motor_get_m1_distance_mm(p_motors),
           motor_get_m2_distance_mm(p_motors),
           motor_get_m1_speed_mm_s(p_motors),
           motor_get_m2_speed_mm_s(p_motors));
}

/*!
 * @brief Lets the current demo move run, then stops and pauses.
 */
static void
finish_demo_move (motor_t * p_motors, uint32_t move_ms)
{
    sleep_ms(move_ms);
    motor_stop(p_motors);
    sleep_ms(DEMO_PAUSE_MS);
}

/*** end of file ***/

