#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "motor_controller.h"
#include "ir_sensor.h"
#include "wifi_log.h"

// Motor pin assignments for the Robo Pico's built-in motor driver terminals. 
// Change accordingly if motor wiring on the board are changed
#define MOTOR1_A_PIN 8
#define MOTOR1_B_PIN 9
#define MOTOR2_A_PIN 10
#define MOTOR2_B_PIN 11


// Encoder A/B pin assignments for the Robo Pico's built-in motor driver terminals.
// Change accordingly if encoder wiring on the board are changed
// Red wire = A pin, Green wire = B pin
// Motor2 encoder pins are swapped to match the physical orientation of the motors on the Robo Pico board.
// Else the direction of motor2 will be reversed compared to motor1
#define MOTOR1_ENCODER_A_PIN 0
#define MOTOR1_ENCODER_B_PIN 1
#define MOTOR2_ENCODER_A_PIN 5
#define MOTOR2_ENCODER_B_PIN 4

// Wheel diameter in millimeters for the Robo Pico's wheels. 
// Change accordingly if different wheels are used
#define WHEEL_DIAMETER_MM 62.0f

// FOR TESTING PURPOSES ONLY, TO REPLACE LATER
// cmake -S . -B build -DWIFI_SSID="YourNetworkName" -DWIFI_PASSWORD="YourNetworkPassword" -DLOG_SERVER_IP="192.168.x.x"
// cmake --build build
// Run wifi_log_listener.py there to see prints from this program over Wi-Fi
#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#endif
#ifndef LOG_SERVER_IP
#define LOG_SERVER_IP ""
#endif
#ifndef LOG_SERVER_PORT
#define LOG_SERVER_PORT 4210
#endif

// Set to 1 to run the motor/encoder demo instead of line following.
#define RUN_MOTOR_DEMO 0

// Set to 1 to test the barcode sensor on its own: motors stay off, push the car
// over a barcode by hand. Prints the raw sensor value with its min/max (use these
// to set BARCODE_BLACK_ON/OFF in ir_sensor.c) and every decoded barcode.
#define RUN_BARCODE_TEST 0

// Set to 1 to test barcode reading with the car really driving, but WITHOUT
// line following: after the start-up delay the car drives straight at a steady
// speed for DRIVE_TEST_MS, stops, and prints what it read. Press RESET to repeat.
// Line the car up so the right sensor will pass over the bars.
// Set all RUN_ options to 0 for normal line following.
#define RUN_BARCODE_DRIVE_TEST 1
#define DRIVE_TEST_SPEED       25      // percent; too low (e.g. 15) makes the motors stall and lurch
#define DRIVE_TEST_MS          2500    // how long to drive; shorten if it runs off the mat

// Drives through forward/backward/turn/spin moves forever, printing encoder readings.
static void run_motor_demo(MotorController *mc);

// Barcode sensor test with the motors off.
static void run_barcode_test(MotorController *mc);

// Barcode test driving straight at a steady speed, no line following.
static void run_barcode_drive_test(MotorController *mc);

int main() {
    stdio_init_all();

    // FOR TESTING PURPOSES ONLY, TO REPLACE LATER
    if (strlen(WIFI_SSID) > 0) {
        if (wifi_log_init(WIFI_SSID, WIFI_PASSWORD, LOG_SERVER_IP, LOG_SERVER_PORT)) {
            printf("Wi-Fi log connected, sending to %s:%d\n", LOG_SERVER_IP, LOG_SERVER_PORT);
        } else {
            printf("Wi-Fi log failed to connect, continuing on USB only\n");
        }
    }


    // Create a MotorController instance and initialize it with the motor pins.
    MotorController motors;
    mc_init(&motors, MOTOR1_A_PIN, MOTOR1_B_PIN, MOTOR2_A_PIN, MOTOR2_B_PIN);
    mc_begin(&motors);
    mc_attach_encoders(&motors,
                       MOTOR1_ENCODER_A_PIN, MOTOR1_ENCODER_B_PIN,
                       MOTOR2_ENCODER_A_PIN, MOTOR2_ENCODER_B_PIN);
    mc_set_wheel_geometry(&motors, WHEEL_DIAMETER_MM, MOTOR_DEFAULT_PULSES_PER_WHEEL_REV);

    ir_init();

    sleep_ms(3000); // time to open the Serial Monitor and place the robot on the line

#if RUN_MOTOR_DEMO
    run_motor_demo(&motors);
#elif RUN_BARCODE_TEST
    run_barcode_test(&motors);
#elif RUN_BARCODE_DRIVE_TEST
    run_barcode_drive_test(&motors);
#else
    printf("Starting line following + barcode scanning\n");
    char code[24];
    while (true) {
        ir_update(&motors);

        if (ir_get_barcode(code, sizeof(code))) {
            printf("BARCODE: %s\n", code);
            // TODO (Vehicle controller): turn the barcode into a navigation
            // command here once the team agrees what each letter means.
        }
    }
#endif
}

static void run_barcode_test(MotorController *mc) {
    mc_stop(mc);
    printf("Barcode test: push the car over a barcode by hand\n");

    uint16_t lowest = 4095, highest = 0;
    uint64_t next_print = 0;
    char code[24];

    while (true) {
        ir_barcode_update(mc);

        uint16_t v = ir_barcode_raw();
        if (v < lowest) lowest = v;
        if (v > highest) highest = v;

        uint64_t now = time_us_64();
        if (now >= next_print) {   // 5 lines a second is enough to read
            printf("right sensor raw=%4u   min=%4u   max=%4u\n", v, lowest, highest);
            next_print = now + 200000;
        }

        if (ir_get_barcode(code, sizeof(code))) {
            printf(">>> BARCODE: %s\n", code);
        }
    }
}

static void run_motor_demo(MotorController *mc) {
    while (true) {
        printf("Forward\n");
        mc_forward(mc, 50);
        sleep_ms(2000);
        mc_stop(mc);
        printf("Pulses -> M1: %d  M2: %d  |  Distance(mm) -> M1: %.1f  M2: %.1f  |  Speed(mm/s) -> M1: %.1f  M2: %.1f\n",
               mc_get_motor1_pulses(mc), mc_get_motor2_pulses(mc),
               mc_get_motor1_distance_mm(mc), mc_get_motor2_distance_mm(mc),
               mc_get_motor1_speed_mm_s(mc), mc_get_motor2_speed_mm_s(mc));
        sleep_ms(1000);

        printf("Backward\n");
        mc_backward(mc, 50);
        sleep_ms(2000);
        mc_stop(mc);
        printf("Pulses -> M1: %d  M2: %d  |  Distance(mm) -> M1: %.1f  M2: %.1f  |  Speed(mm/s) -> M1: %.1f  M2: %.1f\n",
               mc_get_motor1_pulses(mc), mc_get_motor2_pulses(mc),
               mc_get_motor1_distance_mm(mc), mc_get_motor2_distance_mm(mc),
               mc_get_motor1_speed_mm_s(mc), mc_get_motor2_speed_mm_s(mc));
        printf("Wi-Fi log dropped lines so far: %u\n", wifi_log_get_dropped_count());
        sleep_ms(1000);

        printf("Pivot gentle turn left\n");
        mc_turn_left(mc, 50, 30); // sharpness 30 = gentle turn
        sleep_ms(1000);
        mc_stop(mc);
        sleep_ms(1000);
        
        printf("Pivot gentle turn right\n");
        mc_turn_right(mc, 50, 30); // sharpness 30 = gentle turn
        sleep_ms(1000);
        mc_stop(mc);
        sleep_ms(1000);

        printf("Spin left\n");
        mc_spin_left(mc, 50);
        sleep_ms(1000);
        mc_stop(mc);
        sleep_ms(1000);

        printf("Spin right\n");
        mc_spin_right(mc, 50);
        sleep_ms(1000);
        mc_stop(mc);
        sleep_ms(1000);

        printf("Pivot sharp turn left\n");
        mc_turn_left(mc, 50, 70); // sharpness 70 = sharp turn
        sleep_ms(1000);
        mc_stop(mc);
        sleep_ms(1000);
        
        printf("Pivot sharp turn right\n");
        mc_turn_right(mc, 50, 70); // sharpness 70 = sharp turn
        sleep_ms(1000);
        mc_stop(mc);
        sleep_ms(1000);

        sleep_ms(3000);
    }
}

static void run_barcode_drive_test(MotorController *mc) {
    char code[24];
    bool got = false;

    printf("Barcode drive test: driving straight for %d ms\n", DRIVE_TEST_MS);
    mc_set_wheel_speeds(mc, DRIVE_TEST_SPEED, DRIVE_TEST_SPEED);

    // Drive and read. Nothing is printed until the car has stopped, so the
    // timing of the bars isn't disturbed.
    uint64_t stop_at = time_us_64() + (uint64_t)DRIVE_TEST_MS * 1000;
    while (time_us_64() < stop_at) {
        ir_barcode_update(mc);
        if (!got && ir_get_barcode(code, sizeof(code))) got = true;
    }
    mc_stop(mc);

    // Keep reading briefly in case the barcode ended right as the car stopped.
    uint64_t wait_until = time_us_64() + 1500000;
    while (time_us_64() < wait_until) {
        ir_barcode_update(mc);
        if (!got && ir_get_barcode(code, sizeof(code))) got = true;
    }

    if (got) printf(">>> BARCODE: %s\n", code);
    else     printf(">>> no barcode decoded (see the barcode: line above, if any)\n");
    printf("Done. Press RESET to run again.\n");

    while (true) sleep_ms(1000);
}