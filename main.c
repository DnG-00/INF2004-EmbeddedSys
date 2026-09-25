#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "motor_controller.h"
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
    
    sleep_ms(2000);

    while (true) {
        printf("Forward\n");
        mc_forward(&motors, 50);
        sleep_ms(2000);
        mc_stop(&motors);
        printf("Pulses -> M1: %d  M2: %d  |  Distance(mm) -> M1: %.1f  M2: %.1f  |  Speed(mm/s) -> M1: %.1f  M2: %.1f\n",
               mc_get_motor1_pulses(&motors), mc_get_motor2_pulses(&motors),
               mc_get_motor1_distance_mm(&motors), mc_get_motor2_distance_mm(&motors),
               mc_get_motor1_speed_mm_s(&motors), mc_get_motor2_speed_mm_s(&motors));
        sleep_ms(1000);

        printf("Backward\n");
        mc_backward(&motors, 50);
        sleep_ms(2000);
        mc_stop(&motors);
        printf("Pulses -> M1: %d  M2: %d  |  Distance(mm) -> M1: %.1f  M2: %.1f  |  Speed(mm/s) -> M1: %.1f  M2: %.1f\n",
               mc_get_motor1_pulses(&motors), mc_get_motor2_pulses(&motors),
               mc_get_motor1_distance_mm(&motors), mc_get_motor2_distance_mm(&motors),
               mc_get_motor1_speed_mm_s(&motors), mc_get_motor2_speed_mm_s(&motors));
        printf("Wi-Fi log dropped lines so far: %u\n", wifi_log_get_dropped_count());
        sleep_ms(1000);

        printf("Pivot gentle turn left\n");
        mc_turn_left(&motors, 50, 30); // sharpness 30 = gentle turn
        sleep_ms(1000);
        mc_stop(&motors);
        sleep_ms(1000);
        
        printf("Pivot gentle turn right\n");
        mc_turn_right(&motors, 50, 30); // sharpness 30 = gentle turn
        sleep_ms(1000);
        mc_stop(&motors);
        sleep_ms(1000);

        printf("Spin left\n");
        mc_spin_left(&motors, 50);
        sleep_ms(1000);
        mc_stop(&motors);
        sleep_ms(1000);

        printf("Spin right\n");
        mc_spin_right(&motors, 50);
        sleep_ms(1000);
        mc_stop(&motors);
        sleep_ms(1000);

        printf("Pivot sharp turn left\n");
        mc_turn_left(&motors, 50, 70); // sharpness 70 = sharp turn
        sleep_ms(1000);
        mc_stop(&motors);
        sleep_ms(1000);
        
        printf("Pivot sharp turn right\n");
        mc_turn_right(&motors, 50, 70); // sharpness 70 = sharp turn
        sleep_ms(1000);
        mc_stop(&motors);
        sleep_ms(1000);

        sleep_ms(3000);
    }
}
