#pragma once

#include "pico/stdlib.h"
#include <stdint.h>

// MotorController: manages two motors with Robo Pico's built-in driver, plus optional wheel-encoder pulse counting.
// Usage:
//   MotorController motors;
//   mc_init(&motors, 8, 9, 10, 11);
//   mc_begin(&motors);
//   mc_forward(&motors, 60);
typedef struct {
    uint m1_a_pin, m1_b_pin;
    uint m2_a_pin, m2_b_pin;

    // UINT32_MAX until mc_attach_encoders(). Only the A pin is
    // interrupt-driven; B is read at each A edge to tell direction.
    uint m1_pulse_a_pin, m1_pulse_b_pin;
    uint m2_pulse_a_pin, m2_pulse_b_pin;
    volatile int32_t m1_pulses, m2_pulses; // signed: net position since last reset
    volatile uint32_t m1_total_pulses, m2_total_pulses; // odometer: never decreases

    // Debounce (encoder_isr ignores edges within ENCODER_DEBOUNCE_US of the
    // last accepted one) and period-based speed. Both only update when a
    // pulse is accepted, not on every raw edge. Timestamps are the low 32
    // bits of the hardware microsecond timer (time_us_32()) -- wraps every
    // ~71.6 minutes, but unsigned subtraction handles that correctly, and a
    // single 32-bit read/write is atomic against the ISR without needing a
    // critical section.
    volatile uint32_t m1_last_pulse_time_us, m2_last_pulse_time_us;
    volatile int32_t m1_last_pulse_period_us, m2_last_pulse_period_us; // signed: sign = direction, magnitude = time since the previous accepted pulse; 0 = no pulse yet

    float wheel_circumference_mm;      // 0 until mc_set_wheel_geometry()
    uint32_t pulses_per_wheel_rev;
} MotorController;

// Pulses per full wheel revolution: 13 encoder lines on the motor's fast
// shaft x 45 (gearbox ratio) = 585. mc_attach_encoders() only
// counts A-phase edges (1x), just using B phase for direction -- not full 4x
// quadrature decoding, so this is 585, not the datasheet's 2340 (13 x 4 x 45).
#define MOTOR_DEFAULT_PULSES_PER_WHEEL_REV 585u

// Records pin assignments. Call once before mc_begin().
void mc_init(MotorController *mc,
             uint m1_a_pin, uint m1_b_pin,
             uint m2_a_pin, uint m2_b_pin);

// Configures the 4 motor pins for PWM output at ~10kHz. Call once at startup.
void mc_begin(MotorController *mc);

// High-level movement helpers. speed_pct is clamped to 0-100.
void mc_forward(MotorController *mc, uint8_t speed_pct);
void mc_backward(MotorController *mc, uint8_t speed_pct);
void mc_stop(MotorController *mc);

// Turns with adjustable sharpness. The outer wheel always drives at
// speed_pct; sharpness_pct (0-100) controls the inner wheel: 0 leaves it
// stationary (a pivot turn, the widest turn radius this gives you), 100
// reverses it at speed_pct (a full spin turn, same as spin_left/spin_right),
// and values in between blend continuously -- higher sharpness_pct = tighter
// turn radius.
void mc_turn_left(MotorController *mc, uint8_t speed_pct, uint8_t sharpness_pct);  // M1 inner, M2 outer
void mc_turn_right(MotorController *mc, uint8_t speed_pct, uint8_t sharpness_pct); // M2 inner, M1 outer

// Spin turns: both wheels drive in opposite directions to rotate on the spot.
void mc_spin_left(MotorController *mc, uint8_t speed_pct);  // M1 back, M2 forward
void mc_spin_right(MotorController *mc, uint8_t speed_pct); // M1 forward, M2 back

// Low-level escape hatch for anything the helpers above don't cover.
// left_pct/right_pct are signed, -100..100; negative means reverse.
void mc_set_wheel_speeds(MotorController *mc, int16_t left_pct, int16_t right_pct);

// Optional wheel-encoder pulse counting with direction: interrupts on the
// A-phase pin, reads the B-phase pin's level at that instant to tell which
// way the wheel is turning. The Robo Pico has no dedicated encoder header --
// wire both encoder signal wires into a free Grove port and pass those GPIO
// numbers here. Direction sign (which B level means "forward") is a guess --
// verify against a known movement on real hardware and swap if reversed.
void mc_attach_encoders(MotorController *mc,
                         uint m1_pulse_a_pin, uint m1_pulse_b_pin,
                         uint m2_pulse_a_pin, uint m2_pulse_b_pin);
// Current movement: signed net pulse count since the last reset -- forward
// and backward cancel out, so this is position, not total distance moved.
int32_t mc_get_motor1_pulses(const MotorController *mc);
int32_t mc_get_motor2_pulses(const MotorController *mc);

// Total (odometer): every encoder edge counts, regardless of direction, so
// this only ever grows -- total distance moved, like a car's odometer.
uint32_t mc_get_motor1_total_pulses(const MotorController *mc);
uint32_t mc_get_motor2_total_pulses(const MotorController *mc);

// Resets both the current-movement (signed net) and total (odometer) counts.
void mc_reset_encoder_counts(MotorController *mc);

// Wheel geometry needed to turn pulse counts into real-world distance/speed.
// Call once (after mc_init()) before using the functions
// below. Measure your actual wheel diameter -- don't guess.
// pulses_per_wheel_rev: use MOTOR_DEFAULT_PULSES_PER_WHEEL_REV unless you've
// changed how mc_attach_encoders() counts pulses.
void mc_set_wheel_geometry(MotorController *mc, float wheel_diameter_mm, uint32_t pulses_per_wheel_rev);

// Current movement: net distance travelled by each wheel (in mm, signed)
// since the last mc_reset_encoder_counts() -- forward and
// backward motion cancel out, since this reflects position, not total path
// length. Requires wheel geometry to be set.
float mc_get_motor1_distance_mm(const MotorController *mc);
float mc_get_motor2_distance_mm(const MotorController *mc);

// Total (odometer): total distance moved by each wheel (in mm, always >= 0)
// since the last mc_reset_encoder_counts() -- forward and
// backward both add up, this never decreases. Requires wheel geometry to be
// set.
float mc_get_motor1_total_distance_mm(const MotorController *mc);
float mc_get_motor2_total_distance_mm(const MotorController *mc);

// Instantaneous speed (mm/s, signed -- negative while reversing), computed
// from the time between the two most recent accepted encoder pulses (not a
// polling-window average), so it's accurate even at low speed where few
// pulses occur per call. Reads as 0 if no pulse has been accepted in the
// last ~0.5s (i.e. the wheel has actually stopped), not just decayed from
// the last non-zero reading. Can be called any time, any rate.
float mc_get_motor1_speed_mm_s(const MotorController *mc);
float mc_get_motor2_speed_mm_s(const MotorController *mc);
