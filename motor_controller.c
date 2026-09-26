#include "motor_controller.h"

#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

static float s_pwm_clkdiv; // PWM clock divider, set at init
static uint16_t s_pwm_wrap; // PWM wrap value, set at init

// PWM frequency in Hz based on Robo Pico's datasheet. 
// 20 Khz is the max for the motor driver, but set to 10 kHz to reduce audible noise.
#define PWM_FREQ_HZ 10000.0f 

// MotorController instance class that is currently active for encoder pulse counting.
// Only one instance can be active at a time, since the encoder ISR is shared.
static MotorController *s_active_instance = NULL;

// Debounce time for encoder pulses.
// Only edges that are at least this many microseconds after the last accepted edge are counted
// Others are ignored as noise. 
#define ENCODER_DEBOUNCE_US 100u

// Check how long pulse has been since the last accepted pulse.
// If it's been more than this many microseconds, the wheel is considered to have stopped.
#define SPEED_STALE_US 500000u

// Calculates speed_pct (0-100) to PWM duty cycle (0-wrap). 
// Wrap is set at init.
static uint16_t pct_to_duty(uint8_t speed_pct) { 
    if (speed_pct > 100) speed_pct = 100; 
    return (uint16_t)(((uint32_t)speed_pct * s_pwm_wrap) / 100u);
}

// Configures the pin mux, clock divider, wrap value, and enables the PWM slice.
static void init_pwm_pin(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM); // sets the pin mux to PWM mode
    uint slice_num = pwm_gpio_to_slice_num(pin); // each PWM slice drives two pins, so multiple pins can share a slice

    pwm_set_clkdiv(slice_num, s_pwm_clkdiv); // sets the PWM clock divider for this slice, which determines the PWM frequency
    pwm_set_wrap(slice_num, s_pwm_wrap); // sets the PWM wrap value for this slice, which determines the PWM period

    pwm_set_gpio_level(pin, 0); // sets the PWM duty cycle for this pin to 0% (off)
    pwm_set_enabled(slice_num, true); // enables the PWM slice, which starts the PWM output on both pins of the slice
}

// Drives a motor with the given A/B pins at the given speed percentage (-100 to 100). 
// Negative means reverse.
static void drive_motor(uint a_pin, uint b_pin, int16_t pct) { 
    if (pct > 100) pct = 100;
    if (pct < -100) pct = -100;

    uint16_t duty = pct_to_duty((uint8_t)(pct < 0 ? -pct : pct));

    pwm_set_gpio_level(a_pin, pct > 0 ? duty : 0);
    pwm_set_gpio_level(b_pin, pct < 0 ? duty : 0);
}

// Encoder ISR: called on every edge of the A-phase pin for either motor.
// Reads the B-phase pin to determine direction, and updates the pulse counts and speed.
// Debounces edges that are too close together, and ignores edges if no MotorController instance is active.
static void encoder_isr(uint gpio, uint32_t events) {
    (void)events;
    if (!s_active_instance) return;

    uint32_t now_us = time_us_32(); // wrap-safe microsecond timestamp

    // Check if the interrupt is for motor 1's A-phase pin
    if (gpio == s_active_instance->m1_pulse_a_pin) { 
        uint32_t since_last_us = now_us - s_active_instance->m1_last_pulse_time_us; // Calculate time since last accepted pulse, wrap-safe
        if (since_last_us < ENCODER_DEBOUNCE_US) return; // Ignore edges that are too close together (debounce)
        if (since_last_us > (uint32_t)INT32_MAX) since_last_us = (uint32_t)INT32_MAX; // avoid signed overflow below

        // Determine direction based on B-phase pin: 1 for forward, -1 for reverse
        int32_t dir = gpio_get(s_active_instance->m1_pulse_b_pin) ? 1 : -1; 

        // Update pulse counts and speed for motor 1
        s_active_instance->m1_pulses += dir;
        s_active_instance->m1_total_pulses++;
        s_active_instance->m1_last_pulse_period_us = dir * (int32_t)since_last_us;
        s_active_instance->m1_last_pulse_time_us = now_us;

    // Check if the interrupt is for motor 2's A-phase pin
    } else if (gpio == s_active_instance->m2_pulse_a_pin) {
        uint32_t since_last_us = now_us - s_active_instance->m2_last_pulse_time_us; 
        if (since_last_us < ENCODER_DEBOUNCE_US) return;
        if (since_last_us > (uint32_t)INT32_MAX) since_last_us = (uint32_t)INT32_MAX;

        // Determine direction based on B-phase pin: 1 for forward, -1 for reverse
        int32_t dir = gpio_get(s_active_instance->m2_pulse_b_pin) ? 1 : -1;

        // Update pulse counts and speed for motor 2
        s_active_instance->m2_pulses += dir;
        s_active_instance->m2_total_pulses++;
        s_active_instance->m2_last_pulse_period_us = dir * (int32_t)since_last_us;
        s_active_instance->m2_last_pulse_time_us = now_us;
    }
}

// Initializes a MotorController instance with the given motor pins.
void mc_init(MotorController *mc,
             uint m1_a_pin, uint m1_b_pin,
             uint m2_a_pin, uint m2_b_pin) {
    uint32_t clk_sys_hz = clock_get_hz(clk_sys);

    // Calculate the total number of clock cycles per PWM period based on the system clock and desired PWM frequency.
    float total_count = (float)clk_sys_hz / PWM_FREQ_HZ;

    // Calculate the clock divider and wrap value for the PWM slices. The clock divider is the ratio of the total count to the maximum wrap value (65536)
    // and the wrap value is the total count divided by the clock divider, rounded to the nearest integer. 
    // Clamp the clock divider to the valid range of 1.0 to 255.9375 for the RP2040.
    s_pwm_clkdiv = total_count / 65536.0f;
    if (s_pwm_clkdiv < 1.0f) s_pwm_clkdiv = 1.0f;   
    if (s_pwm_clkdiv > 255.9375f) s_pwm_clkdiv = 255.9375f;

    // Auto round the wrap value to the nearest integer and subtract 1 to get the final wrap value for the PWM slices.
    s_pwm_wrap = (uint16_t)((total_count / s_pwm_clkdiv) + 0.5f) - 1;
    
    // Initialize the MotorController instance with the given motor pins
    mc->m1_a_pin = m1_a_pin;
    mc->m1_b_pin = m1_b_pin;
    mc->m2_a_pin = m2_a_pin;
    mc->m2_b_pin = m2_b_pin;
    
    // Set the encoder pins to UINT32_MAX to indicate that they are not yet attached, and initialize the pulse counts and speed to zero.
    mc->m1_pulse_a_pin = UINT32_MAX;
    mc->m1_pulse_b_pin = UINT32_MAX;
    mc->m2_pulse_a_pin = UINT32_MAX;
    mc->m2_pulse_b_pin = UINT32_MAX;
    mc->m1_pulses = 0;
    mc->m2_pulses = 0;
    mc->m1_total_pulses = 0;
    mc->m2_total_pulses = 0;
    mc->m1_last_pulse_time_us = time_us_32();
    mc->m2_last_pulse_time_us = mc->m1_last_pulse_time_us;
    mc->m1_last_pulse_period_us = 0; // 0 = no pulse accepted yet
    mc->m2_last_pulse_period_us = 0;
    
    // Set the wheel geometry to zero until mc_set_wheel_geometry() is called.
    mc->wheel_circumference_mm = 0.0f;
    mc->pulses_per_wheel_rev = 0;
}

// Begins the operation of the MotorController.
void mc_begin(MotorController *mc) {
    init_pwm_pin(mc->m1_a_pin);
    init_pwm_pin(mc->m1_b_pin);
    init_pwm_pin(mc->m2_a_pin);
    init_pwm_pin(mc->m2_b_pin);
}

// Sets the PWM duty cycle for the left and right wheels based on the given speed percentages (-100 to 100).
void mc_set_wheel_speeds(MotorController *mc, int16_t left_pct, int16_t right_pct) {
    drive_motor(mc->m1_a_pin, mc->m1_b_pin, left_pct);
    drive_motor(mc->m2_a_pin, mc->m2_b_pin, right_pct);
}

// Movement helpers that call mc_set_wheel_speeds() with the appropriate speed percentages for forward, backward, stop, turn, and spin movements.
void mc_forward(MotorController *mc, uint8_t speed_pct) {
    mc_set_wheel_speeds(mc, speed_pct, speed_pct);
}

void mc_backward(MotorController *mc, uint8_t speed_pct) {
    mc_set_wheel_speeds(mc, -(int16_t)speed_pct, -(int16_t)speed_pct);
}

void mc_stop(MotorController *mc) {
    mc_set_wheel_speeds(mc, 0, 0);
}

void mc_turn_left(MotorController *mc, uint8_t speed_pct, uint8_t sharpness_pct) {
    if (sharpness_pct > 100) sharpness_pct = 100;
    int16_t inner_pct = -(int16_t)(((uint32_t)speed_pct * sharpness_pct) / 100u);
    mc_set_wheel_speeds(mc, inner_pct, speed_pct);
}

void mc_turn_right(MotorController *mc, uint8_t speed_pct, uint8_t sharpness_pct) {
    if (sharpness_pct > 100) sharpness_pct = 100;
    int16_t inner_pct = -(int16_t)(((uint32_t)speed_pct * sharpness_pct) / 100u);
    mc_set_wheel_speeds(mc, speed_pct, inner_pct);
}

void mc_spin_left(MotorController *mc, uint8_t speed_pct) {
    mc_set_wheel_speeds(mc, -(int16_t)speed_pct, speed_pct);
}

void mc_spin_right(MotorController *mc, uint8_t speed_pct) {
    mc_set_wheel_speeds(mc, speed_pct, -(int16_t)speed_pct);
}

// Helper function to initialize an encoder pin as an input with a pull-up resistor, and optionally install the encoder ISR callback for the A-phase pin.
static void init_encoder_pin(uint pin, bool install_callback) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
    if (install_callback) {
        gpio_set_irq_enabled_with_callback(pin, GPIO_IRQ_EDGE_FALL, true, &encoder_isr);
    }
}

// Attaches the encoder pins to the MotorController instance, and installs the encoder ISR callback for the A-phase pin of motor 1. 
// The B-phase pins are read in the ISR to determine direction.
void mc_attach_encoders(MotorController *mc,
                         uint m1_pulse_a_pin, uint m1_pulse_b_pin,
                         uint m2_pulse_a_pin, uint m2_pulse_b_pin) {
    mc->m1_pulse_a_pin = m1_pulse_a_pin;
    mc->m1_pulse_b_pin = m1_pulse_b_pin;
    mc->m2_pulse_a_pin = m2_pulse_a_pin;
    mc->m2_pulse_b_pin = m2_pulse_b_pin;
    s_active_instance = mc;

    // Only the A pins are interrupt-driven; B pins are plain inputs read for direction inside encoder_isr().
    init_encoder_pin(m1_pulse_a_pin, true);
    init_encoder_pin(m1_pulse_b_pin, false);
    init_encoder_pin(m2_pulse_a_pin, false);
    init_encoder_pin(m2_pulse_b_pin, false);
    gpio_set_irq_enabled(m2_pulse_a_pin, GPIO_IRQ_EDGE_FALL, true);
}

// Functions below are for reading the total pulse counts, total pulses, distances, and speeds for each motor.

// Helpers to read the pulse counts, total pulses, distances, and speeds for each motor.
int32_t mc_get_motor1_pulses(const MotorController *mc) {
    return mc->m1_pulses;
}

int32_t mc_get_motor2_pulses(const MotorController *mc) {
    return mc->m2_pulses;
}

uint32_t mc_get_motor1_total_pulses(const MotorController *mc) {
    return mc->m1_total_pulses;
}

uint32_t mc_get_motor2_total_pulses(const MotorController *mc) {
    return mc->m2_total_pulses;
}

// Resets both the current-movement (signed net) and total (odometer) counts for both motors.
void mc_reset_encoder_counts(MotorController *mc) {
    mc->m1_pulses = 0;
    mc->m2_pulses = 0;
    mc->m1_total_pulses = 0;
    mc->m2_total_pulses = 0;
}

// Sets the wheel geometry for the MotorController instance, which is used to convert pulse counts to real-world distances and speeds.
void mc_set_wheel_geometry(MotorController *mc, float wheel_diameter_mm, uint32_t pulses_per_wheel_rev) {
    mc->wheel_circumference_mm = 3.14159265f * wheel_diameter_mm;
    mc->pulses_per_wheel_rev = pulses_per_wheel_rev;
}

// Converts pulse counts to distance in millimeters based on the wheel geometry and pulses per revolution.
static float distance_mm(int32_t pulses, const MotorController *mc) {
    if (mc->pulses_per_wheel_rev == 0) return 0.0f;
    return ((float)pulses / (float)mc->pulses_per_wheel_rev) * mc->wheel_circumference_mm;
}

// Returns the distance in millimeters for each motor based on the current pulse counts and wheel geometry.
float mc_get_motor1_distance_mm(const MotorController *mc) {
    return distance_mm(mc->m1_pulses, mc);
}

float mc_get_motor2_distance_mm(const MotorController *mc) {
    return distance_mm(mc->m2_pulses, mc);
}

float mc_get_motor1_total_distance_mm(const MotorController *mc) {
    return distance_mm((int32_t)mc->m1_total_pulses, mc);
}

float mc_get_motor2_total_distance_mm(const MotorController *mc) {
    return distance_mm((int32_t)mc->m2_total_pulses, mc);
}


// Functions below are for calculating based on the last pulse time and period.

// Calculates the speed in millimeters per second for a motor based on the last pulse time and period, and the wheel geometry.
static float speed_mm_s(const MotorController *mc, uint32_t last_pulse_time_us, int32_t last_pulse_period_us) {
    if (mc->pulses_per_wheel_rev == 0 || last_pulse_period_us == 0) 
        return 0.0f;

    uint32_t since_last_pulse_us = time_us_32() - last_pulse_time_us; 
    if (since_last_pulse_us > SPEED_STALE_US) 
        return 0.0f; 

    uint32_t period_us = last_pulse_period_us < 0 ? (uint32_t)(-last_pulse_period_us) : (uint32_t)last_pulse_period_us;
    float distance_per_pulse = distance_mm(1, mc); 
    float speed = distance_per_pulse / ((float)period_us / 1000000.0f); // Calculate speed in mm/s based on distance per pulse and period in seconds
    return last_pulse_period_us < 0 ? -speed : speed;
}

// Returns the speed in millimeters per second for each motor based on the last pulse time and period, and the wheel geometry.
float mc_get_motor1_speed_mm_s(const MotorController *mc) {
    return speed_mm_s(mc, mc->m1_last_pulse_time_us, mc->m1_last_pulse_period_us);
}

float mc_get_motor2_speed_mm_s(const MotorController *mc) {
    return speed_mm_s(mc, mc->m2_last_pulse_time_us, mc->m2_last_pulse_period_us);
}
