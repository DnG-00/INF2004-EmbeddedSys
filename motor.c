/** @file motor.c
 *
 * @brief Drives the Robo Pico's two motors with PWM and counts their
 *        wheel-encoder pulses to measure distance and speed.
 */

#include "motor.h"

#include <stddef.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

// PWM frequency in Hz, from the Robo Pico datasheet. 20 kHz is the maximum
// for the motor driver, but 10 kHz is used to reduce audible noise.
#define PWM_FREQ_HZ          10000.0f

// Largest PWM count (wrap + 1) and clock divider range on the RP2040.
#define PWM_MAX_COUNT        65536.0f
#define PWM_MIN_CLKDIV       1.0f
#define PWM_MAX_CLKDIV       255.9375f

// Speeds are percentages from -100 to 100. The signed and unsigned versions
// keep signed and unsigned values from being mixed in comparisons.
#define MAX_SPEED_PCT        100
#define FULL_SCALE_PCT       100U

// Debounce time for encoder pulses. Only edges at least this many
// microseconds after the last accepted edge are counted; others are noise.
#define ENCODER_DEBOUNCE_US  100U

// If no pulse has been accepted for this long, the wheel is considered to
// have stopped. Set in milliseconds; the code compares in microseconds.
#define SPEED_STALE_MS       500U
#define SPEED_STALE_US       (SPEED_STALE_MS * 1000U)

#define PI_F                 3.14159265f
#define US_PER_SECOND        1000000.0f

// PWM clock divider and wrap value, worked out once in motor_init().
static float32_t g_pwm_clkdiv = PWM_MIN_CLKDIV;
static uint16_t  g_pwm_wrap   = 0U;

// The motors currently counting encoder pulses. Only one motor_t can be
// active at a time, because the encoder ISR is shared.
static motor_t * g_p_active_motors = NULL;

static uint16_t  convert_pct_to_duty (uint8_t speed_pct);
static void      init_pwm_pin (uint32_t pin);
static void      drive_motor (motor_wheel_t const * p_wheel, int16_t speed_pct);
static void      encoder_isr (uint gpio, uint32_t events);
static void      record_encoder_pulse (motor_wheel_t * p_wheel,
                                       uint32_t now_us);
static void      init_encoder_pin (uint32_t pin, bool b_install_callback);
static void      init_wheel (motor_wheel_t * p_wheel, uint32_t pwm_a_pin,
                             uint32_t pwm_b_pin, uint32_t now_us);
static float32_t calc_distance_mm (int32_t pulses,
                                   motor_t const * p_motors);
static float32_t calc_speed_mm_s (motor_t const * p_motors,
                                  motor_wheel_t const * p_wheel);

/*!
 * @brief Records the motor pins and works out the PWM settings.
 *
 * Call once, before motor_begin().
 *
 * @param[out] p_motors  The motors to set up.
 * @param[in]  m1_a_pin  M1 (left) driver pin A.
 * @param[in]  m1_b_pin  M1 (left) driver pin B.
 * @param[in]  m2_a_pin  M2 (right) driver pin A.
 * @param[in]  m2_b_pin  M2 (right) driver pin B.
 */
void
motor_init (motor_t * p_motors,
            uint32_t m1_a_pin, uint32_t m1_b_pin,
            uint32_t m2_a_pin, uint32_t m2_b_pin)
{
    uint32_t clk_sys_hz = clock_get_hz(clk_sys);

    // Clock cycles per PWM period. Cast: a clock of at most a few hundred MHz
    // is well within float range.
    float32_t total_count = (float32_t)clk_sys_hz / PWM_FREQ_HZ;

    // The clock divider is the total count over the largest wrap value,
    // clamped to the RP2040's valid range.
    g_pwm_clkdiv = total_count / PWM_MAX_COUNT;
    if (g_pwm_clkdiv < PWM_MIN_CLKDIV)
    {
        g_pwm_clkdiv = PWM_MIN_CLKDIV;
    }

    if (g_pwm_clkdiv > PWM_MAX_CLKDIV)
    {
        g_pwm_clkdiv = PWM_MAX_CLKDIV;
    }

    // The wrap value is the total count over the divider, rounded to the
    // nearest integer, minus 1. Cast: the clamping above keeps the rounded
    // count between 1 and 65536, so it fits in 32 bits, and the count minus
    // 1 fits in 16 bits.
    uint32_t rounded_count = (uint32_t)((total_count / g_pwm_clkdiv) + 0.5f);

    g_pwm_wrap = (uint16_t)(rounded_count - 1U);

    uint32_t now_us = time_us_32();

    init_wheel(&p_motors->m1, m1_a_pin, m1_b_pin, now_us);
    init_wheel(&p_motors->m2, m2_a_pin, m2_b_pin, now_us);

    // No wheel geometry until motor_set_wheel_geometry() is called.
    p_motors->wheel_circumference_mm = 0.0f;
    p_motors->pulses_per_wheel_rev   = 0U;
}

/*!
 * @brief Configures the four motor pins for PWM output.
 *
 * Call once at start-up, after motor_init().
 *
 * @param[in] p_motors  The motors to start.
 */
void
motor_begin (motor_t * p_motors)
{
    init_pwm_pin(p_motors->m1.pwm_a_pin);
    init_pwm_pin(p_motors->m1.pwm_b_pin);
    init_pwm_pin(p_motors->m2.pwm_a_pin);
    init_pwm_pin(p_motors->m2.pwm_b_pin);
}

/*!
 * @brief Sets both wheel speeds directly.
 *
 * Low-level escape hatch for anything the other helpers don't cover.
 *
 * @param[in] p_motors   The motors to drive.
 * @param[in] left_pct   Left (M1) speed, -100 to 100. Negative = reverse.
 * @param[in] right_pct  Right (M2) speed, -100 to 100. Negative = reverse.
 */
void
motor_set_wheel_speeds (motor_t * p_motors, int16_t left_pct,
                        int16_t right_pct)
{
    drive_motor(&p_motors->m1, left_pct);
    drive_motor(&p_motors->m2, right_pct);
}

/*!
 * @brief Drives both wheels forward.
 *
 * @param[in] p_motors   The motors to drive.
 * @param[in] speed_pct  Speed, 0 to 100.
 */
void
motor_drive_forward (motor_t * p_motors, uint8_t speed_pct)
{
    int16_t speed = speed_pct;

    motor_set_wheel_speeds(p_motors, speed, speed);
}

/*!
 * @brief Drives both wheels backward.
 *
 * @param[in] p_motors   The motors to drive.
 * @param[in] speed_pct  Speed, 0 to 100.
 */
void
motor_drive_backward (motor_t * p_motors, uint8_t speed_pct)
{
    int16_t speed = speed_pct;

    motor_set_wheel_speeds(p_motors, -speed, -speed);
}

/*!
 * @brief Stops both wheels.
 *
 * @param[in] p_motors  The motors to stop.
 */
void
motor_stop (motor_t * p_motors)
{
    motor_set_wheel_speeds(p_motors, 0, 0);
}

/*!
 * @brief Turns left with adjustable sharpness (M1 inner, M2 outer).
 *
 * The outer wheel always drives at speed_pct. sharpness_pct controls the
 * inner wheel: 0 leaves it stationary (a pivot turn, the widest radius this
 * gives), 100 reverses it at speed_pct (a full spin turn), and values in
 * between blend continuously. Higher sharpness = tighter turn.
 *
 * @param[in] p_motors       The motors to drive.
 * @param[in] speed_pct      Outer wheel speed, 0 to 100.
 * @param[in] sharpness_pct  Turn sharpness, 0 to 100.
 */
void
motor_turn_left (motor_t * p_motors, uint8_t speed_pct, uint8_t sharpness_pct)
{
    uint32_t sharpness = sharpness_pct;
    uint32_t speed     = speed_pct;

    if (sharpness > FULL_SCALE_PCT)
    {
        sharpness = FULL_SCALE_PCT;
    }

    // Cast: at most 255 x 100 / 100 = 255, which fits in 16 bits.
    int16_t inner_pct = -(int16_t)((speed * sharpness) / FULL_SCALE_PCT);

    motor_set_wheel_speeds(p_motors, inner_pct, speed_pct);
}

/*!
 * @brief Turns right with adjustable sharpness (M2 inner, M1 outer).
 *
 * See motor_turn_left() for how sharpness works.
 *
 * @param[in] p_motors       The motors to drive.
 * @param[in] speed_pct      Outer wheel speed, 0 to 100.
 * @param[in] sharpness_pct  Turn sharpness, 0 to 100.
 */
void
motor_turn_right (motor_t * p_motors, uint8_t speed_pct,
                  uint8_t sharpness_pct)
{
    uint32_t sharpness = sharpness_pct;
    uint32_t speed     = speed_pct;

    if (sharpness > FULL_SCALE_PCT)
    {
        sharpness = FULL_SCALE_PCT;
    }

    // Cast: at most 255 x 100 / 100 = 255, which fits in 16 bits.
    int16_t inner_pct = -(int16_t)((speed * sharpness) / FULL_SCALE_PCT);

    motor_set_wheel_speeds(p_motors, speed_pct, inner_pct);
}

/*!
 * @brief Spins left on the spot (M1 backward, M2 forward).
 *
 * @param[in] p_motors   The motors to drive.
 * @param[in] speed_pct  Speed, 0 to 100.
 */
void
motor_spin_left (motor_t * p_motors, uint8_t speed_pct)
{
    int16_t speed = speed_pct;

    motor_set_wheel_speeds(p_motors, -speed, speed);
}

/*!
 * @brief Spins right on the spot (M1 forward, M2 backward).
 *
 * @param[in] p_motors   The motors to drive.
 * @param[in] speed_pct  Speed, 0 to 100.
 */
void
motor_spin_right (motor_t * p_motors, uint8_t speed_pct)
{
    int16_t speed = speed_pct;

    motor_set_wheel_speeds(p_motors, speed, -speed);
}

/*!
 * @brief Starts counting wheel-encoder pulses, with direction.
 *
 * Interrupts on each A-phase pin; the B-phase pin's level at that instant
 * tells which way the wheel is turning. Which B level means "forward" is a
 * guess: verify it against a known movement and swap the pins if reversed.
 *
 * @param[in,out] p_motors  The motors the encoders belong to.
 * @param[in]     m1_a_pin  M1 (left) encoder A-phase pin.
 * @param[in]     m1_b_pin  M1 (left) encoder B-phase pin.
 * @param[in]     m2_a_pin  M2 (right) encoder A-phase pin.
 * @param[in]     m2_b_pin  M2 (right) encoder B-phase pin.
 */
void
motor_attach_encoders (motor_t * p_motors,
                       uint32_t m1_a_pin, uint32_t m1_b_pin,
                       uint32_t m2_a_pin, uint32_t m2_b_pin)
{
    p_motors->m1.encoder_a_pin = m1_a_pin;
    p_motors->m1.encoder_b_pin = m1_b_pin;
    p_motors->m2.encoder_a_pin = m2_a_pin;
    p_motors->m2.encoder_b_pin = m2_b_pin;
    g_p_active_motors          = p_motors;

    // Only the A pins are interrupt-driven; the B pins are plain inputs read
    // for direction inside encoder_isr().
    init_encoder_pin(m1_a_pin, true);
    init_encoder_pin(m1_b_pin, false);
    init_encoder_pin(m2_a_pin, false);
    init_encoder_pin(m2_b_pin, false);
    gpio_set_irq_enabled(m2_a_pin, GPIO_IRQ_EDGE_FALL, true);
}

/*!
 * @brief Gets M1's signed net pulse count since the last reset.
 *
 * Forward and backward cancel out, so this is position, not total distance.
 *
 * @param[in] p_motors  The motors to read.
 * @return Net pulse count.
 */
int32_t
motor_get_m1_pulses (motor_t const * p_motors)
{
    return p_motors->m1.pulses;
}

/*!
 * @brief Gets M2's signed net pulse count since the last reset.
 *
 * @param[in] p_motors  The motors to read.
 * @return Net pulse count.
 */
int32_t
motor_get_m2_pulses (motor_t const * p_motors)
{
    return p_motors->m2.pulses;
}

/*!
 * @brief Gets M1's odometer pulse count, which only ever grows.
 *
 * @param[in] p_motors  The motors to read.
 * @return Total pulses in either direction since the last reset.
 */
uint32_t
motor_get_m1_total_pulses (motor_t const * p_motors)
{
    return p_motors->m1.total_pulses;
}

/*!
 * @brief Gets M2's odometer pulse count, which only ever grows.
 *
 * @param[in] p_motors  The motors to read.
 * @return Total pulses in either direction since the last reset.
 */
uint32_t
motor_get_m2_total_pulses (motor_t const * p_motors)
{
    return p_motors->m2.total_pulses;
}

/*!
 * @brief Resets both the net and the odometer pulse counts of both motors.
 *
 * @param[in,out] p_motors  The motors to reset.
 */
void
motor_reset_encoder_counts (motor_t * p_motors)
{
    p_motors->m1.pulses       = 0;
    p_motors->m2.pulses       = 0;
    p_motors->m1.total_pulses = 0U;
    p_motors->m2.total_pulses = 0U;
}

/*!
 * @brief Sets the wheel size used to turn pulse counts into distance/speed.
 *
 * Call once, after motor_init(). Measure the real wheel diameter.
 *
 * @param[in,out] p_motors              The motors to configure.
 * @param[in]     wheel_diameter_mm     Wheel diameter in millimetres.
 * @param[in]     pulses_per_wheel_rev  Use MOTOR_DEFAULT_PULSES_PER_WHEEL_REV
 *                                      unless the pulse counting changes.
 */
void
motor_set_wheel_geometry (motor_t * p_motors, float32_t wheel_diameter_mm,
                          uint32_t pulses_per_wheel_rev)
{
    p_motors->wheel_circumference_mm = PI_F * wheel_diameter_mm;
    p_motors->pulses_per_wheel_rev   = pulses_per_wheel_rev;
}

/*!
 * @brief Gets M1's signed net distance since the last reset.
 *
 * Forward and backward cancel out. Requires the wheel geometry to be set.
 *
 * @param[in] p_motors  The motors to read.
 * @return Net distance in millimetres.
 */
float32_t
motor_get_m1_distance_mm (motor_t const * p_motors)
{
    return calc_distance_mm(p_motors->m1.pulses, p_motors);
}

/*!
 * @brief Gets M2's signed net distance since the last reset.
 *
 * @param[in] p_motors  The motors to read.
 * @return Net distance in millimetres.
 */
float32_t
motor_get_m2_distance_mm (motor_t const * p_motors)
{
    return calc_distance_mm(p_motors->m2.pulses, p_motors);
}

/*!
 * @brief Gets M1's odometer distance (always >= 0) since the last reset.
 *
 * @param[in] p_motors  The motors to read.
 * @return Total distance in millimetres.
 */
float32_t
motor_get_m1_total_distance_mm (motor_t const * p_motors)
{
    // Cast: 2^31 pulses is over 700 km of driving, so the count fits.
    return calc_distance_mm((int32_t)p_motors->m1.total_pulses, p_motors);
}

/*!
 * @brief Gets M2's odometer distance (always >= 0) since the last reset.
 *
 * @param[in] p_motors  The motors to read.
 * @return Total distance in millimetres.
 */
float32_t
motor_get_m2_total_distance_mm (motor_t const * p_motors)
{
    // Cast: 2^31 pulses is over 700 km of driving, so the count fits.
    return calc_distance_mm((int32_t)p_motors->m2.total_pulses, p_motors);
}

/*!
 * @brief Gets M1's instantaneous speed.
 *
 * Worked out from the time between the two most recent pulses, so it's
 * accurate even at low speed. Reads 0 if no pulse arrived in the last 0.5 s.
 *
 * @param[in] p_motors  The motors to read.
 * @return Speed in mm/s; negative while reversing.
 */
float32_t
motor_get_m1_speed_mm_s (motor_t const * p_motors)
{
    return calc_speed_mm_s(p_motors, &p_motors->m1);
}

/*!
 * @brief Gets M2's instantaneous speed.
 *
 * @param[in] p_motors  The motors to read.
 * @return Speed in mm/s; negative while reversing.
 */
float32_t
motor_get_m2_speed_mm_s (motor_t const * p_motors)
{
    return calc_speed_mm_s(p_motors, &p_motors->m2);
}

/*!
 * @brief Converts a 0-100 speed to a PWM duty level (0 to wrap).
 */
static uint16_t
convert_pct_to_duty (uint8_t speed_pct)
{
    uint32_t speed = speed_pct;

    if (speed > FULL_SCALE_PCT)
    {
        speed = FULL_SCALE_PCT;
    }

    // Cast: speed is at most 100, so the result is at most the 16-bit wrap.
    return (uint16_t)((speed * g_pwm_wrap) / FULL_SCALE_PCT);
}

/*!
 * @brief Sets up one pin's PWM slice: pin mux, divider and wrap, then enable.
 */
static void
init_pwm_pin (uint32_t pin)
{
    gpio_set_function(pin, GPIO_FUNC_PWM);

    // Each PWM slice drives two pins, so pins can share a slice.
    uint32_t slice_num = pwm_gpio_to_slice_num(pin);

    pwm_set_clkdiv(slice_num, g_pwm_clkdiv);
    pwm_set_wrap(slice_num, g_pwm_wrap);
    pwm_set_gpio_level(pin, 0U);
    pwm_set_enabled(slice_num, true);
}

/*!
 * @brief Drives one motor at -100 to 100 percent. Negative = reverse.
 */
static void
drive_motor (motor_wheel_t const * p_wheel, int16_t speed_pct)
{
    int16_t speed = speed_pct;

    if (speed > MAX_SPEED_PCT)
    {
        speed = MAX_SPEED_PCT;
    }
    else if (speed < -MAX_SPEED_PCT)
    {
        speed = -MAX_SPEED_PCT;
    }
    else
    {
        // Already within range.
    }

    // Cast: speed is now -100 to 100, so its magnitude fits in 8 bits.
    uint8_t  magnitude = (uint8_t)((speed < 0) ? -speed : speed);
    uint16_t duty      = convert_pct_to_duty(magnitude);

    pwm_set_gpio_level(p_wheel->pwm_a_pin, (speed > 0) ? duty : 0U);
    pwm_set_gpio_level(p_wheel->pwm_b_pin, (speed < 0) ? duty : 0U);
}

/*!
 * @brief Encoder interrupt: runs on each falling edge of either A pin.
 *
 * The gpio parameter uses the SDK's uint type because the callback signature
 * (gpio_irq_callback_t) requires it.
 */
static void
encoder_isr (uint gpio, uint32_t events)
{
    // Cast: events is required by the callback signature but not needed.
    (void)events;

    if (NULL != g_p_active_motors)
    {
        uint32_t now_us = time_us_32();

        if (gpio == g_p_active_motors->m1.encoder_a_pin)
        {
            record_encoder_pulse(&g_p_active_motors->m1, now_us);
        }
        else if (gpio == g_p_active_motors->m2.encoder_a_pin)
        {
            record_encoder_pulse(&g_p_active_motors->m2, now_us);
        }
        else
        {
            // Not one of the encoder pins.
        }
    }
}

/*!
 * @brief Counts one encoder pulse for a wheel, unless it's a bounce.
 *
 * Reads the B pin to tell direction, then updates the counts and speed.
 */
static void
record_encoder_pulse (motor_wheel_t * p_wheel, uint32_t now_us)
{
    // Wrap-safe time since the last accepted pulse.
    uint32_t since_last_us = now_us - p_wheel->last_pulse_time_us;

    if (since_last_us >= ENCODER_DEBOUNCE_US)
    {
        // Cast: INT32_MAX is positive, so it converts to uint32_t exactly.
        // Limiting the period keeps the signed multiply below in range.
        if (since_last_us > (uint32_t)INT32_MAX)
        {
            since_last_us = (uint32_t)INT32_MAX;
        }

        // B pin high = forward (+1), low = reverse (-1).
        int32_t direction = gpio_get(p_wheel->encoder_b_pin) ? 1 : -1;

        p_wheel->pulses += direction;
        p_wheel->total_pulses++;

        // Cast: since_last_us was limited to INT32_MAX above.
        p_wheel->last_pulse_period_us = direction * (int32_t)since_last_us;
        p_wheel->last_pulse_time_us   = now_us;
    }
}

/*!
 * @brief Sets up an encoder pin as a pulled-up input, optionally installing
 *        the encoder interrupt callback.
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
                                           &encoder_isr);
    }
}

/*!
 * @brief Sets one wheel's driver pins and clears its encoder state.
 *
 * Encoder pins are UINT32_MAX until motor_attach_encoders() is called.
 */
static void
init_wheel (motor_wheel_t * p_wheel, uint32_t pwm_a_pin, uint32_t pwm_b_pin,
            uint32_t now_us)
{
    p_wheel->pwm_a_pin            = pwm_a_pin;
    p_wheel->pwm_b_pin            = pwm_b_pin;
    p_wheel->encoder_a_pin        = UINT32_MAX;
    p_wheel->encoder_b_pin        = UINT32_MAX;
    p_wheel->pulses               = 0;
    p_wheel->total_pulses         = 0U;
    p_wheel->last_pulse_time_us   = now_us;
    p_wheel->last_pulse_period_us = 0;
}

/*!
 * @brief Converts a pulse count to millimetres using the wheel geometry.
 */
static float32_t
calc_distance_mm (int32_t pulses, motor_t const * p_motors)
{
    float32_t distance_mm = 0.0f;

    if (0U != p_motors->pulses_per_wheel_rev)
    {
        // Cast: pulse counts are far below float's exact-integer limit for
        // any realistic run, and the pulses per rev is a small constant.
        distance_mm = ((float32_t)pulses
                       / (float32_t)p_motors->pulses_per_wheel_rev)
                      * p_motors->wheel_circumference_mm;
    }

    return distance_mm;
}

/*!
 * @brief Works out a wheel's speed from its last pulse time and period.
 */
static float32_t
calc_speed_mm_s (motor_t const * p_motors, motor_wheel_t const * p_wheel)
{
    float32_t speed_mm_s    = 0.0f;
    int32_t   period_us     = p_wheel->last_pulse_period_us;
    uint32_t  since_last_us = time_us_32() - p_wheel->last_pulse_time_us;

    bool b_valid = (0U != p_motors->pulses_per_wheel_rev)
                   && (0 != period_us)
                   && (since_last_us <= SPEED_STALE_US);

    if (b_valid)
    {
        // Cast: period_us is never INT32_MIN (record_encoder_pulse() limits
        // it to +/-INT32_MAX), so its magnitude always fits in uint32_t.
        uint32_t magnitude_us = (period_us < 0) ? (uint32_t)(-period_us)
                                                : (uint32_t)period_us;
        float32_t per_pulse_mm = calc_distance_mm(1, p_motors);

        // Cast: a period of up to 0.5 s is exactly representable as a float.
        speed_mm_s = per_pulse_mm
                     / ((float32_t)magnitude_us / US_PER_SECOND);

        if (period_us < 0)
        {
            speed_mm_s = -speed_mm_s;
        }
    }

    return speed_mm_s;
}

/*** end of file ***/

