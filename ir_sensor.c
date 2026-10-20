/** @file ir_sensor.c
 *
 * @brief Line following (left + centre IR sensors) and Code 39 barcode
 *        scanning (right IR sensor).
 *
 * Sensor wiring (confirmed with the IR sensor test program):
 *   Left   sensor DO -> GP3            (digital, 1 = black)
 *   Centre sensor AO -> GP27 (ADC1)    (analog, ~200 white / ~2500 black)
 *   Right  sensor AO -> GP28 (ADC2)    (analog, barcode scanner, Grove 7)
 *
 * Line following: EDGE FOLLOWING with a PD controller.
 *   The centre sensor's analog reading is turned into a position 0..1
 *   (0 = white, 1 = black). The robot aims for 0.5, i.e. the centre sensor
 *   half over the line's edge, and steers in proportion to how far off it is.
 *   Small drift = small correction, so it no longer snaps left/right.
 *   The side sensor (GP3) is a backstop: if it sees the line, steer hard back.
 *
 * Junctions: when the side sensor AND the right (barcode) sensor both see
 * black, a line crosses the track. The robot drives straight across it
 * instead of treating it as drift, and prints "JUNCTION n".
 *
 * Line lost: if all sensors see white briefly, it brakes and prints
 * "LINE LOST", then sweeps left and right on the spot to look for the line.
 *   Found     -> "LINE FOUND", carries on (it had just drifted off).
 *   Not found -> it's a real dead end: stops and prints "DEAD END".
 * If the "junction" black lasts much longer than a strip of tape (e.g. the
 * robot has reached the edge of the mat and sees the dark floor), it stops
 * and prints "OFF TRACK". Once stopped, put it back on the line to carry on.
 *
 * Barcodes: Code 39 (e.g. *A*, *Z*), printed 141 mm long with 3 mm narrow and
 * 9 mm wide bars, placed 20 mm beside the line. Bar widths are measured with
 * the wheel encoders (distance), so the car's speed doesn't affect the read.
 * The barcode sensor is sampled every 0.2 ms by a repeating timer interrupt,
 * so the main loop being slow (e.g. printing over Wi-Fi) can't distort the
 * widths. The interrupt only records bars and spaces; the finished barcode
 * is decoded (and printed) in the main loop by ir_sensor_update_barcode().
 * Decoded barcodes are collected with ir_sensor_get_barcode() (see main.c).
 *
 * Motors: this module never drives them. ir_sensor_update() fills in an
 * ir_drive_request_t (M1 = left wheel, M2 = right wheel) and the motor
 * control code applies it. The encoders are only read, never reset.
 *
 * Distance: the interrupt keeps its own odometer from the encoders' total
 * pulse counts. It only ever adds the change since the last sample, so
 * another module calling motor_reset_encoder_counts() mid-barcode can't
 * corrupt a bar width.
 */

#include "ir_sensor.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/sync.h"

// ---------------- Sensor pins ----------------
#define LEFT_DO_PIN              3U
#define CENTRE_AO_PIN            27U
#define CENTRE_ADC_CH            1U     // GP27 = ADC1
#define BARCODE_AO_PIN           28U
#define BARCODE_ADC_CH           2U     // GP28 = ADC2

// ---------------- Calibration ----------------
// Centre sensor raw readings on white and on black, from the test program.
#define CENTRE_WHITE             200.0f
#define CENTRE_BLACK             2500.0f
#define CENTRE_SAMPLES           4U     // readings averaged per update

// Barcode edges are found by how far the reading moves, not by fixed
// black/white levels (ADC 0..4095). On black the sensor tracks the highest
// reading (peak); once the reading falls BARCODE_EDGE_DELTA below that peak,
// it has gone white. On white it tracks the lowest reading (dip) and goes
// black once the reading rises BARCODE_EDGE_DELTA above it.
// Why: white paper reads ~200 and bars up to ~3900, but narrow white spaces
// only dip to ~1000-2000 because the sensor's spot is wider than 3 mm. A
// fixed white level misses them; a dip of several hundred counts does not.
//   Too high -> shallow narrow spaces are missed again (bars merge)
//   Too low  -> print specks and noise add extra bars
// Measure with RUN_BARCODE_TEST in main.c (it prints min/max).
#define BARCODE_EDGE_DELTA       500U

// The edge is timed where the reading crossed half of BARCODE_EDGE_DELTA,
// which is nearer the real edge than where the full move completed.
#define BARCODE_EDGE_HALF        (BARCODE_EDGE_DELTA / 2U)

// Only used once, at start-up, to pick the colour the sensor starts on.
#define BARCODE_START_BLACK      1750U
#define ADC_MAX_READING          4095U

// 1 = after each barcode, print all its widths on one line.
#define BARCODE_DEBUG            1

// Bar widths are recorded both as wheel distance (encoders) and as time.
// Distance is used when the encoders actually moved during the barcode (speed
// doesn't matter then); otherwise it falls back to time automatically.
// The barcode must cover at least this distance for distance to be trusted.
#define BARCODE_MIN_ENCODER_MM   60.0f

// A colour change only counts if it lasts this many samples
// (5 x 0.2 ms = 1 ms). Stops dust or print specks from adding extra bars.
#define BARCODE_DEBOUNCE_SAMPLES 5U

// Which side of the line the GP3 sensor sits on: 1.0f = right, -1.0f = left.
// Test: hold the robot so ONLY GP3 is over the line. The wheels must turn the
// robot TOWARDS GP3. If they turn it away, flip this sign.
#define SIDE_SENSOR_SIDE         1.0f

// ---------------- Line following tuning ----------------
// Motor speeds in percent (0..100). Tune slowly first, then speed up.
#define BASE_SPEED               30.0f
#define MAX_SPEED_PCT            100.0f

// Steering = KP * error + KD * rate_of_change_of_error.
// error runs from -0.5 (centre sensor on white) to +0.5 (fully on black).
//   KP too low  -> lazy, drifts off on curves
//   KP too high -> wobbles side to side
//   KD damps the wobble. Tune KP first with KD = 0, then add KD.
#define KP                       25.0f
#define KD                       0.5f

// Errors smaller than this are ignored, so tiny sensor noise around the edge
// doesn't cause constant micro-corrections. Raise to 0.08 if it still wiggles.
#define ERR_DEADBAND             0.07f

// The rate of change of error is smoothed by this fraction each update,
// because differentiating a noisy reading is noisy.
#define ERROR_RATE_FILTER        0.3f

// Centre positions (0 = white, 1 = black).
#define LINE_EDGE_POS            0.5f   // target: half over the line's edge
#define LINE_WHITE_POS           0.1f   // below this the centre is on white
#define BARCODE_LINE_POS         0.3f   // centre still on the line at a barcode

#define SLOW_IN_TURNS            0.6f   // forward speed drops by this x |steer|
#define SIDE_STEER               30.0f  // hard correction when GP3 sees line

// GP3 must stay black this long before the hard correction kicks in. This
// gives the right sensor time to also see black if it's actually a junction.
#define SIDE_CONFIRM_MS          30U
#define SIDE_CONFIRM_US          (SIDE_CONFIRM_MS * 1000U)

// While a barcode is being read (at least this many bars and spaces, and an
// edge this recently), GP3 seeing black is the barcode, not the line.
#define BARCODE_SIDE_MIN_ELEMENTS 2U
#define BARCODE_RECENT_EDGE_MS   500U
#define BARCODE_RECENT_EDGE_US   (BARCODE_RECENT_EDGE_MS * 1000U)

#define JUNCTION_SPEED           30.0f

// A real crossing line passes under ALL three sensors, so the centre sensor
// must also be on solid black. At a barcode the centre stays on the line edge
// (~0.5), so a barcode can't be mistaken for a junction.
#define JUNCTION_MIN_POS         0.8f

// Both outer sensors white this long = junction crossed.
#define JUNCTION_CLEAR_MS        40U
#define JUNCTION_CLEAR_US        (JUNCTION_CLEAR_MS * 1000U)

// A crossing strip of tape is crossed within this distance (measured with the
// wheel encoders, so it works at any speed). Black for longer = not tape
// (mat edge, dark floor) -> stop. JUNCTION_MAX_US is a backup in case the
// encoders aren't counting.
#define JUNCTION_MAX_MM          60.0f
#define JUNCTION_MAX_MS          1500U
#define JUNCTION_MAX_US          (JUNCTION_MAX_MS * 1000U)

// Centre + side on white this long = line lost -> brake.
#define LINE_LOST_MS             120U
#define LINE_LOST_US             (LINE_LOST_MS * 1000U)

// Line search after the line is lost: pivot on the spot one way for
// SWEEP_US, then the other way for 2 x SWEEP_US, then back to the middle.
// Raise SWEEP_US if the sweep is too narrow to find a line it drifted off.
#define SEARCH_SPEED             30.0f
#define SWEEP_MS                 400U
#define SWEEP_US                 (SWEEP_MS * 1000U)

// Forward speed eases towards its target by this fraction each update.
// (Steering is NOT smoothed: smoothing adds lag, and lag causes wobble.)
#define SMOOTHING                0.15f

#define LINE_PERIOD_MS           5U     // line-following update every 5 ms
#define LINE_PERIOD_US           (LINE_PERIOD_MS * 1000U)
#define LINE_PERIOD_S            (LINE_PERIOD_US / 1e6f)
#define BARCODE_SAMPLE_US        200    // barcode sampled every 0.2 ms

// The odometer adds up both wheels' pulses, so it counts 2 per wheel pulse.
#define ODOMETER_WHEELS          2.0f

// A barcode is finished after this much white following the last bar, AND
// the white must also be longer than 4 x the widest element seen so far
// (measured in time), so a slow pass isn't cut off at a wide space. It never
// waits for more than BARCODE_END_MAX_US of white.
#define BARCODE_END_MS           300U   // at least 300 ms
#define BARCODE_END_US           (BARCODE_END_MS * 1000U)
#define BARCODE_END_FACTOR       4U
#define BARCODE_END_MAX_MS       1000U
#define BARCODE_END_MAX_US       (BARCODE_END_MAX_MS * 1000U)

// Any single bar or space longer than this isn't part of a barcode (the car
// was stopped or lifted), so the scan is thrown away and starts fresh.
#define BARCODE_MAX_ELEMENT_MS   1500U
#define BARCODE_MAX_ELEMENT_US   (BARCODE_MAX_ELEMENT_MS * 1000U)

// Backup: give up on a barcode if no new bar arrives for this long
// (e.g. the car stopped on top of it).
#define BARCODE_TIMEOUT_MS       4000U
#define BARCODE_TIMEOUT_US       (BARCODE_TIMEOUT_MS * 1000U)

#define MAX_ELEMENTS             200U   // bars + spaces, ~20 characters

// Code 39 layout: each character is 9 elements (bar, space, ... bar), 3 of
// them wide, and characters are separated by a 1-element gap.
#define CODE39_CHAR_ELEMENTS     9U
#define CODE39_CHAR_BARS         5U     // elements 0, 2, 4, 6, 8
#define CODE39_CHAR_SPACES       4U     // elements 1, 3, 5, 7

// Most characters have 2 wide bars and 1 wide space; $ / + % have no wide
// bars and 3 wide spaces. If the widest bar is less than this many times the
// narrowest, the bars are taken to be all narrow.
#define CODE39_WIDE_BAR_RATIO    2U
#define CODE39_CHAR_PITCH        10U    // 9 elements + 1 gap
#define CODE39_MIN_CHARS         3U     // start '*' + data + stop '*'
#define CODE39_FRAME_CHARS       2U     // the start and stop '*'
#define CODE39_START_STOP        '*'
#define CODE39_TABLE_SIZE        (sizeof(g_code39_table) \
                                  / sizeof(g_code39_table[0]))

#define BARCODE_TEXT_SIZE        24U
#define BARCODE_DECODE_SIZE      32U
#define BARCODE_DECODE_ATTEMPTS  2U     // distance first, then time

#define US_PER_MS                1000.0f

// Size of the debug summary line, and how close to full it may get.
#define BARCODE_LINE_SIZE        ((12 * 200 / 2) + 200)
#define BARCODE_LINE_LIMIT       (BARCODE_LINE_SIZE - 8)

typedef enum
{
    LINE_FOLLOW,
    LINE_JUNCTION,
    LINE_SEARCH,
    LINE_STOPPED
} line_state_t;

// The sensor readings used by one line-following update.
typedef struct
{
    bool      b_side_black;   // GP3 sees black.
    bool      b_right_black;  // The barcode sensor sees black.
    float32_t position;       // Centre position, 0 = white, 1 = black.
    float32_t error;          // position - LINE_EDGE_POS.
} line_reading_t;

// One Code 39 character: 9 bits, bar first, 1 = wide, 0 = narrow.
typedef struct
{
    char     character;
    uint32_t pattern;
} code39_entry_t;

static const code39_entry_t g_code39_table[] =
{
    {'0', 0x034U},  // 000110100
    {'1', 0x121U},  // 100100001
    {'2', 0x061U},  // 001100001
    {'3', 0x160U},  // 101100000
    {'4', 0x031U},  // 000110001
    {'5', 0x130U},  // 100110000
    {'6', 0x070U},  // 001110000
    {'7', 0x025U},  // 000100101
    {'8', 0x124U},  // 100100100
    {'9', 0x064U},  // 001100100
    {'A', 0x109U},  // 100001001
    {'B', 0x049U},  // 001001001
    {'C', 0x148U},  // 101001000
    {'D', 0x019U},  // 000011001
    {'E', 0x118U},  // 100011000
    {'F', 0x058U},  // 001011000
    {'G', 0x00DU},  // 000001101
    {'H', 0x10CU},  // 100001100
    {'I', 0x04CU},  // 001001100
    {'J', 0x01CU},  // 000011100
    {'K', 0x103U},  // 100000011
    {'L', 0x043U},  // 001000011
    {'M', 0x142U},  // 101000010
    {'N', 0x013U},  // 000010011
    {'O', 0x112U},  // 100010010
    {'P', 0x052U},  // 001010010
    {'Q', 0x007U},  // 000000111
    {'R', 0x106U},  // 100000110
    {'S', 0x046U},  // 001000110
    {'T', 0x016U},  // 000010110
    {'U', 0x181U},  // 110000001
    {'V', 0x0C1U},  // 011000001
    {'W', 0x1C0U},  // 111000000
    {'X', 0x091U},  // 010010001
    {'Y', 0x190U},  // 110010000
    {'Z', 0x0D0U},  // 011010000
    {'-', 0x085U},  // 010000101
    {'.', 0x184U},  // 110000100
    {' ', 0x0C4U},  // 011000100
    {'*', 0x094U},  // 010010100
    {'$', 0x0A8U},  // 010101000
    {'/', 0x0A2U},  // 010100010
    {'+', 0x08AU},  // 010001010
    {'%', 0x02AU},  // 000101010
};

// ---------------- Barcode sensor state ----------------
// Written by the barcode timer interrupt. The volatile ones are also read by
// the main loop; all are 32 bits or less, so each read is atomic.
// Time is the low 32 bits of the microsecond timer: it wraps every ~71
// minutes, which unsigned subtraction handles.

// Right sensor colour, also used for junctions.
static volatile bool     g_b_bar_black      = false;
static volatile bool     g_b_scanning       = false;  // a barcode is being read
static volatile uint32_t g_element_count    = 0U;     // bars + spaces so far
static volatile uint32_t g_last_edge_us     = 0U;     // time of the last edge
static volatile uint16_t g_right_raw        = 0U;     // latest reading

// Set by the interrupt when a barcode has ended; the main loop decodes it and
// clears this. The recorded widths aren't touched while it is set.
static volatile bool     g_b_decode_pending = false;

// Odometer: both wheels' pulses added up, since start-up. Unlike the encoder
// counts in motor_t, it is never reset, so it only goes up.
static volatile uint32_t g_odometer         = 0U;
static uint32_t          g_last_m1_pulses   = 0U;     // at the last sample
static uint32_t          g_last_m2_pulses   = 0U;
static motor_t const *   g_p_motors         = NULL;   // encoders to read
static repeating_timer_t g_barcode_timer;

// Widths used for decoding (copied from one of the two arrays after them).
static uint32_t  g_widths[MAX_ELEMENTS];
static uint32_t  g_widths_pulses[MAX_ELEMENTS]; // as odometer pulses
static uint32_t  g_widths_us[MAX_ELEMENTS];     // as time, us

static uint32_t  g_last_edge_pos      = 0U;     // odometer at the last edge
static uint32_t  g_pending_samples    = 0U;     // samples a new colour lasted
static uint32_t  g_pending_pos        = 0U;     // where the new colour began
static uint32_t  g_pending_us         = 0U;
static bool      g_b_pending_crossed  = false;  // g_pending_pos/us are set

// Highest reading since the bar began (on black), or lowest since the space
// began (on white). Edges are measured from here.
static uint32_t  g_extreme            = 0U;

// Darkest and lightest readings in the current element.
static uint32_t  g_element_min        = ADC_MAX_READING;
static uint32_t  g_element_max        = 0U;

// Lowest "darkest" reading of any bar, and highest "lightest" reading of any
// space, in this barcode.
static uint32_t  g_weakest_bar        = ADC_MAX_READING;
static uint32_t  g_weakest_space      = 0U;
static uint32_t  g_widest_us          = 0U;     // widest element (time)

static char      g_last_barcode[BARCODE_TEXT_SIZE];
static bool      g_b_barcode_ready    = false;

// ---------------- Line following state ----------------
static line_state_t g_line_state            = LINE_FOLLOW;
static float32_t    g_forward               = 0.0f;  // forward speed, %
static float32_t    g_prev_error            = 0.0f;
static float32_t    g_error_rate            = 0.0f;  // filtered, 1/s
static uint64_t     g_side_black_since_us   = 0U;    // 0 = side on white
static uint64_t     g_all_white_since_us    = 0U;    // 0 = line visible
static uint64_t     g_junction_clear_since_us = 0U;
static uint64_t     g_junction_start_us     = 0U;
static uint32_t     g_junction_start_pos    = 0U;    // odometer
static uint64_t     g_search_start_us       = 0U;
static int32_t      g_junction_count        = 0;

// ---------------- Scheduling ----------------
static uint64_t     g_next_line_us          = 0U;

static float32_t convert_pulses_to_mm (uint32_t pulses);
static bool      is_line_found (line_reading_t const * p_reading);
static void      resume_line_follow (void);
static uint16_t  read_centre (void);
static float32_t read_centre_position (void);
static int16_t   clamp_speed (float32_t speed);
static void      step_line_follow (uint64_t now_us,
                                   ir_drive_request_t * p_request);
static void      follow_junction (line_reading_t const * p_reading,
                                  uint64_t now_us,
                                  float32_t * p_target_forward);
static void      follow_search (line_reading_t const * p_reading,
                                uint64_t now_us,
                                float32_t * p_target_forward,
                                float32_t * p_steer);
static void      follow_stopped (line_reading_t const * p_reading,
                                 float32_t * p_target_forward);
static void      follow_line (line_reading_t const * p_reading,
                              uint64_t now_us,
                              float32_t * p_target_forward,
                              float32_t * p_steer);
static void      sort_by_width (uint32_t const p_elements[], uint32_t first,
                                uint32_t p_order[]);
static char      decode_char (uint32_t const p_elements[]);
static bool      decode_barcode (bool b_reverse, char * p_out,
                                 size_t out_size);
static void      print_barcode_summary (float32_t total_mm);
static void      finish_barcode (void);
static void      reset_barcode_scan (bool b_scanning);
static void      record_barcode_element (bool b_now_black);
static void      accept_barcode_edge (bool b_now_black);
static void      check_barcode_end (uint32_t now_us);
static uint32_t  get_pulses_since (uint32_t pulses, uint32_t * p_last);
static void      update_odometer (void);
static void      sample_barcode (uint32_t now_us);
static bool      barcode_timer_isr (repeating_timer_t * p_timer);

/*!
 * @brief Sets up the IR sensor pins and the ADC, and starts sampling the
 *        barcode sensor in a timer interrupt.
 *
 * Call once at start-up, after the encoders and wheel geometry are set up.
 *
 * @param[in] p_motors  The motors whose encoders measure bar widths. Only
 *                      read, never changed. Must stay valid from now on.
 */
void
ir_sensor_init (motor_t const * p_motors)
{
    gpio_init(LEFT_DO_PIN);
    gpio_set_dir(LEFT_DO_PIN, GPIO_IN);

    adc_init();
    adc_gpio_init(CENTRE_AO_PIN);
    adc_gpio_init(BARCODE_AO_PIN);

    g_p_motors       = p_motors;
    g_last_m1_pulses = motor_get_m1_total_pulses(p_motors);
    g_last_m2_pulses = motor_get_m2_total_pulses(p_motors);

    // Start from the colour under the sensor now, since edges are only found
    // relative to the last peak or dip.
    adc_select_input(BARCODE_ADC_CH);
    g_extreme     = adc_read();
    g_b_bar_black = (g_extreme > BARCODE_START_BLACK);

    // Negative period = every 0.2 ms from the start of the last call, so the
    // sample rate stays fixed however long the interrupt takes.
    if (!add_repeating_timer_us(-BARCODE_SAMPLE_US, &barcode_timer_isr, NULL,
                                &g_barcode_timer))
    {
        printf("IR: no free timer, barcode scanning disabled\n");
    }
}

/*!
 * @brief Runs line following, and decodes any barcode that has been read.
 *
 * Non-blocking: call it as often as possible from the main loop. The
 * steering is worked out every 5 ms. It does NOT drive the motors: when it
 * returns true, the motor control code should apply *p_request.
 *
 * @param[out] p_request  The wheel speeds line following wants.
 * @return true if *p_request holds a new request.
 */
bool
ir_sensor_update (ir_drive_request_t * p_request)
{
    bool     b_new_request = false;
    uint64_t now_us        = time_us_64();

    ir_sensor_update_barcode();

    if (now_us >= g_next_line_us)
    {
        step_line_follow(now_us, p_request);
        g_next_line_us = now_us + LINE_PERIOD_US;
        b_new_request  = true;
    }

    return b_new_request;
}

/*!
 * @brief Decodes the barcode the timer interrupt has finished reading, if
 *        any (no line following, motors untouched).
 *
 * Called by ir_sensor_update(); call it directly for the barcode tests in
 * main.c. Decoding and printing happen here, not in the interrupt.
 */
void
ir_sensor_update_barcode (void)
{
    if (g_b_decode_pending)
    {
        // Don't let the compiler read the widths before the flag.
        __compiler_memory_barrier();
        finish_barcode();
        __compiler_memory_barrier();

        // Hands the width arrays back to the interrupt.
        g_b_decode_pending = false;
    }
}

/*!
 * @brief Collects a newly decoded barcode.
 *
 * Returns true once for each newly decoded barcode and copies its text
 * (without the * start/stop characters) into p_out, e.g. "A".
 *
 * @param[out] p_out  Buffer for the barcode text.
 * @param[in]  size   Size of p_out in bytes.
 * @return true if a new barcode was copied.
 */
bool
ir_sensor_get_barcode (char * p_out, size_t size)
{
    bool b_got = (g_b_barcode_ready && (size > 0U));

    if (b_got)
    {
        strncpy(p_out, g_last_barcode, size - 1U);
        p_out[size - 1U]  = '\0';
        g_b_barcode_ready = false;
    }

    return b_got;
}

/*!
 * @brief Gets the latest raw barcode sensor reading, for calibration.
 *
 * @return ADC reading, 0 to 4095.
 */
uint16_t
ir_sensor_get_barcode_raw (void)
{
    return g_right_raw;
}

/*!
 * @brief Converts an odometer distance (both wheels' pulses added up) to the
 *        average distance each wheel travelled, in mm.
 *
 * Returns 0 if the wheel geometry hasn't been set.
 */
static float32_t
convert_pulses_to_mm (uint32_t pulses)
{
    float32_t distance_mm = 0.0f;

    if (0U != g_p_motors->pulses_per_wheel_rev)
    {
        // Cast: pulse counts are far below float's exact-integer limit for
        // any realistic run, and the pulses per rev is a small constant.
        distance_mm = ((float32_t)pulses
                       / (ODOMETER_WHEELS
                          * (float32_t)g_p_motors->pulses_per_wheel_rev))
                      * g_p_motors->wheel_circumference_mm;
    }

    return distance_mm;
}

/*!
 * @brief Checks whether the centre sensor is back on the line (and it's not
 *        the dark floor).
 */
static bool
is_line_found (line_reading_t const * p_reading)
{
    bool b_on_line  = (p_reading->position > LINE_EDGE_POS)
                      || p_reading->b_side_black;
    bool b_on_floor = p_reading->b_side_black && p_reading->b_right_black;

    return (b_on_line && !b_on_floor);
}

/*!
 * @brief Goes back to normal line following with fresh timers.
 */
static void
resume_line_follow (void)
{
    g_line_state          = LINE_FOLLOW;
    g_all_white_since_us  = 0U;
    g_side_black_since_us = 0U;
    g_error_rate          = 0.0f;
}

/*!
 * @brief Averages a few ADC samples to take the noise out of the centre
 *        reading.
 */
static uint16_t
read_centre (void)
{
    uint32_t sum = 0U;

    // The barcode timer interrupt also uses the ADC (on another channel).
    // Hold it off for these few reads (~10 us), or it could switch the
    // channel between adc_select_input() and adc_read().
    uint32_t saved_interrupts = save_and_disable_interrupts();

    adc_select_input(CENTRE_ADC_CH);

    for (uint32_t idx = 0U; idx < CENTRE_SAMPLES; idx++)
    {
        sum += adc_read();
    }

    restore_interrupts(saved_interrupts);

    // Cast: the average of 12-bit readings is at most 4095.
    return (uint16_t)(sum / CENTRE_SAMPLES);
}

/*!
 * @brief Reads the centre sensor as a position: 0 = white, 1 = black.
 */
static float32_t
read_centre_position (void)
{
    // Cast: a 12-bit reading is exactly representable as a float.
    float32_t position = ((float32_t)read_centre() - CENTRE_WHITE)
                         / (CENTRE_BLACK - CENTRE_WHITE);

    if (position < 0.0f)
    {
        position = 0.0f;
    }
    else if (position > 1.0f)
    {
        position = 1.0f;
    }
    else
    {
        // Already between 0 and 1.
    }

    return position;
}

/*!
 * @brief Limits a wheel speed to -100..100 percent.
 */
static int16_t
clamp_speed (float32_t speed)
{
    int16_t clamped = 0;

    if (speed < -MAX_SPEED_PCT)
    {
        clamped = -100;
    }
    else if (speed > MAX_SPEED_PCT)
    {
        clamped = 100;
    }
    else
    {
        // Cast: speed is within -100..100 here, so it fits in 16 bits.
        // The fraction is dropped, as before.
        clamped = (int16_t)speed;
    }

    return clamped;
}

/*!
 * @brief One line-following update: reads the sensors, runs the current
 *        state and works out the wheel speeds to ask for.
 */
static void
step_line_follow (uint64_t now_us, ir_drive_request_t * p_request)
{
    line_reading_t reading;

    reading.b_side_black  = gpio_get(LEFT_DO_PIN);
    reading.b_right_black = g_b_bar_black;
    reading.position      = read_centre_position();
    reading.error         = reading.position - LINE_EDGE_POS;

    // Rate of change of error, lightly filtered because differentiating is
    // noisy.
    float32_t rate = (reading.error - g_prev_error) / LINE_PERIOD_S;

    g_error_rate += (rate - g_error_rate) * ERROR_RATE_FILTER;
    g_prev_error  = reading.error;

    // Both outer sensors on black at once = a line crossing the track.
    if ((LINE_FOLLOW == g_line_state) && reading.b_side_black
        && reading.b_right_black && (reading.position > JUNCTION_MIN_POS))
    {
        g_line_state              = LINE_JUNCTION;
        g_junction_clear_since_us = 0U;
        g_junction_start_us       = now_us;
        g_junction_start_pos      = g_odometer;
        g_junction_count++;
        printf("JUNCTION %" PRId32 "\n", g_junction_count);
    }

    float32_t target_forward = BASE_SPEED;
    float32_t steer          = 0.0f;  // positive = turn right

    switch (g_line_state)
    {
        case LINE_JUNCTION:
            follow_junction(&reading, now_us, &target_forward);
        break;

        case LINE_SEARCH:
            follow_search(&reading, now_us, &target_forward, &steer);
        break;

        case LINE_STOPPED:
            follow_stopped(&reading, &target_forward);
        break;

        case LINE_FOLLOW:
            follow_line(&reading, now_us, &target_forward, &steer);
        break;

        default:
            // Unknown state: treat it like normal line following.
            follow_line(&reading, now_us, &target_forward, &steer);
        break;
    }

    g_forward += (target_forward - g_forward) * SMOOTHING;
    p_request->left_pct  = clamp_speed(g_forward + steer);
    p_request->right_pct = clamp_speed(g_forward - steer);
}

/*!
 * @brief Junction state: drives straight across until both outer sensors are
 *        back on white, or stops if the black goes on far too long.
 */
static void
follow_junction (line_reading_t const * p_reading, uint64_t now_us,
                 float32_t * p_target_forward)
{
    *p_target_forward = JUNCTION_SPEED;

    if (!p_reading->b_side_black && !p_reading->b_right_black)
    {
        if (0U == g_junction_clear_since_us)
        {
            g_junction_clear_since_us = now_us;
        }

        if ((now_us - g_junction_clear_since_us) > JUNCTION_CLEAR_US)
        {
            resume_line_follow();
        }
    }
    else
    {
        g_junction_clear_since_us = 0U;

        bool b_too_far  = (convert_pulses_to_mm(g_odometer
                                                - g_junction_start_pos)
                           > JUNCTION_MAX_MM);
        bool b_too_long = ((now_us - g_junction_start_us) > JUNCTION_MAX_US);

        if (b_too_far || b_too_long)
        {
            // Far too long to be a strip of tape -> off the track.
            g_line_state      = LINE_STOPPED;
            *p_target_forward = 0.0f;
            g_forward         = 0.0f;
            printf("OFF TRACK\n");
        }
    }
}

/*!
 * @brief Search state: sweeps on the spot to find the line again.
 *
 * Sweeps away from the side sensor first (the likely way it drifted), then
 * back past the middle to the other side, then returns to the middle.
 */
static void
follow_search (line_reading_t const * p_reading, uint64_t now_us,
               float32_t * p_target_forward, float32_t * p_steer)
{
    uint64_t elapsed_us = now_us - g_search_start_us;

    *p_target_forward = 0.0f;
    g_forward         = 0.0f;

    if (is_line_found(p_reading))
    {
        resume_line_follow();
        printf("LINE FOUND\n");
    }
    else if (elapsed_us < SWEEP_US)
    {
        *p_steer = -SIDE_SENSOR_SIDE * SEARCH_SPEED;
    }
    else if (elapsed_us < (3U * SWEEP_US))
    {
        *p_steer = SIDE_SENSOR_SIDE * SEARCH_SPEED;
    }
    else if (elapsed_us < (4U * SWEEP_US))
    {
        *p_steer = -SIDE_SENSOR_SIDE * SEARCH_SPEED;
    }
    else
    {
        g_line_state = LINE_STOPPED;
        printf("DEAD END\n");
    }
}

/*!
 * @brief Stopped state: waits to be put back on the line, then carries on.
 */
static void
follow_stopped (line_reading_t const * p_reading,
                float32_t * p_target_forward)
{
    bool b_on_floor = p_reading->b_side_black && p_reading->b_right_black;

    *p_target_forward = 0.0f;
    g_forward         = 0.0f;

    if ((p_reading->position > LINE_EDGE_POS) && !b_on_floor)
    {
        resume_line_follow();
        printf("LINE FOUND\n");
    }
}

/*!
 * @brief Normal state: PD steering along the line's edge, with the side
 *        sensor as a backstop and line-lost detection.
 *
 * More black under the centre sensor = the robot has drifted away from the
 * side sensor, so steer towards it; more white = steer away from it.
 */
static void
follow_line (line_reading_t const * p_reading, uint64_t now_us,
             float32_t * p_target_forward, float32_t * p_steer)
{
    float32_t error = p_reading->error;

    if (error > ERR_DEADBAND)
    {
        error -= ERR_DEADBAND;
    }
    else if (error < -ERR_DEADBAND)
    {
        error += ERR_DEADBAND;
    }
    else
    {
        error = 0.0f;
    }

    *p_steer = SIDE_SENSOR_SIDE * ((KP * error) + (KD * g_error_rate));

    // Backstop: side sensor on the line for a while -> steer hard back.
    // While a barcode is being read and the centre is still on the line,
    // GP3 seeing black is the barcode, not the line, so ignore it.
    // Only while a real barcode is under way (a few bars, recent edge).
    // Read the edge time before the clock, so a new edge from the interrupt
    // can't make it later than "now".
    uint32_t last_edge_us      = g_last_edge_us;
    uint32_t since_edge_us     = time_us_32() - last_edge_us;
    bool     b_side_is_barcode = g_b_scanning
        && (g_element_count >= BARCODE_SIDE_MIN_ELEMENTS)
        && (since_edge_us < BARCODE_RECENT_EDGE_US)
        && (p_reading->position > BARCODE_LINE_POS);

    if (p_reading->b_side_black && !b_side_is_barcode)
    {
        if (0U == g_side_black_since_us)
        {
            g_side_black_since_us = now_us;
        }

        if ((now_us - g_side_black_since_us) > SIDE_CONFIRM_US)
        {
            *p_steer = SIDE_SENSOR_SIDE * SIDE_STEER;
        }
    }
    else
    {
        g_side_black_since_us = 0U;
    }

    // Nothing black anywhere for too long -> end of line or ran off it.
    bool b_lost = false;

    if (!p_reading->b_side_black && (p_reading->position < LINE_WHITE_POS))
    {
        if (0U == g_all_white_since_us)
        {
            g_all_white_since_us = now_us;
        }

        if ((now_us - g_all_white_since_us) > LINE_LOST_US)
        {
            b_lost            = true;
            g_line_state      = LINE_SEARCH;
            g_search_start_us = now_us;
            *p_steer          = 0.0f;
            *p_target_forward = 0.0f;
            g_forward         = 0.0f;   // brake now, don't ease off
            printf("LINE LOST\n");
        }
    }
    else
    {
        g_all_white_since_us = 0U;
    }

    // Slow down in proportion to how hard it's turning, except while a
    // barcode is being read: then hold a steady speed so time-based widths
    // stay accurate if the encoders can't be used.
    if (!b_lost)
    {
        *p_target_forward = g_b_scanning
            ? BASE_SPEED
            : (BASE_SPEED - (SLOW_IN_TURNS * fabsf(*p_steer)));
    }
}

/*!
 * @brief Sorts a character's bars or spaces by width, narrowest first.
 *
 * @param[in]  p_elements  The character's 9 widths.
 * @param[in]  first       0 for the 5 bars, 1 for the 4 spaces.
 * @param[out] p_order     Their element indices, narrowest first.
 */
static void
sort_by_width (uint32_t const p_elements[], uint32_t first,
               uint32_t p_order[])
{
    uint32_t count = 0U;

    for (uint32_t idx = first; idx < CODE39_CHAR_ELEMENTS; idx += 2U)
    {
        uint32_t pos = count;

        while ((pos > 0U) && (p_elements[p_order[pos - 1U]] > p_elements[idx]))
        {
            p_order[pos] = p_order[pos - 1U];
            pos--;
        }

        p_order[pos] = idx;
        count++;
    }
}

/*!
 * @brief Decodes 9 element widths into a character.
 *
 * Bars are only compared with bars, and spaces with spaces. The sensor's wide
 * spot shortens bars and stretches the spaces beside them, so a narrow space
 * can measure longer than a wide bar (seen in testing: 31 ms narrow spaces
 * next to 28 ms wide bars).
 *
 * @return The character, or '\0' if the widths don't match one.
 */
static char
decode_char (uint32_t const p_elements[])
{
    uint32_t bars[CODE39_CHAR_BARS];
    uint32_t spaces[CODE39_CHAR_SPACES];

    sort_by_width(p_elements, 0U, bars);
    sort_by_width(p_elements, 1U, spaces);

    uint32_t narrowest_bar = p_elements[bars[0]];
    uint32_t widest_bar    = p_elements[bars[CODE39_CHAR_BARS - 1U]];
    bool     b_wide_bars   = (widest_bar
                              >= (CODE39_WIDE_BAR_RATIO * narrowest_bar));

    // 2 wide bars + 1 wide space, or no wide bars + 3 wide spaces.
    uint32_t wide_bars   = b_wide_bars ? 2U : 0U;
    uint32_t wide_spaces = b_wide_bars ? 1U : 3U;
    uint32_t pattern     = 0U;

    // Bit 8 is element 0, so element idx is bit (8 - idx).
    for (uint32_t count = 1U; count <= wide_bars; count++)
    {
        pattern |= 1U << (CODE39_CHAR_ELEMENTS - 1U
                          - bars[CODE39_CHAR_BARS - count]);
    }

    for (uint32_t count = 1U; count <= wide_spaces; count++)
    {
        pattern |= 1U << (CODE39_CHAR_ELEMENTS - 1U
                          - spaces[CODE39_CHAR_SPACES - count]);
    }

    char result = '\0';

    for (uint32_t idx = 0U; (idx < CODE39_TABLE_SIZE) && ('\0' == result);
         idx++)
    {
        if (g_code39_table[idx].pattern == pattern)
        {
            result = g_code39_table[idx].character;
        }
    }

    return result;
}

/*!
 * @brief Decodes the recorded elements in one direction.
 *
 * @return true on success, with the text (without the '*'s) in p_out.
 */
static bool
decode_barcode (bool b_reverse, char * p_out, size_t out_size)
{
    // n characters = 9n elements + (n - 1) gaps between characters.
    uint32_t n_chars = (g_element_count + 1U) / CODE39_CHAR_PITCH;
    bool     b_ok    = (0U == ((g_element_count + 1U) % CODE39_CHAR_PITCH))
                       && (n_chars >= CODE39_MIN_CHARS)
                       && ((n_chars - CODE39_FRAME_CHARS) < out_size);
    char     chars[(MAX_ELEMENTS / CODE39_CHAR_PITCH) + 1U];

    for (uint32_t char_idx = 0U; b_ok && (char_idx < n_chars); char_idx++)
    {
        uint32_t elements[CODE39_CHAR_ELEMENTS];

        for (uint32_t idx = 0U; idx < CODE39_CHAR_ELEMENTS; idx++)
        {
            uint32_t element_idx = (char_idx * CODE39_CHAR_PITCH) + idx;

            elements[idx] = b_reverse
                ? g_widths[g_element_count - 1U - element_idx]
                : g_widths[element_idx];
        }

        chars[char_idx] = decode_char(elements);
        b_ok            = ('\0' != chars[char_idx]);
    }

    if (b_ok)
    {
        b_ok = (CODE39_START_STOP == chars[0])
               && (CODE39_START_STOP == chars[n_chars - 1U]);
    }

    if (b_ok)
    {
        memcpy(p_out, &chars[1], n_chars - CODE39_FRAME_CHARS);
        p_out[n_chars - CODE39_FRAME_CHARS] = '\0';
    }

    return b_ok;
}

/*!
 * @brief Prints one summary line for the barcode just read.
 */
static void
print_barcode_summary (float32_t total_mm)
{
    char    line[BARCODE_LINE_SIZE];
    int32_t len = snprintf(line, sizeof(line),
                           "barcode: %" PRIu32 " elements, encoders %.0f mm, "
                           "weakest bar %" PRIu32 " / space %" PRIu32
                           " | ms:",
                           g_element_count, total_mm,
                           g_weakest_bar, g_weakest_space);

    // Casts: (float32_t) is for printing only, where float precision is
    // plenty. (size_t)len: snprintf() can only fail on encoding errors, which
    // these formats can't produce, so len is never negative.
    for (uint32_t idx = 0U;
         (idx < g_element_count) && (len < BARCODE_LINE_LIMIT); idx++)
    {
        len += snprintf(&line[len], sizeof(line) - (size_t)len, " %.0f",
                        (float32_t)g_widths_us[idx] / US_PER_MS);
    }

    len += snprintf(&line[len], sizeof(line) - (size_t)len, " | mm:");

    for (uint32_t idx = 0U;
         (idx < g_element_count) && (len < BARCODE_LINE_LIMIT); idx++)
    {
        len += snprintf(&line[len], sizeof(line) - (size_t)len, " %.1f",
                        convert_pulses_to_mm(g_widths_pulses[idx]));
    }

    printf("%s\n", line);
}

/*!
 * @brief Decodes the barcode that has just been read.
 *
 * Tries wheel distance first (if the encoders moved enough), then time. Each
 * is tried in both directions so barcodes read either way round.
 * Main loop only: called while g_b_decode_pending holds the widths still.
 */
static void
finish_barcode (void)
{
    char     text[BARCODE_DECODE_SIZE];
    uint32_t total_pulses = 0U;

    for (uint32_t idx = 0U; idx < g_element_count; idx++)
    {
        total_pulses += g_widths_pulses[idx];
    }

    float32_t total_mm = convert_pulses_to_mm(total_pulses);

#if BARCODE_DEBUG
    print_barcode_summary(total_mm);
#endif

    bool         b_ok   = false;
    char const * p_used = "";

    for (uint32_t attempt = 0U;
         (attempt < BARCODE_DECODE_ATTEMPTS) && !b_ok; attempt++)
    {
        bool b_distance = (0U == attempt);

        if (!b_distance || (total_mm >= BARCODE_MIN_ENCODER_MM))
        {
            for (uint32_t idx = 0U; idx < g_element_count; idx++)
            {
                g_widths[idx] = b_distance ? g_widths_pulses[idx]
                                           : g_widths_us[idx];
            }

            if (decode_barcode(false, text, sizeof(text))
                || decode_barcode(true, text, sizeof(text)))
            {
                b_ok   = true;
                p_used = b_distance ? "distance" : "time";
            }
        }
    }

    if (b_ok)
    {
        strncpy(g_last_barcode, text, sizeof(g_last_barcode) - 1U);
        g_last_barcode[sizeof(g_last_barcode) - 1U] = '\0';
        g_b_barcode_ready = true;     // collected by ir_sensor_get_barcode()
#if BARCODE_DEBUG
        printf("decoded using %s\n", p_used);
#endif
    }
    else if (g_element_count >= CODE39_CHAR_ELEMENTS)
    {
        printf("Barcode read failed (%" PRIu32 " elements)\n",
               g_element_count);
    }
    else
    {
        // Fewer than 9 elements = just a stray black mark, ignore quietly.
    }

    // Cast: p_used is only printed when BARCODE_DEBUG is on.
    (void)p_used;
}

/*!
 * @brief Clears the barcode being read.
 *
 * Timer interrupt only.
 *
 * @param[in] b_scanning  true if a new barcode starts at this edge.
 */
static void
reset_barcode_scan (bool b_scanning)
{
    g_b_scanning    = b_scanning;
    g_element_count = 0U;
    g_widest_us     = 0U;
    g_weakest_bar   = ADC_MAX_READING;
    g_weakest_space = 0U;
}

/*!
 * @brief Records the bar or space that has just ended.
 *
 * Timer interrupt only.
 *
 * @param[in] b_now_black  The colour that is starting.
 */
static void
record_barcode_element (bool b_now_black)
{
    // The odometer never goes backwards, so this can't underflow.
    uint32_t width_pulses = g_pending_pos - g_last_edge_pos;

    // Wrap-safe. A width over 2^32 us (71 minutes) is far past
    // BARCODE_MAX_ELEMENT_US, and is thrown away either way.
    uint32_t width_us = g_pending_us - g_last_edge_us;

    if (width_us > BARCODE_MAX_ELEMENT_US)
    {
        // Far too long to be a bar or space -> throw this scan away.
        // If we've just reached black, that's the start of a new barcode.
        reset_barcode_scan(b_now_black);
    }
    else
    {
        if (g_element_count < MAX_ELEMENTS)
        {
            g_widths_pulses[g_element_count] = width_pulses;
            g_widths_us[g_element_count]     = width_us;
            g_element_count++;
        }

        if (width_us > g_widest_us)
        {
            g_widest_us = width_us;
        }

        // g_b_bar_black is still the colour of the element that just ended.
        if (g_b_bar_black && (g_element_max < g_weakest_bar))
        {
            g_weakest_bar = g_element_max;
        }

        if (!g_b_bar_black && (g_element_min > g_weakest_space))
        {
            g_weakest_space = g_element_min;
        }
    }
}

/*!
 * @brief Accepts a colour change that has lasted long enough.
 *
 * Timer interrupt only.
 *
 * @param[in] b_now_black  The colour that is starting.
 */
static void
accept_barcode_edge (bool b_now_black)
{
    g_pending_samples = 0U;

    if (g_b_scanning)
    {
        record_barcode_element(b_now_black);
    }
    else if (b_now_black && !g_b_decode_pending)
    {
        // First bar of a new barcode. (While the last one is still being
        // decoded its widths are left alone; that takes a few ms, far less
        // than the white needed to end a barcode.)
        reset_barcode_scan(true);
    }
    else
    {
        // White while no barcode is being read: nothing to record.
    }

    g_element_min   = ADC_MAX_READING;
    g_element_max   = 0U;
    g_last_edge_pos = g_pending_pos;
    g_last_edge_us  = g_pending_us;
    g_b_bar_black   = b_now_black;
}

/*!
 * @brief Hands the barcode to the main loop once there has been enough white
 *        after the last bar.
 *
 * Timer interrupt only. Decoding is left to ir_sensor_update_barcode(),
 * because it prints and printing doesn't belong in an interrupt.
 */
static void
check_barcode_end (uint32_t now_us)
{
    if (g_b_scanning && !g_b_bar_black)
    {
        uint32_t gap_us    = now_us - g_last_edge_us;
        uint32_t needed_us = BARCODE_END_FACTOR * g_widest_us;

        if (needed_us < BARCODE_END_US)
        {
            needed_us = BARCODE_END_US;
        }

        if (needed_us > BARCODE_END_MAX_US)
        {
            needed_us = BARCODE_END_MAX_US;
        }

        bool b_gap_done  = (gap_us > needed_us);
        bool b_timed_out = (gap_us > BARCODE_TIMEOUT_US);

        if (b_gap_done || b_timed_out)
        {
            g_b_scanning = false;

            // Make sure the widths are written before the main loop is told.
            __compiler_memory_barrier();
            g_b_decode_pending = true;
        }
    }
}

/*!
 * @brief Gets how many pulses a wheel's total count has gone up by since the
 *        last sample, and remembers the new count.
 *
 * Timer interrupt only.
 *
 * @param[in]     pulses  The wheel's total pulse count now.
 * @param[in,out] p_last  The wheel's count at the last sample.
 * @return Pulses since the last sample.
 */
static uint32_t
get_pulses_since (uint32_t pulses, uint32_t * p_last)
{
    uint32_t delta = pulses;

    if (pulses >= *p_last)
    {
        delta = pulses - *p_last;
    }
    else
    {
        // The count went down: someone called motor_reset_encoder_counts().
        // Everything counted since the reset is new travel. At most the
        // pulse or so between the last sample and the reset is lost.
    }

    *p_last = pulses;

    return delta;
}

/*!
 * @brief Adds the wheels' travel since the last sample to the odometer.
 *
 * Timer interrupt only. Each wheel is tracked on its own, so a reset that
 * lands between clearing M1 and clearing M2 is still handled correctly.
 */
static void
update_odometer (void)
{
    uint32_t m1 = get_pulses_since(motor_get_m1_total_pulses(g_p_motors),
                                   &g_last_m1_pulses);
    uint32_t m2 = get_pulses_since(motor_get_m2_total_pulses(g_p_motors),
                                   &g_last_m2_pulses);

    g_odometer += m1 + m2;
}

/*!
 * @brief Takes one barcode sensor sample and tracks bars and spaces.
 *
 * Timer interrupt only.
 */
static void
sample_barcode (uint32_t now_us)
{
    adc_select_input(BARCODE_ADC_CH);

    uint32_t reading = adc_read();

    // Cast: the ADC is 12-bit, so the reading fits in 16 bits.
    g_right_raw = (uint16_t)reading;

    if (reading < g_element_min)
    {
        g_element_min = reading;
    }

    if (reading > g_element_max)
    {
        g_element_max = reading;
    }

    // How far the reading has moved back from this colour's peak (on black)
    // or dip (on white), towards the other colour.
    uint32_t moved = 0U;

    if (g_b_bar_black ? (reading > g_extreme) : (reading < g_extreme))
    {
        // Still getting darker on a bar / lighter on a space.
        g_extreme = reading;
    }
    else
    {
        moved = g_b_bar_black ? (g_extreme - reading) : (reading - g_extreme);
    }

    if (moved < BARCODE_EDGE_HALF)
    {
        // Back near the peak or dip: any half-made edge was noise.
        g_b_pending_crossed = false;
    }
    else if (!g_b_pending_crossed)
    {
        // Halfway to an edge: time the edge from here if it completes.
        g_b_pending_crossed = true;
        g_pending_pos       = g_odometer;
        g_pending_us        = now_us;
    }
    else
    {
        // Already past halfway; keep the first crossing.
    }

    if (moved >= BARCODE_EDGE_DELTA)
    {
        // Possible edge: accept it once it has lasted.
        g_pending_samples++;

        if (g_pending_samples >= BARCODE_DEBOUNCE_SAMPLES)
        {
            accept_barcode_edge(!g_b_bar_black);

            // The new colour's peak or dip is measured from here.
            g_extreme           = reading;
            g_b_pending_crossed = false;
        }
    }
    else
    {
        // The move didn't last -> it was a speck, ignore it.
        g_pending_samples = 0U;
        check_barcode_end(now_us);
    }
}

/*!
 * @brief Barcode timer interrupt: runs every BARCODE_SAMPLE_US.
 *
 * Kept short: no printing, no floating point, no blocking.
 *
 * @param[in] p_timer  The timer (required by the callback signature).
 * @return true to keep the timer repeating.
 */
static bool
barcode_timer_isr (repeating_timer_t * p_timer)
{
    // Cast: p_timer is required by the callback signature but not needed.
    (void)p_timer;

    update_odometer();
    sample_barcode(time_us_32());

    return true;
}

/*** end of file ***/
