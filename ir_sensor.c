// Line following (left + centre IR sensors) + Code 39 barcode scanning (right IR sensor).
//
// Sensor wiring (confirmed with ir_sensor_test.c):
//   Left   sensor DO -> GP3            (digital, 1 = black)
//   Centre sensor AO -> GP27 (ADC1)    (analog, ~200 white / ~2500 black)
//   Right  sensor AO -> GP28 (ADC2)    (analog, barcode scanner)
//
// Line following: EDGE FOLLOWING with a PD controller.
//   The centre sensor's analog reading is turned into a position 0..1
//   (0 = white, 1 = black). The robot aims for 0.5, i.e. the centre sensor
//   half over the line's edge, and steers in proportion to how far off it is.
//   Small drift = small correction, so it no longer snaps left/right.
//   The side sensor (GP3) is a backstop: if it sees the line, steer hard back.
//
// Junctions: when the side sensor AND the right (barcode) sensor both see
// black, a line crosses the track. The robot drives straight across it
// instead of treating it as drift, and prints "JUNCTION n".
//
// Line lost: if all sensors see white briefly, it brakes and prints "LINE LOST",
// then sweeps left and right on the spot to look for the line.
//   Found  -> "LINE FOUND", carries on (it had just drifted off).
//   Not found -> it's a real dead end: stops and prints "DEAD END".
// If the "junction" black lasts much longer than a strip of tape (e.g. the
// robot has reached the edge of the mat and sees the dark floor/bed), it stops
// and prints "OFF TRACK". Once stopped, put it back on the line to carry on.
//
// Decoded barcodes are printed on the Serial Monitor.
// Motors are driven through the MotorController passed to ir_update()
// (M1 = left wheel, M2 = right wheel).

#include "ir_sensor.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"

// ---------------- Sensor pins ----------------
#define LEFT_DO_PIN      3
#define CENTRE_AO_PIN    27
#define CENTRE_ADC_CH    1      // GP27 = ADC1
#define BARCODE_AO_PIN   28
#define BARCODE_ADC_CH   2      // GP28 = ADC2

// ---------------- Calibration ----------------
// Centre sensor raw readings on white and on black, from your test program.
#define CENTRE_WHITE        200
#define CENTRE_BLACK       2500

// Barcode sensor black/white thresholds with hysteresis (ADC 0..4095).
// Measure the RIGHT sensor's own white/black values and set these between them.
#define BARCODE_BLACK_ON   1500   // becomes black above this
#define BARCODE_BLACK_OFF   900   // becomes white below this

// Which side of the line the GP3 sensor sits on: 1 = right, -1 = left.
// Test: hold the robot so ONLY GP3 is over the line. The wheels must turn the
// robot TOWARDS GP3. If they turn it away, flip this sign.
#define SIDE_SENSOR_SIDE   1

// ---------------- Line following tuning ----------------
// Motor speeds in percent (0..100). Tune slowly first, then speed up.
#define BASE_SPEED         35

// Steering = KP * error + KD * rate_of_change_of_error.
// error runs from -0.5 (centre sensor on white) to +0.5 (fully on black).
//   KP too low  -> lazy, drifts off on curves
//   KP too high -> wobbles side to side
//   KD damps the wobble. Tune KP first with KD = 0, then add KD.
#define KP                 25.0f
#define KD                 0.5f

// Errors smaller than this are ignored, so tiny sensor noise around the edge
// doesn't cause constant micro-corrections. Raise to 0.08 if it still wiggles.
#define ERR_DEADBAND       0.07f

#define SLOW_IN_TURNS      0.6f   // forward speed drops by this x |steering|
#define SIDE_STEER         30     // hard correction when GP3 sees the line

// GP3 must stay black this long before the hard correction kicks in. This
// gives the right sensor time to also see black if it's actually a junction.
#define SIDE_CONFIRM_US    30000

#define JUNCTION_SPEED     30
#define JUNCTION_CLEAR_US  40000   // both outer sensors white this long = junction crossed
// A crossing strip of tape is crossed within this distance (measured with the
// wheel encoders, so it works at any speed). Black for longer = not tape
// (mat edge, dark floor) -> stop. JUNCTION_MAX_US is a backup in case the
// encoders aren't counting.
#define JUNCTION_MAX_MM    60.0f
#define JUNCTION_MAX_US    1500000

#define LINE_LOST_US       120000  // centre + side on white this long = line lost -> brake

// Line search after the line is lost: pivot on the spot one way for
// SWEEP_US, then the other way for 2 x SWEEP_US, then back to the middle.
// Raise SWEEP_US if the sweep is too narrow to find a line it drifted off.
#define SEARCH_SPEED       30
#define SWEEP_US           400000

// Forward speed eases towards its target by this fraction each update.
// (Steering is NOT smoothed: smoothing adds lag, and lag causes wobble.)
#define SMOOTHING          0.15f

#define LINE_PERIOD_US     5000   // line-following update every 5 ms
#define BARCODE_SAMPLE_US  200    // barcode sampled every 0.2 ms

// A barcode is considered finished after this long on white.
// Increase it if the robot moves slowly and barcodes get cut off.
#define BARCODE_END_US     300000
#define MAX_ELEMENTS       200    // bars + spaces, enough for ~20 characters

// Right sensor state, updated by barcode_sample() and also used for junctions.
static bool bar_black = false;

// ============================================================
// Line following
// ============================================================

typedef enum { LINE_FOLLOW, LINE_JUNCTION, LINE_SEARCH, LINE_STOPPED } LineState;

static LineState line_state = LINE_FOLLOW;
static float forward = 0.0f;              // current forward speed in %
static float prev_err = 0.0f;
static float d_err = 0.0f;                // filtered rate of change of error (1/s)
static uint64_t side_black_since = 0;     // 0 = side sensor on white
static uint64_t all_white_since = 0;      // 0 = line visible
static uint64_t junction_clear_since = 0;
static uint64_t junction_start_us = 0;
static float junction_start_mm = 0.0f;
static uint64_t search_start_us = 0;

// Average distance both wheels have travelled, from Buddy 2's encoders.
static float travelled_mm(MotorController *mc) {
    return (fabsf(mc_get_motor1_distance_mm(mc)) + fabsf(mc_get_motor2_distance_mm(mc))) / 2.0f;
}

// True when the centre sensor is back on the line (and it's not the dark floor).
static bool line_found(float pos, bool side_black, bool right_black) {
    return (pos > 0.5f || side_black) && !(side_black && right_black);
}
static int junction_count = 0;

// Averages a few ADC samples to take the noise out of the centre reading.
static uint16_t read_centre(void) {
    adc_select_input(CENTRE_ADC_CH);
    uint32_t sum = 0;
    for (int i = 0; i < 4; i++) sum += adc_read();
    return (uint16_t)(sum / 4);
}

// Centre reading as a position: 0 = white, 1 = black.
static float centre_position(void) {
    float p = ((float)read_centre() - CENTRE_WHITE) / (float)(CENTRE_BLACK - CENTRE_WHITE);
    if (p < 0.0f) return 0.0f;
    if (p > 1.0f) return 1.0f;
    return p;
}

static int16_t clamp_speed(float speed) {
    if (speed < -100.0f) return -100;
    if (speed > 100.0f) return 100;
    return (int16_t)speed;
}

static void line_follow_step(MotorController *mc, uint64_t now) {
    bool side_black = gpio_get(LEFT_DO_PIN);
    bool right_black = bar_black;
    float pos = centre_position();
    float err = pos - 0.5f;

    // Rate of change of error, lightly filtered because differentiating is noisy.
    float d = (err - prev_err) / (LINE_PERIOD_US / 1e6f);
    d_err += (d - d_err) * 0.3f;
    prev_err = err;

    // Both outer sensors on black at once = a line crossing the track.
    if (line_state == LINE_FOLLOW && side_black && right_black) {
        line_state = LINE_JUNCTION;
        junction_clear_since = 0;
        junction_start_us = now;
        junction_start_mm = travelled_mm(mc);
        junction_count++;
        printf("JUNCTION %d\n", junction_count);
    }

    float target_forward = BASE_SPEED;
    float steer = 0.0f;   // positive = turn right

    switch (line_state) {
    case LINE_JUNCTION:
        // Drive straight across until both outer sensors are back on white.
        target_forward = JUNCTION_SPEED;
        if (!side_black && !right_black) {
            if (junction_clear_since == 0) junction_clear_since = now;
            if (now - junction_clear_since > JUNCTION_CLEAR_US) {
                line_state = LINE_FOLLOW;
                side_black_since = 0;
                all_white_since = 0;
                d_err = 0.0f;
            }
        } else {
            junction_clear_since = 0;
            if (travelled_mm(mc) - junction_start_mm > JUNCTION_MAX_MM ||
                now - junction_start_us > JUNCTION_MAX_US) {
                // Far too long to be a strip of tape -> off the track.
                line_state = LINE_STOPPED;
                target_forward = 0.0f;
                forward = 0.0f;
                printf("OFF TRACK\n");
            }
        }
        break;

    case LINE_SEARCH: {
        // Sweep: away from the side sensor first (the likely way it drifted),
        // then back past the middle to the other side, then return to middle.
        target_forward = 0.0f;
        forward = 0.0f;
        uint64_t t = now - search_start_us;
        if (line_found(pos, side_black, right_black)) {
            line_state = LINE_FOLLOW;
            all_white_since = 0;
            side_black_since = 0;
            d_err = 0.0f;
            printf("LINE FOUND\n");
        } else if (t < SWEEP_US) {
            steer = -SIDE_SENSOR_SIDE * SEARCH_SPEED;
        } else if (t < 3 * SWEEP_US) {
            steer = SIDE_SENSOR_SIDE * SEARCH_SPEED;
        } else if (t < 4 * SWEEP_US) {
            steer = -SIDE_SENSOR_SIDE * SEARCH_SPEED;
        } else {
            line_state = LINE_STOPPED;
            printf("DEAD END\n");
        }
        break;
    }

    case LINE_STOPPED:
        target_forward = 0.0f;
        forward = 0.0f;
        // Put back on the line -> carry on.
        if (pos > 0.5f && !(side_black && right_black)) {
            line_state = LINE_FOLLOW;
            all_white_since = 0;
            side_black_since = 0;
            d_err = 0.0f;
            printf("LINE FOUND\n");
        }
        break;

    case LINE_FOLLOW:
    default:
        // Proportional + derivative steering. More black under the centre
        // sensor = robot has drifted away from the side sensor, so steer
        // towards it; more white = steer away from it.
        {
            float e = err;
            if (e > ERR_DEADBAND) e -= ERR_DEADBAND;
            else if (e < -ERR_DEADBAND) e += ERR_DEADBAND;
            else e = 0.0f;
            steer = SIDE_SENSOR_SIDE * (KP * e + KD * d_err);
        }

        // Backstop: side sensor on the line for a while -> steer hard back.
        if (side_black) {
            if (side_black_since == 0) side_black_since = now;
            if (now - side_black_since > SIDE_CONFIRM_US) {
                steer = SIDE_SENSOR_SIDE * SIDE_STEER;
            }
        } else {
            side_black_since = 0;
        }

        // Nothing black anywhere for too long -> end of line or ran off it.
        if (!side_black && pos < 0.1f) {
            if (all_white_since == 0) all_white_since = now;
            if (now - all_white_since > LINE_LOST_US) {
                line_state = LINE_SEARCH;
                search_start_us = now;
                steer = 0.0f;
                target_forward = 0.0f;
                forward = 0.0f;              // brake now, don't ease off
                printf("LINE LOST\n");
                break;
            }
        } else {
            all_white_since = 0;
        }

        // Slow down in proportion to how hard it's turning.
        target_forward = BASE_SPEED - SLOW_IN_TURNS * fabsf(steer);
        break;
    }

    forward += (target_forward - forward) * SMOOTHING;
    mc_set_wheel_speeds(mc, clamp_speed(forward + steer), clamp_speed(forward - steer));
}


// ============================================================
// Barcode (Code 39)
// ============================================================

// Each character is 9 elements (bar, space, bar, ... bar), 1 = wide, 0 = narrow.
static const struct { char c; uint16_t pattern; } CODE39[] = {
    {'0', 0b000110100}, {'1', 0b100100001}, {'2', 0b001100001}, {'3', 0b101100000},
    {'4', 0b000110001}, {'5', 0b100110000}, {'6', 0b001110000}, {'7', 0b000100101},
    {'8', 0b100100100}, {'9', 0b001100100}, {'A', 0b100001001}, {'B', 0b001001001},
    {'C', 0b101001000}, {'D', 0b000011001}, {'E', 0b100011000}, {'F', 0b001011000},
    {'G', 0b000001101}, {'H', 0b100001100}, {'I', 0b001001100}, {'J', 0b000011100},
    {'K', 0b100000011}, {'L', 0b001000011}, {'M', 0b101000010}, {'N', 0b000010011},
    {'O', 0b100010010}, {'P', 0b001010010}, {'Q', 0b000000111}, {'R', 0b100000110},
    {'S', 0b001000110}, {'T', 0b000010110}, {'U', 0b110000001}, {'V', 0b011000001},
    {'W', 0b111000000}, {'X', 0b010010001}, {'Y', 0b110010000}, {'Z', 0b011010000},
    {'-', 0b010000101}, {'.', 0b110000100}, {' ', 0b011000100}, {'*', 0b010010100},
    {'$', 0b010101000}, {'/', 0b010100010}, {'+', 0b010001010}, {'%', 0b000101010},
};

static uint32_t widths[MAX_ELEMENTS];   // duration of each bar/space in microseconds
static int n_elements = 0;
static bool scanning = false;
static uint64_t last_edge_us = 0;

// Decodes 9 element widths into a character, or returns 0 if invalid.
static char decode_char(const uint32_t e[9]) {
    // Sort a copy: the 3 widest elements are wide, the other 6 narrow.
    uint32_t s[9];
    memcpy(s, e, sizeof(s));
    for (int i = 1; i < 9; i++) {
        uint32_t v = s[i];
        int j = i - 1;
        while (j >= 0 && s[j] > v) { s[j + 1] = s[j]; j--; }
        s[j + 1] = v;
    }
    uint32_t threshold = (s[5] + s[6]) / 2;

    uint16_t pattern = 0;
    for (int i = 0; i < 9; i++) {
        pattern <<= 1;
        if (e[i] > threshold) pattern |= 1;
    }
    for (size_t k = 0; k < sizeof(CODE39) / sizeof(CODE39[0]); k++) {
        if (CODE39[k].pattern == pattern) return CODE39[k].c;
    }
    return 0;
}

// Decodes the recorded elements in one direction. Returns true on success.
static bool decode_barcode(bool reverse, char *out, size_t out_size) {
    // n characters = 9n elements + (n - 1) gaps between characters
    if ((n_elements + 1) % 10 != 0) return false;
    int n_chars = (n_elements + 1) / 10;
    if (n_chars < 3) return false;              // start '*' + data + stop '*'
    if ((size_t)(n_chars - 2) >= out_size) return false;

    char chars[MAX_ELEMENTS / 10 + 1];
    for (int k = 0; k < n_chars; k++) {
        uint32_t e[9];
        for (int i = 0; i < 9; i++) {
            int idx = k * 10 + i;
            e[i] = reverse ? widths[n_elements - 1 - idx] : widths[idx];
        }
        chars[k] = decode_char(e);
        if (chars[k] == 0) return false;
    }
    if (chars[0] != '*' || chars[n_chars - 1] != '*') return false;

    memcpy(out, &chars[1], n_chars - 2);
    out[n_chars - 2] = '\0';
    return true;
}

static void barcode_finished(void) {
    char text[32];
    // Try both directions so barcodes can be read either way round.
    if (decode_barcode(false, text, sizeof(text)) ||
        decode_barcode(true, text, sizeof(text))) {
        printf("BARCODE: %s\n", text);
    } else if (n_elements >= 9) {
        printf("Barcode read failed (%d elements)\n", n_elements);
    }
    // Fewer than 9 elements = just a stray black mark, ignore quietly.
}

static void barcode_sample(uint64_t now) {
    adc_select_input(BARCODE_ADC_CH);
    uint16_t v = adc_read();

    bool black = bar_black;
    if (!bar_black && v > BARCODE_BLACK_ON) black = true;
    else if (bar_black && v < BARCODE_BLACK_OFF) black = false;

    if (black != bar_black) {
        if (scanning) {
            if (n_elements < MAX_ELEMENTS) {
                widths[n_elements++] = (uint32_t)(now - last_edge_us);
            }
        } else if (black) {
            // First bar of a new barcode
            scanning = true;
            n_elements = 0;
        }
        last_edge_us = now;
        bar_black = black;
    } else if (scanning && !black && now - last_edge_us > BARCODE_END_US) {
        // Long white after the last bar -> barcode finished
        scanning = false;
        barcode_finished();
    }
}

// ============================================================
// Public API
// ============================================================

static uint64_t next_barcode_us = 0;
static uint64_t next_line_us = 0;

void ir_init(void) {
    gpio_init(LEFT_DO_PIN);
    gpio_set_dir(LEFT_DO_PIN, GPIO_IN);

    adc_init();
    adc_gpio_init(CENTRE_AO_PIN);
    adc_gpio_init(BARCODE_AO_PIN);
}

void ir_update(MotorController *mc) {
    uint64_t now = time_us_64();
    if (now >= next_barcode_us) {
        barcode_sample(now);
        next_barcode_us = now + BARCODE_SAMPLE_US;
    }
    if (now >= next_line_us) {
        line_follow_step(mc, now);
        next_line_us = now + LINE_PERIOD_US;
    }
}