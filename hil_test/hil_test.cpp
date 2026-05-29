// hil_test.cpp — Hardware-in-the-Loop test firmware for a second Pico
//
// Wiring (HIL Pico  ←→  DUT Pico):
//   HIL GPIO 0  ──────────────────►  DUT GPIO 10  (signal generator → DUT input)
//   HIL GPIO 1  ◄──────────────────  DUT GPIO 14  (DUT output → checker input)
//   HIL GND     ──────────────────   DUT GND
//
// The HIL Pico generates a known pulse train, waits for the DUT to settle,
// then measures the DUT output frequency and asserts the expected 4× ratio.
// Results are printed over USB serial as a simple PASS / FAIL log.
//
// Calibration constant (must match speedo_logic.h):
//   Input freq at 1 mph  = 8.89 / 4 = 2.2225 Hz  → period = 450 113 µs
//   Output freq at 1 mph = 8.89 Hz                → period = 112 528 µs
//   Generator half-period at N mph = 450113 / N / 2  µs
//   Expected output period at N mph = 112528 / N     µs

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"

// ---------------------------------------------------------------------------
// Pin assignments
// ---------------------------------------------------------------------------
#define GEN_PIN     0   // Square-wave output to DUT GPIO 10
#define CHECK_PIN   1   // Rising-edge input from DUT GPIO 14

// ---------------------------------------------------------------------------
// DUT timing constants (kept in sync with speedo_logic.h)
// ---------------------------------------------------------------------------
#define DUT_MIN_PULSE_US     3200U
#define DUT_MAX_PULSE_US     750000U
#define DUT_TIMEOUT_US       1000000U
#define DUT_AVG_INTERVAL_MS  300U
#define DUT_WINDOW_SIZE      3U

// Calibration: input period (µs) at 1 mph
static const double INPUT_PERIOD_1MPH_US = 450113.0;
// Calibration: output period (µs) at 1 mph
static const double OUTPUT_PERIOD_1MPH_US = 112528.0;

// ---------------------------------------------------------------------------
// Test tolerances
// ---------------------------------------------------------------------------
#define FREQ_TOLERANCE_PCT   5.0    // ±5 % on output frequency
#define SETTLE_EXTRA_MS      700U   // extra margin on top of 3 DUT avg cycles
#define MEASURE_SAMPLES      10U    // rising edges to average for frequency check
#define MEASURE_TIMEOUT_MS   5000U  // give up waiting for samples after this long

// ---------------------------------------------------------------------------
// Generator state
// ---------------------------------------------------------------------------
static struct repeating_timer  g_gen_timer;
static volatile bool           g_gen_pin_state = false;
static volatile bool           g_gen_running   = false;

static bool gen_timer_callback(struct repeating_timer *t) {
    (void)t;
    g_gen_pin_state = !g_gen_pin_state;
    gpio_put(GEN_PIN, g_gen_pin_state);
    return true;
}

static void gen_start(uint64_t half_period_us) {
    if (g_gen_running) {
        cancel_repeating_timer(&g_gen_timer);
    }
    g_gen_pin_state = false;
    gpio_put(GEN_PIN, false);
    add_repeating_timer_us(-(int64_t)half_period_us, gen_timer_callback,
                           NULL, &g_gen_timer);
    g_gen_running = true;
}

static void gen_stop(void) {
    if (g_gen_running) {
        cancel_repeating_timer(&g_gen_timer);
        g_gen_running = false;
    }
    gpio_put(GEN_PIN, false);
}

// ---------------------------------------------------------------------------
// Checker — measures rising-edge periods on CHECK_PIN
// ---------------------------------------------------------------------------
#define CHECKER_BUF_SIZE 32U

static volatile uint64_t  g_check_times[CHECKER_BUF_SIZE];
static volatile uint32_t  g_check_write = 0;   // next slot to write
static volatile uint32_t  g_check_read  = 0;   // next slot to read

static void check_gpio_callback(uint gpio, uint32_t events) {
    (void)gpio; (void)events;
    uint64_t now = to_us_since_boot(get_absolute_time());
    uint32_t next = (g_check_write + 1) % CHECKER_BUF_SIZE;
    if (next != g_check_read) {           // drop if full
        g_check_times[g_check_write] = now;
        g_check_write = next;
    }
}

// Discard buffered samples (call before starting a new measurement window)
static void checker_flush(void) {
    uint32_t irq = save_and_disable_interrupts();
    g_check_read  = g_check_write;
    restore_interrupts(irq);
}

// Block until `n` rising-edge periods have been measured or timeout_ms elapses.
// Returns the average period in µs, or 0 on timeout.
static uint64_t checker_measure_avg_period(uint32_t n, uint32_t timeout_ms) {
    checker_flush();

    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    uint64_t prev_time = 0;
    bool     have_prev = false;
    uint64_t sum       = 0;
    uint32_t count     = 0;

    while (count < n) {
        if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0) {
            return 0;   // timeout
        }

        uint32_t irq = save_and_disable_interrupts();
        bool have_sample = (g_check_read != g_check_write);
        uint64_t t = have_sample ? g_check_times[g_check_read] : 0;
        if (have_sample) g_check_read = (g_check_read + 1) % CHECKER_BUF_SIZE;
        restore_interrupts(irq);

        if (!have_sample) {
            sleep_us(50);
            continue;
        }

        if (have_prev) {
            uint64_t period = t - prev_time;
            // Sanity-check: ignore wildly out-of-range periods
            if (period > 100 && period < 2000000) {
                sum += period;
                ++count;
            }
        }
        prev_time = t;
        have_prev = true;
    }

    return sum / count;
}

// ---------------------------------------------------------------------------
// Settle time calculation
// DUT needs: 3 full input periods to fill its window + one AVG_INTERVAL
// ---------------------------------------------------------------------------
static uint32_t settle_ms(uint64_t gen_half_period_us) {
    uint64_t input_period_us = gen_half_period_us * 2;
    uint32_t three_cycles_ms = (uint32_t)((input_period_us * DUT_WINDOW_SIZE) / 1000 + 1);
    return three_cycles_ms + DUT_AVG_INTERVAL_MS + SETTLE_EXTRA_MS;
}

// ---------------------------------------------------------------------------
// Helper: mph → generator half-period (µs)
// ---------------------------------------------------------------------------
static uint64_t mph_to_gen_half_period(double mph) {
    return (uint64_t)(INPUT_PERIOD_1MPH_US / mph / 2.0 + 0.5);
}

// ---------------------------------------------------------------------------
// Helper: mph → expected DUT output period (µs)
// ---------------------------------------------------------------------------
static uint64_t mph_to_expected_output_period(double mph) {
    return (uint64_t)(OUTPUT_PERIOD_1MPH_US / mph + 0.5);
}

// ---------------------------------------------------------------------------
// Test result bookkeeping
// ---------------------------------------------------------------------------
static int g_tests_run    = 0;
static int g_tests_passed = 0;

static void report(const char *name, bool passed, const char *detail) {
    ++g_tests_run;
    if (passed) ++g_tests_passed;
    printf("[%s] %s  %s\n", passed ? "PASS" : "FAIL", name, detail);
}

// ---------------------------------------------------------------------------
// Individual test cases
// ---------------------------------------------------------------------------

// TC-1  Steady speed — verify output freq is 4× input freq within tolerance
static void tc_steady_speed(const char *label, double mph) {
    char detail[128];

    uint64_t gen_half_us     = mph_to_gen_half_period(mph);
    uint64_t expected_out_us = mph_to_expected_output_period(mph);
    double   expected_out_hz = 1e6 / expected_out_us;

    gen_start(gen_half_us);
    sleep_ms(settle_ms(gen_half_us));

    uint64_t measured_out_us = checker_measure_avg_period(MEASURE_SAMPLES,
                                                          MEASURE_TIMEOUT_MS);
    gen_stop();

    if (measured_out_us == 0) {
        snprintf(detail, sizeof(detail),
                 "%.1f mph — TIMEOUT waiting for %u samples", mph, MEASURE_SAMPLES);
        report(label, false, detail);
        return;
    }

    double measured_out_hz = 1e6 / measured_out_us;
    double error_pct = fabs(measured_out_hz - expected_out_hz) / expected_out_hz * 100.0;
    bool passed = (error_pct <= FREQ_TOLERANCE_PCT);

    snprintf(detail, sizeof(detail),
             "%.1f mph | expected %.2f Hz | measured %.2f Hz | error %.1f%%",
             mph, expected_out_hz, measured_out_hz, error_pct);
    report(label, passed, detail);
}

// TC-2  Signal dropout — output must stop within DUT_TIMEOUT_US + margin
static void tc_dropout(void) {
    const double  mph          = 55.0;
    uint64_t      gen_half_us  = mph_to_gen_half_period(mph);
    const uint32_t stop_margin_ms = 1500;   // 1 s timeout + 500 ms margin

    // Start signal and let DUT lock on
    gen_start(gen_half_us);
    sleep_ms(settle_ms(gen_half_us));

    // Verify signal is running before we drop it
    uint64_t pre = checker_measure_avg_period(4, MEASURE_TIMEOUT_MS);
    if (pre == 0) {
        report("TC-2 Dropout", false, "DUT not outputting before dropout");
        gen_stop();
        return;
    }

    // Drop the input signal
    gen_stop();
    checker_flush();

    // Poll: check whether DUT output has gone quiet
    absolute_time_t deadline = make_timeout_time_ms(stop_margin_ms);
    bool output_stopped = false;
    while (absolute_time_diff_us(get_absolute_time(), deadline) > 0) {
        // Try to collect a sample; if none arrive for 1.1× the expected period
        // the output has stopped
        uint64_t period = checker_measure_avg_period(1, 1100);
        if (period == 0) {
            output_stopped = true;
            break;
        }
    }

    report("TC-2 Dropout",
           output_stopped,
           output_stopped
               ? "DUT output stopped within 1.5 s of signal loss"
               : "DUT output still running >1.5 s after signal loss");
}

// TC-3  Resume after dropout — output must restart within a reasonable window
static void tc_resume(void) {
    const double  mph         = 55.0;
    uint64_t      gen_half_us = mph_to_gen_half_period(mph);

    // Ensure DUT is in dropout state first
    gen_stop();
    sleep_ms(1500);
    checker_flush();

    // Restart the signal
    gen_start(gen_half_us);

    // DUT needs to see WINDOW_SIZE pulses before it can drive output again
    uint32_t resume_window_ms = settle_ms(gen_half_us);
    uint64_t measured = checker_measure_avg_period(MEASURE_SAMPLES,
                                                   resume_window_ms + MEASURE_TIMEOUT_MS);
    gen_stop();

    uint64_t expected_out_us = mph_to_expected_output_period(mph);
    double   expected_hz     = 1e6 / expected_out_us;
    bool     got_signal      = (measured != 0);
    double   error_pct       = 0.0;
    if (got_signal) {
        error_pct = fabs((1e6 / measured) - expected_hz) / expected_hz * 100.0;
    }
    bool passed = got_signal && (error_pct <= FREQ_TOLERANCE_PCT);

    char detail[128];
    if (!got_signal) {
        snprintf(detail, sizeof(detail), "DUT output did not resume within %u ms",
                 resume_window_ms + MEASURE_TIMEOUT_MS);
    } else {
        snprintf(detail, sizeof(detail),
                 "output resumed at %.2f Hz (expected %.2f Hz, error %.1f%%)",
                 1e6 / measured, expected_hz, error_pct);
    }
    report("TC-3 Resume", passed, detail);
}

// TC-4  Glitch rejection — inject pulses below MIN_PULSE_US; DUT output
//        should be unaffected (continue at the pre-glitch frequency)
static void tc_glitch_rejection(void) {
    const double  mph         = 55.0;
    uint64_t      gen_half_us = mph_to_gen_half_period(mph);

    // Establish a clean baseline
    gen_start(gen_half_us);
    sleep_ms(settle_ms(gen_half_us));

    uint64_t baseline_us = checker_measure_avg_period(MEASURE_SAMPLES, MEASURE_TIMEOUT_MS);
    if (baseline_us == 0) {
        report("TC-4 Glitch", false, "could not establish baseline");
        gen_stop();
        return;
    }

    // Stop the real generator, then fire 8 short glitch pulses manually
    // (each much shorter than DUT_MIN_PULSE_US = 3200 µs)
    cancel_repeating_timer(&g_gen_timer);
    g_gen_running = false;
    sleep_us(500);

    const uint32_t GLITCH_HALF_US = 500;   // 1 ms full period — well below 3.2 ms min
    for (int i = 0; i < 16; ++i) {         // 8 full glitch cycles
        gpio_put(GEN_PIN, (i & 1) ? true : false);
        sleep_us(GLITCH_HALF_US);
    }
    gpio_put(GEN_PIN, false);

    // Restart real signal and let DUT re-stabilise
    gen_start(gen_half_us);
    sleep_ms(settle_ms(gen_half_us));

    uint64_t post_us = checker_measure_avg_period(MEASURE_SAMPLES, MEASURE_TIMEOUT_MS);
    gen_stop();

    if (post_us == 0) {
        report("TC-4 Glitch", false, "DUT output lost after glitch injection");
        return;
    }

    double pre_hz   = 1e6 / baseline_us;
    double post_hz  = 1e6 / post_us;
    double drift_pct = fabs(post_hz - pre_hz) / pre_hz * 100.0;
    bool   passed   = (drift_pct <= FREQ_TOLERANCE_PCT);

    char detail[128];
    snprintf(detail, sizeof(detail),
             "pre %.2f Hz → post %.2f Hz, drift %.1f%%",
             pre_hz, post_hz, drift_pct);
    report("TC-4 Glitch", passed, detail);
}

// TC-5  Speed ramp — step through several speeds and verify output tracks
static void tc_speed_ramp(void) {
    static const double speeds_mph[] = { 5.0, 20.0, 55.0, 88.0, 55.0, 20.0 };
    static const int    N = (int)(sizeof(speeds_mph) / sizeof(speeds_mph[0]));

    bool all_passed = true;
    char summary[256] = "";

    for (int i = 0; i < N; ++i) {
        double    mph            = speeds_mph[i];
        uint64_t  gen_half_us   = mph_to_gen_half_period(mph);
        uint64_t  exp_out_us    = mph_to_expected_output_period(mph);
        double    exp_hz        = 1e6 / exp_out_us;

        gen_start(gen_half_us);
        sleep_ms(settle_ms(gen_half_us));

        uint64_t measured_us = checker_measure_avg_period(MEASURE_SAMPLES,
                                                          MEASURE_TIMEOUT_MS);
        if (measured_us == 0) {
            all_passed = false;
            char seg[48];
            snprintf(seg, sizeof(seg), "%.0fmph:TIMEOUT ", mph);
            strncat(summary, seg, sizeof(summary) - strlen(summary) - 1);
            continue;
        }

        double meas_hz   = 1e6 / measured_us;
        double error_pct = fabs(meas_hz - exp_hz) / exp_hz * 100.0;
        bool   ok        = (error_pct <= FREQ_TOLERANCE_PCT);
        if (!ok) all_passed = false;

        char seg[48];
        snprintf(seg, sizeof(seg), "%.0fmph:%.1f%%(%s) ",
                 mph, error_pct, ok ? "ok" : "FAIL");
        strncat(summary, seg, sizeof(summary) - strlen(summary) - 1);
    }

    gen_stop();
    report("TC-5 Ramp", all_passed, summary);
}

// TC-6  Boundary: input just at MIN_PULSE_US — should be accepted
static void tc_boundary_min(void) {
    // gen half period = MIN_PULSE_US / 2
    uint64_t gen_half_us = DUT_MIN_PULSE_US / 2;

    gen_start(gen_half_us);
    sleep_ms(settle_ms(gen_half_us));

    uint64_t measured_us = checker_measure_avg_period(MEASURE_SAMPLES, MEASURE_TIMEOUT_MS);
    gen_stop();

    bool got_output = (measured_us != 0);
    char detail[64];
    if (got_output) {
        snprintf(detail, sizeof(detail),
                 "output period %llu us at MIN_PULSE input", (unsigned long long)measured_us);
    } else {
        snprintf(detail, sizeof(detail), "no output at MIN_PULSE_US input");
    }
    report("TC-6 Boundary MIN", got_output, detail);
}

// TC-7  Boundary: input just above MAX_PULSE_US — DUT should stop output
static void tc_boundary_max(void) {
    // First establish a running state at a moderate speed
    const double mph = 30.0;
    uint64_t gen_half_us = mph_to_gen_half_period(mph);
    gen_start(gen_half_us);
    sleep_ms(settle_ms(gen_half_us));

    uint64_t pre = checker_measure_avg_period(4, MEASURE_TIMEOUT_MS);
    if (pre == 0) {
        report("TC-7 Boundary MAX", false, "could not establish baseline at 30 mph");
        gen_stop();
        return;
    }

    // Switch to a period just over MAX_PULSE_US — DUT should treat this as
    // a signal-loss / invalid pulse and stop output
    uint64_t over_max_half = (DUT_MAX_PULSE_US + 10000) / 2;
    gen_start(over_max_half);

    // Give the DUT one over-max pulse to trigger reset, plus settle time
    sleep_ms(2 * (over_max_half / 1000) + DUT_AVG_INTERVAL_MS + 500);

    // DUT output should now be silent
    checker_flush();
    uint64_t post = checker_measure_avg_period(1, 1500);
    gen_stop();

    bool output_stopped = (post == 0);
    report("TC-7 Boundary MAX",
           output_stopped,
           output_stopped
               ? "DUT output stopped on over-max pulse"
               : "DUT output still running after over-max pulse");
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main() {
    stdio_init_all();

    // Brief wait for USB serial to enumerate on the host
    sleep_ms(2000);

    printf("\n");
    printf("============================================================\n");
    printf("  Speedo Mustang — Hardware-in-the-Loop Test Suite\n");
    printf("  Generator → DUT GPIO10  |  DUT GPIO14 → Checker\n");
    printf("  Tolerance: ±%.0f %%\n", FREQ_TOLERANCE_PCT);
    printf("============================================================\n\n");

    // Generator output pin
    gpio_init(GEN_PIN);
    gpio_set_dir(GEN_PIN, GPIO_OUT);
    gpio_put(GEN_PIN, false);

    // Checker input pin — rising-edge ISR
    gpio_init(CHECK_PIN);
    gpio_set_dir(CHECK_PIN, GPIO_IN);
    gpio_pull_down(CHECK_PIN);
    gpio_set_irq_enabled_with_callback(CHECK_PIN, GPIO_IRQ_EDGE_RISE,
                                       true, &check_gpio_callback);

    // Ensure DUT starts from a clean idle state
    gen_stop();
    sleep_ms(1500);

    // ------------------------------------------------------------------
    // Run test suite
    // ------------------------------------------------------------------
    tc_steady_speed("TC-1a Steady  5 mph",   5.0);
    tc_steady_speed("TC-1b Steady 30 mph",  30.0);
    tc_steady_speed("TC-1c Steady 55 mph",  55.0);
    tc_steady_speed("TC-1d Steady 88 mph",  88.0);
    tc_dropout();
    tc_resume();
    tc_glitch_rejection();
    tc_speed_ramp();
    tc_boundary_min();
    tc_boundary_max();

    // ------------------------------------------------------------------
    // Summary
    // ------------------------------------------------------------------
    printf("\n============================================================\n");
    printf("  Results: %d / %d passed\n", g_tests_passed, g_tests_run);
    if (g_tests_passed == g_tests_run) {
        printf("  ALL TESTS PASSED\n");
    } else {
        printf("  *** %d TEST(S) FAILED ***\n", g_tests_run - g_tests_passed);
    }
    printf("============================================================\n");

    // Hold LED on/off to signal result without a serial terminal
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    while (true) {
        if (g_tests_passed == g_tests_run) {
            // Slow blink = all pass
            gpio_put(PICO_DEFAULT_LED_PIN, true);
            sleep_ms(900);
            gpio_put(PICO_DEFAULT_LED_PIN, false);
            sleep_ms(900);
        } else {
            // Rapid blink = failure
            gpio_put(PICO_DEFAULT_LED_PIN, true);
            sleep_ms(150);
            gpio_put(PICO_DEFAULT_LED_PIN, false);
            sleep_ms(150);
        }
    }
}
