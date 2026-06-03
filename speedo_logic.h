#pragma once
#include <stdint.h>
#include <stdbool.h>

#define WINDOW_SIZE         40U  // physical buffer capacity (max samples stored)
#define MEASUREMENT_WINDOW_US 300000U

// Effective window size used for averaging will vary with speed to improve
// noise rejection at higher speeds. The runtime chooses a value between
// `MIN_EFFECTIVE_WINDOW` and `WINDOW_SIZE` based on recent pulse timing.
#define MIN_EFFECTIVE_WINDOW 3U
#define MIN_PULSE_US        3000U
#define MAX_PULSE_US        750000U
#define MAX_OUTPUT_PULSE_US 93750U
#define PULSE_TIMEOUT_US    1000000U

// ---------------------------------------------------------------------------
// SpeedoState — all mutable algorithm state in one place.
//
// In the firmware this struct is declared as a single volatile global and
// accessed only under save_and_disable_interrupts(). In tests it is a plain
// local variable; no volatile, no locks needed.
// ---------------------------------------------------------------------------
struct SpeedoState {
    uint64_t pulse_in_window[WINDOW_SIZE];
    uint64_t pulse_out_window[WINDOW_SIZE];
    uint64_t pulse_pos;
    bool     window_filled;

    uint64_t last_edge_us;       // absolute microseconds of last rising edge
    bool     last_edge_valid;    // false until first edge seen

    uint64_t last_input_interval_us;
    uint64_t bad_pulse_count;

    // Outputs computed by speedo_compute_metrics()
    uint64_t input_interval_us;
    uint64_t output_interval_us;
    uint64_t last_output_interval_us;
};

// ---------------------------------------------------------------------------
// SpeedoHal — hardware actions the logic layer needs to perform.
//
// The firmware fills this with real Pico SDK calls. Tests fill it with
// simple fakes that record what was called.
// ---------------------------------------------------------------------------
struct SpeedoHal {
    // Drive the output GPIO high or low (used when disabling output)
    void (*set_output_pin)(bool state);

    // Stop the repeating output timer
    void (*cancel_timer)(void);

    // Start (or restart) the repeating output timer at the given half-period
    void (*start_timer)(uint64_t interval_us);

    // Request an interval change that takes effect after the current output
    // pulse completes (pin returns high). Safe to call from the main loop
    // while the timer ISR is running — avoids mid-pulse cancellation.
    void (*set_pending_interval)(uint64_t interval_us);

    // Diagnostic logging — may be NULL to suppress all output
    void (*log)(const char *msg);
};

// ---------------------------------------------------------------------------
// Logic API
// ---------------------------------------------------------------------------

// Zero-init all state fields. Call on startup and on signal loss/timeout.
void speedo_reset(SpeedoState &s);

// Feed one measured pulse width into the pipeline (called from GPIO ISR).
// Returns true if the pulse was accepted into the buffer, false if it was
// out of range (caller should cancel the output timer on false + reset).
bool speedo_process_pulse(SpeedoState &s, uint64_t width_us);

// Compute rolling average and jitter from the current window.
// now_us  — current time in microseconds (injected so tests can control time)
// Returns true if metrics were computed, false if timeout/no data.
// On timeout the state is reset internally; caller should cancel timer.
bool speedo_compute_metrics(SpeedoState &s, uint64_t now_us,
                            double &std_dev, uint64_t &count);

// Compare current output_interval_us to last_output_interval_us.
// If the change exceeds 1 %, restart (or stop) the output timer via hal.
// Call after speedo_compute_metrics().
void speedo_update_output(SpeedoState &s, SpeedoHal &hal);
