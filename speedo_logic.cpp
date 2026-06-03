#include "speedo_logic.h"
#include <math.h>
#include <stdio.h>   // snprintf for log helpers

// ---------------------------------------------------------------------------
// speedo_reset
// ---------------------------------------------------------------------------
void speedo_reset(SpeedoState &s) {
    for (uint32_t i = 0; i < WINDOW_SIZE; ++i) {
        s.pulse_in_window[i]  = 0;
        s.pulse_out_window[i] = 0;
    }
    s.pulse_pos              = 0;
    s.window_filled          = false;
    s.last_edge_us           = 0;
    s.last_edge_valid        = false;
    s.last_input_interval_us = 0;
    s.input_interval_us      = 0;
    s.output_interval_us     = 0;
    s.last_output_interval_us = 0;
    // bad_pulse_count is intentionally NOT reset — it's a lifetime counter
}

// ---------------------------------------------------------------------------
// speedo_process_pulse
// Called from the GPIO ISR with the measured width between two rising edges.
// ---------------------------------------------------------------------------
bool speedo_process_pulse(SpeedoState &s, uint64_t width_us) {
    s.last_input_interval_us = width_us;

    if (width_us < MIN_PULSE_US) {
        // Too short — noise/glitch
        ++s.bad_pulse_count;
        return false;
    }

    if (width_us > MAX_PULSE_US) {
        // Too long — signal loss; reset everything
        speedo_reset(s);
        return false;
    }

    // Valid pulse — push into circular buffer
    s.pulse_in_window[s.pulse_pos]  = width_us;
    s.pulse_out_window[s.pulse_pos] = width_us / 8;   // ÷8: 1/4 freq × 2 half-cycles
    ++s.pulse_pos;
    if (s.pulse_pos >= WINDOW_SIZE) {
        s.pulse_pos    = 0;
        s.window_filled = true;
    }
    return true;
}

// ---------------------------------------------------------------------------
// speedo_compute_metrics
// Called from the main loop (not ISR). now_us is injected by the caller so
// tests can control time without mocking a global.
// ---------------------------------------------------------------------------
bool speedo_compute_metrics(SpeedoState &s, uint64_t now_us,
                            double &std_dev, uint64_t &count) {
    // Determine how many samples are actually available in the circular buffer
    uint32_t available = s.window_filled ? WINDOW_SIZE : (uint32_t)s.pulse_pos;

    // Choose an effective window size such that the averaged samples cover
    // roughly `MEASUREMENT_WINDOW_US` of signal time, up to `WINDOW_SIZE`.
    // This gives more noise rejection at higher speed while keeping low-speed
    // responsiveness bounded by `MIN_EFFECTIVE_WINDOW`.
    uint32_t effective;
    uint64_t basis_interval = s.input_interval_us ? s.input_interval_us : s.last_input_interval_us;
    if (basis_interval == 0) {
        // No prior interval available; preserve the current behavior by using
        // all available samples.
        effective = available;
    } else {
        effective = (uint32_t)((MEASUREMENT_WINDOW_US + basis_interval / 2) / basis_interval);
        if (effective < MIN_EFFECTIVE_WINDOW) {
            effective = MIN_EFFECTIVE_WINDOW;
        } else if (effective > WINDOW_SIZE) {
            effective = WINDOW_SIZE;
        }
    }

    // Final count is the min of available samples and the chosen effective size.
    count = (uint64_t)(available < effective ? available : effective);
    std_dev  = 0.0;
    s.input_interval_us  = 0;
    // Do NOT zero output_interval_us here — leave last good value visible

    if (!s.last_edge_valid) {
        return false;
    }

    // Timeout: no edge for >= 1 second
    if (now_us - s.last_edge_us >= PULSE_TIMEOUT_US) {
        speedo_reset(s);
        count = 0;
        return false;
    }

    if (count == 0) {
        return false;
    }

    // Rolling average
    uint64_t sum = 0, output_sum = 0;
    // When the buffer is filled and we're using fewer than WINDOW_SIZE
    // samples, use the most-recent `count` entries. If the buffer hasn't
    // wrapped yet, use the first `count` entries.
    if (s.window_filled && count < WINDOW_SIZE) {
        // Use the most recent `count` samples ending at (pulse_pos - 1)
        int idx = (int)s.pulse_pos - 1;
        if (idx < 0) idx += WINDOW_SIZE;
        for (uint64_t i = 0; i < count; ++i) {
            sum        += s.pulse_in_window[idx];
            output_sum += s.pulse_out_window[idx];
            idx--;
            if (idx < 0) idx += WINDOW_SIZE;
        }
    } else {
        for (uint64_t i = 0; i < count; ++i) {
            sum        += s.pulse_in_window[i];
            output_sum += s.pulse_out_window[i];
        }
    }
    s.input_interval_us  = sum        / count;
    s.output_interval_us = output_sum / count;

    // Jitter (std dev of input widths)
    double variance = 0.0;
    if (s.window_filled && count < WINDOW_SIZE) {
        int idx = (int)s.pulse_pos - 1;
        if (idx < 0) idx += WINDOW_SIZE;
        for (uint64_t i = 0; i < count; ++i) {
            double diff = (double)s.pulse_in_window[idx] - (double)s.input_interval_us;
            variance += diff * diff;
            idx--;
            if (idx < 0) idx += WINDOW_SIZE;
        }
    } else {
        for (uint64_t i = 0; i < count; ++i) {
            double diff = (double)s.pulse_in_window[i] - (double)s.input_interval_us;
            variance += diff * diff;
        }
    }
    std_dev = sqrt(variance / count);

    return true;
}

// ---------------------------------------------------------------------------
// speedo_update_output
// Compares output_interval_us to last_output_interval_us. If the change
// exceeds 1 % the timer is restarted (or stopped if interval is too large).
// ---------------------------------------------------------------------------
void speedo_update_output(SpeedoState &s, SpeedoHal &hal) {
    float change_rate = 0.0f;
    if (s.output_interval_us > 0) {
        change_rate = (float)((int64_t)s.output_interval_us -
                              (int64_t)s.last_output_interval_us)
                      / (float)s.output_interval_us * 100.0f;
    } else if (s.last_output_interval_us == 0) {
        return;   // Still stopped — no change
    } else {
        change_rate = 100.0f; // Transition from running to stopped
    }

    if (change_rate > -1.0f && change_rate < 1.0f) {
        return;   // No significant change — leave timer alone
    }

    char buf[128];
    if (hal.log) {
        snprintf(buf, sizeof(buf),
                 "Output interval changed: %llu -> %llu us\n",
                 (unsigned long long)s.last_output_interval_us,
                 (unsigned long long)s.output_interval_us);
        hal.log(buf);
    }

    if (s.output_interval_us > 0 && s.output_interval_us < MAX_OUTPUT_PULSE_US) {
        if (hal.log) {
            snprintf(buf, sizeof(buf), "Setting output interval to %llu us\n",
                     (unsigned long long)s.output_interval_us);
            hal.log(buf);
        }
        if (s.last_output_interval_us == 0) {
            // Timer not running — start immediately rather than deferring.
            hal.cancel_timer();
            hal.start_timer(s.output_interval_us);
        } else {
            // Timer already running — let the current pulse finish first.
            hal.set_pending_interval(s.output_interval_us);
        }
    } else {
        hal.cancel_timer();
        if (hal.log) {
            snprintf(buf, sizeof(buf),
                     "Output interval %llu us out of range, disabling output.\n",
                     (unsigned long long)s.output_interval_us);
            hal.log(buf);
        }
        hal.set_output_pin(true);   // Park output high (idle state)
    }

    s.last_output_interval_us = s.output_interval_us;
}
