// speedo_mustang.cpp — Pico firmware wrapper
// Pure algorithm logic lives in speedo_logic.cpp/.h.
// This file owns: Pico SDK includes, ISR, real HAL, main().

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"

#include "speedo_logic.h"

// ---------------------------------------------------------------------------
// Pin assignments
// ---------------------------------------------------------------------------
#define PIN_INPUT  10
#define PIN_OUTPUT 14
#define AVG_INTERVAL_MS 300

// ---------------------------------------------------------------------------
// Global state — volatile so the compiler treats every ISR access as a real
// memory operation. Accessed only under save_and_disable_interrupts() in both
// the ISR and the main loop, which provides the required memory barrier.
// ---------------------------------------------------------------------------
static volatile SpeedoState g_state;
static volatile bool        g_output_pin_state = true;
static struct repeating_timer g_output_timer;

// ---------------------------------------------------------------------------
// Real HAL — called by speedo_update_output() in the main loop
// ---------------------------------------------------------------------------
static void hal_set_output_pin(bool state) {
    g_output_pin_state = state;
    gpio_put(PIN_OUTPUT, state);
}

static void hal_cancel_timer(void) {
    cancel_repeating_timer(&g_output_timer);
}

static bool timer_callback(struct repeating_timer *t) {
    g_output_pin_state = !g_output_pin_state;
    gpio_put(PIN_OUTPUT, g_output_pin_state);
    return true;
}

static void hal_start_timer(uint64_t interval_us) {
    // Negative value: period measured from the *start* of each callback,
    // giving a stable frequency regardless of callback execution time.
    add_repeating_timer_us(-(int64_t)interval_us, timer_callback, NULL,
                           &g_output_timer);
}

static void hal_log(const char *msg) {
    printf("%s", msg);
}

static SpeedoHal g_hal = {
    .set_output_pin = hal_set_output_pin,
    .cancel_timer   = hal_cancel_timer,
    .start_timer    = hal_start_timer,
    .log            = hal_log,
};

// ---------------------------------------------------------------------------
// GPIO ISR — fires on rising edge of input signal
// ---------------------------------------------------------------------------
static void gpio_callback(uint gpio, uint32_t events) {
    uint32_t irq_status = save_and_disable_interrupts();

    absolute_time_t now   = get_absolute_time();
    uint64_t        now_us = to_us_since_boot(now);

    uint64_t width_us = 0;
    if (g_state.last_edge_valid) {
        width_us = now_us - g_state.last_edge_us;

        // Cast away volatile for the logic call — safe because we hold the lock
        bool accepted = speedo_process_pulse(
            const_cast<SpeedoState &>(g_state), width_us);

        if (!accepted) {
            if (width_us > MAX_PULSE_US) {
                // A long timeout indicates signal loss; reset state and restart
                // the reference from the current edge.
                hal_cancel_timer();
                g_state.last_edge_us    = now_us;
                g_state.last_edge_valid = true;
            }
            restore_interrupts(irq_status);
            return;
        }

        g_state.last_edge_us    = now_us;
        g_state.last_edge_valid = true;
    } else {
        g_state.last_edge_us    = now_us;
        g_state.last_edge_valid = true;
    }

    restore_interrupts(irq_status);
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main() {
    stdio_init_all();

    if (watchdog_caused_reboot()) {
        printf("Rebooted by Watchdog!\n");
    }
    // Watchdog: reboot if main loop stalls for > 5 s; pause on debug
    watchdog_enable(5000, 1);

    // Input pin
    gpio_init(PIN_INPUT);
    gpio_set_dir(PIN_INPUT, GPIO_IN);
    gpio_set_irq_enabled_with_callback(PIN_INPUT, GPIO_IRQ_EDGE_RISE,
                                       true, &gpio_callback);

    // Output pin — start high (idle)
    gpio_init(PIN_OUTPUT);
    gpio_set_dir(PIN_OUTPUT, GPIO_OUT);
    gpio_put(PIN_OUTPUT, 1);

    // Init state
    speedo_reset(const_cast<SpeedoState &>(g_state));

    absolute_time_t next_avg_time = make_timeout_time_ms(AVG_INTERVAL_MS);

    while (true) {
        watchdog_update();

        if (absolute_time_diff_us(get_absolute_time(), next_avg_time) <= 0) {
            double   std_dev = 0.0;
            uint64_t count   = 0;

            // --- snapshot volatile state under lock for main-loop use ---
            uint32_t irq_status = save_and_disable_interrupts();
            SpeedoState snap = const_cast<SpeedoState &>(g_state);
            restore_interrupts(irq_status);

            uint64_t now_us = to_us_since_boot(get_absolute_time());
            bool have_data  = speedo_compute_metrics(snap, now_us, std_dev, count);

            // Write computed averages back under lock
            irq_status = save_and_disable_interrupts();
            g_state.input_interval_us  = snap.input_interval_us;
            g_state.output_interval_us = snap.output_interval_us;
            if (!have_data) {
                // timeout reset — propagate full reset back and park the output.
                const_cast<SpeedoState &>(g_state) = snap;
                hal_cancel_timer();
                hal_set_output_pin(true);
            }
            restore_interrupts(irq_status);

            // Diagnostics
            float freq_in  = snap.input_interval_us  > 0
                           ? 1e6f / snap.input_interval_us  : 0.0f;
            float freq_out = snap.output_interval_us > 0
                           ? 1e6f / snap.output_interval_us : 0.0f;
            printf("Freq: %.2f Hz | Avg: %llu us | Jitter: %.2f us | "
                   "Out: %.2f Hz | Out width: %llu us | "
                   "Prev out: %llu | Bad: %llu | Samples: %llu\n",
                   freq_in,  (unsigned long long)snap.input_interval_us,
                   std_dev,  freq_out,
                   (unsigned long long)snap.output_interval_us,
                   (unsigned long long)snap.last_output_interval_us,
                   (unsigned long long)snap.bad_pulse_count,
                   (unsigned long long)count);

            // Update output timer if interval changed meaningfully
            speedo_update_output(snap, g_hal);

            // Write last_output_interval_us back if update_output changed it
            irq_status = save_and_disable_interrupts();
            g_state.last_output_interval_us = snap.last_output_interval_us;
            restore_interrupts(irq_status);

            next_avg_time = make_timeout_time_ms(AVG_INTERVAL_MS);
        }

        sleep_ms(100);
    }

    return 0;
}
