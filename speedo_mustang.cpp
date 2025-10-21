#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h"
//#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include <math.h>

#define PIN_INPUT 10
#define PIN_OUTPUT 14
#define WINDOW_SIZE 3 // Size of the circular buffer for pulse widths
#define AVG_INTERVAL_MS 300
//#define NUM_ZEROS 10 // Number of samples to wait before zeroing the pulse width

// output pulse frequency
// 8.89Hz=1mph  782Hz=88mph  489Hz=55mph  1160=130mph 
// 1/4th frequency for input pulse so 2.22Hz=1mph  195.5Hz=88mph  122.25Hz=55mph  290Hz=130mph
// 1/4th frequency means output pulse width is 1/4th the input pulse width

// Output pulse period of 0.8ms is 1250Hz which is 140.1mph (assuming 8.89Hz=1mph)
// 0.8ms is a bit fast for a speedo signal, but some bikes can do that at high speed
// input is 1/4th of the frequency so 0.8ms out is 3.2ms in
#define MIN_PULSE_US 3200 
// 8.89Hz / 112.5ms output period is 1mph 
// input is 1/4th frequency so 112.5ms out is 450ms in
// 750ms would be 0.593mph 
#define MAX_PULSE_US 750000
#define MAX_OUTPUT_PULSE_US 93750 // 1/8th of the max input pulse width (750ms) for max output pulse width
#define PULSE_TIMEOUT_US 1000000 // 1 second timeout for pulse width measurement

// Circular buffer for pulse widths
volatile uint64_t pulse_in_window[WINDOW_SIZE];
volatile uint64_t pulse_pos = 0;
volatile bool window_filled = false;

// Input Timing variables
absolute_time_t last_edge = nil_time;
volatile uint64_t last_input_interval_us = 0;
volatile uint64_t bad_pulse_count = 0;

// Average input pulse width
volatile uint64_t input_interval_us = 0; // Average input pulse width in microseconds

// Output pulse generation
volatile uint64_t pulse_out_window[WINDOW_SIZE];
volatile uint64_t output_interval_us = 0;
volatile bool output_pin_state = true;
volatile uint64_t last_output_interval_us = 0;

struct repeating_timer output_timer;


void reset_circular_buffer() {
    for (uint64_t i = 0; i < WINDOW_SIZE; ++i) {
        pulse_in_window[i] = 0;
        pulse_out_window[i] = 0;
    }
    pulse_pos = 0;
    window_filled = false;
    output_interval_us = 0; 
    //last_output_interval_us = 0;
    input_interval_us = 0;
    last_edge = nil_time; // Reset last edge time
        
}

// ISR:  Toggles PIN_OUTPUT pin on each timer callback
bool repeating_timer_callback(struct repeating_timer *t) {
    output_pin_state = !output_pin_state;
    gpio_put(PIN_OUTPUT, output_pin_state);
    return true;
}

// ISR: On rising edge of input pulse
void gpio_callback(uint gpio, uint32_t events) {
    uint32_t irq_status = save_and_disable_interrupts();

    absolute_time_t now = get_absolute_time();
    uint64_t width_us = 0;

    if (!is_nil_time(last_edge)) {
        width_us = absolute_time_diff_us(last_edge, now);
        last_input_interval_us = width_us;

        if (width_us >= MIN_PULSE_US) {
            if ( width_us > MAX_PULSE_US) {
                //width_us = MAX_PULSE_US; // Cap the pulse width to prevent overflow
                printf("Bad pulse width: %lu µs\n", width_us);
                reset_circular_buffer();
                cancel_repeating_timer(&output_timer);  // Stop current timer
                    
                printf("Resetting circular buffer and cancelling timer due to bad pulse width.\n");    
            }
                
            else {
                pulse_in_window[pulse_pos] = width_us;
                pulse_out_window[pulse_pos ] = (width_us==MAX_PULSE_US)?0:(width_us / 8); // Store output pulse width
                // this is 1/4th the input pulse width. Use 8 because there are 2 half cycles / toggles in a full cycle
                pulse_pos++;
                if (pulse_pos >= WINDOW_SIZE) {
                    pulse_pos = 0;
                    window_filled = true;
                }
            }
        } else {
            bad_pulse_count++;
        }
        
    }
    last_edge = now;
    
    
    

    restore_interrupts(irq_status);
}



// Signal quality metrics: avg, jitter
void compute_quality_metrics(double& std_dev, uint64_t& count) {
    
    count = window_filled ? WINDOW_SIZE : pulse_pos;
    input_interval_us = 0;
    std_dev = 0.0;

    uint32_t irq_status = save_and_disable_interrupts();
    if (!is_nil_time(last_edge)) {
        absolute_time_t now = get_absolute_time();
        uint64_t width_us = absolute_time_diff_us(last_edge, now);
        if (width_us >= PULSE_TIMEOUT_US) {
            // If the last pulse was too long, reset the circular buffer
            reset_circular_buffer();
            cancel_repeating_timer(&output_timer);  // Stop current timer
                    
            printf("Resetting circular buffer and cancelling timer due to timeout.\n");
        }
        else if (count > 0) {
            uint64_t sum = 0;
            uint64_t output_sum = 0;
            uint64_t diff = 0;

            double variance = 0.0;
            for (uint64_t i = 0; i < count; ++i) {
                sum += pulse_in_window[i];
                output_sum += pulse_out_window[i];
            }
            input_interval_us = sum / count;
            output_interval_us = output_sum / count;

            for (uint64_t i = 0; i < count; ++i) {

                diff = pulse_in_window[i] - input_interval_us;
                variance += diff * diff;
            }

            std_dev = sqrt(variance / count);
        }
    }
    restore_interrupts(irq_status);

}





int main()
{
    stdio_init_all();

    // Watchdog example code
    if (watchdog_caused_reboot()) {
        printf("Rebooted by Watchdog!\n");
        // Whatever action you may take if a watchdog caused a reboot
    }
    
    // Enable the watchdog, requiring the watchdog to be updated every 1000ms or the chip will reboot
    // second arg is pause on debug which means the watchdog will pause when stepping through code
    watchdog_enable(5000, 1);

    int empty_count = 0;


    // Input setup
    gpio_init(PIN_INPUT);
    gpio_set_dir(PIN_INPUT, GPIO_IN);
    //gpio_pull_up(PIN_INPUT);
    gpio_set_irq_enabled_with_callback(PIN_INPUT, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

    // Output setup
    gpio_init(PIN_OUTPUT);
    gpio_set_dir(PIN_OUTPUT, GPIO_OUT);
    //gpio_pull_down(PIN_OUTPUT);
    gpio_put(PIN_OUTPUT, 1);

    // Main processing loop
    absolute_time_t next_avg_time = make_timeout_time_ms(AVG_INTERVAL_MS);

    //DEBUG DEBUG
    //output_interval_us=2500;

    while (true) {

        // You need to call this function at least more often than the 1000ms in the enable call to prevent a reboot
        watchdog_update();


        if (absolute_time_diff_us(get_absolute_time(), next_avg_time) <= 0) {
            uint64_t avg = 0, count = 0;
            double std_dev = 0.0;

            //DEBUG DEBUG
            compute_quality_metrics(std_dev, count);
            
            float freq_hz = input_interval_us > 0 ? 1e6f / input_interval_us : 0;
            float output_freq_hz = output_interval_us > 0 ? 1e6f / output_interval_us : 0;
            printf("Freq: %.2f Hz | Avg Width: %u µs | Jitter: %.2f µs | Output: %.2f Hz | Out width: %u µs | Previous Out: %u | Bad Pulses: %u | Samples: %u\n",
                   freq_hz, input_interval_us, std_dev, output_freq_hz, output_interval_us, last_output_interval_us, bad_pulse_count, count);
  
            float change_rate = 0.0;
            if (output_interval_us > 0) {
                change_rate = (float)(output_interval_us - last_output_interval_us) / output_interval_us * 100.0f;
            }
            //if ( output_interval_us - last_output_interval_us > 2*std_dev || output_interval_us - last_output_interval_us < -2*std_dev ) {
            if ( change_rate > 1 || change_rate < -1 ) {    
                printf("Output interval changed significantly: %X -> %X\n", last_output_interval_us, output_interval_us);
                // Restart the output pulse timer with the new interval
                if (output_interval_us < MAX_OUTPUT_PULSE_US) {
                    cancel_repeating_timer(&output_timer);  // Stop current timer
                    //negative interval calculates the period from the start of each callback
                    //positive interval calculates the period from the end of the callback 
                    printf("Setting output interval to %u µs\n", output_interval_us);
                    add_repeating_timer_us(-output_interval_us, repeating_timer_callback, NULL, &output_timer);
                    empty_count = 0; // Reset empty count when output is enabled
                }
                else { 
                    cancel_repeating_timer(&output_timer);  // Stop current timer
                    printf("Output interval %u µs exceeds maximum %u µs, disabling output.\n", output_interval_us, MAX_OUTPUT_PULSE_US);
                    gpio_put(PIN_OUTPUT, 1);
                    output_pin_state = true;
                    empty_count = 0; // Reset empty count when output is disabled
                }
                last_output_interval_us = output_interval_us;
                printf("Last output interval updated to %X µs\n", last_output_interval_us);
            }

            
            next_avg_time = make_timeout_time_ms(AVG_INTERVAL_MS);
        }

        sleep_ms(100);
    }

    return 0;


}
