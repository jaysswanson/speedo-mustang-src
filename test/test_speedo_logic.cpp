// test_speedo_logic.cpp — GoogleTest unit tests for speedo_logic.cpp
// Build with: cmake -DSPEEDO_UNIT_TEST=ON .. && make && ./speedo_tests
//
// These tests run entirely on the host — no Pico SDK or hardware needed.

#include <gtest/gtest.h>
#include "../speedo_logic.h"

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static SpeedoState make_state() {
    SpeedoState s{};
    speedo_reset(s);
    return s;
}

// Fake HAL that records calls for assertion
struct FakeHal {
    int    cancel_count  = 0;
    int    start_count   = 0;
    int    set_pin_count = 0;
    bool   last_pin_state = false;
    uint64_t last_interval = 0;

    SpeedoHal as_speedo_hal() {
        return SpeedoHal{
            .set_output_pin = [](bool s) { /* captured via pointer below */ },
            .cancel_timer   = nullptr,
            .start_timer    = nullptr,
            .log            = nullptr,
        };
    }
};

// Because SpeedoHal uses plain C function pointers we thread state through
// a module-level pointer for tests that need to inspect HAL calls.
static FakeHal *g_fake = nullptr;

static SpeedoHal make_fake_hal(FakeHal &fake) {
    g_fake = &fake;
    return SpeedoHal{
        .set_output_pin = [](bool s) {
            g_fake->set_pin_count++;
            g_fake->last_pin_state = s;
        },
        .cancel_timer = []() { g_fake->cancel_count++; },
        .start_timer  = [](uint64_t iv) {
            g_fake->start_count++;
            g_fake->last_interval = iv;
        },
        .log = nullptr,   // suppress output during tests
    };
}

// ---------------------------------------------------------------------------
// speedo_reset
// ---------------------------------------------------------------------------
TEST(Reset, ZeroesAllFields) {
    SpeedoState s{};
    s.bad_pulse_count = 99;    // must survive reset
    s.pulse_pos       = 2;
    s.window_filled   = true;
    s.output_interval_us = 12345;

    speedo_reset(s);

    EXPECT_EQ(s.pulse_pos, 0U);
    EXPECT_FALSE(s.window_filled);
    EXPECT_FALSE(s.last_edge_valid);
    EXPECT_EQ(s.output_interval_us, 0U);
    EXPECT_EQ(s.input_interval_us,  0U);
    // bad_pulse_count is a lifetime counter — NOT reset
    EXPECT_EQ(s.bad_pulse_count, 99U);
}

// ---------------------------------------------------------------------------
// speedo_process_pulse — rejection cases
// ---------------------------------------------------------------------------
TEST(ProcessPulse, TooShortIncrementsBadCount) {
    auto s = make_state();
    bool ok = speedo_process_pulse(s, MIN_PULSE_US - 1);
    EXPECT_FALSE(ok);
    EXPECT_EQ(s.bad_pulse_count, 1U);
    EXPECT_EQ(s.pulse_pos, 0U);
}

TEST(ProcessPulse, TooLongResetsState) {
    auto s = make_state();
    // Put some data in first
    speedo_process_pulse(s, 10000);
    ASSERT_EQ(s.pulse_pos, 1U);

    bool ok = speedo_process_pulse(s, MAX_PULSE_US + 1);
    EXPECT_FALSE(ok);
    EXPECT_EQ(s.pulse_pos, 0U);          // reset clears buffer
    EXPECT_FALSE(s.window_filled);
}

// ---------------------------------------------------------------------------
// speedo_process_pulse — acceptance and scaling
// ---------------------------------------------------------------------------
TEST(ProcessPulse, ValidPulseStoredAndScaled) {
    auto s = make_state();
    bool ok = speedo_process_pulse(s, 80000);   // 80 ms input → 10 ms output
    EXPECT_TRUE(ok);
    EXPECT_EQ(s.pulse_in_window[0],  80000U);
    EXPECT_EQ(s.pulse_out_window[0], 10000U);   // 80000 / 8
    EXPECT_EQ(s.pulse_pos, 1U);
    EXPECT_FALSE(s.window_filled);
}

TEST(ProcessPulse, BufferWrapsAndSetsWindowFilled) {
    auto s = make_state();
    for (int i = 0; i < (int)WINDOW_SIZE; ++i) {
        speedo_process_pulse(s, 10000);
    }
    EXPECT_TRUE(s.window_filled);
    EXPECT_EQ(s.pulse_pos, 0U);   // wrapped back to start
}

TEST(ProcessPulse, AtMinBoundaryAccepted) {
    auto s = make_state();
    EXPECT_TRUE(speedo_process_pulse(s, MIN_PULSE_US));
}

TEST(ProcessPulse, AtMaxBoundaryRejected) {
    auto s = make_state();
    // MAX_PULSE_US itself is > MAX_PULSE_US — false; equal is also > so rejected
    EXPECT_FALSE(speedo_process_pulse(s, MAX_PULSE_US + 1));
}

// ---------------------------------------------------------------------------
// speedo_compute_metrics — no data / timeout
// ---------------------------------------------------------------------------
TEST(ComputeMetrics, NoEdgeReturnsFalse) {
    auto s = make_state();   // last_edge_valid = false
    double std_dev; uint64_t count;
    bool ok = speedo_compute_metrics(s, 5000000, std_dev, count);
    EXPECT_FALSE(ok);
    EXPECT_EQ(count, 0U);
}

TEST(ComputeMetrics, TimeoutResetsAndReturnsFalse) {
    auto s = make_state();
    s.last_edge_valid = true;
    s.last_edge_us    = 0;
    // now_us far in the future → timeout
    double std_dev; uint64_t count;
    bool ok = speedo_compute_metrics(s, PULSE_TIMEOUT_US + 1, std_dev, count);
    EXPECT_FALSE(ok);
    EXPECT_FALSE(s.last_edge_valid);   // reset was called
}

// ---------------------------------------------------------------------------
// speedo_compute_metrics — happy path
// ---------------------------------------------------------------------------
TEST(ComputeMetrics, ConstantPulsesGiveZeroJitter) {
    auto s = make_state();
    const uint64_t W = 10000;
    for (uint32_t i = 0; i < WINDOW_SIZE; ++i) {
        s.pulse_in_window[i]  = W;
        s.pulse_out_window[i] = W / 8;
    }
    s.window_filled   = true;
    s.last_edge_valid = true;
    s.last_edge_us    = 1000;   // recent edge

    double std_dev; uint64_t count;
    bool ok = speedo_compute_metrics(s, 1000 + 500, std_dev, count);

    EXPECT_TRUE(ok);
    EXPECT_EQ(count, (uint64_t)WINDOW_SIZE);
    EXPECT_DOUBLE_EQ(std_dev, 0.0);
    EXPECT_EQ(s.input_interval_us,  W);
    EXPECT_EQ(s.output_interval_us, W / 8);
}

TEST(ComputeMetrics, VaryingPulsesGiveNonZeroJitter) {
    auto s = make_state();
    s.pulse_in_window[0]  = 9000;
    s.pulse_in_window[1]  = 10000;
    s.pulse_in_window[2]  = 11000;
    s.pulse_out_window[0] = 9000  / 8;
    s.pulse_out_window[1] = 10000 / 8;
    s.pulse_out_window[2] = 11000 / 8;
    s.window_filled   = true;
    s.last_edge_valid = true;
    s.last_edge_us    = 500;

    double std_dev; uint64_t count;
    speedo_compute_metrics(s, 500 + 100, std_dev, count);

    EXPECT_GT(std_dev, 0.0);
    EXPECT_EQ(s.input_interval_us, 10000U);
}

// ---------------------------------------------------------------------------
// speedo_update_output — hysteresis / no change
// ---------------------------------------------------------------------------
TEST(UpdateOutput, SmallChangeDoesNotRestartTimer) {
    auto s = make_state();
    FakeHal fake;
    SpeedoHal hal = make_fake_hal(fake);

    s.output_interval_us      = 10000;
    s.last_output_interval_us = 10050;   // 0.5 % change — below threshold

    speedo_update_output(s, hal);

    EXPECT_EQ(fake.cancel_count, 0);
    EXPECT_EQ(fake.start_count,  0);
}

TEST(UpdateOutput, LargeChangeRestartsTimer) {
    auto s = make_state();
    FakeHal fake;
    SpeedoHal hal = make_fake_hal(fake);

    s.output_interval_us      = 10000;
    s.last_output_interval_us = 5000;   // 50 % change

    speedo_update_output(s, hal);

    EXPECT_EQ(fake.cancel_count, 1);
    EXPECT_EQ(fake.start_count,  1);
    EXPECT_EQ(fake.last_interval, 10000U);
    EXPECT_EQ(s.last_output_interval_us, 10000U);
}

TEST(UpdateOutput, IntervalExceedsMaxDisablesOutput) {
    auto s = make_state();
    FakeHal fake;
    SpeedoHal hal = make_fake_hal(fake);

    s.output_interval_us      = MAX_OUTPUT_PULSE_US + 1;
    s.last_output_interval_us = 10000;

    speedo_update_output(s, hal);

    EXPECT_EQ(fake.cancel_count,  1);
    EXPECT_EQ(fake.start_count,   0);       // timer NOT started
    EXPECT_EQ(fake.set_pin_count, 1);       // output parked high
    EXPECT_TRUE(fake.last_pin_state);
}

TEST(UpdateOutput, ZeroIntervalDisablesOutput) {
    auto s = make_state();
    FakeHal fake;
    SpeedoHal hal = make_fake_hal(fake);

    s.output_interval_us      = 0;
    s.last_output_interval_us = 10000;   // was running — now stopped

    speedo_update_output(s, hal);

    EXPECT_EQ(fake.cancel_count,  1);
    EXPECT_EQ(fake.start_count,   0);
    EXPECT_EQ(fake.set_pin_count, 1);
}

// ---------------------------------------------------------------------------
// Scaling sanity: 55 mph input should produce correct output interval
// 55 mph → input freq = 55 × 2.22 Hz = 122.1 Hz → period ≈ 8184 µs
// output period = 8184 / 8 ≈ 1023 µs
// ---------------------------------------------------------------------------
TEST(Scaling, FiftyFiveMphRoundTrip) {
    auto s = make_state();
    const uint64_t input_period_us = 8184;
    for (uint32_t i = 0; i < WINDOW_SIZE; ++i) {
        s.pulse_in_window[i]  = input_period_us;
        s.pulse_out_window[i] = input_period_us / 8;
    }
    s.window_filled   = true;
    s.last_edge_valid = true;
    s.last_edge_us    = 1000;

    double std_dev; uint64_t count;
    speedo_compute_metrics(s, 2000, std_dev, count);

    float out_freq = 1e6f / (2.0f * s.output_interval_us);
    // expect ~489 Hz full-wave output frequency (8.89 Hz/mph × 55 mph)
    EXPECT_NEAR(out_freq, 489.0f, 10.0f);
}
