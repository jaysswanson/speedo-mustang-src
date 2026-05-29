# Host-side unit tests

## Prerequisites

GoogleTest installed on your host machine:

```bash
# Ubuntu/Debian
sudo apt install libgtest-dev cmake build-essential

# macOS (Homebrew)
brew install googletest cmake

# Windows (vcpkg)
vcpkg install gtest
```

## Build and run

From the repo root — use a **separate build directory** so the test build
doesn't conflict with the Pico firmware build directory:

```bash
cmake -B build-test -DSPEEDO_UNIT_TEST=ON -DCMAKE_BUILD_TYPE=Debug
cmake --build build-test
ctest --test-dir build-test --output-on-failure
```

Or run the test binary directly for verbose output:

```bash
./build-test/speedo_tests
```

## What is tested

| Test group           | What it covers                                             |
|----------------------|------------------------------------------------------------|
| `Reset`              | All fields zeroed; `bad_pulse_count` survives reset        |
| `ProcessPulse`       | Short/long rejection, valid acceptance, ÷8 scaling, wrap  |
| `ComputeMetrics`     | No-data, timeout reset, zero jitter, non-zero jitter       |
| `UpdateOutput`       | Hysteresis (<1 % no restart), large change restarts timer, |
|                      | over-range and zero interval disable output                |
| `Scaling`            | 55 mph round-trip frequency sanity check                   |
