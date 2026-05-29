# Speedo Mustang — Technical Design Document

Firmware for the Raspberry Pi Pico that converts a Mustang speed sensor's
pulse train into a calibrated output pulse stream for an analog speedometer
gauge.  The firmware runs on a bare Pico board wired in-line between the
vehicle's transmission speed sensor and the gauge cluster.

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Hardware Design](#2-hardware-design)
3. [Signal Characteristics](#3-signal-characteristics)
4. [Frequency Scaling and Calibration](#4-frequency-scaling-and-calibration)
5. [Software Architecture](#5-software-architecture)
6. [Core Data Structures](#6-core-data-structures)
7. [Algorithm Detail](#7-algorithm-detail)
8. [Interrupt and Concurrency Model](#8-interrupt-and-concurrency-model)
9. [Diagnostics](#9-diagnostics)
10. [Build, Flash, and Debug](#10-build-flash-and-debug)
11. [Testing](#11-testing)
12. [Configuration Reference](#12-configuration-reference)

---

## 1. System Overview

```
  Transmission speed sensor (magnetic reluctor sensor)
        │
        │  raw pulse train (low freq, 1/4× output)
        ▼
  ┌─────────────┐
  │  Pico       │  GPIO 10 ──► interrupt → measure width → filter → average
  │  firmware   │
  │             │  GPIO 14 ──► repeating timer toggles → scaled output pulses
  └─────────────┘
        │
        │  conditioned pulse train (speedometer frequency)
        ▼
  Computer input to speedometer gauge
```

The sensor produces a pulse frequency that is **one-quarter** of the frequency
the gauge expects.  The Pico measures the period of each incoming pulse,
applies a three-sample rolling average to remove noise, and re-generates
pulses at the correct frequency (period ÷ 8, because dividing the period by 8
is equivalent to multiplying frequency by 8 — covering both the 4× ratio and
the two half-cycles required to produce a full square-wave toggle cycle).

---

## 2. Hardware Design

### 2.1 Components

| Component | Details |
|---|---|
| Microcontroller | Raspberry Pi Pico (RP2040, dual-core ARM Cortex-M0+, 133 MHz) |
| Input pin | GPIO 10 — rising-edge interrupt |
| Output pin | GPIO 14 — square-wave driven by hardware repeating timer |
| Power | 3.3 V from Pico VSYS / onboard regulator |
| Debug interface | SWD via `raspberrypi-swd.cfg` (3-wire: SWDIO, SWCLK, GND) |
| Diagnostics | USB serial (UART disabled; USB stdio enabled in CMake) |

### 2.2 Pin Connections

```
Vehicle harness                  Pico
─────────────────────────────────────────────────────
Transmission speed sensor signal ──────► GPIO 10  (PIN_INPUT)
Speedometer gauge signal  ◄──────  GPIO 14  (PIN_OUTPUT)
Ground                    ─────── GND
+5 V / +3.3 V             ──────► VSYS / 3V3
```

### 2.3 Watchdog

The hardware watchdog is enabled with a **5 000 ms** timeout.  The main loop
calls `watchdog_update()` every 100 ms.  If the main loop stalls for any
reason the Pico automatically resets, and a `"Rebooted by Watchdog!"` message
is printed on the next boot.

---

## 3. Signal Characteristics

### 3.1 Input signal

| Parameter | Value | Notes |
|---|---|---|
| Signal type | Digital square wave | Rising-edge triggered |
| Minimum period | 3 200 µs (3.2 ms) | = 312.5 Hz = ~140 mph input |
| Maximum period | 750 000 µs (750 ms) | = 1.33 Hz = ~0.6 mph |
| Timeout threshold | 1 000 000 µs (1 s) | No edge → signal loss |
| Frequency ratio | 1/4× output frequency | Sensor runs slower than gauge expects |

Pulses shorter than 3 200 µs are counted as noise/glitch
(`bad_pulse_count`) and discarded.  Pulses longer than 750 000 µs indicate a
stationary vehicle or lost signal; the state machine resets and the output
timer is cancelled.

> NOTE: The input signal from the transmission speed sensor is a magnetic reluctor
> signal. The raw signal is conditioned by a MAX9925. The output of the conditioner
> is a clean 5v square wave.  That signal is level shifted to 3.3v for the Pico
> by a BC848 NPN transistor

### 3.2 Output signal

| Parameter | Value | Notes |
|---|---|---|
| Signal type | 50 % duty-cycle square wave | Timer toggles each half-period |
| Minimum period | 800 µs (0.8 ms) | ~1 250 Hz = 140 mph |
| Maximum period | 93 750 µs (93.75 ms) | ~10.67 Hz = ~1.2 mph |
| Idle state | GPIO 14 held high | When no valid signal present |

> NOTE: The computer that controls the spedometer is expecting a signal from the 
> magnetic reluctor. To simulate that, the output drives a mosfet that drives a 
> 1:1 transformer. The output signal from the secondary is close enough to the
> signal from a reluctor that the computer is happy with it. The frequency-shifted
> signal is at the correct frequency so that the spedometer is accurate. The output
> pin signal from the Pico (at 3.3v) is buffered by a BC848 NPN transistor, which
> in turn drives the transformer.

---

## 4. Frequency Scaling and Calibration

### 4.1 Calibration constant

```
8.89 Hz output = 1 mph
```

Derived values:

| Speed | Output frequency | Output period | Input period (÷4) |
|---|---|---|---|
| 1 mph | 8.89 Hz | 112 500 µs | 450 000 µs |
| 55 mph | 489 Hz | 2 045 µs | 8 180 µs |
| 88 mph | 782 Hz | 1 279 µs | 5 115 µs |
| 130 mph | 1 160 Hz | 862 µs | 3 448 µs |
| 140 mph | 1 250 Hz | 800 µs | 3 200 µs |

### 4.2 Period scaling formula

The sensor input frequency is **one-quarter** of the gauge output frequency:

```
f_output = 4 × f_input
```

Expressed in periods:

```
T_output = T_input / 4
```

The hardware timer toggles the output pin on each callback (one toggle = one
half-cycle), so two callbacks produce one full output cycle.  The timer
interval must therefore be **half** the desired output period:

```
timer_interval = T_output / 2 = T_input / 8
```

In code:

```cpp
pulse_out_window[i] = width_us / 8;
```

### 4.3 Output disable threshold

If the computed `output_interval_us` equals or exceeds `MAX_OUTPUT_PULSE_US`
(93 750 µs) the gauge would read below ~1 mph.  Rather than drive the gauge
with an unreliable slow signal, the output is disabled and GPIO 14 is parked
high.

---

## 5. Software Architecture

After the refactor the codebase is split into three layers:

```
┌─────────────────────────────────────────────────────────┐
│  speedo_mustang.cpp  (firmware wrapper)                 │
│  • Pico SDK #includes                                   │
│  • volatile SpeedoState global                          │
│  • Real SpeedoHal (gpio_put, repeating_timer, printf)   │
│  • gpio_callback ISR                                    │
│  • main() loop                                          │
├─────────────────────────────────────────────────────────┤
│  speedo_logic.cpp / speedo_logic.h  (pure logic)        │
│  • speedo_reset()                                       │
│  • speedo_process_pulse()                               │
│  • speedo_compute_metrics()                             │
│  • speedo_update_output()                               │
│  No Pico SDK headers — compiles on any host             │
├─────────────────────────────────────────────────────────┤
│  test/test_speedo_logic.cpp  (GoogleTest, host-only)    │
│  • Fake SpeedoHal                                       │
│  • 15 unit tests covering all logic paths               │
└─────────────────────────────────────────────────────────┘
```

### 5.1 Execution contexts

| Context | Trigger | Functions called |
|---|---|---|
| GPIO ISR | Rising edge on GPIO 10 | `speedo_process_pulse()` |
| Timer ISR | Repeating hardware timer | Output pin toggle (inline in callback) |
| Main loop | Every 300 ms | `speedo_compute_metrics()`, `speedo_update_output()` |

### 5.2 Main loop timing

```
┌── every 100 ms ──────────────────────────────────────────────┐
│  watchdog_update()                                           │
│  if 300 ms elapsed:                                          │
│    snapshot volatile state under IRQ lock                    │
│    speedo_compute_metrics()   — rolling average + jitter     │
│    write averages back under IRQ lock                        │
│    speedo_update_output()     — restart timer if needed      │
│    write last_output_interval back under IRQ lock            │
│    printf diagnostics                                        │
│    schedule next 300 ms window                               │
└──────────────────────────────────────────────────────────────┘
```

---

## 6. Core Data Structures

### 6.1 `SpeedoState`

All mutable algorithm state in a single struct.  In the firmware this is
declared as a `volatile SpeedoState` global; in tests it is a plain local
variable.

| Field | Type | Description |
|---|---|---|
| `pulse_in_window[3]` | `uint64_t[]` | Circular buffer of raw input pulse widths (µs) |
| `pulse_out_window[3]` | `uint64_t[]` | Corresponding scaled output half-periods (µs) |
| `pulse_pos` | `uint64_t` | Next write index into the circular buffer |
| `window_filled` | `bool` | True once the buffer has been written at least once fully |
| `last_edge_us` | `uint64_t` | Absolute timestamp (µs since boot) of most recent rising edge |
| `last_edge_valid` | `bool` | False until the first edge is seen |
| `last_input_interval_us` | `uint64_t` | Raw width of the most recent pulse (pre-validation) |
| `bad_pulse_count` | `uint64_t` | Lifetime count of rejected short pulses (never reset) |
| `input_interval_us` | `uint64_t` | Rolling average input period (output of `compute_metrics`) |
| `output_interval_us` | `uint64_t` | Rolling average output half-period (drives timer) |
| `last_output_interval_us` | `uint64_t` | Previous output interval (used for change-rate hysteresis) |

### 6.2 `SpeedoHal`

Four plain C function pointers injected at runtime.  The firmware provides
real Pico SDK implementations; tests provide lightweight fakes.

| Field | Firmware implementation | Test fake |
|---|---|---|
| `set_output_pin(bool)` | `gpio_put(PIN_OUTPUT, …)` | Records state |
| `cancel_timer()` | `cancel_repeating_timer(&g_output_timer)` | Increments counter |
| `start_timer(uint64_t)` | `add_repeating_timer_us(…)` | Records interval |
| `log(const char*)` | `printf("%s", …)` | `nullptr` (silent) |

---

## 7. Algorithm Detail

### 7.1 `speedo_process_pulse(state, width_us)`

Called from the GPIO ISR with the measured interval between consecutive rising
edges.

```
width_us < MIN_PULSE_US (3 200)?
  → discard, increment bad_pulse_count, return false

width_us > MAX_PULSE_US (750 000)?
  → speedo_reset(), return false  (caller cancels timer)

otherwise:
  → write to pulse_in_window[pulse_pos]
  → write width_us / 8 to pulse_out_window[pulse_pos]
  → advance pulse_pos; set window_filled when it wraps
  → return true
```

### 7.2 `speedo_compute_metrics(state, now_us, &std_dev, &count)`

Called from the main loop every 300 ms.  `now_us` is injected so tests can
control time.

```
last_edge_valid == false?  → return false (no data yet)

now_us - last_edge_us >= PULSE_TIMEOUT_US (1 000 000)?
  → speedo_reset(), return false  (signal lost)

count = window_filled ? 3 : pulse_pos

input_interval_us  = mean(pulse_in_window[0..count-1])
output_interval_us = mean(pulse_out_window[0..count-1])
std_dev            = sqrt( Σ(sample - mean)² / count )   (jitter metric)
```

Note: `diff` is computed as `double` to avoid unsigned underflow when a sample
is smaller than the mean.

### 7.3 `speedo_update_output(state, hal)`

Called from the main loop after `compute_metrics`.

```
change_rate = (output_interval_us - last_output_interval_us)
              / output_interval_us × 100 %

|change_rate| < 1 %?  → return (hysteresis — no unnecessary timer restart)

otherwise:
  hal.cancel_timer()

  output_interval_us in (0, MAX_OUTPUT_PULSE_US)?
    → hal.start_timer(output_interval_us)
  else:
    → hal.set_output_pin(true)   // park high, disable output

  last_output_interval_us = output_interval_us
```

The 1 % hysteresis band prevents the hardware timer from being restarted on
every loop cycle due to minor measurement noise.

### 7.4 State machine summary

```
         ┌─────────────────────┐
  boot   │                     │  bad pulse (too long)
 ───────►│      IDLE           │◄─────────────────────────────────┐
         │  (no valid signal)  │                                  │
         └────────┬────────────┘                                  │
                  │ first valid pulse                             │
                  ▼                                               │
         ┌─────────────────────┐    timeout (>1 s silence)        │
         │    ACCUMULATING     │──────────────────────────────────┤
         │  (buffer filling)   │                                  │
         └────────┬────────────┘                                  │
                  │ window_filled == true                         │
                  ▼                                               │
         ┌─────────────────────┐    timeout or bad long pulse     │
         │      RUNNING        │──────────────────────────────────┘
         │  (timer active,     │
         │   gauge driven)     │
         └─────────────────────┘
```

---

## 8. Interrupt and Concurrency Model

The RP2040 is single-core for this firmware (only core 0 is used).  Two
interrupt sources share the `SpeedoState` global with the main loop:

| Source | Modifies | Reads |
|---|---|---|
| GPIO ISR (`gpio_callback`) | `pulse_in/out_window`, `pulse_pos`, `window_filled`, `last_edge_*`, `bad_pulse_count` | — |
| Timer ISR (`timer_callback`) | `g_output_pin_state`, GPIO 14 | — |
| Main loop | `last_output_interval_us` | All of the above |

**Mutual exclusion strategy:** all shared-state accesses in the main loop are
wrapped in `save_and_disable_interrupts()` / `restore_interrupts()`.  The main
loop takes a full snapshot of `SpeedoState` under the lock, operates on the
snapshot, then writes only the two fields it computes (`input_interval_us`,
`output_interval_us`, `last_output_interval_us`) back under a second lock.
This keeps the critical section as short as possible and avoids holding the
lock across floating-point math or `printf`.

The timer ISR does not access `SpeedoState`; it only reads and toggles
`g_output_pin_state` (a `volatile bool`) and calls `gpio_put`, so no lock is
needed inside the timer callback.

---

## 9. Diagnostics

Every 300 ms the main loop prints a single diagnostic line over USB serial
(115 200 baud, no parity):

```
Freq: 122.25 Hz | Avg: 8184 us | Jitter: 12.30 us | Output: 489.00 Hz | Out width: 1023 us | Prev out: 1025 | Bad: 0 | Samples: 3
```

| Field | Meaning |
|---|---|
| `Freq` | Input pulse frequency (Hz) derived from rolling average period |
| `Avg` | Rolling average input pulse width (µs) |
| `Jitter` | Standard deviation of input pulse widths in the current window (µs) |
| `Output` | Output pulse frequency (Hz) |
| `Out width` | Output timer half-period (µs) |
| `Prev out` | Output half-period from the previous update cycle |
| `Bad` | Lifetime count of rejected short pulses |
| `Samples` | Number of samples in the current averaging window (1–3) |

Connect any USB serial terminal at 115 200 baud to observe the output:

```powershell
# Windows — substitute the correct COM port
mode COM3: BAUD=115200 PARITY=n DATA=8 STOP=1
```

---

## 10. Build, Flash, and Debug

### 10.1 Prerequisites

- Raspberry Pi Pico SDK 2.2.0 (auto-located via `pico_sdk_import.cmake`)
- CMake 3.13+
- Ninja build system
- ARM GCC toolchain (installed by the VS Code Pico extension under
  `~/.pico-sdk/toolchain/`)
- picotool 2.2.0-a4 (for USB loading)
- OpenOCD 0.12.0+dev (for SWD flash/debug)

### 10.2 Firmware build

```powershell
# Configure (first time only)
cmake -B build -G Ninja

# Build
cmake --build build

# Output
build/speedo_mustang.uf2   # drag-and-drop or picotool image
build/speedo_mustang.elf   # ELF for GDB / disassembly
build/speedo_mustang.dis   # pre-generated disassembly
```

VS Code shortcut: **Run Task → Compile Project**

### 10.3 Flash via SWD (OpenOCD)

```powershell
openocd -s ~/.pico-sdk/openocd/0.12.0+dev/scripts `
        -f raspberrypi-swd.cfg `
        -f target/rp2040.cfg `
        -c "adapter speed 5000; program build/speedo_mustang.elf verify reset exit"
```

VS Code shortcut: **Run Task → Flash**

### 10.4 Fast load via USB (picotool)

Hold BOOTSEL while connecting USB, then:

```powershell
~/.pico-sdk/picotool/2.2.0-a4/picotool/picotool.exe load build/speedo_mustang.uf2 -fx
```

VS Code shortcut: **Run Task → Run Project**

### 10.5 Debug with GDB

```powershell
# Terminal 1 — start OpenOCD GDB server
openocd -s ~/.pico-sdk/openocd/0.12.0+dev/scripts `
        -f raspberrypi-swd.cfg -f target/rp2040.cfg

# Terminal 2 — connect GDB
arm-none-eabi-gdb build/speedo_mustang.elf
(gdb) target remote :3333
(gdb) monitor reset halt
(gdb) continue
```

The VS Code Cortex-Debug launch configuration in the workspace handles this
automatically via **Run → Start Debugging**.

---

## 11. Testing

### 11.1 Host unit tests

The pure logic in `speedo_logic.cpp` is tested without hardware using
GoogleTest.  See [`test/README.md`](test/README.md) for full build
instructions.

```powershell
cmake -B build-test -DSPEEDO_UNIT_TEST=ON -DCMAKE_BUILD_TYPE=Debug
cmake --build build-test
ctest --test-dir build-test --output-on-failure
```

**Test coverage:**

| Suite | Cases | What it verifies |
|---|---|---|
| `Reset` | 1 | All fields zeroed; `bad_pulse_count` preserved |
| `ProcessPulse` | 5 | Short/long rejection, valid acceptance, ÷8 scaling, buffer wrap |
| `ComputeMetrics` | 4 | No-edge guard, timeout reset, zero jitter, non-zero jitter |
| `UpdateOutput` | 4 | <1 % hysteresis, >1 % restart, over-range disable, zero disable |
| `Scaling` | 1 | 55 mph round-trip frequency sanity check |

### 11.2 Hardware-in-the-loop (suggested)

A second Pico acting as a signal generator can drive GPIO 10 at known
frequencies while a logic analyzer or the first Pico's own diagnostic output
verifies:

- Output frequency = input frequency × 4
- Output stops within 1 s of signal dropout
- Output resumes correctly after signal returns
- `bad_pulse_count` increments on injected glitches

---

## 12. Configuration Reference

All tuneable constants are defined at the top of `speedo_logic.h`:

| Constant | Default | Description |
|---|---|---|
| `WINDOW_SIZE` | `3` | Number of samples in the rolling average |
| `MIN_PULSE_US` | `3 200` | Shortest accepted input pulse (µs) — glitch filter |
| `MAX_PULSE_US` | `750 000` | Longest accepted input pulse (µs) — signal-loss threshold |
| `MAX_OUTPUT_PULSE_US` | `93 750` | Output half-period above which the output is disabled (µs) |
| `PULSE_TIMEOUT_US` | `1 000 000` | Silence duration that triggers a state reset (µs) |

Pin assignments are in `speedo_mustang.cpp`:

| Constant | Default | Description |
|---|---|---|
| `PIN_INPUT` | `10` | GPIO pin for rising-edge sensor input |
| `PIN_OUTPUT` | `14` | GPIO pin for square-wave speedometer output |
| `AVG_INTERVAL_MS` | `300` | Main loop averaging and diagnostic interval (ms) |
