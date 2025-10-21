# Copilot Instructions for speedo_mustang (Raspberry Pi Pico Project)

## Project Overview
- This project is a firmware for the Raspberry Pi Pico, written in C++ (see `speedo_mustang.cpp`).
- It processes input pulses (e.g., from a speed sensor) and generates output pulses for a speedometer, with configurable scaling and filtering.
- Uses the Raspberry Pi Pico SDK and hardware libraries (see `CMakeLists.txt`).

## Architecture & Key Files
- `speedo_mustang.cpp`: Main logic, including pulse input handling, output generation, and timing.
- `CMakeLists.txt`: Build configuration, SDK/library setup, board selection, and output settings.
- `raspberrypi-swd.cfg`: SWD configuration for flashing/debugging.
- `build/`: CMake/Ninja build output, including binaries (`speedo_mustang.uf2`), disassembly, and dependency builds.

## Build & Flash Workflow
- **Build:**
  - Use the VS Code task "Compile Project" (runs Ninja in `build/`).
  - Output: `build/speedo_mustang.uf2` (firmware image).
- **Flash:**
  - Use the "Flash" task (OpenOCD with SWD config) to program the Pico.
  - Alternatively, use "Run Project" to load via `picotool`.
- **Debug:**
  - Debugging is typically via SWD (`raspberrypi-swd.cfg`).
  - Output logs via USB (UART disabled, USB enabled in CMake).

## Conventions & Patterns
- **Pulse Handling:**
  - Input pulses are measured using hardware interrupts and timers.
  - Output pulses are generated with a repeating timer callback.
  - Circular buffers (`pulse_in_window`, `pulse_out_window`) smooth pulse width measurements.
- **Error Handling:**
  - Out-of-range pulse widths reset buffers and cancel timers (see `gpio_callback`).
  - Diagnostic messages printed via `printf` (USB output).
- **SDK Integration:**
  - All Pico SDK setup is in `CMakeLists.txt` (do not edit auto-generated lines for VS Code extension compatibility).
  - Libraries: `hardware_timer`, `hardware_watchdog`, `hardware_clocks`.

## External Dependencies
- Raspberry Pi Pico SDK (imported via `pico_sdk_import.cmake`).
- Ninja (build system), OpenOCD (flashing), picotool (loading firmware).

## Examples
- To add a new sensor, connect to a GPIO and update `PIN_INPUT` in `speedo_mustang.cpp`.
- To change output scaling, adjust the division in `pulse_out_window` assignment.
- For custom board support, set `PICO_BOARD` in `CMakeLists.txt`.

## Key Directories
- `build/`: All build artifacts and intermediate files.
- `generated/`, `pico-sdk/`, `picotool/`: SDK and tool dependencies (do not modify unless updating SDK/tool).

---

For questions or unclear sections, review `speedo_mustang.cpp` and `CMakeLists.txt` for implementation details. If conventions change, update this file to reflect actual project practices.
