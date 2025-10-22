# Copilot Instructions for speedo_mustang (Raspberry Pi Pico firmware)

Purpose: help an AI coding agent become productive quickly in this repo — build, flash, debug, and modify firmware safely.

## Quick snapshot (what this repo is)
- Firmware for the Raspberry Pi Pico written in C++.
- Core logic lives in `speedo_mustang.cpp` (pulse input measurement, filtering, and output pulse generation).
- Uses the Raspberry Pi Pico SDK; build outputs land in `build/` including `build/speedo_mustang.uf2`.

## Where to look first (key files)
- `speedo_mustang.cpp` — main application: look for `gpio_callback`, `repeating_timer_callback`, `pulse_in_window`, `pulse_out_window`, and constants like `PIN_INPUT`.
- `CMakeLists.txt` and `pico_sdk_import.cmake` — SDK integration and board settings (do not remove auto-generated VS Code extension lines).
- `raspberrypi-swd.cfg` — OpenOCD SWD board config used by the "Flash" task.
- `generated/pico_base/` and `pico-sdk/` — SDK headers and generated config (read-only for most changes).

## Build / flash / run (concrete commands)
Preferred: use the workspace VS Code tasks (they set the right SDK/tool paths):

Run the build task (Compile Project):
```powershell
# runs ninja in the build folder (task provided in workspace)
# VS Code: Run Task → Compile Project
${env:USERPROFILE}/.pico-sdk/ninja/v1.12.1/ninja.exe -C ${workspaceFolder}/build
```

Flash via USB SWD (OpenOCD task in workspace):
```powershell
# VS Code: Run Task → Flash
openocd -s ${userHome}/.pico-sdk/openocd/0.12.0+dev/scripts -f raspberrypi-swd.cfg -f target/<target>.cfg -c "adapter speed 5000; program \"${command:raspberry-pi-pico.launchTargetPath}\" verify reset exit"
```

Fast load via picotool (Run Project task):
```powershell
# VS Code: Run Task → Run Project
${env:USERPROFILE}/.pico-sdk/picotool/2.2.0-a4/picotool/picotool.exe load ${command:raspberry-pi-pico.launchTargetPath} -fx
```

Notes:
- The workspace already defines tasks named "Compile Project", "Flash", and "Run Project" — use them unless you must customize paths.
- Build output: `build/speedo_mustang.uf2` and `build/speedo_mustang.dis` (disassembly for diagnostics).

## Code patterns and hotspots (do this, not generic advice)
- Pulse input: interrupts capture pulse widths; look at `gpio_callback` for validation and buffer resets on out-of-range values.
- Smoothing: `pulse_in_window` and `pulse_out_window` are circular buffers used to average or filter measurements before generating the output pulse.
- Output generation: implemented with a repeating timer callback (search for `repeating_timer_callback` or `add_repeating_timer_ms`).
- Diagnostics: uses `printf` for USB serial output — the code expects USB-enabled stdio (UART is disabled in the CMake configuration).
- Error handling: when an input pulse is invalid the code clears windows and cancels timers rather than trying to continue with bad data.

Practical examples to search/edit:
- Change input pin: edit `PIN_INPUT` in `speedo_mustang.cpp`.
- Change scaling/filter: adjust the division/weights applied to `pulse_out_window` or the window sizes near the buffer declarations.

## Editing rules & safety
- Do not edit SDK files under `pico-sdk/` or `generated/` unless explicitly updating the SDK — breakage here is hard to debug.
- Keep CMake auto-generated sections intact; the VS Code Pico extension expects some lines to remain.
- Hardware changes should be tested incrementally; use printf logs to validate behavior before flashing.

## Debugging tips
- SWD via OpenOCD using `raspberrypi-swd.cfg` is the recommended debug path.
- Use `build/speedo_mustang.dis` to inspect generated assembly if timing/optimizations are suspicious.
- When investigating timing issues, log the raw pulse widths printed by the ISR and the averaged values from the windows.

## External tools and dependencies
- Raspberry Pi Pico SDK (imported through `pico_sdk_import.cmake`).
- Ninja, OpenOCD, and picotool are used by the workspace tasks; task definitions reference paths under `${env:USERPROFILE}/.pico-sdk`.

## What an AI agent should do first
1. Open `speedo_mustang.cpp` and locate the symbols named above (`gpio_callback`, `pulse_in_window`, etc.).
2. Build with the VS Code "Compile Project" task and confirm `build/speedo_mustang.uf2` is produced.
3. Use `printf` traces in code to iterate: add small logs, rebuild, and load via the "Run Project" task.

## Limitations / missing pieces
- There are no unit tests or simulator harness in the repo — hardware testing is required for most validation.
- Assumes a Pico toolchain installed under the user's profile; if not present, update workspace tasks to point to local tool installs.

---

If anything here is unclear or you want more detail (e.g., a quick grep of `speedo_mustang.cpp` with annotated hotspots), tell me which area to expand.

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
