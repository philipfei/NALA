# NALA_v0 Change Notes

Date: 2026-10-06 · Version: v0.1.0 · Based on: motor_driver_C (Floris van Mourik / Mathan / sojim, 2020–2022)

## Layout
| Path | Description |
|---|---|
| `motor_driver_C/` | The **modified** Studio project (open `motor_driver_C.atsln` to build) |
| `motor_driver_C.atsln` | The Studio **solution** file. It is only a container that registers one project, `motor_driver_C\motor_driver_C.cproj`. **Open this one in Studio.** |
| `original_ref/` | Read-only snapshot of the original sources before any edit (file-by-file hash-identical to the originals in the OneDrive repo), for comparison only. **Unrelated to the solution; do not open it in Studio** (its project file was renamed to `.cproj.orig` so Studio does not treat it as a project). |
| `build_cli/` | Output of the command-line verification builds (Debug / Release) and the Studio build log |
| `tests/` | Model tests for the serial frame parser (`python rx_parser_model_test.py`) |

The original code in the OneDrive repo has **not been modified** (all file modification times are still 2022-01-26, and the hashes match `original_ref`).

## Mapping to the 6 requirements
1. **Complete the headers**: added `config.h`, `ADC.h`, `encoder.h`, `motor_functions.h`, `pwm.h`, `timer.h`; removed 3 never-implemented prototypes from `Usart.h`; completed the includes (`stdio` / `stdint` / `Usart.h`, etc.) in `main.c` and `ADC.c`. Functions that previously compiled only through implicit declarations now have prototypes. The three duplicate `F_CPU` definitions were unified in `config.h`.
2. **File header comments**: every file has author, version (v0.1.0), date (2026-10-06) and a summary; `main.c` contains the full architecture, speed-measurement principle and protocol description. Original authors are credited. The author field is set to "HY (NALA v0 revision)"; edit the file headers to change it.
3. **No blocking + accurate measurement period** (details below).
4. **Parameters**: wheel radius 0.040 m, half track 0.160 m, half wheelbase 0.130 m, in `config.h` as `WHEEL_RADIUS_M / HALF_TRACK_M / HALF_WHEELBASE_M`. The yaw lever arm `W+H` changes from 0.315 to **0.290 m**.
5. **Encoder ↔ motor mapping**: the "WIRING MAP" at the end of `config.h`:
   - `MOTORn_ENCODER` (n = 1..4, value 1..4): which encoder provides the speed feedback of motor n;
   - `MOTORn_ENC_SIGN` (±1): sign of that encoder's count (left/right motors are mounted as mirror images; originally M1, M2 inverted and M3, M4 not inverted);
   - `ENCn_*`: A/B pins and interrupt configuration of each encoder; `MOTORn_DIR_BIT`: direction pins;
   - the documentation table lists the PWM pin of each motor (fixed to the timer outputs) and its direction pin;
   - compile-time checks: an out-of-range number, or one encoder assigned to two motors, triggers `#error`.
6. **Serial feedback and protocol**: the velocity frame format is unchanged (`0x80 0x86 Vx Vy w`, int8, 9600 8N1), but the **scale changed from /100 to /50 (0.02 per count, later change)**; the feedback format is unchanged (`%f \t %f \t %f \t %f\n`, measured M1..M4 speeds in rad/s).
   **Added (a separate frame type that does not affect the velocity frame)**: command-echo switch `0x80 0x87 E`, E=1 on, E=0 off, other values ignored; the MCU replies `echo on\n` / `echo off\n`. The power-on default is set by `ECHO_COMMAND` in `config.h` (default 0).
   Compatibility precondition: the host never sends the byte pair `0x80 0x87` outside a velocity frame (the existing Pi code does not).

## Removing blocking and the measurement period (detail of item 3)
| | Original code | NALA_v0 |
|---|---|---|
| Encoder counting | Polled in the main loop in two ~10 ms windows (M3/M4 in one, M1/M2 in the other, **not simultaneous**, pulses outside the windows are missed) | Pin-change interrupts count **continuously**, all four channels at once, nothing missed |
| Period source | `_delay_ms` + blocking print, period undefined (estimated 90–100 ms) | Timer3 CTC compare interrupt, **exactly 100 ms** (OCR3A = 24999, verified) |
| Time used in the speed formula | Hard-coded 0.010 s; PID dt hard-coded 0.045 s | Actual elapsed time = number of elapsed ticks × period (still correct if the main loop is occasionally late) |
| Serial transmit | `usart_send` busy-waits, one line takes ~45 ms | Ring buffer + transmit interrupt; if the feedback does not fit it is dropped, **never blocks** |
| Serial receive interrupt | Does sprintf, transmit and computation inside (~25 ms) | Only receives bytes and sets a flag; computation happens in the main loop |
| Speed conversion | `×6.28` | `×2π` (6.28318…) |

The counting rule is unchanged (count every edge of channel A, B≠A counts +1), so the calibration `ENC_COUNTS_PER_REV = 1536` still applies.

## Behavioural differences from the original (please note)
1. **The effective PID gains change.** The control law is unchanged (error = measured − target, `pwm -= u`, u truncated to int, gains Kp=0.74 / Ki=3.7 / Kd=0.0644), but the original dt was wrongly written as 0.045 while the real period was about 0.09–0.1, so the Ki term was effectively about half of its design value and the Kd term about double. dt is now the real value, so the **Ki term becomes about 2× larger and the Kd term about 2× smaller**. The default control period of 100 ms was chosen so that the per-second action of the proportional term stays close to the original. **After installing on the robot, check for oscillation and retune if necessary.**
2. **Commands take effect immediately**: the feed-forward PWM is output as soon as a command arrives, instead of waiting for the next loop (originally it waited until the current loop finished).
3. **An extra `a\n` at start-up**: originally `adc_init` was called before the UART was enabled, so this character was never actually sent; the UART is now enabled first, so it is sent.
4. **Command echo is off by default** (originally every command was echoed as a `Vx Vy w` line). Turn it on with the serial frame `80 87 01` and off with `80 87 00`, or set `ECHO_COMMAND` in `config.h` to 1 as the power-on default.
5. **Fixed the memset ordering bug**: the original `main.c` cleared the buffer first and then used it to compute the speed, so the command was always 0; the new code takes the values first and then processes them. (The backed-up firmware itself did not have this problem.)
6. `readVelCmd` no longer exists (hand-over is done directly in the interrupt); in the backed-up firmware it was a 2-byte int, so there is no need to chase byte-identical output.
7. Removed the commented-out old code in the main loop (the old loop and the PID test loop); it is still in `original_ref/main.c`.
8. **Velocity frame scale changed from /100 to /50** (`CMD_SCALE` in `config.h`): 0.02 per count, range ±2.54. Vx, Vy and w share this factor; the Pi must send `value × 50` (reported as already updated on the Pi).
   **Axes follow the ROS standard**: Vx forward, Vy left, w counter-clockwise positive. The kinematic equations are unchanged; they are consistent with the position assumption "M1 = front-left, M2 = rear-left, M3 = rear-right, M4 = front-right".
9. **Automatic stop on command timeout** (`config.h`: `CMD_TIMEOUT_MS`, default 500 ms, 0 = disabled): while the robot is moving, if no velocity frame has been received before the timeout, all targets are set to zero and `cmd timeout` is sent once; a new velocity frame resumes operation immediately. The check runs every 100 ms, so the actual stop happens 500–600 ms after the last frame. A zero velocity frame never times out; the echo-switch frame does not refresh the timeout.
   **This requires the Pi to keep re-sending velocity frames.** The local copy of `motor_uart_comms.py` (the 2022 original) only sends when the velocity changes (the `isclose` de-duplication at lines 52–53), so without re-sending the robot would stop after 0.5 s of constant-speed driving. Please confirm the version on the Pi handles this.

## Project file changes (`motor_driver_C.cproj`)
- Registered the new `encoder.c` and the headers.
- DFP path 1.6.364 → **1.7.374**: only 1.7.374 is installed in Studio on this machine; 1.6.364 does not exist.
- Added `-lprintf_flt` to the Release configuration: Release previously did not link the floating-point printf, so the `%f` feedback printed garbage in Release and only worked in Debug.

## Verification status
- Studio command-line build (`AtmelStudio.exe motor_driver_C.atsln /build Debug`): **succeeded**, Flash 9356 B (28.6%), RAM 332 B (16.2%).
- Command-line avr-gcc 5.4.0, Debug (-Og) and Release (-Os), both with `-Wall -Wextra`: **0 warnings**. Release: text 8380 B.
- Model test of the serial frame parser, 13/13 passed: velocity frame, negative values, leading garbage bytes, back-to-back frames, `80 87` inside a payload does not trigger, echo on/off/invalid value, and 20,000 random byte streams without 0x87 give results **identical to the original parser**. Note that this is a model test of the logic, not a test of the AVR firmware on real hardware.
- Disassembly check: all 6 interrupts (PCINT0/1/2, USART RX/TX, Timer3 compare) are in the vector table; no delay functions in the firmware; Timer3 compare value 0x61A7 = 24999.
- **Not yet verified on real hardware** (this firmware has not been flashed). Before putting it on the robot, check in this order:
  1. After power-up the serial port should first show `a`, `test`, then a line of four numbers every 100 ms;
  2. Turn each wheel by hand and check the sign and magnitude of the corresponding column (M1, M2 negative for clockwise, M3, M4 positive, consistent with the earlier measurements);
  3. With the wheels lifted, send `80 86 05 00 00` at low speed (+Vx = 0.10 m/s): all four wheels should turn in the same sense and the measured speed should be close to 2.5 rad/s; then try `80 86 00 05 00` (+Vy = strafe left) and `80 86 00 00 19` (w = 0.50 rad/s, counter-clockwise) and see which wheels turn and in which direction;
  4. Timeout: send one `80 86 05 00 00` and then nothing; after about 0.5–0.6 s the robot should stop and the serial port should receive `cmd timeout`;
  5. At high speed, check whether the encoder interrupts lose counts (about 2400 edges/s/wheel, estimated CPU load about 5–6%, not measured).

## Known leftovers (not changed)
- The position assumption (M1 = front-left, M2 = rear-left, M3 = rear-right, M4 = front-right; x forward, y left, w counter-clockwise) is still an assumption and must be confirmed with the wheels lifted.
- `ENC_COUNTS_PER_REV = 1536` is inherited from the original and has not been re-checked on this hardware.
- The timeout stop only acts while the robot is "moving", and the stop is done by letting the PID bring the target to zero, so if the wheels are still spinning there is some active reverse braking (the same effect as sending a stop frame).
- The old `start_timer3()` in `timer.c` selects the external clock (CS32:0 = 111), which does not match its comment; it is never called, and Timer3 is now dedicated to the control tick, so do not use it again.
