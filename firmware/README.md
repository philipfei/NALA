# NALA firmware (ATmega328PB motor driver)

Firmware for the NALA base. It receives body-velocity commands from the Pi over UART, runs a speed controller on four mecanum wheels and reports the measured wheel speeds. Current version: **v1.0.1**. Toolchain: Atmel/Microchip Studio 7 (avr-gcc 5.4.0, ATmega_DFP 1.7.374), 16 MHz external crystal.

> ### HARDWARE POLARITY NOTE (read before touching the motor wiring)
>
> - The motor supply cables of **M2 and M3 are wired with opposite polarity to M1 and M4**. This is how the hardware is built and it must stay that way.
> - The firmware therefore does **no software inversion**: `MOTORn_DIR_INVERT = 0` for all four motors (`config.h`). With the current wiring a positive command turns all four wheels the same way.
> - **Do not invert M2/M3 in software.** That makes M2 and M3 turn against M1 and M4.
> - If the supply cables of a motor are re-soldered, that wheel reverses: set that motor's `MOTORn_DIR_INVERT` to 1 (and change nothing else), so wiring and flag stay consistent.
> - A wiring/flag mismatch gives the speed loop positive feedback on that wheel: it runs away to full power.

**This file is the source of truth for the Pi <-> MCU protocol.** 

Pi side: [`../src/nala_base/nala_base/mcu_protocol.py`](../src/nala_base/nala_base/mcu_protocol.py), [`../docs/mcu_protocol.md`](../docs/mcu_protocol.md).

## 1. Protocol

UART0, **9600 8N1**, no flow control (MCU RXD0 = PD0, TXD0 = PD1, TTL levels).

### Host -> MCU

| Frame | Bytes | Meaning |
|---|---|---|
| Velocity | `80 86 Vx Vy w` | Set the body velocity |
| Echo switch | `80 87 E` | `E=01` command echo on, `E=00` off, other values ignored |

- `Vx Vy w` are signed **int8**: `value = count * 0.02`, so the host sends `round(value * 50)`. Range -2.56 .. +2.54. Units: m/s, m/s, rad/s.
- Axes follow ROS (REP-103): Vx forward, Vy left, w counter-clockwise positive.
- :fire:**Examples:** 
  - stop `80 86 00 00 00` 
  - forward 0.10 m/s `80 86 05 00 00` 
  - reverse 0.10 m/s `80 86 FB 00 00` 
  - left 0.10 m/s `80 86 00 05 00` 
  - CCW 0.50 rad/s `80 86 00 00 19`

- The parser finds frames by their two header bytes. Payload bytes are not searched for headers; a lost byte shifts the frame until the next header. A new frame replaces an unprocessed one.
- A command is applied immediately (feed-forward) and then refined by the controller every 50 ms.
- :fire:**Command timeout:** while the robot is moving, if no velocity frame arrives for **200 ms** (`CMD_TIMEOUT_MS`, 0 disables), all targets are set to zero and `cmd timeout` is sent once. The stop happens 200-250 ms after the last frame. A zero command never times out; the echo frame does not refresh the timeout. **The host must keep resending the command** (at most 150 ms apart, 20 Hz recommended).

### MCU -> host (text lines ending in `\n`)

| Line | When |
|---|---|
| `%f \t %f \t %f \t %f` - measured speed of M1..M4 in rad/s, positive = the wheel pushes the robot forward | :exclamation:every 100 ms (10 Hz) |
| `a`, `test` | once after reset |
| `Vx Vy w` (3 floats) | after each velocity frame, only when echo is on (default off) |
| `echo on` / `echo off` | after an echo frame |
| `cmd timeout` | once, when the timeout stops the robot |

> **Link budget** (9600 baud = 960 byte/s). A velocity frame is 5 bytes, so 20 Hz uses 10 % of the Pi -> MCU wire. A wheel-speed line is up to 50 bytes (about 52 ms): 10 Hz uses 52 % of the MCU -> Pi wire, 20 Hz would need 104 % and does not fit. A line that does not fit into the 128-byte TX buffer is dropped, never waited for. With echo on and 20 Hz commands the total exceeds the wire capacity, so use echo for debugging only. If the Pi does not read the feedback, its RX buffer fills up (harmless); flush it (`reset_input_buffer()`) before reading.

## 2. Interface

| Motor | PWM pin | Direction pin | Encoder (A / B) | Encoder sign | Kinematic signs Vx / Vy / w |
|---|---|---|---|---|---|
| M1 | PD6 (OC0A) | PE0 | E1: PB3 / PB4 | -1 | + / - / - |
| M2 | PD5 (OC0B) | PE1 | E2: PD2 / PD3 | -1 | + / + / - |
| M3 | PB1 (OC1A) | PE2 | E3: PD4 / PD7 | +1 | + / - / + |
| M4 | PB2 (OC1B) | PE3 | E4: PC2 / PC3 | +1 | + / + / + |

- Wheel target: `M1 = (Vx - Vy - k*w)/R`, `M2 = (Vx + Vy - k*w)/R`, `M3 = (Vx - Vy + k*w)/R`, `M4 = (Vx + Vy + k*w)/R`, with `k = half track + half wheelbase = 0.290 m`, `R = 0.040 m`.
- Assumed layout *(unverified)*: M1 front-left, M2 rear-left, M3 rear-right, M4 front-right.
- Motor input polarity: **no motor is inverted in software** (`MOTORn_DIR_INVERT` = 0 for all four). The M2/M3 supply cables are wired opposite to M1/M4 in the hardware; see the **hardware polarity note** at the top.
- Encoders: only the A pins raise pin-change interrupts; B is sampled in the interrupt. Every A edge counts, direction from B.
- The motor <-> encoder association (`MOTORn_ENCODER`), encoder signs (`MOTORn_ENC_SIGN`) and all pins are in the "WIRING MAP" of `config.h`. Invalid mappings fail at compile time.

## 3. Parameters (`motor_driver_C/config.h`)

| Parameter | Value |
|---|---|
| Wheel radius / half track / half wheelbase | 0.040 / 0.160 / 0.130 m |
| Encoder counts per wheel revolution | 1536 (inherited; marked as confirmed in `../docs/mcu_protocol.md`) |
| Control period (Timer3, `OCR3A = 12499`) | 50 ms (20 Hz); one encoder count = 0.082 rad/s |
| Feedback every | 2 periods (10 Hz) |
| Command timeout | 200 ms |
| UART / command scale | 9600 baud / 50 counts per unit (0.02) |
| Speed controller | positional PI + feed-forward + anti-windup: Kp 1.0, Ki 8.0, integral limit 25 % |
| Feed-forward / output limit | 4.0 % duty per rad/s / +-100 % (8-bit PWM, 1/255 resolution) |
| Command echo at power-up | off (`ECHO_COMMAND`) |

Controller per wheel: `e = target - measured`, `pwm = 4.0*target + Kp*e + I`, `I += Ki*e*dt` (clamped, not integrated while saturated). Target 0 gives output 0 and clears `I`; `I` is also cleared when the target changes sign.

## 4. Firmware structure

Nothing blocks: interrupts only collect data and set flags, the main loop does the work. Diagram: [`docs/state_machine.svg`](docs/state_machine.svg) (labels in Chinese).

| Interrupt | Trigger | Does |
|---|---|---|
| `PCINT0/1/2_vect` | edge on an encoder A pin (ports B / C / D) | counts the edge (+1/-1 from B) |
| `TIMER3_COMPA_vect` | every 50 ms, exact | snapshots the encoder counts, sets the control tick |
| `USART0_RX_vect` | byte received | frame parser; sets `cmd_ready` / `echo_ack` |
| `USART0_UDRE_vect` | TX register empty | sends the next byte from the 128-byte ring buffer |

Main loop: acknowledge an echo frame, apply a new command, and on each tick do speed measurement (counts / real elapsed time), timeout check, controller, motor output and the feedback line.

## 5. Build, flash, test

- **Build:** open `motor_driver_C.atsln` in Studio and press F7. Output: `motor_driver_C/Release/motor_driver_C.hex` (or `Debug/`). The Studio output folders are git-ignored.
- **Flash** (ArduinoISP; Studio itself cannot use it): `avrdude -c stk500v1 -P COMx -b 19200 -p m328pb -U flash:w:motor_driver_C.hex:i` (needs a recent avrdude that knows `m328pb`; 8.0 from the Arduino IDE works). Fuses on the board are `lfuse 0xFF, hfuse 0xD1, efuse 0xF7`; do not write them. Wheels off the ground for the first run.
- **ISP wiring** (an Arduino Uno running the *ArduinoISP* sketch, `File > Examples > 11.ArduinoISP`, as programmer):

  | Uno | Target ATmega328PB | Signal |
  |---|---|---|
  | D10 | RESET (PC6) | reset |
  | D11 | PB3 | MOSI |
  | D12 | PB4 | MISO |
  | D13 | PB5 | SCK |
  | GND | GND | common ground (always) |
  | 5V | VCC | only if the target has no supply of its own |

  Order:
  1. Upload ArduinoISP to the Uno (board "Arduino Uno", its COM port), then close the serial monitor.
  2. Power everything off and unplug the encoder and motor-driver connectors (PB3/PB4 are also encoder E1). Connect **GND first**, then SCK, MISO, MOSI, RESET.
  3. Power the target: either from its own supply (then do **not** connect the Uno 5V) or from the Uno 5V.
  4. Test the link without writing anything: `avrdude -c stk500v1 -P COMx -b 19200 -p m328pb -v` must print the signature `1E 95 16`.
  5. Flash (command above) and wait for `verified`.
  6. Remove the ISP wires, at least RESET, so the chip runs on its own. Then watch the serial port at 9600 baud: `a`, `test`, then a wheel-speed line every 100 ms.
- **Tests** (models and simulation, not hardware): `python tests/rx_parser_model_test.py`, `timeout_model_test.py`, `motor_polarity_model_test.py`, `tx_budget.py`, `pid_sim.py`.

## 6. Changes per version

**Baseline** - `motor_driver_C` by the previous team (2021-22): blocking main loop, encoders polled in two ~10 ms windows, blocking UART, PID with an assumed time step.

**v0 (2026-10-06)**
- Restructured: `config.h`, complete headers, file headers, explicit motor <-> encoder map.
- Encoders counted by pin-change interrupts; control tick from Timer3 (100 ms), so the measured speed uses the real time step.
- Non-blocking UART: TX ring buffer with interrupt, RX interrupt only parses.
- Geometry set to 0.040 / 0.160 / 0.130 m.
- Protocol: command scale 0.01 -> 0.02, ROS axes, echo-switch frame, command timeout 500 ms.
- Fixed a bug where the buffer was cleared before it was used (every command was 0); Release now links floating-point printf.

**v1.0.0 (2026-10-07)**
- M2 and M3 motor input polarity inverted in software.
- 20 Hz control (50 ms); feedback stays at 10 Hz; timeout 500 -> 200 ms.
- Speed controller: the v0 incremental PID (a double integrator, `int` truncation, integral kept after a stop) made the wheels oscillate for a long time before stopping. Replaced by positional PI + feed-forward + anti-windup. On 24 assumed motor models the old law (at its 100 ms period) reversed the wheels after a stop in 22 cases, the new one in none. The gains come from that simulation, not from tuning on the robot. A stop now switches the output off and the wheels coast.
- PWM output resolution 1 % -> 1/255.
- Added `tests/`; merged all documentation into this README.

**v1.0.1 (2026-10-07)**
- Software polarity of M2 and M3 reverted (all `MOTORn_DIR_INVERT` = 0): the M2/M3 supply cables are wired with opposite polarity to M1/M4 in the hardware, so no software inversion is needed (see the hardware polarity note). Nothing else changed.

## 7. Known limitations

- The controller gains come from simulation of an *assumed* motor model, not from tuning on the robot.
- Wheel layout, axis directions and the 1536 counts/rev still need to be confirmed on the robot.
- Speeds above what the motors can do (for example 1.0 m/s needs 25 rad/s = 100 % feed-forward) saturate; the real speed is then lower than commanded.
- Stopping is by coasting, so the stopping distance depends on friction.

---
Last updated: 2026-10-07 14:57 · Stiffeel :octocat:
