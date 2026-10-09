# NALA firmware (ATmega328PB motor driver)

Firmware for the NALA base. It receives body-velocity commands from the Pi over I2C, runs a speed controller on four mecanum wheels and gives the cumulative encoder counts back. Current version: **v2.0.0**. Toolchain: Atmel/Microchip Studio 7 (avr-gcc 5.4.0, ATmega_DFP 1.7.374), 16 MHz external crystal.

> ### HARDWARE POLARITY NOTE (read before touching the motor wiring)
>
> - The motor supply cables of **M2 and M3 are wired with opposite polarity to M1 and M4**. This is how the hardware is built and it must stay that way.
> - The firmware therefore does **no software inversion**: `MOTORn_DIR_INVERT = 0` for all four motors (`config.h`). With the current wiring a positive command turns all four wheels the same way.
> - **Do not invert M2/M3 in software.** That makes M2 and M3 turn against M1 and M4.
> - If the supply cables of a motor are re-soldered, that wheel reverses: set that motor's `MOTORn_DIR_INVERT` to 1 (and change nothing else), so wiring and flag stay consistent.
> - A wiring/flag mismatch gives the speed loop positive feedback on that wheel: it runs away to full power.

**This file is the source of truth for the Pi <-> MCU protocol.** 

Pi side: [`../src/nala_base/nala_base/mcu_protocol.py`](../src/nala_base/nala_base/mcu_protocol.py), [`../docs/mcu_protocol.md`](../docs/mcu_protocol.md).

## 1. Protocol (v2.0.0: I2C)

**I2C, the MCU is the slave at address `0x10`** (`I2C_ADDRESS`), the Pi is the master (bus 1, 100 kHz).
MCU TWI0: PC4 = SDA, PC5 = SCL. Pi: GPIO2 = SDA, GPIO3 = SCL. The Pi works at 3.3 V and the MCU at 5 V:
**a level shifter on the board** converts and has the pull-ups (the MCU internal pull-ups are off).
The Pi UART pins (GPIO14/15) belong to the IMU since v2.0.0.

### Pi -> MCU: command (write, 8 bytes)

| Byte | 0 | 1-2 | 3-4 | 5-6 | 7 |
|---|---|---|---|---|---|
| Content | `0x01` | vx | vy | w | CRC-8 of bytes 0-6 |

- vx, vy in **mm/s**, w in **mrad/s**, **int16 little endian**. Axes follow ROS (REP-103): vx forward, vy left, w counter-clockwise positive.
- Examples: stop `01 00 00 00 00 00 00 crc`; forward 0.3 m/s `01 2C 01 00 00 00 00 crc`; right 0.1 m/s `01 00 00 9C FF 00 00 crc`.
- A frame with a wrong id, length or CRC is ignored. A valid command is applied immediately (feed-forward) and then refined by the controller every 50 ms.
- :fire:**Command timeout:** while the robot is moving, if no valid command arrives for **200 ms** (`CMD_TIMEOUT_MS`, 0 disables), all targets are set to zero and the state flag bit1 is set (until the next valid command); the debug UART prints `cmd timeout`. A zero command never times out. **The Pi must keep sending the command** (20 Hz).

### MCU -> Pi: state (read, 19 bytes, no register address: just read)

| Byte | 0 | 1 | 2-5 | 6-9 | 10-13 | 14-17 | 18 |
|---|---|---|---|---|---|---|---|
| Content | seq | flags | M1 | M2 | M3 | M4 | CRC-8 of bytes 0-17 |

- seq: control period counter (uint8, wraps at 256). The difference of two reads = the number of 50 ms periods between them: the exact time base for the speeds.
- flags: bit0 = first frame after power-up (the counters started at 0; cleared after the Pi has read one frame), bit1 = stopped by the command timeout.
- Mn: **cumulative encoder counts** since power-up (int32 little endian, 1536 per wheel turn), positive = the wheel drove the robot forward. The Pi uses the difference of two reads, so a failed read loses no distance.
- The frame is updated every control period (50 ms).

**CRC-8:** polynomial 0x07, initial value 0, no final XOR (`"123456789"` -> `0xF4`). Same code in `main.c` (`crc8`) and on the Pi (`mcu_protocol.crc8`).

**Short interrupts (Pi 4 clock stretching):** the TWI holds SCL low until the interrupt has run, and the Pi 4 I2C handles that badly. So the TWI interrupt does no computation: received bytes are only stored (the CRC check is in the main loop), and the state frame is built with its CRC by the main loop in a **double buffer**; the interrupt only locks the ready buffer at SLA+R and then indexes into it. If errors still occur on the robot: slower I2C, or software I2C (`i2c-gpio`) on the Pi.

**UART:** only debug text for a USB-serial adapter on PD1 (38400 8N1): `a`, `test` after reset, `cmd timeout`. Received bytes are dropped.

## 2. Interface

| Motor | PWM pin | Direction pin | Encoder (A / B) | Encoder sign | Kinematic signs Vx / Vy / w |
|---|---|---|---|---|---|
| M1 | PD6 (OC0A) | PE0 | E1: PB3 / PB4 | -1 | + / - / - |
| M2 | PD5 (OC0B) | PE1 | E2: PD2 / PD3 | -1 | + / + / - |
| M3 | PB1 (OC1A) | PE2 | E3: PD4 / PD7 | +1 | + / - / + |
| M4 | PB2 (OC1B) | PE3 | E4: PC2 / PC3 | +1 | + / + / + |

- Wheel target: `M1 = (Vx - Vy - k*w)/R`, `M2 = (Vx + Vy - k*w)/R`, `M3 = (Vx - Vy + k*w)/R`, `M4 = (Vx + Vy + k*w)/R`, with `k = half track + half wheelbase = 0.290 m`, `R = 0.040 m`.
- Layout: M1 front-left, M2 rear-left, M3 rear-right, M4 front-right. Forward driving and the odometry scale were verified on the robot (2026-10-07: 1.17 m driven = 1.17 m odometry); strafe and turn worked as expected in the teleop test.
- Motor input polarity: **no motor is inverted in software** (`MOTORn_DIR_INVERT` = 0 for all four). The M2/M3 supply cables are wired opposite to M1/M4 in the hardware; see the **hardware polarity note** at the top.
- Encoders: only the A pins raise pin-change interrupts; B is sampled in the interrupt. Every A edge counts, direction from B.
- The motor <-> encoder association (`MOTORn_ENCODER`), encoder signs (`MOTORn_ENC_SIGN`) and all pins are in the "WIRING MAP" of `config.h`. Invalid mappings fail at compile time.

## 3. Parameters (`motor_driver_C/config.h`)

| Parameter | Value |
|---|---|
| Wheel radius / half track / half wheelbase | 0.040 / 0.160 / 0.130 m |
| Encoder counts per wheel revolution | 1536 (verified on the robot, 2026-10-07) |
| Control period (Timer3, `OCR3A = 12499`) | 50 ms (20 Hz); one encoder count = 0.082 rad/s |
| State frame update | every period (20 Hz), cumulative encoder counts |
| Command timeout | 200 ms |
| Pi link / command units | I2C slave 0x10 (TWI0) / int16 mm/s and mrad/s; UART 38400 baud for debug text |
| Speed controller | positional PI + feed-forward + anti-windup: Kp 1.0, Ki 8.0, integral limit 25 % |
| Feed-forward | table speed -> PWM from step tests (v1.1.1, `FF_*` in `config.h`): 0 -> 9 % (dead zone), 3.5 -> 15 %, 6.6 -> 21.5 %, 8.1 -> 29.5 %, 9.4 -> 37.5 %, 10.7 -> 45.5 %, 11.5 -> 54 %, 11.85 -> 63 %, 12.2 -> 72 %, 13.0 rad/s -> 100 % |
| Output limit | +-100 % (8-bit PWM, 1/255 resolution) |
| Measured top speed (100 % PWM) | 12.6-13.1 rad/s = 0.50-0.52 m/s rim speed, with and without load (2026-10-07) |

Controller per wheel: `e = target - measured`, `pwm = FF(target) + Kp*e + I`, `I += Ki*e*dt` (clamped, not integrated while saturated). Target 0 gives output 0 and clears `I`; `I` is also cleared when the target changes sign.

## 4. Firmware structure

Nothing blocks: interrupts only collect data and set flags, the main loop does the work. Diagram: [`docs/state_machine.svg`](docs/state_machine.svg) (labels in Chinese; it still shows the v1.x UART parser, outdated since v2.0.0).

| Interrupt | Trigger | Does |
|---|---|---|
| `PCINT0/1/2_vect` | edge on an encoder A pin (ports B / C / D) | counts the edge (+1/-1 from B) |
| `TIMER3_COMPA_vect` | every 50 ms, exact | snapshots the encoder counts, sets the control tick |
| `TWI0_vect` | I2C event (status 0x60, 0x80, 0xA0, 0xA8, 0xB8, 0xC0/0xC8, others = recover) | stores command bytes and sets `cmd_ready` at STOP; sends the bytes of the ready state frame |
| `USART0_RX_vect` | byte received (debug UART) | drops it |
| `USART0_UDRE_vect` | TX register empty | sends the next debug byte from the 128-byte ring buffer |

Main loop: check (CRC) and apply a new command, and on each tick do speed measurement (counts / real elapsed time), timeout check, controller, motor output and build the next state frame.

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
  6. Remove the ISP wires, at least RESET, so the chip runs on its own. Then the debug UART (38400 baud) shows `a`, `test`; on the Pi, `i2cdetect -y 1` must show `10`.
- **Compile check on the Pi** (no flashing, avr-gcc 7.3 from apt):
  `avr-gcc -mmcu=atmega328pb -DF_CPU=16000000UL -Os -Wall -Wextra -o /tmp/fw.elf motor_driver_C/*.c`.
- **Tests** (models and simulation, not hardware): `python tests/i2c_frame_model_test.py`, `timeout_model_test.py`, `motor_polarity_model_test.py`, `pid_sim.py`.

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

**v1.1.0 (2026-10-07)** - from measurements on the robot. **Needs the matching Pi code** (same commit).
- Feedback: the cumulative encoder counts of M1..M4 (`c n1 n2 n3 n4`) every period (20 Hz), instead of the wheel speeds of every 2nd period (10 Hz). v1.0.x never reported the counts of the other period, and a dropped line lost 100 ms of motion; now the host gets every count.
- UART 9600 -> 38400 baud (one line takes ~7 ms instead of ~48 ms, so the odometry time stamps are more exact).
- Feed-forward 4.0 -> 6.5 % per rad/s: 100 % PWM gives only ~13 rad/s, and with 4.0 the integral sat at its 25 % limit at 11 rad/s (the wheels needed ~1 s to reach the target).

**v1.1.1 (2026-10-07)** - feed-forward from a measured table.
- The step tests with v1.1.0 showed that the motor curve is not linear: no motion below ~8 %, 15 % -> 3.5 rad/s, 30 % -> 8 rad/s, 100 % -> 13 rad/s. 6.5 % per rad/s overshot by 30-90 % and settled only after ~2.5 s. The feed-forward is now a 10-point table (`FF_SPEED_RAD_S`, `FF_PWM_PCT`) with linear interpolation. Nothing else changed.
- Checked on the robot (wheels in the air, steps 0.05-0.45 m/s): 90 % of the target in 0.2-0.4 s, overshoot 1-27 % (most <= 15 %), settled within ~1 s.

**v2.0.0 (2026-10-09)** - the Pi link is I2C. **Needs the matching Pi code** (same commit).
- The Pi UART pins (GPIO14/15) now belong to the IMU. The MCU is an I2C slave (TWI0, PC4/PC5, address 0x10) behind a level shifter.
- Command: `0x01` + vx, vy, w as int16 (mm/s, mrad/s; was int8 x 0.02) + CRC-8. State: seq, flags, 4 x int32 cumulative counts + CRC-8, read by the Pi.
- The TWI interrupt does no computation (Pi 4 clock stretching): double-buffered state frame, CRC check in the main loop.
- Removed: UART command parser, echo frame, UART counts line. The UART only prints debug text.
- Not used any more: `printf` (the `-lprintf_flt` linker flag is harmless).

## 7. Known limitations

- The PI gains come from simulation of an *assumed* motor model. The feed-forward table is measured (wheels in the air); check it with step tests after flashing v1.1.1.
- v2.0.0 is compile-checked only (avr-gcc 7.3 on the Pi); I2C on the robot is not tested yet.
- The motors reach only ~13 rad/s (0.50-0.52 m/s). Faster targets saturate at 100 % PWM; the Pi limits every wheel to 0.45 m/s (`max_wheel_speed` in `config/base.yaml`).
- Stopping is by coasting, so the stopping distance depends on friction.

---
Last updated: 2026-10-09 (v2.0.0) · Stiffeel :octocat: · Claude
