# NALA firmware (ATmega328PB motor driver)

Firmware for the NALA base. It receives body-velocity commands from the Pi over UART, runs a speed controller on four mecanum wheels and sends the cumulative encoder counts back. Current version: **v2.3.3**. Toolchain: Atmel/Microchip Studio 7 (avr-gcc 5.4.0, ATmega_DFP 1.7.374), 16 MHz external crystal.

> ### HARDWARE POLARITY NOTE (read before touching the motor wiring)
>
> - The motor supply cables of **M2 and M3 are wired with opposite polarity to M1 and M4**. This is how the hardware is built and it must stay that way.
> - The firmware therefore does **no software inversion**: `MOTORn_DIR_INVERT = 0` for all four motors (`config.h`). With the current wiring a positive command turns all four wheels the same way.
> - **Do not invert M2/M3 in software.** That makes M2 and M3 turn against M1 and M4.
> - If the supply cables of a motor are re-soldered, that wheel reverses: set that motor's `MOTORn_DIR_INVERT` to 1 (and change nothing else), so wiring and flag stay consistent.
> - A wiring/flag mismatch gives the speed loop positive feedback on that wheel: it runs away to full power.

**This file is the source of truth for the Pi <-> MCU protocol.** 

Pi side: [`../src/nala_base/nala_base/mcu_protocol.py`](../src/nala_base/nala_base/mcu_protocol.py), [`../docs/mcu_protocol.md`](../docs/mcu_protocol.md).

## 1. Protocol (v2.3.0: UART, binary frames, setting mode)

UART0, **38400 8N1**, no flow control. MCU **PD0 = RXD0, PD1 = TXD0** (5 V). The Pi (GPIO14 TX / GPIO15 RX, 3.3 V,
`/dev/ttyAMA0`) is connected through the level shifter on the board.

> **Level shifter:** the schematic says TXS0104E, but the part on the board is a **TXB** type (auto direction).
> A TXB is fine for UART (push-pull signals) but **cannot do I2C** (it oscillates against pull-ups).
> That is why the I2C link of v2.0.0 never worked. Do not put pull-up resistors on lines that go through the TXB.

> **UART wiring: check every pin, do not trust the order.**
> - The same signals (GND, 3.3 V, TX, RX) are on three connectors: the motor driver connector and the UART header row of the
>   breakout board, and P1 on the motor driver board. The **pin order is different on each of the three.**
> - The TX/RX labels on the breakout board are from the **Pi's point of view**. So breakout **TX = motor driver RX**, and
>   breakout **RX = motor driver TX**. The labels on P1 are from the MCU's point of view (P1 TX = MCU PD1).
> - The 3.3 V pin of P1 supplies the 3.3 V side of the level shifter (U2, VCCA and OE). It must be connected.

Only binary frames are sent, in both directions: no text. Every frame is `0xA5` (sync), an id, the data and a
**CRC-8** of id + data (polynomial 0x07, initial value 0, no final XOR; `"123456789"` -> `0xF4`).
Same code in `main.c` (`crc8`) and on the Pi (`mcu_protocol.crc8`).

### Pi -> MCU: command frame (9 bytes)

| Byte | 0 | 1 | 2-3 | 4-5 | 6-7 | 8 |
|---|---|---|---|---|---|---|
| Content | `0xA5` | `0x01` | vx | vy | w | CRC-8 of bytes 1-7 |

- vx, vy in **mm/s**, w in **mrad/s**, **int16 little endian**. Axes follow ROS (REP-103): vx forward, vy left, w counter-clockwise positive.
- Examples: stop `A5 01 00 00 00 00 00 00 DF`; forward 0.3 m/s `A5 01 2C 01 00 00 00 00 crc`; right 0.1 m/s `A5 01 00 00 9C FF 00 00 crc`.
- Receive state machine (`USART0_RX_vect`): **IDLE** (only here `0xA5` starts a frame) -> **HEADER_ACK** (`0x01` -> PAYLOAD, `0xA5` -> stay, other -> IDLE) -> **PAYLOAD** (6 data bytes + crc; a `0xA5` in here is data and never restarts the frame) -> complete (`cmd_ready`). The CRC is checked in the main loop; a frame with a wrong CRC is ignored.
- A valid zero command stops the wheels at once (they coast). A non-zero command is applied by the next control step (<= 50 ms), so every output comes from the full controller (since v2.2.0).
- While the feed-forward table is invalid (state flag bit3), non-zero commands are ignored (treated as a stop).
- :fire:**Command timeout:** while the robot is moving, if no valid command arrives for **200 ms** (`CMD_TIMEOUT_MS`, 0 disables), all targets are set to zero and state flag bit1 is set (until the next valid command). A zero command never times out. **The Pi must keep sending the command** (20 Hz).

### MCU -> Pi: state frame (21 bytes, every control period = 20 Hz)

| Byte | 0 | 1 | 2 | 3 | 4-7 | 8-11 | 12-15 | 16-19 | 20 |
|---|---|---|---|---|---|---|---|---|---|
| Content | `0xA5` | `0x02` | seq | flags | M1 | M2 | M3 | M4 | CRC-8 of bytes 1-19 |

- seq: control period counter (uint8, wraps at 256). The difference of two frames = the number of 50 ms periods between them: the exact time base for the speeds.
- flags: bit0 = no valid command since power-up (the counters started at 0), bit1 = stopped by the command timeout, bit2 = the parameters in RAM are the EEPROM block (loaded or saved, unchanged since), bit3 = the feed-forward table is invalid (non-zero drive commands are ignored).
- Mn: **cumulative encoder counts** since power-up (int32 little endian, 1536 per wheel turn), positive = the wheel drove the robot forward. The Pi uses the difference of two frames, so a lost frame loses no distance.

> **Link budget** (38400 baud = 3840 byte/s): commands 9 bytes x 20 Hz = 5 % of the Pi -> MCU wire, state frames 21 bytes x 20 Hz = 11 % of the MCU -> Pi wire. A frame that does not fit into the TX ring buffer (128 bytes, checked at compile time to be >= 32) is dropped, never waited for.

### Setting mode (v2.2.0, per-motor FF rows since v2.3.0): parameter request and reply

The controller parameters live in RAM (`param[]` in `main.c`). They can be read and changed over the UART and
saved in the EEPROM. Tool on the Pi: `scripts/tune_motors.py` (see the main README).

Pi -> MCU, **parameter request, 9 bytes** (same length as a command: 6 data bytes + CRC after sync and id, no padding):

| Byte | 0 | 1 | 2 | 3 | 4-7 | 8 |
|---|---|---|---|---|---|---|
| Content | `0xA5` | `0x03` | op | idx | value (float32 LE) | CRC-8 of bytes 1-7 |

MCU -> Pi, **parameter reply, 10 bytes**:

| Byte | 0 | 1 | 2 | 3 | 4 | 5-8 | 9 |
|---|---|---|---|---|---|---|---|
| Content | `0xA5` | `0x04` | op | idx | status | value (float32 LE) after the op | CRC-8 of bytes 1-8 |

- op: 0 get, 1 set (RAM), 2 save all to the EEPROM, 3 load the `config.h` defaults (RAM), 4 reload from the EEPROM (RAM). idx is ignored by ops 2-4 (the Pi sends 0).
- status: 0 ok, 1 bad index, 2 out of range (also NaN and +-Inf; the value is not changed), 3 busy, 4 bad feed-forward table, 5 bad op, 6 no valid EEPROM block.
- **Only while stopped:** every op except get needs all wheel targets at 0, otherwise the reply is "busy".
- The Pi sends **one request at a time** and waits for the reply (0.2 s; save 1.0 s). A request with a wrong CRC gets no reply.
- The RX state machine keeps **one slot per id** (command, parameter request): drive commands that arrive while the main loop is busy never overwrite a waiting request.
- A parameter request is not a drive command: it does not restart the command timeout and does not clear the boot flag.

| idx | name | default (`config.h`) | range |
|---|---|---|---|
| 0 | kp | `PI_KP` 1.0 % per rad/s | 0..50 |
| 1 | ki | `PI_KI` 8.0 % per rad/s per s | 0..200 |
| 2 | i_limit | `PI_I_LIMIT` 25 % | 0..100 |
| 3 | accel_limit | `ACCEL_LIMIT` 0 rad/s^2 (0 = off) | 0..200 |
| 4-13 | ff_speed0..9 | `FF_SPEED_RAD_S` | 0..20 rad/s |
| 14-23 | ff_m1_pwm0..9 | `FF_M1_PWM_PCT` (PWM row of M1, at the ff_speed points) | 0..100 % |
| 24-33 | ff_m2_pwm0..9 | `FF_M2_PWM_PCT` | 0..100 % |
| 34-43 | ff_m3_pwm0..9 | `FF_M3_PWM_PCT` | 0..100 % |
| 44-53 | ff_m4_pwm0..9 | `FF_M4_PWM_PCT` | 0..100 % |

- **Feed-forward table edits:** a set of one point only checks that value's range. The whole table (ff_speed0 = 0, speeds strictly increasing) is checked at the next non-zero drive command and at save. Invalid: drive commands are ignored and flag bit3 is set (until an edit makes it valid again); save is refused (status 4).
- **EEPROM:** one block = layout version (`EEPROM_LAYOUT_VERSION`) + the 54 floats + **CRC-16** (XMODEM: poly 0x1021, init 0) over version and data, 219 bytes (layout version 2 since v2.3.0; a v2.2.0 block, layout 1, is ignored). At power-up a block with the right version and CRC, finite values within range and a valid table replaces the defaults; otherwise the defaults are used. The fuse `hfuse 0xD1` has EESAVE set, so **a reflash keeps the saved parameters** (use op 3 + save, or `tune_motors.py defaults` + `save`, to go back to `config.h`). Change `EEPROM_LAYOUT_VERSION` when the parameter list changes.
- **Save** blocks the main loop for up to ~0.35 s (3.4 ms per changed byte). Interrupts keep running (encoders, Timer3, UART RX into its slots, UART TX). The next control step uses the real elapsed time for the speeds and `seq`, but at most one period for the integral and the ramp; a command that waited during the save does not time out at once.
- Concurrency: `param[]` and the controller state are only used in the main loop, never in an interrupt, so a float is never read half-written.

## 2. Interface

| Motor | PWM pin | Direction pin | Encoder (A / B) | Encoder sign | Kinematic signs Vx / Vy / w |
|---|---|---|---|---|---|
| M1 | PD6 (OC0A) | PE0 | E1: PB3 / PB4 | -1 | + / - / - |
| M2 | PD5 (OC0B) | PE1 | E2: PD2 / PD3 | -1 | + / + / - |
| M3 | PB1 (OC1A) | PE2 | E3: PD4 / PD7 | +1 | + / - / + |
| M4 | PB2 (OC1B) | PE3 | E4: PC2 / PC3 | +1 | + / + / + |

- Wheel target: `M1 = (Vx - Vy - k*w)/R`, `M2 = (Vx + Vy - k*w)/R`, `M3 = (Vx - Vy + k*w)/R`, `M4 = (Vx + Vy + k*w)/R`, with `k = half track + half wheelbase = 0.330 m`, `R = 0.040 m`.
- Layout: M1 front-left, M2 rear-left, M3 rear-right, M4 front-right. Forward driving and the odometry scale were verified on the robot (2026-10-07: 1.17 m driven = 1.17 m odometry); strafe and turn worked as expected in the teleop test.
- Motor input polarity: **no motor is inverted in software** (`MOTORn_DIR_INVERT` = 0 for all four). The M2/M3 supply cables are wired opposite to M1/M4 in the hardware; see the **hardware polarity note** at the top.
- Encoders: only the A pins raise pin-change interrupts; B is sampled in the interrupt. Every A edge counts, direction from B.
- The motor <-> encoder association (`MOTORn_ENCODER`), encoder signs (`MOTORn_ENC_SIGN`) and all pins are in the "WIRING MAP" of `config.h`. Invalid mappings fail at compile time.

## 3. Parameters (`motor_driver_C/config.h`)

| Parameter | Value |
|---|---|
| Wheel radius / half track / half wheelbase | 0.040 / 0.160 / 0.170 m (v2.3.3; 0.130 before v2.3.2) |
| Encoder counts per wheel revolution | 1536 (verified on the robot, 2026-10-07) |
| Control period (Timer3, `OCR3A = 12499`) | 50 ms (20 Hz); one encoder count = 0.082 rad/s |
| State frame | sent every period (20 Hz), cumulative encoder counts |
| Command timeout | 200 ms |
| Pi link / command units | UART0 38400 baud, binary frames with CRC-8 / int16 mm/s and mrad/s |
| Speed controller | positional PI + feed-forward + anti-windup + target ramp (no D term). Defaults: Kp 1.0, Ki 8.0, integral limit 25 %, ramp off. Run-time values: setting mode (section 1), saved in the EEPROM |
| Feed-forward | table speed -> PWM from step tests (v1.1.1, `FF_*` in `config.h`): 0 -> 9 % (dead zone), 3.5 -> 15 %, 6.6 -> 21.5 %, 8.1 -> 29.5 %, 9.4 -> 37.5 %, 10.7 -> 45.5 %, 11.5 -> 54 %, 11.85 -> 63 %, 12.2 -> 72 %, 13.0 rad/s -> 100 % |
| Output limit | +-100 % (8-bit PWM, 1/255 resolution) |
| Measured top speed (100 % PWM) | 12.6-13.1 rad/s = 0.50-0.52 m/s rim speed, with and without load (2026-10-07) |

Controller per wheel: `e = target - measured`, `pwm = FF(target) + Kp*e + I`, `I += Ki*e*dt` (clamped, not integrated while saturated). Target 0 gives output 0 and clears `I`; `I` is also cleared when the target changes sign. With `accel_limit` > 0 the target moves toward the commanded wheel speed by at most `accel_limit * dt` per step (a stop is always at once). `dt` for the integral and the ramp is at most one period.

## 4. Firmware structure

Nothing blocks: interrupts only collect data and set flags, the main loop does the work. Diagram: [`docs/state_machine.svg`](docs/state_machine.svg) (labels in Chinese; it shows the v1.x text protocol: the frame parser and the feedback are different since v2.1.0).

| Interrupt | Trigger | Does |
|---|---|---|
| `PCINT0/1/2_vect` | edge on an encoder A pin (ports B / C / D) | counts the edge (+1/-1 from B) |
| `TIMER3_COMPA_vect` | every 50 ms, exact | snapshots the encoder counts, sets the control tick |
| `USART0_RX_vect` | byte received | frame state machine (IDLE, HEADER_ACK, PAYLOAD) for commands and parameter requests; one slot each, sets `cmd_ready` / `param_ready` |
| `USART0_UDRE_vect` | TX register empty | sends the next byte from the 128-byte ring buffer |

Main loop: check (CRC) and apply a new command; check (CRC) and answer a parameter request (setting mode); on each tick do speed measurement (counts / real elapsed time), timeout check, ramp, controller, motor output and send the state frame.

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
  6. Remove the ISP wires, at least RESET, so the chip runs on its own. Then the UART (38400 baud) sends a 21-byte state frame (`A5 02 ...`) every 50 ms.
- **Compile check on the Pi** (no flashing; avr-gcc 7.3 from apt). Ubuntu's avr-libc has no ATmega328PB header,
  so take it from the same device pack Studio uses (unzip `gcc/dev/atmega328pb/` and `include/` to `/tmp/dfp/pack`):
  `curl -fsSL -o /tmp/dfp/dfp.zip http://packs.download.atmel.com/Atmel.ATmega_DFP.1.7.374.atpack`, then
  `avr-gcc -B /tmp/dfp/pack/gcc/dev/atmega328pb/ -I /tmp/dfp/pack/include -mmcu=atmega328pb -DF_CPU=16000000UL -Os -Wall -Wextra -o /tmp/fw.elf motor_driver_C/*.c`.
  v2.0.0 (2026-10-09): 0 errors, 0 warnings, 7.3 KB flash, 428 B RAM. v2.1.0 (2026-10-12): 0 errors, 0 warnings, 7.0 KB flash, 374 B RAM. v2.2.0 (2026-10-15): 0 errors, 0 warnings, 9.3 KB flash (9547 B), 610 B RAM, 99 B EEPROM. v2.3.0 (2026-10-15): 0 errors, 0 warnings, 9.7 KB flash (9899 B), 970 B RAM, 219 B EEPROM.
- **Tests** (models and simulation, not hardware): `python tests/uart_frame_model_test.py`, `timeout_model_test.py`, `param_model_test.py`, `motor_polarity_model_test.py`, `pid_sim.py`.

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

**v2.1.0 (2026-10-12)** - back to UART. **Needs the matching Pi code** (same commit).
- The I2C link of v2.0.0 never worked on the robot: the level shifter on the board is a TXB type, which cannot do I2C. The TWI code is removed.
- UART on PD0/PD1 again (38400 baud), now with binary frames that keep the v2.0.0 contents: command `A5 01` + vx, vy, w (int16 mm/s, mrad/s) + CRC-8; state `A5 02` + seq, flags, 4 x int32 cumulative counts + CRC-8, sent every 50 ms.
- Receive state machine IDLE -> HEADER_ACK -> PAYLOAD; a `0xA5` inside the data never restarts a frame.
- No text on the UART any more (boot text `a`, `test` and `cmd timeout` removed; the state flags carry that). `adc_init()` is no longer called.

**v2.2.0 (2026-10-15)** - setting mode. **Needs the matching Pi code** (same commit).
- Why: the motor test with v2.1.0 (2026-10-15, wheels in the air) showed 25-30 % overshoot at 3.75 rad/s and, at 1.25 rad/s, a start 0.6-1 s late, then 2-3x overshoot and stick-slip. Each gain change needed a reflash.
- Parameter request `0xA5 0x03` / reply `0xA5 0x04` (section 1): kp, ki, i_limit, accel_limit and the 10-point feed-forward table can be read and changed while stopped, and saved in the EEPROM (CRC-16 block, loaded at power-up).
- New target ramp (`accel_limit`, default 0 = off). No D term (the speed estimate is quantised to 0.082 rad/s).
- A non-zero command is applied by the next control step: v2.1.0 wrote `FF + I` (without the P term) when a command arrived, between the control steps.
- Integral and ramp use at most one period as `dt` after a late loop; the command timeout counts only the periods after a command (no false timeout after an EEPROM save).
- State flags bit2 (parameters = EEPROM block) and bit3 (feed-forward table invalid).
- The defaults (`config.h`) are the v2.1.0 values. With them the only change in behaviour is the start of a command: up to 50 ms later, through the full controller.

**v2.3.0 (2026-10-15)** - one feed-forward PWM row per motor. **Needs the matching Pi code** (same commit).
- Why: steady-state test (2026-10-15, no wheels, open loop, 3 s settle then 2 s measured): the motors differ. For 5 rad/s M1 needs 17.3 %, M2 14.5 %, M3 13.6 %, M4 19.9 % PWM; M4 needs ~1.5x the PWM of M3 between 3.5 and 11 rad/s. With one shared row M2/M3 overshot by 40-83 % at 0.05 m/s.
- `ff_m1_pwm0..9` .. `ff_m4_pwm0..9` (parameters 14-53) replace `ff_pwm0..9`; the speed points `ff_speed0..9` stay shared. 54 parameters, EEPROM layout version 2.
- Defaults: the v2.2.0 row for all four motors (behaviour unchanged until `tune_motors.py ff --apply` + `save`).

**v2.3.1 (2026-10-15)** - new defaults only (no code change): the values tuned on the floor.

**v2.3.3 (2026-10-19)** - half wheelbase 0.170 m (wheelbase 340 mm, measured by the user; v2.3.2 had 0.1675 m). Same value in `config/base.yaml` (same commit). The rotation arm k is 0.330 m. Flashed by the user. Geometry only; the EEPROM values stay valid.

**v2.3.2 (2026-10-19)** - half wheelbase 0.1675 m (wheelbase 335 mm, measured by the user 2026-10-16; was 0.130 m). **Needs the matching Pi code** (`half_wheelbase` in `config/base.yaml`, same commit) **and a reflash**: the geometry is compiled in, not an EEPROM parameter. The rotation arm k goes from 0.290 to 0.3275 m: commanded turns were ~11 % too slow, the wheel odometry turn ~12.9 % too high. The EEPROM controller values stay valid (they are per wheel, in rad/s).

| Parameter | Value (saved in the MCU EEPROM 2026-10-15 = `config.h` defaults since v2.3.1) |
|---|---|
| kp / ki / i_limit / accel_limit | 2.0 / 8.0 / 10 % / 0 (off) |
| ff_speed0..9 [rad/s] | 0, 1, 2, 3.5, 5, 6.6, 8.1, 9.4, 11.5, 13 |
| ff_m1_pwm0..9 [%] | 10.71, 12.80, 14.89, 17.86, 21.05, 25.35, 30.72, 37.82, 67.37, 100 |
| ff_m2_pwm0..9 [%] | 10.88, 12.58, 14.30, 16.72, 19.53, 24.04, 29.14, 35.88, 63.73, 98.73 |
| ff_m3_pwm0..9 [%] | 11.07, 12.49, 13.92, 16.16, 18.98, 23.67, 28.66, 35.48, 63.35, 99.49 |
| ff_m4_pwm0..9 [%] | 10.86, 12.94, 15.02, 18.39, 21.92, 27.34, 33.42, 41.49, 73.74, 100 |

How they were found (`scripts/tune_motors.py`, robot on the floor, 2 m free, a person watching):
- Open-loop FF measurement on the floor works only at first: with the same PWM the strong rear-right side pushes harder
  and the robot turns, then the wheels scrub and the friction rises (second run: no motion below ~20 %). On the floor the
  four wheels are coupled, so a per-motor row cannot be measured there.
- So each row = the floor load (first floor run, robot still straight; the same for all four) + 0.5 x the difference of
  that motor from the mean, measured in the air without wheels. With the full difference M4 ran 5-13 % too fast, without
  it the result was about the same as with half.
- Floor steps, forward and back, legs <= 0.5 m: overshoot 5-23 % and steady error 0-9 % at 0.20-0.30 m/s; 18-50 %
  overshoot at 0.05-0.10 m/s (stick-slip of the whole robot: it needs ~12-17 % PWM to break free). The firmware defaults
  of v2.3.0 gave 8-77 % overshoot and up to 52 % steady error. A ramp (accel_limit 10) and ki 15 were worse.
- What is left (overshoot, slow settling) comes from the integral during the rise; a firmware change could help
  (integrate only near the target).
- The tuned values are in the EEPROM; a reflash keeps them (EESAVE). `tune_motors.py defaults` loads the same values.

## 7. Known limitations

- The PI gains come from simulation of an *assumed* motor model. The feed-forward table is measured (wheels in the air); check it with step tests after flashing v1.1.1.
- **v2.1.0**: on the MCU 2026-10-14/15: state frames at 20 Hz without CRC errors; motors, encoders and kinematics signs correct (wheels in the air).
- **v2.2.0**: flashed 2026-10-15, setting mode works on the robot.
- **v2.3.0**: flashed 2026-10-15. Tuned on the floor and saved in the EEPROM (section 6, v2.3.1). v2.3.1 only changes the defaults: no reflash needed.
- **v2.3.3**: flashed 2026-10-19 (half wheelbase 0.170 m). To do: repeat the 360 deg turn test (wheels and IMU).
- The speed controller is tuned on the floor (section 6, v2.3.1), but the overshoot at low speed is still high.
- The motors reach only ~13 rad/s (0.50-0.52 m/s). Faster targets saturate at 100 % PWM; the Pi limits every wheel to 0.45 m/s (`max_wheel_speed` in `config/base.yaml`).
- Stopping is by coasting, so the stopping distance depends on friction.

---
Last updated: 2026-10-19 (v2.3.3) · Stiffeel :octocat: · Claude
