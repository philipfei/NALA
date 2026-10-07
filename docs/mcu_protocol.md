# Pi <-> MCU Protocol

The firmware is owned by a teammate, who builds and flashes it (Claude may edit it). Its own documents are the source of truth:

- [firmware/README.md](../firmware/README.md): the protocol, parameters and changes per version.

This file is the Pi-side summary, for firmware **v1.1.0 (NALA_v1, 2026-10-07)**.
v1.1.0 changed the baud rate and the feedback line: the Pi code only works with v1.1.0 or newer.
When the firmware protocol changes, update the Pi code (`src/nala_base/nala_base/mcu_protocol.py`)
and this file in the same commit.

Items marked **(verify)** are not confirmed on the robot yet.

## Link

| Item | Value |
|---|---|
| MCU | ATmega328PB, 16 MHz, USART0 (RXD0 = PD0, TXD0 = PD1, TTL level) |
| Port on Pi | `/dev/ttyS0` (GPIO UART). `scripts/setup_pi.sh` removes the Linux serial console from it |
| Settings | 38400 baud (v1.1.0; was 9600), 8N1, no flow control |

## Pi -> MCU

| Frame | Bytes | Use |
|---|---|---|
| Velocity | `80 86 Vx Vy w` | Set the body speed |
| Echo control | `80 87 E` | `E=01` echo on, `E=00` echo off. **The Pi does not send it** (echo is off by default) |

- `Vx`, `Vy`, `w` are **int8**, **0.02 per count** (`count = value * 50`, rounded, limited to +-127, so at most +-2.54).
- Axes are the **ROS axes** (REP 103), so a `geometry_msgs/Twist` maps 1:1:
  `Vx = linear.x` (forward), `Vy = linear.y` (left), `w = angular.z` (counter-clockwise).
  Checked on the robot 2026-10-07 (forward, strafe left, turn).
- No checksum. The MCU finds frames by the `80 86` / `80 87` header.

Examples: stop `80 86 00 00 00`, forward 0.10 m/s `80 86 05 00 00`, backward 0.10 m/s `80 86 FB 00 00`,
left 0.10 m/s `80 86 00 05 00`, counter-clockwise 0.50 rad/s `80 86 00 00 19`.

### MCU command timeout

If the robot is moving and no velocity frame arrives for **200 ms**, the MCU stops and prints `cmd timeout` once
(the stop happens 200-250 ms after the last frame). A zero frame never times out.
So the Pi must keep sending while the robot moves: 20 Hz, never slower than one frame per 150 ms.
A stop (target 0) lets the motors coast: no active braking.

### How the Pi sends (nala_base)

- The Pi sends the current command at a fixed rate (`command_rate` in `config/base.yaml`, 20 Hz), also when it is zero.
  That is 4 frames in the 200 ms MCU timeout. The firmware control loop also runs at 20 Hz.
- Each frame is one `write` of 5 bytes.
- If `/cmd_vel` is older than `cmd_vel_timeout` (0.6 s), the Pi sends zero.
- Sending is **not** synced to the counts feedback, on purpose:
  - The UART is full duplex (separate TX and RX wires), so sending and receiving at the same time do not collide.
  - The new firmware never blocks: the RX interrupt only stores bytes, TX uses a ring buffer.
  - The MCU may drop a feedback line when its TX buffer is full. If sending waited for feedback,
    a dropped line would delay the next command and could trigger the MCU timeout. It would also add delay.
- Line load at 38400 baud: commands (20 Hz x 5 bytes) use about 3 % of the Pi -> MCU wire,
  feedback (20 Hz x up to 50 bytes, about 25 when driving) at most 26 % of the MCU -> Pi wire.

## MCU -> Pi: text lines (end with `\n`)

| Line | Format | When |
|---|---|---|
| **Encoder counts** | `c n1 n2 n3 n4` (4 int32: cumulative counts of M1..M4 since the MCU start) | Every 50 ms (20 Hz, every control tick) |
| Boot | `a`, then `test` | Once after reset |
| Command echo | `Vx Vy w` (3 floats; m/s, m/s, rad/s) | After each velocity frame, only when echo is on |
| Echo ack | `echo on` / `echo off` | After an echo control frame |
| Timeout | `cmd timeout` | Once, when the MCU stops because of the timeout |

Parsing rule (`parse_line`): split on whitespace. `c` + 4 integers = counts, 3 floats = echo, other text = message.
Lines can be dropped by the MCU. The counts are cumulative, so a dropped line loses no distance.

## Odometry from the counts (`base_node`)

- The first counts line after the start (or after `test`, an MCU restart: the counters start again at 0)
  is only the reference.
- Each next line: count change per wheel (`count_delta`, int32 wrap-around safe)
  x 2*pi / 1536 = wheel angle change -> `wheels_to_body` -> body movement (dx, dy, dyaw) -> pose.
- `/odom` speed = movement / (MCU periods covered x 0.05 s). The periods come from the arrival time
  (normally 1, 2 after a dropped line). Parameters: `encoder_counts_per_rev`, `feedback_period` in `config/base.yaml`.

## Motors and wheel speeds

```
        front (+Vx)
 M1 (FL)    M4 (FR)         left = +Vy
 M2 (RL)    M3 (RR)         counter-clockwise = +w
        rear
```

- M1 front-left, M2 rear-left, M3 rear-right, M4 front-right (given by the user; firmware math agrees).
- Counts sign: **positive = the wheel drove the robot forward** (the firmware corrects the mirrored encoders). Checked 2026-10-07.
- The firmware does **not** invert any motor in software (`MOTORn_DIR_INVERT` is 0 for all four in `config.h`).
  The supply cables of M2 and M3 are wired with opposite polarity to M1 and M4 in the hardware, and the firmware relies on that: do not invert M2/M3 in software.
  A positive command turns all four wheels the same way. The encoder signs did not change.
- Encoder: 1536 counts per wheel revolution (checked 2026-10-07: 1.17 m driven = 1.17 m odometry).
- Firmware geometry: R = 0.040 m, half track 0.160 m, half wheelbase 0.130 m (k = 0.290 m).

Firmware inverse kinematics (rad/s):

```
M1 = (Vx - Vy - k*w) / R
M2 = (Vx + Vy - k*w) / R
M3 = (Vx - Vy + k*w) / R
M4 = (Vx + Vy + k*w) / R
```

## Pi forward kinematics (odometry: `wheels_to_body` in `src/nala_base/nala_base/kinematics.py`)

```
Vx = R/4     * ( M1 + M2 + M3 + M4)
Vy = R/4     * (-M1 + M2 - M3 + M4)
w  = R/(4*k) * (-M1 - M2 + M3 + M4)
```

## First test on the robot (wheels off the ground)

1. After reset the Pi log shows `MCU: a` and `MCU: test`. Then counts lines at 20 Hz (`log_level:=debug`).
2. Teleop key `i` (forward): all 4 wheels turn forward, all 4 wheel speeds in the debug log positive.
3. `J` (Shift+j, strafe left), `j` (turn counter-clockwise): the wheels turn as in the formulas above.
4. Release the key: the wheels stop after about 0.6 s (Pi `/cmd_vel` timeout). There should be no `cmd timeout`,
   because the Pi keeps sending (zero) frames. The wheels coast to a stop (no braking).
5. Watch for oscillation. v1 has a new PI speed controller; v1.1.0 raised the feed-forward to 6.5 (from a
   measurement): step test (for example 0.3 m/s) to check the rise time and overshoot. See `firmware/README.md`.
6. Top speed: 1.0 m/s needs 25 rad/s = 100 % feed-forward. The firmware clips each wheel at 100 % on its own,
   so the Pi keeps every wheel below `max_wheel_speed` (`config/base.yaml`, measured top rim speed).
   Measured 2026-10-07: 100 % PWM gives only 12.6-13.1 rad/s (0.50-0.52 m/s), with and without load.
