# Pi <-> MCU Protocol

The firmware is written and owned by a teammate. Its own documents are the source of truth:

- [firmware/docs/protocol.md](../firmware/docs/protocol.md) (Chinese): the protocol.
- [firmware/CHANGES.md](../firmware/CHANGES.md) (English) / `firmware/变化.md` (Chinese): what changed from the old firmware.

This file is the Pi-side summary, for firmware **v0.1.0 (NALA_v0, 2026-10-06)**.
When the firmware protocol changes, update the Pi code (`src/nala_base/nala_base/mcu_protocol.py`)
and this file in the same commit.

Items marked **(verify)** are not confirmed on the robot yet.

## Link

| Item | Value |
|---|---|
| MCU | ATmega328PB, 16 MHz, USART0 (RXD0 = PD0, TXD0 = PD1, TTL level) |
| Port on Pi | `/dev/ttyS0` (GPIO UART). `scripts/setup_pi.sh` removes the Linux serial console from it |
| Settings | 9600 baud, 8N1, no flow control |

## Pi -> MCU

| Frame | Bytes | Use |
|---|---|---|
| Velocity | `80 86 Vx Vy w` | Set the body speed |
| Echo control | `80 87 E` | `E=01` echo on, `E=00` echo off. **The Pi does not send it** (echo is off by default) |

- `Vx`, `Vy`, `w` are **int8**, **0.02 per count** (`count = value * 50`, rounded, limited to +-127, so at most +-2.54).
- Axes are the **ROS axes** (REP 103), so a `geometry_msgs/Twist` maps 1:1:
  `Vx = linear.x` (forward), `Vy = linear.y` (left), `w = angular.z` (counter-clockwise). **(verify)**
- No checksum. The MCU finds frames by the `80 86` / `80 87` header.

Examples: stop `80 86 00 00 00`, forward 0.10 m/s `80 86 05 00 00`, backward 0.10 m/s `80 86 FB 00 00`,
left 0.10 m/s `80 86 00 05 00`, counter-clockwise 0.50 rad/s `80 86 00 00 19`.

### MCU command timeout

If the robot is moving and no velocity frame arrives for **500 ms**, the MCU stops and prints `cmd timeout` once.
A zero frame never times out. So the Pi must keep sending while the robot moves.

### How the Pi sends (nala_base)

- The Pi sends the current command at a fixed rate (`command_rate` in `config/base.yaml`, 10 Hz), also when it is zero.
  10 Hz is 5 times faster than the 500 ms MCU timeout.
- If `/cmd_vel` is older than `cmd_vel_timeout` (0.6 s), the Pi sends zero.
- Sending is **not** synced to the wheel speed feedback, on purpose:
  - The UART is full duplex (separate TX and RX wires), so sending and receiving at the same time do not collide.
  - The new firmware never blocks: the RX interrupt only stores bytes, TX uses a ring buffer.
  - The MCU may drop a feedback line when its TX buffer is full. If sending waited for feedback,
    a dropped line would delay the next command and could trigger the MCU timeout. It would also add up to 100 ms delay.
- Line load at 9600 baud: commands use about 5 % of the Pi -> MCU wire, feedback about 50 % of the MCU -> Pi wire.

## MCU -> Pi: text lines (end with `\n`)

| Line | Format | When |
|---|---|---|
| **Measured wheel speeds** | `M1 \t M2 \t M3 \t M4` (4 floats, rad/s) | Every 100 ms (hardware timer) |
| Boot | `a`, then `test` | Once after reset |
| Command echo | `Vx Vy w` (3 floats; m/s, m/s, rad/s) | After each velocity frame, only when echo is on |
| Echo ack | `echo on` / `echo off` | After an echo control frame |
| Timeout | `cmd timeout` | Once, when the MCU stops because of the timeout |

Parsing rule (`parse_line`): split on whitespace. 4 floats = wheel speeds, 3 floats = echo, other text = message.
Lines can be dropped by the MCU, so never depend on every line arriving.

## Motors and wheel speeds

```
        front (+Vx)
 M1 (FL)    M4 (FR)         left = +Vy
 M2 (RL)    M3 (RR)         counter-clockwise = +w
        rear
```

- M1 front-left, M2 rear-left, M3 rear-right, M4 front-right (given by the user; firmware math agrees).
- Measured wheel speed sign: **positive = the wheel pushes the robot forward** (the firmware corrects the mirrored encoders). **(verify)**
- Encoder: 1536 counts per wheel revolution (confirmed by the user).
- Firmware geometry: R = 0.040 m, half track 0.160 m, half wheelbase 0.130 m (k = 0.290 m).

Firmware inverse kinematics (rad/s):

```
M1 = (Vx - Vy - k*w) / R
M2 = (Vx + Vy - k*w) / R
M3 = (Vx - Vy + k*w) / R
M4 = (Vx + Vy + k*w) / R
```

## Pi forward kinematics (for odometry, stage 2)

```
Vx = R/4     * ( M1 + M2 + M3 + M4)
Vy = R/4     * (-M1 + M2 - M3 + M4)
w  = R/(4*k) * (-M1 - M2 + M3 + M4)
```

## First test on the robot (wheels off the ground)

1. After reset the Pi log shows `MCU: a` and `MCU: test`.
2. Teleop key `i` (forward): all 4 wheels turn forward, all 4 measured speeds positive.
3. `J` (Shift+j, strafe left), `j` (turn counter-clockwise): the wheels turn as in the formulas above.
4. Release the key: the wheels stop after about 0.6 s (Pi `/cmd_vel` timeout). There should be no `cmd timeout`,
   because the Pi keeps sending (zero) frames.
5. Watch for PID oscillation (the firmware PID gains act differently now, see `firmware/CHANGES.md`).
