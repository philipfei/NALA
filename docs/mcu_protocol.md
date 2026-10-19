# Pi <-> MCU Protocol

The firmware is owned by a teammate, who builds and flashes it (Claude may edit it). Its own documents are the source of truth:

- [firmware/README.md](../firmware/README.md): the protocol, parameters and changes per version.

This file is the Pi-side summary, for firmware **v2.3.0 (2026-10-15)**: UART with binary frames, plus the
setting mode (parameter frames). `base_node` works with v2.1.0 or newer; `scripts/tune_motors.py` needs v2.3.0 (54 parameters).
When the firmware protocol changes, update the Pi code (`src/nala_base/nala_base/mcu_protocol.py`)
and this file in the same commit.

## Link

| Item | Value |
|---|---|
| MCU | ATmega328PB, 5 V, USART0: PD0 = RXD0, PD1 = TXD0 |
| Pi | GPIO14 (TX) / GPIO15 (RX), 3.3 V, PL011 UART = `/dev/ttyAMA0` (`dtoverlay=miniuart-bt`, `scripts/setup_pi.sh`) |
| Between them | the level shifter on the board. It is a **TXB** type (the schematic says TXS0104E): fine for UART, **not for I2C** |
| Settings | 38400 baud, 8N1, no flow control. Binary frames only, no text |

History: v2.0.0 used I2C for this link (2026-10-09). It never worked, because a TXB level shifter oscillates on I2C lines.

## Frames

Every frame: `0xA5` (sync), id, data, CRC-8 of id + data.
CRC-8: polynomial 0x07, initial value 0 (`crc8("123456789") = 0xF4`), same code on both sides.

### Pi -> MCU: command (9 bytes)

`0xA5, 0x01, vx, vy, w, crc`: vx, vy in mm/s, w in mrad/s, each **int16 little endian**.

- Axes are the **ROS axes** (REP 103), so a `geometry_msgs/Twist` maps 1:1:
  `vx = linear.x` (forward), `vy = linear.y` (left), `w = angular.z` (counter-clockwise).
  Checked on the robot 2026-10-07 (forward, strafe left, turn) with the older firmware.
- Resolution 1 mm/s and 1 mrad/s (`encode_command` rounds and clamps to +-32767).
- A frame with a wrong CRC is ignored by the MCU.

### MCU command timeout

If the robot is moving and no valid command arrives for **200 ms**, the MCU stops (state flag bit1 until the next
valid command). A zero command never times out. A stop lets the motors coast: no active braking.

### MCU -> Pi: state (21 bytes, every MCU control period = 20 Hz)

`0xA5, 0x02, seq, flags, M1, M2, M3, M4, crc`: seq uint8, flags uint8, Mn int32 little endian.

| Field | Meaning |
|---|---|
| seq | MCU control period counter (50 ms, wraps at 256) |
| flags bit0 | no valid command since the MCU started: its counters start at 0 |
| flags bit1 | stopped by the command timeout |
| flags bit2 | the MCU parameters are its EEPROM block (loaded or saved, unchanged since) (v2.2.0) |
| flags bit3 | the feed-forward table is invalid: the MCU ignores non-zero drive commands (v2.2.0; `base_node` logs an error) |
| M1..M4 | cumulative encoder counts since the MCU start, + = the wheel drove the robot forward |

### Setting mode (v2.2.0; per-motor FF rows since v2.3.0): parameter request (9 bytes) and reply (10 bytes)

Used only by `scripts/tune_motors.py` (with `base_node` stopped). Details and the parameter table: `firmware/README.md`.

- Pi -> MCU `0xA5, 0x03, op, idx, value (float32 LE), crc`: same length as a command (6 data bytes + crc).
  op 0 get, 1 set (RAM), 2 save to the EEPROM, 3 load the firmware defaults, 4 reload from the EEPROM.
- MCU -> Pi `0xA5, 0x04, op, idx, status, value (float32 LE), crc`. status 0 ok, 1 bad index, 2 out of range
  (or not finite), 3 busy (not stopped), 4 bad feed-forward table, 5 bad op, 6 no valid EEPROM block.
- Parameters (`mcu_protocol.PARAM_NAMES`, same order as the firmware): 0 kp, 1 ki, 2 i_limit, 3 accel_limit,
  4-13 ff_speed0..9 (shared), 14-23 ff_m1_pwm0..9, 24-33 ff_m2_pwm0..9, 34-43 ff_m3_pwm0..9, 44-53 ff_m4_pwm0..9.
- Every op except get only while the robot is stopped. One request at a time.
- The saved parameters are in the **MCU EEPROM** (user decision 2026-10-15), not in `config/`. A reflash keeps them.
  `tune_motors.py show` prints what the MCU uses.
- `split_frames` returns both frame types: `(states, replies, rest, bad)`. `base_node` ignores the replies.

## How the Pi talks to the MCU (`base_node`)

- A timer at `command_rate` (20 Hz) writes the command frame (the current command, zero if `/cmd_vel` is older
  than `cmd_vel_timeout` 0.6 s). Sending is not synced to the state frames (the UART is full duplex).
- A reader thread blocks on the UART and cuts the byte stream into state frames (`split_frames`):
  it looks for `0xA5 0x02` and checks the CRC; other bytes are skipped. A frame with a wrong CRC is counted
  (throttled warning) and skipped.
- No valid state for `feedback_timeout` (0.5 s): warning, odometry stops until frames come again.

## Odometry from the counts (`base_node`)

- The first frame after the start (or a frame with flags bit0: the MCU restarted) is only the reference.
- Each next frame with a new seq: count change per wheel (`count_delta`, int32 wrap-around safe)
  x 2*pi / 1536 = wheel angle change -> `wheels_to_body` -> body movement (dx, dy, dyaw) -> pose.
  A lost frame loses no distance (the counts are cumulative).
- Speed in `/odom` = movement / (periods x 0.05 s): periods = seq difference (exact MCU time).
  `header.stamp` = Pi time when the frame arrived. Twist covariance: `odom_twist_variance` (vx, vy) for the EKF.
- `publish_tf`: TF odom -> base_footprint from the wheels. App 2 turns it off; there the EKF publishes it
  (wheel vx, vy + IMU turn rate, `docs/imu_ekf.md`).

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
- Firmware geometry: R = 0.040 m, half track 0.160 m, half wheelbase 0.170 m (k = 0.330 m; firmware v2.3.3, 0.130 / 0.290 before).

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

1. App 1 with `log_level:=debug`: no CRC warnings, no "No valid state" warning, `wheels M1..M4` lines at 20 Hz.
2. Turn each wheel forward by hand: its value in the debug log is positive.
3. Teleop key `i` (forward): all 4 wheels turn forward, all 4 wheel speeds positive.
4. `J` (Shift+j, strafe left), `j` (turn counter-clockwise): the wheels turn as in the formulas above.
5. Release the key: the wheels stop after about 0.6 s (Pi `/cmd_vel` timeout). No MCU timeout warning,
   because the Pi keeps sending (zero) commands. The wheels coast to a stop (no braking).
6. Speed control: tune with `scripts/tune_motors.py` (firmware v2.3.0 setting mode): `ff --apply` and `sweep` in the air, then `sweep --floor`.
7. Top speed: 100 % PWM gives only 12.6-13.1 rad/s (0.50-0.52 m/s, measured 2026-10-07). The firmware clips each
   wheel at 100 % on its own, so the Pi keeps every wheel below `max_wheel_speed` (`config/base.yaml`).
