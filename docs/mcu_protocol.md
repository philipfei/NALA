# Pi <-> MCU Protocol

The firmware is owned by a teammate, who builds and flashes it (Claude may edit it). Its own documents are the source of truth:

- [firmware/README.md](../firmware/README.md): the protocol, parameters and changes per version.

This file is the Pi-side summary, for firmware **v2.0.0 (NALA_v1, 2026-10-09)**.
v2.0.0 moved the link from UART to I2C: the Pi code only works with v2.0.0 or newer.
When the firmware protocol changes, update the Pi code (`src/nala_base/nala_base/mcu_protocol.py`)
and this file in the same commit.

## Link

| Item | Value |
|---|---|
| Bus | I2C bus 1 (`/dev/i2c-1`, `dtparam=i2c_arm=on`), 100 kHz, Pi = master |
| Pins | Pi GPIO2 SDA / GPIO3 SCL (3.3 V) <-> level shifter <-> MCU PC4 SDA0 / PC5 SCL0 (5 V) |
| MCU address | `0x10` (`i2c_address: 16` in `config/base.yaml`, `I2C_ADDRESS` in firmware `config.h`) |
| Check | `i2cdetect -y 1` shows `10` |

The Pi UART on GPIO14/15 belongs to the IMU (`docs/imu_ekf.md`).

## Pi -> MCU: command (I2C write, 8 bytes)

`0x01, vx, vy, w, crc`: vx, vy in mm/s, w in mrad/s, each **int16 little endian**, then CRC-8 of the first 7 bytes.

- Axes are the **ROS axes** (REP 103), so a `geometry_msgs/Twist` maps 1:1:
  `vx = linear.x` (forward), `vy = linear.y` (left), `w = angular.z` (counter-clockwise).
  Checked on the robot 2026-10-07 (forward, strafe left, turn).
- Resolution 1 mm/s and 1 mrad/s (`encode_command` rounds and clamps to +-32767).
- A frame with a wrong id or CRC is ignored by the MCU.

### MCU command timeout

If the robot is moving and no valid command arrives for **200 ms**, the MCU stops (state flag bit1 until the next
valid command). A zero command never times out. A stop lets the motors coast: no active braking.

## MCU -> Pi: state (I2C read, 19 bytes, no register address)

`seq, flags, M1, M2, M3, M4, crc`: seq uint8, flags uint8, Mn int32 little endian, CRC-8 of the first 18 bytes.

| Field | Meaning |
|---|---|
| seq | MCU control period counter (50 ms, wraps at 256) |
| flags bit0 | first frame after the MCU started: its counters start at 0 |
| flags bit1 | stopped by the command timeout |
| M1..M4 | cumulative encoder counts since the MCU start, + = the wheel drove the robot forward |

CRC-8: polynomial 0x07, initial value 0 (`crc8("123456789") = 0xF4`), same code on both sides.

## How the Pi talks to the MCU (`base_node`)

- A timer at `command_rate` (20 Hz) does **two separate I2C transfers** (`smbus2.i2c_rdwr`), each with its own STOP:
  first the write (the current command, zero if `/cmd_vel` is older than `cmd_vel_timeout` 0.6 s),
  then the read (the state frame). No repeated START.
- The kernel I2C timeout is set to `i2c_timeout` (20 ms) and retries to 0, so a hanging transfer does not block.
  Every transfer is in `try/except OSError`: a NACK or timeout skips this period (counted, throttled warning).
- A state frame with a wrong CRC is skipped (counted, throttled warning).
- No valid state for `feedback_timeout` (0.5 s): warning, odometry stops until frames come again.

## Odometry from the counts (`base_node`)

- The first frame after the start (or a frame with flags bit0: the MCU restarted) is only the reference.
- Each next frame with a new seq: count change per wheel (`count_delta`, int32 wrap-around safe)
  x 2*pi / 1536 = wheel angle change -> `wheels_to_body` -> body movement (dx, dy, dyaw) -> pose.
  A failed read loses no distance (the counts are cumulative).
- Speed in `/odom` = movement / (periods x 0.05 s): periods = seq difference (exact MCU time).
  `header.stamp` = Pi time of the read. Twist covariance: `odom_twist_variance` (vx, vy) for the EKF.
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

1. `i2cdetect -y 1` shows `10`. App 1 with `log_level:=debug`: no I2C or CRC warnings, `wheels M1..M4` lines at 20 Hz.
2. Turn each wheel forward by hand: its value in the debug log is positive.
3. Teleop key `i` (forward): all 4 wheels turn forward, all 4 wheel speeds positive.
4. `J` (Shift+j, strafe left), `j` (turn counter-clockwise): the wheels turn as in the formulas above.
5. Release the key: the wheels stop after about 0.6 s (Pi `/cmd_vel` timeout). No MCU timeout warning,
   because the Pi keeps sending (zero) commands. The wheels coast to a stop (no braking).
6. Speed control: v1.1.1 feed-forward table, step tests 0.05-0.45 m/s (see `firmware/README.md`).
7. Top speed: 100 % PWM gives only 12.6-13.1 rad/s (0.50-0.52 m/s, measured 2026-10-07). The firmware clips each
   wheel at 100 % on its own, so the Pi keeps every wheel below `max_wheel_speed` (`config/base.yaml`).
