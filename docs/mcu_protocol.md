# Pi <-> MCU Protocol

This describes the **baseline** firmware (`firmware/`, an unchanged copy of `ref/motor_driver`,
which is what is flashed on the MCU now). It was learned from:

- `firmware/motor_driver_C/main.c` and `USART.c` (MCU side)
- `ref/raspberry-pi/src/motor_comms/src/motor_uart_comms.py` (old Pi side, reference only)

Items marked **(verify)** are not confirmed on hardware yet. Check them in stage 1.
When the firmware protocol changes, update this file in the same commit.

## Link

| Item | Value |
|---|---|
| MCU | ATmega328PB, 16 MHz |
| Port on MCU | USART0 |
| Port on Pi | GPIO UART (old code used `/dev/ttyS0`) **(verify device name on Ubuntu 24.04)** |
| Settings | 9600 baud, 8 data bits, no parity, 1 stop bit, no flow control |

## Pi -> MCU: velocity command (5 bytes, binary)

| Byte | Type | Meaning |
|---|---|---|
| 0 | `0x80` | Start flag 1 |
| 1 | `0x86` | Start flag 2 |
| 2 | int8 | `Vx_mcu` in 0.01 m/s |
| 3 | int8 | `Vy_mcu` in 0.01 m/s |
| 4 | int8 | `Wz_mcu` in 0.01 rad/s |

- Range: -128..127, so at most about +-1.27 m/s and +-1.27 rad/s. Resolution 0.01.
- No checksum, no reply other than the echo line below.
- The MCU keeps the last command until a new one arrives. There is **no timeout**.

### MCU axes vs ROS axes

The old Pi code converted a ROS `geometry_msgs/Twist` like this:

```
Vx_mcu = int(-100 * linear.y)
Vy_mcu = int( 100 * linear.x)
Wz_mcu = int( 100 * angular.z)
```

So MCU `+Vy` = robot forward and MCU `+Vx` = robot right (ROS `-y`).
A comment in that old file says "X = left", which does not match the sign in the code.
**(verify both linear signs and the sign of Wz)**

## MCU inverse kinematics (firmware `calc_angular_speed`)

Firmware constants: `R = 0.04`, `W = 0.165`, `H = 0.15` (so `W + H = 0.315` m).

```
M1 = (Vx - Vy - (W+H) * w) / R
M2 = (Vx + Vy - (W+H) * w) / R
M3 = (Vx - Vy + (W+H) * w) / R
M4 = (Vx + Vy + (W+H) * w) / R      [rad/s, MCU axes]
```

Each motor then gets a start PWM of `4 * M` (clamped to +-100), and a PID loop per wheel
(Kp 0.74, Ki 3.7, Kd 0.0644) tracks `M1..M4` with the measured wheel speeds.

Which motor (M1..M4) sits at which corner, and its positive direction, is **unknown (verify)**.

## MCU -> Pi: text lines

All lines end with `\n`. Numbers are printed with `%f` (works because the project links `-lprintf_flt`).

| Line | Format | When |
|---|---|---|
| Boot | `test` | Once after reset |
| Command echo | `Vx Vy w` (3 floats, space separated; m/s, m/s, rad/s) | After every command. Sent from inside the UART RX interrupt |
| **Measured wheel speeds** | `M1 \t M2 \t M3 \t M4` (4 floats, separated by space-tab-space; rad/s) | Every main loop, about 10 Hz (estimate) |

Parsing rule: split on whitespace. 4 floats = wheel speeds. 3 floats = echo. Anything else: ignore.

The measured speeds use the same sign convention as `M1..M4` above (the PID only works if they match).
They are computed as `counts / 1536 / 0.010 * 6.28` from encoder edges counted in a busy-wait window.
1536 counts per wheel revolution is what the firmware assumes **(verify)**.

The previous team never used these measured speeds. **This project uses them for wheel odometry.**

## Pi forward kinematics (for odometry, derived from the equations above)

Measured wheel speeds are real wheel speeds, so use the **real** geometry here:
`k = half_track + half_wheelbase = 0.160 + 0.130 = 0.290` m, `R = 0.040` m.

```
Vx_mcu = R/4     * ( M1 + M2 + M3 + M4)
Vy_mcu = R/4     * (-M1 + M2 - M3 + M4)
w      = R/(4*k) * (-M1 - M2 + M3 + M4)

ROS:  linear.x = Vy_mcu,  linear.y = -Vx_mcu,  angular.z = w      (verify signs)
```

## Known issues in the baseline firmware (fix in stage 1)

1. **Commands are always zero.** The RX interrupt calls `memset(inputValues, 0, ...)` before it
   uses `inputValues` in `calc_angular_speed`.
2. **Wrong geometry.** `W + H = 0.315` m, but half track + half wheelbase = 0.290 m.
   The robot turns about 8.6 % faster than commanded.
3. **No command timeout.** If the Pi program stops, the robot keeps the last speed.
4. **Echo is sent inside the RX interrupt.** About 27 characters at 9600 baud block the interrupt
   for about 28 ms. Bytes that arrive in that time can be lost.
5. **Speed measuring window is not exact.** It is 1000 loops of `_delay_ms(0.01)` plus loop overhead,
   so the real window is longer than the 0.010 s in the formula. Measured speeds read too high by an
   unknown factor. Calibrate it, or measure with a hardware timer.
6. **Low feedback rate.** About 20 ms measuring + about 45 ms to send about 45 characters at 9600 baud
   + 20 ms delay, so roughly 10 Hz (estimate).
7. **No checksum** on commands.
