# Handover (2026-10-16)

For the next Claude Code session, after the Pi is reinstalled. Read `CLAUDE.md` first: it has the rules,
the hardware facts and the open questions. This file says where the work stopped and what to do next.

## Where we are

- App 1 (teleop) was driven on the robot on 2026-10-07 (firmware v1.1.1, UART text protocol).
- App 2 (mapping): code done. IMU driver and EKF were checked on the Pi on 2026-10-09
  (gyro sign, 50 Hz, EKF at 30 Hz, 360 deg turns by hand). No driving test and no SLAM tuning yet.
- App 3 (coverage) was merged into main on 2026-10-16 (simulation only, not run on the robot yet):
  see "2026-10-16: app 3 merged" below.
- 2026-10-09: the MCU link was moved to I2C (firmware v2.0.0). It never worked. Cause found on 2026-10-12:
  the level shifter on the board is a **TXB** type (the schematic says TXS0104E). A TXB cannot do I2C.
- 2026-10-12 (this session, **code only, nothing was tested on hardware**): the MCU link is UART again,
  with new binary frames (firmware v2.1.0); the IMU moves to a CH340G USB adapter; new battery monitor node.

## Hardware now (user, 2026-10-12)

| Part | Connection |
|---|---|
| MCU (ATmega328PB, 5 V) | UART PD0 RX / PD1 TX <-> level shifter (TXB) <-> Pi GPIO14 TX / GPIO15 RX = `/dev/ttyAMA0`, 38400 baud |
| IMU (WT901C-TTL, 3.3 V) | CH340G USB-to-TTL adapter -> Pi USB = `/dev/ttyIMU` (udev rule). Already set to 115200 baud, 50 Hz |
| Battery monitor (INA260) | Pi I2C bus 1 (GPIO2 SDA / GPIO3 SCL), address not checked (0x40 expected) |
| Battery | 4S2P Li-ion 18650, 3300 mAh per cell (16.8 V full, alarm at 14.0 V, 6.6 Ah) |
| LiDAR (RPLIDAR A2M8) | CP2102 USB adapter, unchanged |
| Xbox controller | Bluetooth to the Pi (Bluetooth on the mini UART) |

Do not put pull-up resistors on lines that go through the TXB level shifter.
The I2C pull-ups for the INA260 must go to 3.3 V (the Pi has 1.8 kOhm on GPIO2/3 already).

## What this session changed (all pushed to GitHub, branch main)

- `firmware/` **v2.1.0**: UART binary frames `0xA5, id, data, crc8`. Compile-checked 2026-10-12, **not flashed.**
- `nala_base`: `mcu_protocol.py` and `base_node.py` for the UART frames (reader thread); new `ina260.py`,
  `battery_node.py` (`/battery_state`, `/battery_low`), `config/battery.yaml`.
- `config/imu.yaml`: port `/dev/ttyIMU`. `config/base.yaml`: `/dev/ttyAMA0`, 38400 baud.
- `scripts/setup_pi.sh` for a fresh Ubuntu Desktop: `openssh-server`, removes `brltty`, stops ModemManager,
  udev rule for `/dev/ttyIMU`. **These new lines were never run.**
- `CLAUDE.md` is tracked in git again. `JY901 uno r3/` is ignored.

Tested on the PC only: unit tests (35), `firmware/tests/uart_frame_model_test.py` (12), `base_node` with a fake
MCU on a pty (garbage bytes, CRC error, dropped frame), `battery_node` with a fake INA260.

## Steps for the user after installing Ubuntu Desktop 24.04 on the Pi

1. At the installation: user name **nala**, computer name **NALA** (so `nala@nala.local` works).
2. Join the WiFi of the PC's shared network (10.42.0.x).
3. In a terminal on the Pi: `sudo apt update && sudo apt install -y openssh-server git`.
4. On the PC: `ssh-keygen -R nala.local`, then `ssh-copy-id nala@nala.local`.
5. Give the Pi sudo password to Claude in the chat (it is never written to a file).

## Phase A: the Pi alone (no MCU, no IMU, no LiDAR, no chassis connected)

**Done 2026-10-12.** `setup_pi.sh` ran without changes; all checks of step 2 passed; build and 35 unit tests
passed; firmware v2.1.0 compiles with 0 errors and 0 warnings (7.0 KB flash, 374 B RAM); RViz runs on the Pi
desktop and the PC receives the Pi's TF. Notes:
- That day the PC and the Pi were on a WiFi 192.168.200.0/24, not on 10.42.0.0/24. The tests used a temporary
  copy of `cyclonedds.xml` with that address (`CYCLONEDDS_URI`); the repo config was not changed (user decision).
- GUI programs started over SSH on the Pi desktop (Wayland): `DISPLAY=:0` and
  `XAUTHORITY=/run/user/1000/.mutter-Xwaylandauth.*`.
- `git`, `curl` and `tmux` were missing on the fresh Pi (`sudo apt install -y git curl tmux` first).

Goal: everything installed and built, and RViz works on the Pi desktop. Claude does this over SSH:

1. `git clone https://github.com/philipfei/NALA.git ~/NALA`, `bash ~/NALA/scripts/setup_pi.sh`
   (about 15 minutes; installs ROS, RViz and all tools), then reboot.
   If the script fails on Ubuntu Desktop: fix it on the PC, push, pull, run again.
2. Pi system: `/dev/ttyAMA0` exists; `sudo vcgencmd measure_clock core` = 250 MHz; `hciconfig` shows hci0;
   `id` has dialout, input, i2c; `/etc/udev/rules.d/99-nala.rules` exists; `dpkg -l brltty` = not installed.
3. Build: `cd ~/NALA && colcon build --symlink-install --base-paths src`; the unit tests pass
   (`python3 -m pytest -p no:cacheprovider src/nala_base/test src/nala_teleop/test`).
4. Firmware v2.1.0: compile-check on the Pi (command in `firmware/README.md` section 5; it needs the ATmega DFP
   download). Fix errors and warnings, push. The teammate can flash it later.
5. **RViz test** on the Pi screen (the user looks at the screen): on the Pi desktop (`DISPLAY=:0`, user logged in),
   start the robot model (`ros2 launch nala_description description.launch.py robot_config:=$HOME/NALA/config/robot.yaml`)
   and RViz with the mapping layout (`ros2 launch nala_bringup app2_slam_pc.launch.py`).
   Without sensors there is no map or scan: set the fixed frame to `base_footprint` and check that the robot outline
   and the frames `base_footprint`, `base_link`, `laser`, `imu_link` are shown. Also check RViz on the PC
   (`app2_slam_pc.launch.py` on the PC) sees the Pi's topics over the network.
6. **Stop here and wait for the user's "ok"** before anything is connected.

## Phase B: after the user's "ok": connect the hardware one part at a time

Status 2026-10-14 (network back on 10.42.0.0/24, the Pi is 10.42.0.154):
- IMU: done. `/dev/ttyIMU` OK; 0x51 + 0x52 at 50 Hz, no bad frames (read only, `setup_imu.py` not run again);
  `/imu` 50 Hz; turned 90 deg counter-clockwise by hand -> gyro z positive, integral +90 deg.
- LiDAR: done. `/scan` 13.7 Hz, 720 points; an object behind the robot shows at laser angle 0 deg (mount yaw
  180 deg OK); the PC receives `/scan`. Closest range about 0.18 m (A2M8 minimum about 0.15 m).
- Battery (INA260): **on hold** (user). Address 0x4F (A0, A1 to SCL), set in `config/battery.yaml`. Nothing
  answers on I2C bus 1, though GPIO2/3 are in I2C mode and idle high; the IC may be broken.
  `battery_node` only warns when it cannot read, so app 1 still starts.
- MCU link: done (2026-10-14, firmware v2.1.0 flashed, motor power and encoders disconnected).
  100 state frames in 5 s, 0 CRC errors, seq steps of 1; zero commands clear the boot flag; `app1_teleop_pi`
  runs, `/odom` 20.0 Hz, no warnings (only the INA260 one). Before it worked: a loose UART cable, and pull-up
  resistors on the TXB lines (removed; a TXB cannot pull a line low against them). With v1.1.1 and later
  v2.1.0 frames then arrived; the exact last fix was not confirmed.
- Motors + encoders: done (2026-10-15, motor power on, wheels in the air, v2.1.0). Forward, back, strafe left
  and turn counter-clockwise at 0.05 m/s: every wheel has the right sign (M1..M4 = + + + +, - - - -, - + - +,
  - - + +). Speed control: at 0.15 m/s good enough (target in ~0.5 s, 25-30 % overshoot); at 0.05 m/s bad
  (0.6-1 s late start, 2-3x overshoot, stick-slip, once all wheels stopped for 0.6 s while commanded).
  Raw test with a pyserial script, base_node stopped. Not confirmed yet by the user: that the wheels turned the
  right way physically (the encoders only show their own sign).
- Xbox: not paired yet (fresh Pi). A scan for 3 min found nothing; the user pairs it later.
- Boot splash: `scripts/setup_splash.sh` ran (theme `nala` in the initramfs). Not seen yet (no display).
- Open: firmware v2.2.0 (flash, then tune), Xbox, chassis on the floor, app 2.

## 2026-10-15: firmware v2.2.0 setting mode (code done, **not flashed**)

- The controller parameters (kp, ki, i_limit, accel_limit = new target ramp, feed-forward table) can be read and
  set over the UART while stopped, and saved in the MCU EEPROM (user decision: EEPROM, not `config/`).
  Protocol: `firmware/README.md` section 1. Pi tool: `scripts/tune_motors.py` (README, "Tune the wheel speed controller").
- Tested on the PC: unit tests (48), firmware model tests (uart 22, timeout 13, param 37), the tool against a fake
  MCU on a pty (show/set/save/reload/defaults, NaN/Inf rejected, busy while driving, slow save, Ctrl+C restore,
  restore failure exits in 0.4 s, sweep --floor, ff), base_node with the v2.2.0 frames (`/odom` 20 Hz).
  Compile check on the Pi: 0 errors, 0 warnings.
- Defaults = the v2.1.0 values, so the behaviour is the same until parameters are set (only: a command starts
  at the next control step, <= 50 ms later).

### Status 2026-10-15 afternoon

- v2.2.0 flashed: `tune_motors.py show` works (defaults, EEPROM empty). Nothing saved yet.
- First measurements (in the air): dead zone 11.6-13.6 % PWM, big differences between the wheels (front M1/M4 need
  more power at low speed). Best so far with the shared table: kp 4, ki 20, accel_limit 20 (overshoot 6-24 % at
  0.20 m/s, 30-57 % on the rear wheels at 0.05-0.10 m/s). Higher kp (8) makes the start overshoot worse.
- The user fixed a mechanical problem at M2. After that **M3 sticks**: no motion at 15 and 21.5 % PWM, then
  0.7 s late start and 73 % overshoot. Waiting for a mechanical check of M3.
- Without wheels (all 4 removed): M3 is normal (its wheel was the problem). The motors differ in gain up to 2x at
  low PWM (M4 weakest, M3 strongest), so one shared FF table overshoots on M2/M3 (40-83 % at 0.05 m/s). Next:
  firmware v2.3.0 (one ff_pwm row per motor) is built (2026-10-15), waiting for the flash; then tune again with the wheels on.
  Steady-state check (3 s settle, 2 s measured per PWM): same differences, so they are not a transient.
  For 5 rad/s: M1 17.3 %, M2 14.5 %, M3 13.6 %, M4 19.9 % PWM.
  The MCU runs the defaults (nothing saved in the EEPROM).
- SD card full from vc4 kernel warnings (core_freq=250): fixed with core_freq=500 (CLAUDE.md open questions).

### Next steps (in this order)

1. (Done 2026-10-15) v2.3.0 flashed (second try; the first flash still had v2.2.0). Test without wheels:
   speed points 0, 1, 2, 3.5, 5, 6.6, 8.1, 9.4, 11.5, 13 rad/s (better than the default points at the low end),
   two `ff --apply` rounds -> all motors within ~6 % of the table from 2 rad/s up, dead zones 7.2-7.9 %.
   Open loop the motors then reach the target WITHOUT overshoot; closed loop the overshoot comes from the
   integral during the rise (0.3-0.7 s) and the ~50-75 ms measurement delay. Best without wheels: kp 2, ki 8,
   i_limit 3, accel_limit 0 (overshoot 7-9 % at 0.30 m/s, 13-18 % at 0.20, 15-31 % at 0.05-0.10; steady error <= 3 %).
   Not saved (wheels and floor change friction and load). The MCU runs the defaults again.
2. (Done 2026-10-15) Tuned on the floor and saved in the EEPROM: kp 2, ki 8, i_limit 10, accel_limit 0, new speed
   points, per-motor rows (values and method: `firmware/README.md` section 6, v2.3.1). Open-loop `ff --floor` is not
   usable on its own: the robot turns. Overshoot at 0.05-0.10 m/s is still 18-50 % (whole-robot stick-slip).
3. Optional later: firmware change against the windup during the rise (integrate only near the target), if the
   driving (app 1/2, Nav2) needs a smoother start.
4. (Done 2026-10-16) Driving test on the floor: forward and strafe odometry exact; turn in place: wheel odometry
   360 deg, real ~315-330 deg (so the turn comes only from the IMU, as designed). Stop accuracy 3 mm with a planned
   deceleration of 0.1 m/s^2 (details: CLAUDE.md open questions). Next: the IMU yaw in a 360 deg turn (app 2).
5. (Done) The final values are in `config.h` (v2.3.1) and the docs.
6. Xbox pairing (README "Setup, Pi"), drive with the controller, then app 2 (SLAM test).

The user connects the parts. Claude tests each one before the next:

1. IMU (CH340G): `ls -l /dev/ttyIMU`; `python3 scripts/setup_imu.py --from 115200` must report 50 Hz, 0x51 + 0x52.
   If `/dev/ttyIMU` is missing: `lsusb` (CH340 = 1a86:7523?), `dpkg -l brltty`.
2. Battery: `i2cdetect -y 1` (the INA260 should be the only device, at 0x40; else change `config/battery.yaml`).
   `ros2 run nala_base battery_node --ros-args --params-file config/battery.yaml`, `ros2 topic echo /battery_state`:
   plausible voltage (12-16.8 V); the current must be negative while the robot runs (else `current_sign: -1.0`).
3. Pair the Xbox controller again (README, "Setup, Pi").
4. MCU (after the teammate flashed v2.1.0): `ros2 launch nala_bringup app1_teleop_pi.launch.py log_level:=debug`:
   no "No valid state" and no CRC warnings; `wheels M1..M4` lines at 20 Hz; turn a wheel forward by hand: positive.
   If nothing arrives: check the wiring (Pi TX GPIO14 -> MCU PD0 RX, MCU PD1 TX -> Pi RX GPIO15) with pyserial
   on `/dev/ttyAMA0` at 38400: the MCU sends `a5 02 ...` 21-byte frames every 50 ms.
5. LiDAR: `rplidar_node` gives `/scan` at about 14 Hz.
6. Chassis with motors: Xbox drive, strafe + turn; a 1 m drive against a tape measure (odometry scale);
   step tests if the speed control looks different.
7. App 2: `app2_slam_pi.launch.py` + RViz; EKF without TF warnings; 360 deg turn; map a room;
   `scripts/save_map.sh`. Then SLAM tuning in the real house.

Tricks that worked before:
- sudo over non-interactive SSH: send the password on stdin (`read -r PW`), export it in that one remote shell,
  and use a temporary `sudo -A` wrapper with an askpass script that prints the variable. Remove them afterwards.
- `git push` from the PC: HTTPS has no saved login; use `git push git@github.com:philipfei/NALA.git main`.
- Long-running launch on the Pi for tests: start it in `tmux` (Ctrl+C then works like in a terminal).
- The user moves the robot or connects the motors when asked. Ask before anything moves.

## Known open points

- Firmware v2.1.0 was written without a compiler. Expect small compile errors.
- Gyro bias: about -5 deg/min of yaw drift at rest, not compensated (the user said: leave it for now).
- EKF variances (`odom_twist_variance`, `angular_velocity_variance`) are start values.
- LiDAR height and IMU mount position are placeholders in `config/robot.yaml`.
- Battery percentage is a rough linear value; INA260 address and current sign are not checked.
- Feed-forward table: 20-27 % overshoot at 0.10-0.15 m/s (lower two table points when the firmware is touched).
- App 3: AMCL values were tuned in simulation only; the coverage sensor values are placeholders.
- `firmware/docs/state_machine.svg` still shows the old text protocol.

## 2026-10-16: app 3 merged

`stage3-coverage` is merged into main and adapted to the current main:
- `app3_coverage_pi.launch.py` starts base_node (`publish_tf: False`), battery_node, imu_node and the EKF like app 2.
- Acceleration limits from the floor test: 0.1 m/s^2 and 0.35 rad/s^2 (`config/coverage.yaml`, `nav2_params.yaml`).
- `scripts/setup_pi_screen.sh` keeps GNOME: it only installs the app 3 packages and adds two autostart entries
  (`start_panel.sh`, `screen_always_on.sh`). Never run on the Pi yet.

Next steps on the Pi (in this order; ask before anything moves):

1. `git pull`, `bash ~/NALA/scripts/setup_pi_screen.sh` (sudo), build, then the app 3 tests in the isolated
   sim environment (README "Tests (PC)", the same on the Pi).
2. Reboot with the user watching the screen: the panel must come up full-screen on the Waveshare (C), touch must
   land where tapped, the screen must stay on. If the panel does not come up: `journalctl --user -b | grep -i
   start_panel` (the script names the failed display check).
3. App 2: map the real test area, `save_map.sh`. On the screen: base station, a short plan.
4. Robot on the floor (user present): "First run on the robot" in the README (localization, costmaps, a short
   plan, an obstacle, the return).

## Prompt for the next conversation

Copy this into a new Claude Code session in the repo folder:

```
Read CLAUDE.md and docs/handover.md first.

The Pi is reinstalled with Ubuntu Desktop 24.04 (user nala, hostname NALA), it is on the PC's network and
openssh-server is installed. Right now ONLY the Pi is connected: no MCU, no IMU, no LiDAR, no chassis.
The Pi sudo password is: <give it here>.

Do "Phase A" in docs/handover.md: clone the repo on the Pi, run scripts/setup_pi.sh, reboot, check the system,
build, run the tests, compile-check firmware v2.1.0, then start RViz on the Pi desktop with the robot model so I
can look at the screen. Then stop and wait for my "ok".
After my "ok", do "Phase B" one part at a time: tell me which part to connect next, test it, then the next.
Ask me before anything moves.
```
