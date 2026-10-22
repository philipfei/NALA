# CLAUDE.md

Behavioral guidelines to reduce common LLM coding mistakes. Merge with project-specific instructions as needed.

**Tradeoff:** These guidelines bias toward caution over speed. For trivial tasks, use judgment.

## 1. Think Before Coding

**Don't assume. Don't hide confusion. Surface tradeoffs.**

Before implementing:
- State your assumptions explicitly. If uncertain, ask.
- If multiple interpretations exist, present them - don't pick silently.
- If a simpler approach exists, say so. Push back when warranted.
- If something is unclear, stop. Name what's confusing. Ask.

## 2. Simplicity First

**Minimum code that solves the problem. Nothing speculative.**

- No features beyond what was asked.
- No abstractions for single-use code.
- No "flexibility" or "configurability" that wasn't requested.
- No error handling for impossible scenarios.
- If you write 200 lines and it could be 50, rewrite it.

Ask yourself: "Would a senior engineer say this is overcomplicated?" If yes, simplify.

## 3. Surgical Changes

**Touch only what you must. Clean up only your own mess.**

When editing existing code:
- Don't "improve" adjacent code, comments, or formatting.
- Don't refactor things that aren't broken.
- Match existing style, even if you'd do it differently.
- If you notice unrelated dead code, mention it - don't delete it.

When your changes create orphans:
- Remove imports/variables/functions that YOUR changes made unused.
- Don't remove pre-existing dead code unless asked.

The test: Every changed line should trace directly to the user's request.

## 4. Goal-Driven Execution

**Define success criteria. Loop until verified.**

Transform tasks into verifiable goals:
- "Add validation" → "Write tests for invalid inputs, then make them pass"
- "Fix the bug" → "Write a test that reproduces it, then make it pass"
- "Refactor X" → "Ensure tests pass before and after"

For multi-step tasks, state a brief plan:
```
1. [Step] → verify: [check]
2. [Step] → verify: [check]
3. [Step] → verify: [check]
```

Strong success criteria let you loop independently. Weak criteria ("make it work") require constant clarification.

---

**These guidelines are working if:** fewer unnecessary changes in diffs, fewer rewrites due to overcomplication, and clarifying questions come before implementation rather than after mistakes.

---

# NALA Project Instructions

Everything above this line is the andrej-karpathy-skills section. **Never edit it. Always follow it.**
Everything below is project-specific and must be kept up to date.

## Role and communication

- Act as an experienced robotics architect.
- Talk to the user in **Chinese** in chat. Write every file (code, comments, docs, commit messages) in **simple English**.
- Never guess missing or unclear information (hardware, wiring, requirements, numbers). Ask the user.
- At the end of every conversation turn, send the user "喵" (user, 2026-10-09).

## Documents to maintain

- `CLAUDE.md` (this file): rules, project facts, decisions, open questions, current stage.
- `README.md`:
  - The full file tree, the purpose of each file, and whether it is empty. Update it in the same commit as the file change.
  - After each app is finished: the exact commands to run it, separately for the **PC terminal** and the **Pi terminal**.
  - How to get new code onto the Pi: in the Pi SSH terminal, `git pull` from GitHub, then build.
- `requirements.yaml`: system versions and every dependency (PC, Pi, firmware). Update it when a dependency is added or removed.
- `docs/mcu_protocol.md`: the Pi <-> MCU protocol. Update it in the same commit as any protocol change.
- `docs/handover.md`: where the work stopped and the next steps on the hardware. Update it when a session ends with open hardware work.

## Config folder

- Every setting the user may want to change lives in `config/` at the repo root: speed limits, timeouts,
  serial port, chassis parameters, LiDAR, SLAM, Nav2, coverage, teleop start speeds, DDS/network.
- No tunable numbers in code or launch files. Nodes declare parameters without defaults, so a missing value fails loudly.
- **Exception (user decision 2026-10-15):** the MCU speed controller parameters (kp, ki, i_limit, accel_limit,
  feed-forward table) are kept in the **MCU EEPROM** (firmware v2.2.0 setting mode), set with `scripts/tune_motors.py`.
  Their defaults are in `firmware/motor_driver_C/config.h`. After tuning, write the final values into `config.h` too.
- `nala_bringup` installs `config/` into its share folder. Launch files read it from there.
  With `--symlink-install`, an edited file works after `git pull` without a rebuild (a new file needs a rebuild).
- Config files are edited on the PC and pushed, like code.

## Git and deployment

- Remote: <https://github.com/philipfei/NALA> (public), branch `main`. Claude may pull and push.
- Code is changed on the development machine and pushed to GitHub.
  The Pi only pulls (`git pull` in the Pi SSH terminal). Never edit code on the Pi.
- Never commit: `ref/`, `build/`, `install/`, `log/`, map files in `maps/`, passwords.
- Line endings are LF for all files (see `.gitattributes`), because the code runs on Linux.
- Claude may SSH to the Pi to run setup, `git pull`, build, tests and the apps (code still only comes from GitHub).
- Build with `colcon build --symlink-install --base-paths src`. `--base-paths src` is needed on the PC,
  because `ref/` contains old ROS 1 / CMake packages.
- The PC repo is at `/home/philip/Documents/NALA` (ext4). Building on the PC is allowed (user, 2026-10-07).
  Never build on an NTFS drive: a `colcon build` on NTFS crashed the ntfs3 kernel driver (2026-10-06).
  Pure-logic tests on the PC:
  `PYTHONPATH=src/nala_base:src/nala_teleop python3 -m pytest -p no:cacheprovider src/nala_base/test src/nala_teleop/test`.
  App 3 tests (after the build, `source tools/sim/env.sh`: domain 77, localhost only; never `config/ros_env.sh`,
  whose domain 0 reaches the robot):
  `python3 -m pytest -q -p no:cacheprovider src/nala_coverage/test src/nala_panel/test` (about 3 minutes).

## Packages and versions

- The Pi follows the PC. If PC and Pi versions conflict, the Pi uses the version the PC has.
  Never reinstall or upgrade PC packages without the user's approval. Installing a missing PC package also needs approval.
- The user runs PC `sudo` commands. Claude has no PC sudo password.

## System

| | PC | Pi |
|---|---|---|
| Hardware | - | Raspberry Pi 4B, 8 GB RAM |
| OS | Ubuntu 24.04 | Ubuntu 24.04 (Desktop after the reinstall of 2026-10-12; Server before) |
| Architecture | amd64 | arm64 |
| ROS 2 | Jazzy | Jazzy |
| Python | 3.12 | 3.12 |

- Pi login user: `nala`, hostname `NALA`. The password is not stored in this public repo. Ask the user.
- Reach the Pi: `ssh nala@nala.local` (mDNS).
- Network: the PC shares its connection on USB Ethernet (`enx00e17c6840b1`, 10.42.0.1/24).
  The Pi is on WiFi in that network (10.42.0.x, DHCP). The PC also has eduroam WiFi, which ROS must not use.
- DDS: Cyclone DDS on both (the PC has only Cyclone, no Fast DDS). `ROS_DOMAIN_ID=0`.
  The simulation (`tools/sim/env.sh`) uses Cyclone too, in domain 77 on loopback only (`tools/sim/cyclonedds_sim.xml`).
  Set by `config/ros_env.sh`; `config/cyclonedds.xml` limits DDS to the 10.42.0.0/24 network.
- PC firewall: ufw is active (input policy DROP). It must allow 10.42.0.0/24, or DDS from the Pi is blocked.
- Install Python libraries with apt (`python3-*`). Ubuntu 24.04 blocks system-wide pip (PEP 668).
- Firmware is built with Microchip Studio on Windows and flashed with avrdude (ArduinoISP), by the teammate.
  Claude compile-checks it on the Pi (`gcc-avr`, `avr-libc`; command in `firmware/README.md`).

## Hardware facts

**Be careful: half vs full track width and wheelbase, radius vs diameter.**

| Item | Value |
|---|---|
| Drive | 4 mecanum wheels |
| Track width (left-right wheel centers, full) | 0.320 m |
| Half track width | 0.160 m |
| Wheelbase (front-rear axles, full) | **0.335 m** (user, 2026-10-16). In `base.yaml` and firmware v2.3.2 `config.h` since 2026-10-19 (v2.3.2 not flashed yet; until then the MCU still uses 0.260 m) |
| Half wheelbase | **0.1675 m** |
| Wheel **radius** | 0.040 m (diameter 0.080 m) |
| Mecanum rotation term = half track + half wheelbase | **0.3275 m** with the 335 mm wheelbase (0.290 m before 2026-10-19: wheel odometry turn +12.9 %, commanded turns ~11 % slow; likely the main cause of the "10-14 % too high" wheel turn rate of 2026-10-16) |
| Robot outer size (length x width) | **0.650 m x 0.360 m, not centred** (`config/robot.yaml` `outline`: front 0.4425, back 0.2075, left/right 0.18 m; since 2026-10-19). User, 2026-10-16/19: body 0.415 m x 0.360 m, **centred on the wheel centre** (turning centre), plus a rod 0.235 m forward (tip at x = +0.4425 m) = 0.650 m total length; so the turning centre is no longer the centre of the whole outline. Track width still 0.320 m. See `docs/handover.md` section A |
| Motors | M1 front-left, M2 rear-left, M3 rear-right, M4 front-right |
| Encoder | 1536 counts per wheel revolution (confirmed: 1.17 m driven = 1.17 m odometry, 2026-10-07) |
| Teleop speeds | 0.45 m/s linear (= motor limit), 0.5 rad/s angular (keys and full stick; `config/teleop.yaml`; 1.0 until 2026-10-16, slower is better for SLAM) |
| Motor top speed | Measured 2026-10-07 at 100 % PWM: 12.6-13.1 rad/s = 0.50-0.52 m/s rim speed (M4 slowest), the same with wheels in the air and on the floor. Firmware assumed 25 rad/s. `max_wheel_speed: 0.45` in `config/base.yaml` |
| Game controller | Xbox Wireless Controller `C8:3F:26:93:1B:B2`, paired with the **Pi** over Bluetooth (`/dev/input/js0`; Bluetooth runs on the mini UART since 2026-10-09, `dtoverlay=miniuart-bt`). In `joy` (SDL): LB = button 4, B = button 1, left stick x/y = axes 0/1, right stick x = axis 3, triggers = axes 2/5 (1.0 released). Checked 2026-10-07, B 2026-10-16. The left stick rests at up to ~0.12 (raw): joy deadzone 0.15 |
| LiDAR | RPLIDAR A2M8 (firmware 1.28), USB CP2102 adapter (`/dev/serial/by-id/usb-Silicon_Labs_CP2102_...`), 115200 baud, driver `rplidar_ros` (`rplidar_composition`). Sensitivity mode: 16 m, about 7900 points/s, about 14 scans/s. Mounted at the chassis center (x = y = 0), facing backward (yaw 180 deg, checked 2026-10-07 with objects behind and right of the robot). Height not measured (placeholder 0.20 m in `config/robot.yaml`) |
| Motor driver MCU | ATmega328PB, 16 MHz, 5 V. **UART** (user, 2026-10-12): MCU PD0 RX / PD1 TX <-> level shifter <-> Pi GPIO14 TX / GPIO15 RX = PL011 `/dev/ttyAMA0`, 38400 baud. (2026-10-09 to 12: I2C on PC4/PC5, never worked) |
| Level shifter | The schematic says TXS0104E, but the part on the board is a **TXB** type (found 2026-10-12). A TXB works for UART but **cannot do I2C** (it oscillates against pull-ups; seen as random ACKs and a ~60 kHz square wave on SCL/SDA). No pull-ups on lines through it |
| IMU | WitMotion WT901C-TTL, 3.3 V, flat with the chip side up, on a **CH340G USB-to-TTL adapter** (USB id 1a86:7523) = `/dev/ttyIMU` (udev rule from `setup_pi.sh`). Already set to 115200 baud / 50 Hz / acc + gyro (2026-10-09). Position on the robot not measured (placeholder in `config/robot.yaml`). Example code (Arduino) in `JY901 uno r3/` (local, ignored by git) |
| Battery | 4S2P Li-ion 18650, 3300 mAh per cell: 16.8 V full (4.2 V/cell), 6.6 Ah. Low-voltage alarm at 14.4 V (3.6 V/cell, user, 2026-10-21), 0 % at 12.8 V (3.2 V/cell). 16.1 V measured 2026-10-15 |
| Motor supply | Always 12 V from a DC-DC converter (user, 2026-10-15): the motor speed for a PWM does not depend on the battery voltage |
| Battery monitor | TI **INA219** at **0x40** on the Pi I2C bus 1 (GPIO2 SDA / GPIO3 SCL, directly, not through the level shifter), voltage only (IN+ and IN- shorted, no current; user, 2026-10-21). Works: 16.44 V read on 2026-10-21 (`docs/battery.md`). The old INA260 (0x4F) never answered and is dead |
| Pi pins | GPIO14/15 = UART to the MCU, GPIO2/3 = I2C to the INA219, USB = LiDAR (CP2102) and IMU (CH340G). Two devices must never share a UART (two TX outputs would drive one RX pin) |
| Robot screen (app 3) | Waveshare 7 inch HDMI LCD (C), 1024 x 600, capacitive touch over USB, on the Pi HDMI port. The Pi runs headed for app 3 (user, 2026-10-08) |

## Reference code (`ref/`, local only)

- `ref/` holds the previous team's repos. It is read-only and not in git. **Never modify anything in `ref/`.**
- `ref/raspberry-pi`: reference only, to learn how the old Pi talked to the MCU.
  **Never import, copy into a build, or depend on any file in it.** It will not be used.
- `ref/motor_driver`: the old firmware of the previous team. `firmware/` now holds the teammate's new version.
- The previous team did not use the measured wheel speeds that the MCU sends back.
  **This project must use them** (wheel odometry).

## Firmware

- A teammate owns the firmware (`firmware/`) and builds and flashes it. **Claude may edit `firmware/`** (user, 2026-10-07).
  The PC has no avr-gcc (the user chose not to install it); Claude compile-checks on the Pi (avr-gcc 7.3).
- Version in the repo: **v2.3.1** (2026-10-15, by Claude; v2.3.0 + the tuned values as `config.h` defaults, no code change;
  the MCU runs v2.3.0 with the same values in its EEPROM). v2.3.0 = v2.2.0 + one feed-forward PWM row per motor
  (`ff_m1_pwm0..9` .. `ff_m4_pwm0..9`, shared `ff_speed0..9`; 54 parameters, EEPROM layout 2). Compile-checked on the
  Pi (0 errors, 0 warnings, 9.7 KB flash, 970 B RAM, 219 B EEPROM). Firmware and Pi code must match.
  v2.3.0 flashed 2026-10-15 (second try; the first flash still had v2.2.0). Test without wheels:
  speed points 0, 1, 2, 3.5, 5, 6.6, 8.1, 9.4, 11.5, 13 rad/s (better than the default points at the low end),
  two `ff --apply` rounds -> all motors within ~6 % of the table from 2 rad/s up, dead zones 7.2-7.9 %.
  Open loop the motors then reach the target WITHOUT overshoot; closed loop the overshoot comes from the
  integral during the rise (0.3-0.7 s) and the ~50-75 ms measurement delay. Best without wheels: kp 2, ki 8,
  i_limit 3, accel_limit 0 (overshoot 7-9 % at 0.30 m/s, 13-18 % at 0.20, 15-31 % at 0.05-0.10; steady error <= 3 %).
  Then **tuned on the floor and saved in the EEPROM** (2026-10-15, user permission, 2 m free, watched): kp 2, ki 8,
  i_limit 10, accel_limit 0; FF rows = floor load + 0.5 x the motor's air difference (open-loop FF on the floor makes
  the robot turn). Values: `firmware/README.md` section 6 (v2.3.1) = `config.h`. Floor result: 5-23 % overshoot at
  0.20-0.30 m/s, 18-50 % at 0.05-0.10 m/s (whole-robot stick-slip). Left: integral windup during the rise.
- v2.2.0 setting mode: parameter request `0xA5 0x03 op idx float32 crc` (9 bytes, like a command), reply
  `0xA5 0x04 op idx status float32 crc`. ops get / set / save (EEPROM, CRC-16 block) / defaults / reload. Every op
  except get only while stopped ("busy"). Target ramp `accel_limit` (default 0 = off). No D term (user decision).
  State flags bit2 = parameters are the EEPROM block, bit3 = FF table invalid (drive commands ignored).
  EEPROM survives a reflash (EESAVE fuse). Pi tool: `scripts/tune_motors.py` (show, set, save, reload, defaults,
  step, sweep [--floor], ff [--apply]); it needs base_node stopped. `ff` measures each motor in steady state
  (from rest, 3 s settle, 2 s measured) and suggests (or `--apply` writes) one PWM row per motor.
- v2.1.0 flashed 2026-10-14: UART link checked (20 Hz frames, no CRC errors, `/odom` 20 Hz).
  2026-10-15 (wheels in the air): wheel numbering, encoder signs and kinematics signs correct in forward, back,
  strafe and turn. 0.15 m/s (3.75 rad/s): target in ~0.5 s, 25-30 % overshoot. 0.05 m/s (1.25 rad/s): starts
  0.6-1 s late, then 2-3x overshoot and stick-slip (FF table too low near the dead zone, integral winds up).
- Protocol source of truth: `firmware/README.md` (protocol, parameters, changes per version).
- v2.1.0: UART 38400, binary frames `0xA5, id, data, crc8` (CRC-8 poly 0x07 of id + data). Command (id 0x01):
  vx, vy, w as int16 LE in mm/s, mrad/s. State (id 0x02, every 50 ms): seq, flags, 4 x int32 cumulative counts.
  RX state machine IDLE -> HEADER_ACK -> PAYLOAD (a 0xA5 in the data never restarts a frame). No text on the UART.
- v2.0.0 (I2C slave) was dropped: it could never work with the TXB level shifter.
- v1.1.0: control loop and feedback 20 Hz, cumulative encoder counts, MCU command timeout 200 ms, stop = coast. v1.1.1: feed-forward from a measured speed -> PWM table (`FF_*`):
  the motor curve is not linear (dead zone ~8 %, 30 % -> 8 rad/s, 100 % -> 13 rad/s); the single factor 6.5 of
  v1.1.0 overshot by 30-90 % (step tests 2026-10-07). M2/M3 direction pins
  are not inverted in software: the M2/M3 supply cables are wired opposite to M1/M4 in the hardware (nothing to do on the Pi).
  When it changes, update the Pi code and `docs/mcu_protocol.md` (Pi-side summary) in the same commit.
- Open `firmware/motor_driver_C.atsln` in Microchip Studio. (`-lprintf_flt` is not needed since v2.0.0 but harmless.)

## Applications (stages)

| Stage | App | Goal | Status |
|---|---|---|---|
| 1 | Teleop | Drive the chassis with the keyboard or the Xbox controller | Done and driven on the robot (2026-10-07, firmware v1.1.1). v2.3.0: speed control tuned on the floor and saved (2026-10-15). Driving test with the Xbox controller 2026-10-16 (open questions) |
| 2 | Mapping | Teleop + real-time LiDAR SLAM + RViz on the PC + save the map and copy it back to the PC | Code done. IMU + EKF checked (2026-10-09, 360 deg turn 2026-10-16). First map of one room 2026-10-16: clean. SLAM in the real house pending |
| 3 | Coverage | On the robot screen: choose a map from app 2, set the base station, plan a path (start at the base station or at another start point; optionally only an area), choose a saved plan and drive it. The robot starts at the base station, covers, then returns to it | Code done 2026-10-08 (coverage stack from a teammate's PC, adapted to this repo). Tested in simulation only (`tools/sim`, panel end to end). Merged into main 2026-10-16. Not tested on the robot |

**Current stage: 3** (coverage; user, 2026-10-16). App 3 is merged and set up on the Pi. Robot test 2026-10-16
(room "Test", run 8): **98.9 % covered** after the supervisor/CPU fixes. Open: the changed robot dimensions (rod in
front, non-centred outline) and long stops at corners (Nav2 RPP turn-in-place deadlock). Details, data, test
history and the next prompt: `docs/handover.md`.
App 2 is still open in the real house (SLAM with several similar rooms, EKF check while driving).

Stage 2 special requirement: the house has many very similar rooms.
SLAM must not jump to a wrong but similar-looking place.

## Architecture (current decisions)

- The repo root is the colcon workspace. ROS 2 packages are in `src/`. Our nodes are Python (rclpy).
- Packages: `nala_description` (URDF), `nala_base` (Pi <-> MCU driver, odometry), `nala_teleop` (PC keyboard teleop, Pi Xbox teleop),
  `nala_bringup` (launch files and RViz layouts for all apps; installs `config/`), `nala_coverage` (app 3 mission,
  velocity gate, coverage meter, base station), `nala_panel` (robot screen for app 3).
  The coverage planner is `coverage_tool/` (Python, no ROS, not a ROS package), run as `plan_cli.py`.
- A package that is still empty has a `COLCON_IGNORE` file. Remove it in the stage that fills the package.
- The Pi runs: `nala_base` (`base_node`, `battery_node`, `imu_node`), robot_localization EKF (apps 2 and 3), Xbox controller (`joy` + `nala_teleop` `xbox_teleop`: hold LB, left stick = drive + strafe,
  right stick = turn), `rplidar_ros`, `robot_state_publisher`, `slam_toolbox` (app 2), AMCL + Nav2 + coverage
  mission + velocity gate (app 3), the robot screen panel (`nala_panel`, app 3).
- The PC runs: keyboard teleop (`nala_teleop`, own node: IJKL keys, Shift = strafe, the user chose it over WASD;
  no speed keys, speeds only in `config/teleop.yaml`), RViz, and optionally for app 3 the base station picker and
  `coverage_tool` (planning on the PC).
- Keyboard and joystick both publish `/cmd_vel` only while used, so they do not fight (no mux).
- Xbox teleop is our own node `xbox_teleop` (user decision 2026-10-16, replaced `teleop_twist_joy`): after an LB press
  the command is zero until both sticks were centred once (no jump with a held stick); LB release = one zero;
  no `/joy` for `joy_timeout` 0.2 s while LB is held = one zero, then LB must be pressed again. joy_node deadzone
  0.15 (user, 2026-10-16; 0.05 failed: the left stick rests at up to ~0.12 raw, so it never read exactly 0 and crept). Same axes and scales as before. Logic in `xbox.py` (no ROS, unit tests).
- PC and Pi talk over the LAN with ROS 2 DDS (Cyclone, `ROS_DOMAIN_ID=0`). Their clocks must be in sync (both use NTP).
- `nala_base` sends the current command to the MCU at a fixed rate (20 Hz), not synced to the MCU feedback (20 Hz)
  (reasons in `docs/mcu_protocol.md`).
- Frames (REP 105): `map -> odom -> base_footprint -> base_link -> laser`.
  Body frame: x forward, y left, z up. Positive angular z = counter-clockwise seen from above.
  `base_footprint` and `base_link` are the same pose (on the floor under the chassis center).
  `nala_base` publishes `/odom` and TF `odom -> base_footprint`; `robot_state_publisher` the rest of the robot;
  `slam_toolbox` `map -> odom`.
- Odometry: from the cumulative encoder counts in the MCU state frames (firmware v2.1.0, UART, 20 Hz). A timer
  writes the command frame at 20 Hz; a reader thread blocks on the UART and cuts the stream into frames
  (`split_frames`: sync, id, CRC; bad frames are counted and skipped). Count change -> wheel angle -> body
  movement -> pose. A lost frame loses no distance. `/odom` speed = movement / (seq periods x 0.05 s, exact MCU
  time); stamp = Pi time of arrival (no artificial time stamps: clock drift would make jumps).
  Tested on the PC with a fake MCU on a pty (garbage, CRC error, dropped frame).
- Battery: `battery_node` reads the INA219 (I2C, voltage only) at 1 Hz -> `/battery_state` (percentage is rough: linear in the
  voltage, the 18650 curve is not) and `/battery_low` (true below 14.4 V), for a later "drive home on low battery".
  Started by the app 1 and app 3 launch files. The robot screen panel reads the INA219 itself (battery icon on every
  screen, 20 s moving average; user, 2026-10-21), because battery_node runs only with an app.
- IMU + EKF (apps 2 and 3, `docs/imu_ekf.md`, user decision 2026-10-09): robot_localization EKF fuses `/odom` vx, vy
  and **only the IMU gyro z** for yaw (no wheel turn rate: mecanum slip). No IMU orientation (magnetometer is
  distorted indoors), no acceleration. TF odom -> base_footprint: base_node in app 1 (`publish_tf`), the EKF
  in app 2 (`odom_tf:=false`) and app 3 (`publish_tf: False` in its launch). The EKF output topic is
  `/odometry/filtered`; `/odom` stays the wheel odometry (Nav2 and the app 3 supervisor read it for speed and
  standstill only; user decision 2026-10-16: no `odom_raw` rename). If the IMU fails the EKF has no yaw source:
  imu_node reports an error.
- The URDF reads its numbers from `config/robot.yaml` (xacro `load_yaml`), passed in as launch arg `robot_config`.
- Maps: `scripts/save_map.sh <name>` on the Pi saves `map.pgm/.yaml` (map_saver_cli) and
  `map.posegraph/.data` (slam_toolbox serialize_map). The PC copies the folder with `scp`.
- Keep pure logic (protocol, kinematics, coverage planning) in ROS-free modules with pytest unit tests.
- Speeds: input speeds (keys, stick) only in `config/teleop.yaml`; one motor limit `max_wheel_speed` (wheel rim m/s)
  in `config/base.yaml`. A mecanum command needs |vx| + |vy| + k*|wz| on its fastest wheel; above the limit
  the Pi scales vx, vy, wz by the same factor. Reason: the firmware clips each wheel at 100 % PWM on its own,
  which cut the turn when strafing and turning at full stick (2026-10-07). No body speed limits (user decision).
- Safety: the Pi sends zero if `/cmd_vel` is older than 0.6 s. The firmware stops after 200 ms without a frame.
- Maps are saved in `maps/<name>/` and copied between Pi and PC with `scp`.

### App 3 (coverage) decisions

- Everything runs on the Pi. The robot screen (`nala_panel`, PyQt5) is the normal way to use it; the PC is optional
  (RViz, planning with `coverage_tool`). The panel starts at login in the existing Ubuntu Desktop session
  (GNOME on Wayland, autologin of `nala`; user, 2026-10-16: no Xfce/LightDM): `scripts/setup_pi_screen.sh` adds
  two GNOME autostart entries, `start_panel.sh` (PyQt5 on XWayland: `QT_QPA_PLATFORM=xcb`; waits up to 10 s for
  the X display; over SSH it takes DISPLAY/XAUTHORITY of the desktop session from `systemctl --user
  show-environment`: the Pi has two X sockets, X0 and X1, and the session uses :0, checked 2026-10-16) and
  `screen_always_on.sh` (gsettings: no blanking, dimming, lock or suspend; needs the session bus, so at login).
- Map library: `maps/<name>/map.yaml/.pgm` (app 2), `base_station.yaml`, `plans/<time>.yaml/.png`, `runs/<time>/`.
- Base station = a pose only, no dock or charger (user, 2026-10-08). The robot starts and ends there.
  Coverage may start at another start point; the robot still stands on the base station and Nav2 drives it
  to the start first (user, 2026-10-08). A plan records its base station; the robot refuses a plan made for
  another one (`PLAN_BASE_MISMATCH`).
- Localization in app 3: AMCL on `map.yaml` (not slam_toolbox), initial pose = the base station, sent by the
  supervisor with a spread of 0.1 m / 10 deg. AMCL has no global relocalization, but app 3 does not assume the
  robot starts on the base station (user, 2026-10-19): `locator_node` (`locate.py`) searches the whole map with the
  scan every 2 s while no task drives -> `/coverage/located` (ON_BASE / BACKWARDS / NOT_ON_BASE + pose). The screen
  draws the robot there and says "not on the base station" / "the wrong way round"; Start needs ON_BASE; when the
  robot arrives on the base station the supervisor sends AMCL the base station pose again (settings `locator`). TF odom -> base_footprint comes from the
  EKF (wheel vx, vy + IMU turn rate), as in app 2; `hardware:=false` (tools/sim) leaves out base_node, battery,
  IMU, EKF and LiDAR (fake_nala publishes `/odom` and the TF).
- Nav2 without BT navigator and behavior server: the supervisor calls `ComputePathToPose` and `FollowPath`
  (Regulated Pure Pursuit) itself. Autodrive is forward only; `linear.y` is always 0.
- Velocity gate (`nala_coverage/gate_node.py`) is the only `/cmd_vel` publisher in app 3: Nav2 -> `/cmd_vel_nav`,
  manual -> `/cmd_vel_remote` (`coverage_teleop` or the panel's arrows, only while paused). App 3 does not start
  `xbox_teleop`; another `/cmd_vel` publisher stops the robot (`VELOCITY_GRAPH_CONFLICT`).
- Xbox B pauses the task (user decision 2026-10-16): app 3 starts `joy_node` + `xbox_pause` (nala_coverage);
  on the B rising edge (`pause_button: 1`, `config/teleop.yaml`, checked 2026-10-16) it calls
  `/coverage/pause` with `call_async` (no waiting in callbacks; no service = warning only; one open request).
- Planner (coverage_tool, 2026-10-19, user: cover a corner fully when the robot is there): aimed goals are also
  offered at the ends of route pieces (turn on the spot there), and `LocalSearch.reaim` re-places every aimed goal
  after ordering. Room Test plan `2026-10-19_151921`: 77 s planned (was 87 s), each corner visited once.
- Plan check ("Checking the plan", supervisor.plan_preview), 2026-10-19: reachable floor via connected areas of the
  safe cells (same result, ~150x faster); the coverable floor is saved next to the plan (`<plan>.check.npz`) with a
  fingerprint of map, plan, settings, reachable floor and area, and used again while it matches. Pi 4, hospital
  map (68 m2): first check ~19 s, then ~1 s. AMCL is restarted when the locator's pose and AMCL differ by more than
  locator.base_tolerance_m/deg (not against the base station pose: that restarted AMCL every 2 s and dropped the
  plan check when the robot stood 10-15 cm off).
- Corner turns (user, 2026-10-19): plan pieces up to planning.turn_merge_length_m (the aims' 5 cm moves) are no
  Nav2 goals; plan_file.merge_turns joins them with the stop before into one 'turn' target, and the supervisor
  (state TURN) turns the robot on the spot through all its headings itself, via /cmd_vel_nav and the velocity gate,
  slowing only before the last heading in one direction. No motion for no_motion_s skips the rest of the turn
  (TURN_SKIPPED), never a pause. Simulation room Test: 1 standstill > 0.4 s in the whole run.
  Only corner sweeps ('aim' in the drive_path 'kinds' the planner exports since 2026-10-20) become turns; another
  short piece, e.g. a 5 cm step between two lanes, joins the pieces before and after it into one piece: no turn
  and no stop (room Test: the bottom lane follows the south wall's pixel staircase; the real wall is straight).
  Plans without 'kinds' merge every short piece as before.
- Full coverage first (user decision 2026-10-20): the planner keeps covering every reachable cell, also the few
  against the walls (edge_missed_weight stays 10000). Dropping the corner sweeps (room Test 58 s instead of 77 s,
  99.8 % instead of 100 %) was rejected.
- Reversing out of pockets (user, 2026-10-20: "it should only drive backwards on already covered area, so that we
  are sure it doesn't drive into a contamination"). Planner: drive-ins are pieces of the route optimiser
  (coverage_tool/coverage/spurs.py, coverage_tool/CLAUDE.md): straight in from a start with turning room, the same
  way back out, chosen by the cost function like every piece; pocket floor counts as floor to cover. User decisions
  2026-10-20: no post-processing of a finished path (the first version added drive-ins afterwards and then merged,
  trimmed and straightened the path: rejected); only as deep as the sensor model needs; drive-in time at the robot's
  drive-in speed (half); start anywhere with turning room (robot radius + 3 cm). test_basement2: 100 %, 2 drive-ins.
  Robot (user 2026-10-20: the drive-in is a normal part of the path): driving in is a normal plan piece that Nav2
  follows (plan_file: piece kind 'plan' with drive_in = its start P; supervisor.follow checks the stretch from P with
  the outline + planning_padding_m instead of the turning circle). A stop inside the pocket (Nav2 failure, timeout,
  pause) ends the drive-in there (SPUR_SHORTENED) and the robot backs out. Backing out ('reverse', state SPUR) is the
  supervisor's: on odometry along the line put into odom when it starts (steering on AMCL zigzagged), at
  spur.speed_m_s (0.12 m/s = normal speed since 2026-10-21, user; LiDAR stop distance behind spur.clear_m 0.10 m),
  via /cmd_vel_reverse (the gate passes it only while NAV owns the robot, only <= 0, <= spur.speed_m_s, only from
  coverage_supervisor). Before it every cell under the bare outline must be measured (or under the robot at P), else
  a pause (SPUR_REVERSE_NOT_MEASURED); LiDAR behind: SPUR_REVERSE_BLOCKED. (Contamination ends the task, below.)
  It ends at or just past P (it turns there). Earlier versions (same day) drove in with the supervisor too and
  needed a turn-drive-turn onto P after Nav2's 5 cm point goal: many random turns; rejected.
  A piece Nav2 finished up to rejoin_max_m off its end (base: locator.base_tolerance_m) counts as done
  (ENDPOINT_MISSED_ACCEPTED). The meter's denominator includes the floor the drive-ins reach.
- Passages (user, 2026-10-21: a room on the RDG map has turning room inside but a 0.70 m entrance, narrower than the
  1.06 m turning circle): coverage_tool/coverage/passages.py joins such areas with straight passages that the outline
  can drive forwards both ways (margin as drive-ins); the planner routes through them like any connection and plans
  the way out when a route would end inside (user choices). Drive path kind 'passage': plan_file and supervisor.follow
  check it with the outline + planning_padding_m; the plan check counts the areas behind passages as reachable
  (resume inside works). If a Nav2-planned detour would have to go through a passage, the robot pauses (user choice).
- Contamination (user, 2026-10-22: "I don't want it to drive around the contamination"): the robot stops and the task
  ends when radiation.alarm_readings of the last alarm_window readings (2 of 3, user: "any 2 in a short time") reach
  the operator's threshold, at any time in a task (driving, paused, background, way home; user). Threshold set on the
  robot screen (Maps -> Settings, for every map, maps/contamination_threshold.yaml, latched /radiation/threshold):
  counts/s or a factor x the background (user: the operator chooses); the background is always measured at the start
  (user). Every reading counts, also without a trusted pose (user; the spot is then "approximate"). Stops like a pause
  (user: zero at once, ~2 cm coasting; not a 0.1 m/s^2 ramp), then FAILED / CONTAMINATION_FOUND: no resume, no
  driving home, the robot stays (user: end task only). Screen: full-screen red warning with readings, threshold,
  time, sensor position until Acknowledge (user), then the spot on the map; the task report keeps it (Results tab).
  The keepout zone, the Nav2 keepout layer and every "drive around / skip contaminated floor" rule were removed
  (user). radiation.enabled is still false (no scintillator driver). Simulation (Test room, one source): stopped
  0.3 m before the source, no contact.
- Wall distance (user, 2026-10-22: the RDG route was wobbly, with many stops along diagonal walls at the safety
  limit): no fixed inset and no "walls first" rule; the optimiser's shift move (coverage_tool/coverage/localsearch.py)
  tries each wall stretch further from the wall (up to where the sensor still reaches the wall floor) and the cost
  decides. Every reachable cell up to the wall stays covered. Stops cost 3 s extra (stop_penalty_s 4 s) and the
  planner uses the robot's speeds (motion in config/coverage.yaml). RDG plan 2026-10-22_104734: 140 stops (was 180),
  100 % of the reachable floor, 42.5 min estimated (real speeds).
- CPU (robot 2026-10-20: load 15 on the Pi 4, SCAN_STALE / AMCL_STALE because the supervisor handled messages too
  late; the supervisor logs its timing every health.timing_log_period_s). 75-80 % of the supervisor's and the gate's
  CPU was rclpy's default executor (wait set rebuilt per message). All NALA Python nodes now use rclpy's EventsExecutor
  (health.events_executor; ros_common.make_executor / run_until_stopped): blocking spin() in a background thread,
  the main thread waits for its own Ctrl-C/SIGTERM flag (rclpy initialized with SignalHandlerOptions.NO: rclpy's own
  handler made destroy_node() wait for ever), then the node's own shutdown steps run single-threaded on a default
  executor as before (the gate's stop burst). spin_once() in a loop with the events executor cost MORE CPU than the
  default executor. Simulation: supervisor 19 -> 8 %, gate 12 -> 4 %, meter 8 -> 4 %; every node exits cleanly.
  Meter: trace messages built once per point (cached), covered cells no longer in /coverage/measurement (the
  supervisor reads /coverage/covered). Panel: same executor switch, map repaints only when its data changed.
- Apps 1 and 2 on the robot screen (user, 2026-10-19): buttons "Drive (app 1)" and "Make a map (app 2)" on the map
  list (`nala_panel/apps.py`, `ui.AppScreen`). They start the same launch files as over ssh (log in `log/`), STOP /
  Back stop them with Ctrl-C. Driving only with the Xbox controller (user: no on-screen arrows). App 2 shows the live
  /map with the robot (TF map -> base_footprint) and saves with scripts/save_map.sh; the name is typed on an
  on-screen keyboard (user), letters/digits/-/_ only, never an existing map. A start is refused while another app's
  nodes (base_node, slam_toolbox, coverage_supervisor, velocity gate) run.
- Rejoining the plan (user, 2026-10-20: reconnect to the actual path smoothly, few turns on the spot): when a piece
  starts and the robot is beside it (more than plan_join_tolerance_m from its start, up to rejoin_max_m 0.3 m from
  the piece), one path from the robot straight to the point rejoin_ahead_m 0.4 m further along the piece and on
  along it (REJOINED event); corner turns are done where the robot stands. Otherwise (or after a detour around an
  obstacle, or when that straight line is not safe) a planned connection as before. Robot 2026-10-20: a pause 12-16 cm
  beside the lane after a corner turn led to a planned connection with turns on the spot and 35 s of pauses.
- Results tab (user, 2026-10-20): all previous scans of a map (library.list_runs: run folders with a task or
  coverage report), newest first, scrollable; tap one: heatmap, hotspots, covered floor and driven path (load_run).
- Scan screen (user, 2026-10-20): it switches to the big driving view by itself when a scan drives; after "Show
  map" the map stays until "Big view" or a new scan (it used to switch back after every pause or state change).
- Map screen: no zoom buttons (they did not work on the robot screen; pinch and drag do; user 2026-10-19).
- Auto-resume (user, 2026-10-19: rather localize longer than stop with an error): a pause for a localization or
  late-data reason (`AUTO_RESUME` in supervisor.py) ends by itself once trust is back (settle_s), within
  `recovery.auto_resume_s` (60 s); the screen says "Finding the position again - continues by itself". Not for STOP,
  operator pause, blocked paths or LOCALIZATION_JUMP (mirror image risk in symmetric rooms).
- Localization trust (user decision 2026-10-19): AMCL sigma above position_sigma_m / heading_sigma_deg stops the
  robot only if the live scan does not fit the map (matched_ratio) or the sigma is `uncertain_hard_factor` (2) x the
  limit. AMCL's sigma was wrong both ways on the robot; the scan check caught every real error.
- Robot outline for coverage: from `config/robot.yaml` (`outline`: front, back, left, right of base_link; not centred:
  rod in front, as wide as the body, below the LiDAR; user 2026-10-19), never copied. Collision radius = farthest
  corner + `planning_padding_m` (0.478 + 0.02 = 0.498 m); plans use 0.53 m. The tests use a fixed test robot (the old
  centred 410 x 360 mm box), so they do not depend on the real outline.
- The panel runs `plan_cli.py` as a child process: coverage_tool's package is called `coverage`, the same
  name as the system package `python3-coverage`.
- Speeds (`motion` in `config/coverage.yaml`): 0.12 m/s, 0.4 rad/s (start values), slowest 0.04 m/s near a goal
  (the MCU protocol has 1 mm/s steps). Acceleration limits = floor test 2026-10-16 (user decision): 0.1 m/s^2
  (velocity gate) and 0.35 rad/s^2 (velocity gate). RPP `max_angular_accel` = `controller_angular_accel_rad_s2`
  3.0: RPP limits its turn command to the measured rate +- accel x 0.05 s, and 0.35 could neither start a turn
  (0.0175 rad/s, below the breakaway) nor slow down in time (extra turns). The gate lifts a Nav2 turn on the spot
  from standstill only to `turn_start_rad_s: 0.3` (ramped; lifting while turning made the robot overshoot and turn
  extra rounds). User decisions 2026-10-19, handover section B.
  RPP has no `acc_lim_*`/`decel_lim_*` (DWB parameters): not added (user decision 2026-10-16).
- Global costmap robot_radius = collision radius + half a cell: NavFn measures from obstacle cell centres, the
  supervisor checks planner paths exactly against cell edges (else detours and the way back can be refused).

### Plan against SLAM pose jumps (tune and verify in stage 2)

- Good wheel odometry from the encoder counts (scale checked 2026-10-07), heading from the IMU gyro (EKF, 2026-10-09).
- Tune `slam_toolbox` (`config/slam_mapping.yaml`) to trust odometry: small scan-match search window,
  no response expansion, loop closure only close to the estimate (small loop search distance and window)
  and with strict match thresholds, Huber loss in the solver. Start values are set (2026-10-07), not tuned yet.
  Note: in Karto a SMALLER `distance/angle_variance_penalty` means trusting the odometry MORE.
- Stage 3: start localization at the known base station pose. No global relocalization.

## Open questions (ask before the stage that needs them)

- [1] Firmware v1.1.0 checked 2026-10-07: counts at 20 Hz, odometry 1.10 m = tape 1.10 m.
  v1.1.1 step tests (wheels in the air, 0.05-0.45 m/s): 90 % of the target in 0.2-0.4 s (6.5: overshoot 30-90 %),
  overshoot 1-27 % (most <= 15 %), settled within ~1 s. Xbox strafe + turn on the floor works (2026-10-07).
- [2] LiDAR mount height above the floor (placeholder 0.20 m in `config/robot.yaml`).
- [2] Driving test 2026-10-16 (floor, firmware v2.3.0 tuned values, app 1, /odom):
  - Forward 1.053 m by odometry = 1.053 m tape (scale exact, ended straight). Strafe left 1.022 m by odometry
    = ~1.022 m tape (difference below the tape resolution). Wheel odometry is good for vx, vy.
  - Turn in place 360.3 deg by wheel odometry, real only ~315-330 deg (user, tape line): the wheel turn rate is
    10-14 % too high (roller slip). Confirms the design: the EKF (app 2) takes only vx, vy from /odom and the turn
    rate only from the IMU gyro (`config/ekf.yaml`). IMU check (app 2): stopped at 360.7 deg by the EKF yaw
    (wheels: 408.3 deg); the user saw the robot back on the tape line: the IMU yaw is right.
  - Stopping (user choice: plan A = slow down before the goal, no MCU brake): a stop command = coasting
    (~0.38 m/s^2, 5 cm from 0.2 m/s). While the command is above 0 the robot follows only about 0.1-0.14 m/s^2
    of deceleration (the FF keeps pushing). With a planned deceleration of 0.1 m/s^2 and a 0.03 m/s creep the robot
    stopped 3 mm from the goal (0.3 deg in a turn with 0.35 rad/s^2). Use these limits for Nav2 (stage 3).
  - First SLAM test 2026-10-16 (Xbox, one room ~5.8 x 2.2 m): clean map, single walls, user: everything looks good.
    `save_map.sh Test` and `scp` to the PC work (4 files; maps/Test is git-ignored).
  - Then: SLAM in the real house with several similar rooms (no jumps), and an EKF check while driving.
- [2] Checked 2026-10-09 (old Pi installation): miniuart-bt + core_freq=250 -> `/dev/ttyAMA0`, core clock fixed
  250 MHz (since 2026-10-15: 500 MHz, see the SD card note), Bluetooth and the Xbox controller work on the mini UART; `setup_imu.py` -> 115200 baud, 50 Hz, only
  0x51/0x52; `/imu` 50 Hz; gyro sign correct (counter-clockwise = positive); EKF 30 Hz without TF warnings,
  360 deg turns by hand tracked.
- [1] **After the Pi reinstall (Ubuntu Desktop 24.04): follow `docs/handover.md`.** Phase A done (2026-10-12).
  Phase B: IMU, LiDAR, MCU link, motors + encoders in the air done (2026-10-14/15). INA260 on hold.
  Open: speed control tuning (firmware v2.2.0, after the teammate flashes it), Xbox pairing (the user does it later),
  chassis on the floor (only with the user's permission), app 2.
- [1] Pi SD card was 99 % full (2026-10-15): `core_freq=250` (from `setup_pi.sh`) is below the 500 MHz the Pi 4
  display driver (vc4 KMS) needs, so the kernel logged 4 warnings with stack traces per screen update (~3 GB/day
  in syslog + kern.log). Fixed: `core_freq=500` + `core_freq_min=500` (still a fixed clock for the mini UART);
  0 warnings after the reboot, Bluetooth and `/dev/ttyAMA0` OK. Logs emptied, journal 200 MB, apt cache cleaned (4.3 GB free).
- [1] Motor test 2026-10-15 (v2.2.0, in the air): wheel directions confirmed by the user. Dead zone 11.6-13.6 % PWM
  (not 8 % as in v1.1.1), the wheels differ a lot at low PWM. After the user fixed M2 mechanically, **M3 does not
  turn below ~25 % PWM** and starts 0.7 s late with 73 % overshoot: check M3 mechanically before tuning.
  Test without wheels (user removed all 4, 2026-10-15): M3 normal again (the problem was in its wheel). No dead zone
  above 9.5 % PWM and no breakaway jump without wheels: the wheels add the stick-slip. The motors differ in gain:
  at 15 % PWM M1 3.5, M2 5.1, M3 5.6, M4 2.8 rad/s; M4 needs ~1.4x the PWM of M3 for the same mid speed.
  With the shared FF table: overshoot M2/M3 40-83 % at 0.05 m/s, 28 % at 0.30 m/s; M4 7-24 %. kp 4 / ki 20 /
  accel_limit 20 settles faster (0.4-0.9 s, steady error < 1 %) but does not remove the start overshoot.
  Steady state checked (open loop, each PWM from rest, 3 s settle, then 2 s measured; std <= 0.11 rad/s): the
  differences are real, not a transient. PWM needed for the same speed (no wheels):
  5 rad/s: M1 17.3, M2 14.5, M3 13.6, M4 19.9 %; 9 rad/s: 29.1, 24.6, 23.9, 36.8 %. M4 needs ~1.5x the PWM of M3
  between 3.5 and 11 rad/s. Front motors (M1, M4) are weaker than the rear ones (M2, M3).
  Firmware v2.3.0 with one ff_pwm row per motor: flashed 2026-10-15 (results in the Firmware section).
- [1] Pi boot splash: `scripts/setup_splash.sh` ran 2026-10-15 (Plymouth theme `nala` with `firmware/prayer.jpeg`).
  Shown correctly at the reboot of 2026-10-15 (monitor connected). The headless boot waits ~31 s in `plymouth-quit-wait` for the GNOME
  autologin session: normal, SSH works earlier.
- [2] IMU gyro bias at rest: about -0.0015 rad/s = -5 deg/min of yaw drift while standing (2026-10-09).
  Not compensated yet (idea: average the gyro while standing still at start, or while the wheels do not move).
- [2] IMU mount position and x direction (placeholders in `config/robot.yaml`; do not change the turn rate).
- [2] EKF variances (`odom_twist_variance`, `angular_velocity_variance`) are start values: tune while driving.
- [3] Base station: answered, a pose only (user, 2026-10-08). Still open: how exact must the return be?
- [3] Coverage sensor (scintillator): position and size unknown. `sensor_offset_m: 0.26` and
  `coverage_disk_radius_m: 0.6` in `config/coverage.yaml` are placeholders, and the sensor is assumed not to stick
  out of the outline. Whether the rod carries it: not known yet (user, 2026-10-19). Change them and plan again when known.
- [3] First robot tests of app 3: localization at the base station, costmaps, a short plan, an obstacle, the
  return; coverage speeds (`motion` in `config/coverage.yaml`: 0.12 m/s, 0.4 rad/s are start values; the
  acceleration limits 0.1 m/s^2 / 0.35 rad/s^2 come from the floor test). Battery check stays off
  (`battery.enabled: false`) until it is turned on (the INA219 works since 2026-10-21).
- [3] Obstacles lower or higher than the LiDAR scan plane are not seen (no bumper).
- [3] Pi set up for app 3 on 2026-10-16: `setup_pi_screen.sh` ran, after a reboot the panel starts full-screen by
  itself (GNOME autostart), touch and picture checked by the user, the screen stays on. Xbox on the Pi without
  base_node: LB with a held stick = 0, after centring 0.45 m/s, B = button 1. Still open: planning time on the Pi 4.
- [3] Tests on the Pi 4: 111 of 113 app 3 tests pass. `test_supervisor_launch_no_motion_and_continuous_trust` and
  `test_older_plan_is_split_at_corners_and_followed_exactly` (test_ros.py) always fail there and fail the same way
  before the merge adaptation: fixed waits (e.g. 3.4 s for localization) are too short for the Pi. On the PC the
  second one fails now and then, also before the merge. `coverage_tool/tests/regression.py --quick`: `bumpy` misses
  2 cells, the same on the original branch. Teammate's code, not changed.
- [3] Robot dimensions changed (user, 2026-10-16): wheelbase 0.335 m, body 0.415 x 0.360 m, rod 0.235 m in front
  (0.650 m total), turning centre not at the outline centre; track 0.320 m unchanged. The odometry scale check
  (rear-wheel travel) stays valid. Body is centred on the wheel centre (user); rod: as wide as the body, below the LiDAR
  (user, 2026-10-19); whether it carries the coverage sensor: not known yet. Done 2026-10-19: non-centred outline in
  robot.yaml / params.py / coverage_tool / URDF / sim, half_wheelbase 0.1675 (base.yaml + firmware v2.3.2), new plan
  `maps/Test/plans/2026-10-19_104154` (the old plans are refused). Open: flash v2.3.2, then the 360 deg turn test.
- [3] Long stops at corners: Nav2 RPP turn-in-place from standstill is limited to 0.0175 rad/s (measured rate +
  max_angular_accel x 0.05 s), below the breakaway -> deadlock until the progress checker gives up (15 s).
  2026-10-19: RPP max_angular_accel 3.0 helped but 0.15 rad/s still stalled once; now the velocity gate lifts a
  Nav2 turn on the spot to turn_start_rad_s 0.3 and RPP is back at 0.35 (user decisions). AMCL alpha1 and alpha4
  0.05 -> 0.03 (user decision; 0.02/0.01 made AMCL keep a pose 0.6 m off with sigma 0.017 m). Map changes node
  slowed (scan_period_s 2.0, beam_step 4): Nav2's controller and the EKF missed their rates (CPU). Turns can still
  overshoot ~20 deg: the gate's 0.35 rad/s^2 slow-down after RPP hands over to driving (not changed).
- [3] Robot placement at the base station (2026-10-19): room Test is almost symmetric under a 180 deg turn around
  the base station, so a robot put on it backwards is only partly caught (AMCL starts at the base station heading
  +-10 deg, no global relocalization). The user placed it backwards at least once. Open: mark the base station
  direction on the floor; maybe a start check that the scan fits much better at the base heading than turned 180 deg.
- [3] DDS without the PC network (2026-10-20): with none of cyclonedds.xml's networks up, Cyclone could not create
  a node and the panel did not start after a boot without WiFi. Now ros_env.sh uses config/cyclonedds_local.xml
  (loopback) then, start_panel.sh waits up to 30 s for the WiFi first and fixes its choice for everything it starts
  (NALA_CYCLONEDDS_URI). Not loopback + network together: that disables multicast and the PC no longer finds the
  robot (tested). Still to check on the robot: a boot with the laptop network off.
