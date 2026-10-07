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

## Documents to maintain

- `CLAUDE.md` (this file): rules, project facts, decisions, open questions, current stage.
- `README.md`:
  - The full file tree, the purpose of each file, and whether it is empty. Update it in the same commit as the file change.
  - After each app is finished: the exact commands to run it, separately for the **PC terminal** and the **Pi terminal**.
  - How to get new code onto the Pi: in the Pi SSH terminal, `git pull` from GitHub, then build.
- `requirements.yaml`: system versions and every dependency (PC, Pi, firmware). Update it when a dependency is added or removed.
- `docs/mcu_protocol.md`: the Pi <-> MCU protocol. Update it in the same commit as any protocol change.

## Config folder

- Every setting the user may want to change lives in `config/` at the repo root: speed limits, timeouts,
  serial port, chassis parameters, LiDAR, SLAM, Nav2, coverage, teleop start speeds, DDS/network.
- No tunable numbers in code or launch files. Nodes declare parameters without defaults, so a missing value fails loudly.
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

## Packages and versions

- The Pi follows the PC. If PC and Pi versions conflict, the Pi uses the version the PC has.
  Never reinstall or upgrade PC packages without the user's approval. Installing a missing PC package also needs approval.
- The user runs PC `sudo` commands. Claude has no PC sudo password.

## System

| | PC | Pi |
|---|---|---|
| Hardware | - | Raspberry Pi 4B, 8 GB RAM |
| OS | Ubuntu 24.04 | Ubuntu 24.04 |
| Architecture | amd64 | arm64 |
| ROS 2 | Jazzy | Jazzy |
| Python | 3.12 | 3.12 |

- Pi login user: `nala`, hostname `NALA`. The password is not stored in this public repo. Ask the user.
- Reach the Pi: `ssh nala@nala.local` (mDNS).
- Network: the PC shares its connection on USB Ethernet (`enx00e17c6840b1`, 10.42.0.1/24).
  The Pi is on WiFi in that network (10.42.0.x, DHCP). The PC also has eduroam WiFi, which ROS must not use.
- DDS: Cyclone DDS on both (the PC has only Cyclone, no Fast DDS). `ROS_DOMAIN_ID=0`.
  Set by `config/ros_env.sh`; `config/cyclonedds.xml` limits DDS to the 10.42.0.0/24 network.
- PC firewall: ufw is active (input policy DROP). It must allow 10.42.0.0/24, or DDS from the Pi is blocked.
- Install Python libraries with apt (`python3-*`). Ubuntu 24.04 blocks system-wide pip (PEP 668).
- Firmware is built and flashed with Microchip Studio on Windows.

## Hardware facts

**Be careful: half vs full track width and wheelbase, radius vs diameter.**

| Item | Value |
|---|---|
| Drive | 4 mecanum wheels |
| Track width (left-right wheel centers, full) | 0.320 m |
| Half track width | 0.160 m |
| Wheelbase (front-rear axles, full) | 0.260 m |
| Half wheelbase | 0.130 m |
| Wheel **radius** | 0.040 m (diameter 0.080 m) |
| Mecanum rotation term = half track + half wheelbase | 0.290 m |
| Robot outer size (length x width) | 0.410 m x 0.360 m |
| Motors | M1 front-left, M2 rear-left, M3 rear-right, M4 front-right |
| Encoder | 1536 counts per wheel revolution (confirmed) |
| Teleop speeds and limits | 1.0 m/s linear, 1.0 rad/s angular (keys, full stick and Pi limit; `config/teleop.yaml`, `config/base.yaml`) |
| Game controller | Xbox Wireless Controller `C8:3F:26:93:1B:B2`, paired with the **Pi** over Bluetooth (`/dev/input/js0`) |
| LiDAR | RPLIDAR A2M8, USB, driver `rplidar_ros`. Mounted at the chassis center (x = y = 0), facing backward (yaw 180 deg). Height unknown |
| Motor driver MCU | ATmega328PB, 16 MHz, on the Pi GPIO UART `/dev/ttyS0` |
| IMU | None yet. May be added later (model unknown) |

## Reference code (`ref/`, local only)

- `ref/` holds the previous team's repos. It is read-only and not in git. **Never modify anything in `ref/`.**
- `ref/raspberry-pi`: reference only, to learn how the old Pi talked to the MCU.
  **Never import, copy into a build, or depend on any file in it.** It will not be used.
- `ref/motor_driver`: the old firmware of the previous team. `firmware/` now holds the teammate's new version.
- The previous team did not use the measured wheel speeds that the MCU sends back.
  **This project must use them** (wheel odometry).

## Firmware

- A teammate writes and owns the firmware (`firmware/`, v0.1.0 since 2026-10-06). **Claude never edits `firmware/`.**
- Protocol source of truth: `firmware/docs/protocol.md` (and `firmware/CHANGES.md`).
  When it changes, update the Pi code and `docs/mcu_protocol.md` (Pi-side summary) in the same commit.
- Open `firmware/motor_driver_C.atsln` in Microchip Studio. The linker needs `-lprintf_flt`.

## Applications (stages)

| Stage | App | Goal | Status |
|---|---|---|---|
| 1 | Teleop | Drive the chassis with the keyboard or the Xbox controller | Code done (Xbox, fixed speeds, 20 Hz added 2026-10-07). Hardware test pending (motors not connected yet) |
| 2 | Mapping | Teleop + real-time LiDAR SLAM + RViz on the PC + save the map and copy it back to the PC | Not started |
| 3 | Coverage | The user sets a base station on the PC, in the map saved by app 2. The robot starts at the base station, covers the whole house, then returns to it | Not started |

**Current stage: 1** (teleop code done, waiting for the hardware test).

Stage 2 special requirement: the house has many very similar rooms.
SLAM must not jump to a wrong but similar-looking place.

## Architecture (current decisions)

- The repo root is the colcon workspace. ROS 2 packages are in `src/`. Our nodes are Python (rclpy).
- Packages: `nala_description` (URDF), `nala_base` (Pi <-> MCU driver, odometry), `nala_teleop` (PC keyboard teleop),
  `nala_bringup` (launch files and RViz layouts for all apps; installs `config/`), `nala_coverage` (coverage planner, mission, base station picker).
- A package that is still empty has a `COLCON_IGNORE` file. Remove it in the stage that fills the package.
- The Pi runs: `nala_base`, Xbox controller (`joy` + `teleop_twist_joy`: hold LB, left stick = drive + strafe,
  right stick = turn), `rplidar_ros`, `robot_state_publisher`, `slam_toolbox`, Nav2, coverage mission.
- The PC runs: keyboard teleop (`nala_teleop`, own node: IJKL keys, Shift = strafe, the user chose it over WASD;
  no speed keys, speeds only in `config/teleop.yaml`), RViz, base station picker.
- Keyboard and joystick both publish `/cmd_vel` only while used, so they do not fight (no mux).
- PC and Pi talk over the LAN with ROS 2 DDS (Cyclone, `ROS_DOMAIN_ID=0`). Their clocks must be in sync (both use NTP).
- `nala_base` sends the current command to the MCU at a fixed rate (20 Hz), not synced to the MCU feedback (10 Hz)
  (reasons in `docs/mcu_protocol.md`).
- Frames (REP 105): `map -> odom -> base_footprint -> base_link -> laser`.
  Body frame: x forward, y left, z up. Positive angular z = counter-clockwise seen from above.
- Keep pure logic (protocol, kinematics, coverage planning) in ROS-free modules with pytest unit tests.
- Safety: the Pi sends zero if `/cmd_vel` is older than 0.6 s. The firmware stops after 500 ms without a frame.
- Maps are saved in `maps/<name>/` and copied between Pi and PC with `scp`.

### Plan against SLAM pose jumps (tune and verify in stage 2)

- Good wheel odometry from the measured wheel speeds (calibrated).
- Tune `slam_toolbox` to trust odometry: small scan-match search window, higher odometry variance penalties,
  loop closure only close to the odometry estimate (small loop search distance) and with strict match thresholds.
- Stage 3: start localization at the known base station pose. No global relocalization.
- An IMU would help a lot (mecanum wheels slip). Ask the user about adding one if odometry alone is not good enough.

## Open questions (ask before the stage that needs them)

- [1] Hardware test with the new firmware (wheels off the ground): axes, wheel speed signs, PID behavior.
  Checklist in `docs/mcu_protocol.md`.
- [1] Firmware update for 20 Hz commands (teammate, not pushed yet; feedback stays 10 Hz).
  When pushed: read `firmware/docs/protocol.md` and `firmware/CHANGES.md`, sync the Pi code and `docs/mcu_protocol.md`.
- [2] LiDAR mount height (z above the floor or above `base_link`).
- [3] Base station: only a pose, or a physical dock or charger? How exact must the return be?
- [3] Coverage width (tool width) and any coverage pattern requirements.
- [any] IMU model, if one is added.
