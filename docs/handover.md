# Handover (2026-10-16, evening): app 3 merged, first robot tests

For the next person and their Claude Code session. Read `CLAUDE.md` first (rules, hardware facts, decisions,
open questions), then this file. It says what the session of 2026-10-16 did, where the work stopped, and what
to do next. Older handover text (Pi reinstall, phase A/B, firmware tuning) is in the git history:
`git log -p -- docs/handover.md`. Everything in it is done or copied into `CLAUDE.md`.

## Where we are

- Apps 1 and 2 work on the robot (driving test and a first SLAM map of the room "Test", 2026-10-16).
- App 3 (coverage) is merged into `main` and set up on the Pi. On the robot it **starts to drive, but pauses
  after about 30 cm** because messages arrive too late on the busy Pi 4 (details below). This is the open problem.
- Current stage: 3 (user decision).

## What the session of 2026-10-16 did (all on `main`, pushed)

In order (`git log --oneline c2417cf..HEAD`):

| Commit | What |
|---|---|
| `de8480a` | Merge of `stage3-coverage` (teammate's app 3, made 2026-10-08 against an old main). Git conflicts only in docs |
| `c54dd96` | App 3 adapted to the current main: IMU + EKF + battery_node in `app3_coverage_pi.launch.py` (base_node `publish_tf: False`, like app 2); accel limits 0.1 m/s^2 / 0.35 rad/s^2 from the floor test (`config/coverage.yaml`, RPP `max_angular_accel` in `config/nav2_params.yaml`); `tools/sim/fake_nala.py` uses `to_int16` (protocol is 1 mm/s now); `setup_pi_screen.sh` keeps GNOME (no Xfce/LightDM), GNOME autostart for the panel and `screen_always_on.sh`; `start_panel.sh` runs PyQt5 on XWayland and waits for the display |
| `573110f` | Own Xbox node `nala_teleop xbox_teleop` replaces `teleop_twist_joy` (apps 1, 2): after an LB press the robot only drives after both sticks were centred once; LB release = one zero; no `/joy` for 0.2 s = stop |
| `0909bb8` | App 3: Xbox **B** pauses the task (`joy_node` + `nala_coverage xbox_pause` -> `/coverage/pause`). The controller cannot drive in app 3 |
| `7dd8c5a` | Simulation (`tools/sim/env.sh`) on Cyclone DDS (was Fast DDS), loopback only (`tools/sim/cyclonedds_sim.xml`), domain 77 |
| `ea5ad61` | `start_panel.sh` over SSH: uses the GNOME session's DISPLAY (the Pi has X0 and X1, the session is :0) |
| `060de61` | joy deadzone 0.15: the left stick rests at up to ~0.12 (measured); 0.05 made the robot creep |
| `27e6817` | Docs: Pi set up for app 3 (panel autostart, touch and controller checked) |
| `71afd5d` | Supervisor fix 1: a scan newer than the odom TF is not a TF error (use the newest TF) |
| `4349c15` | Supervisor fix 2: `main()` spins a `MultiThreadedExecutor(2)` so TF (reentrant group) is received while slow callbacks run |
| `2aff7fe` | Supervisor fix 3: scan-to-map check with a `cKDTree` (55 ms -> 1 ms per scan on the Pi; it used half a core) |
| `79bd5b8` | Supervisor fix 4: pose lookup at `now - tf_delay_s` falls back to the newest TF if it is not older than `health.odom_age_s` |
| (this commit) | This handover; `tools/debug/` (diagnosis scripts); docs |

Verification of the merge (before the robot tests):
- PC: build OK; `nala_base` + `nala_teleop` tests 55/55; app 3 tests now 115/115 (run in `tools/sim/env.sh`);
  coverage_tool `test_sensor.py` 8/8; closed-loop simulation `run_sim.py room`: `RETURNED_TO_BASE`, 99.3 % covered,
  0 contacts (also after fix 2).
- Known failures that are **not** from this session (same on the original branch, teammate's code, not changed):
  `coverage_tool/tests/regression.py --quick`: map `bumpy` misses 2 cells. On the Pi 4 two `test_ros.py` tests
  with fixed waits fail every time (`test_supervisor_launch_no_motion_and_continuous_trust`,
  `test_older_plan_is_split_at_corners_and_followed_exactly`); on the PC the second one fails now and then.
- Pi: `setup_pi_screen.sh` ran (Nav2, PyQt5, scipy, matplotlib installed). After a reboot the panel starts
  full-screen by itself; picture, touch and "screen stays on" checked by the user. Xbox checked without base_node:
  LB with a held stick = 0, after centring 0.45 m/s, B = button 1.

## Robot tests of app 3 (2026-10-16, room "Test", map `maps/Test/` on the Pi)

Base station `maps/Test/base_station.yaml` (x 0.183, y -0.468, yaw -0.118), plan
`maps/Test/plans/2026-10-16_131806.yaml`. Each run has a log and a task report in `maps/Test/runs/<time>/`
(`launch.log`, `task_*.json` with events and root cause). Recordings of the drives (`tools/debug/record_drive.py`
format) are on the Pi in `~/nala_debug/drive_1..4_*.csv`.

1. **Never trusted (TF_UNAVAILABLE, POSITION_UNCERTAIN).** The trust check looked up the laser pose at the scan's
   stamp; the scan arrives ~30 ms after its stamp, often before the EKF's TF for that time. -> fix 1.
2. **TF_UNAVAILABLE again, buffer up to 0.5 s behind.** The supervisor received TF only between its other
   callbacks (single-threaded spin; the tests use a MultiThreadedExecutor, so they never showed it). -> fix 2.
3. **Drove a few cm, paused: Nav2 "collision ahead", NO_MOVEMENT.** The user thought the speed (0.12 m/s) was too
   low. The recording showed the motors do follow; the stops came from the supervisor (stale data). Speed was
   **not** changed (0.3 m/s would need ~0.45 m to slow down at the measured 0.1 m/s^2; plan pieces are ~0.5 m).
4. **SCAN_MAP_MISMATCH, Start never active.** Caused by Claude: a test left a second `rplidar_composition` running;
   two drivers on one serial port gave a curved, broken scan (see `tools/debug/scan_on_map.py`). Killed;
   the LiDAR motor then needed an app 3 restart.
5. **Jolt, then pause (SENSOR_STALE, TF_UNAVAILABLE).** The scan check took 55 ms per scan. -> fix 3.
6. **Instant pause (TF_UNAVAILABLE).** Under load the EKF TF is sometimes >100 ms old (measured: median 22 ms,
   max 106 ms during app 3; 8 ms / 25 ms alone). -> fix 4.
7. **Last run (`runs/2026-10-16_144126`, `~/nala_debug/drive_4_after_79bd5b8.csv`): drives ~30 cm at
   0.12-0.15 m/s, then pauses.** Timeline: Start, the gate ramps 0 -> 0.12 m/s in 1.2 s, the wheels follow (odom up
   to 0.15 m/s), trust OK. After ~2.5 s the supervisor cancels the FollowPath goal: root cause
   **`SAFETY_GATE_STALE`** (the supervisor did not get `/safety/state` from the velocity gate within
   `health.gate_age_s` = 0.5 s). Later events: `SCAN_STALE`, `ODOM_STALE`. The trust check still reports
   `TF_UNAVAILABLE` sometimes (124 times in 6 min, mostly right after start-up).

## The open problem: late messages on the Pi 4

All remaining pauses are "data too old" checks of the app 3 nodes (supervisor, velocity gate, meter: Python,
`nala_coverage`). They were tuned in simulation on a fast PC. CPU on the Pi during app 3 (top, 4 cores):
panel ~57 % (map redraw while driving), supervisor ~52-80 %, AMCL ~33 %, velocity gate ~33 %, base_node ~14 %,
controller ~14 %, EKF ~10 %. `ekf_filter_node` once "Failed to meet update rate" (36 ms).

Suggested order (measure first with `tools/debug/`, change one thing at a time, ask the user before the robot moves):
1. Find out which messages are late and why: the gate publishes `/safety/state` at 20 Hz; record its arrival
   times at the supervisor side (add `/safety/state` to `record_drive.py`) and the CPU per process while driving.
   Check whether the gate (`gate_node.py`: `check_graph()` every 0.5 s calls `get_publishers_info_by_topic` and
   `get_node_names_and_namespaces`, 20 Hz tick) or the supervisor (single-threaded callbacks, the planning worker)
   is the slow one. The gate spins with `rclpy.spin_once` in its own loop.
2. Reduce load: panel `update_period` (0.3 s) in `config/panel.yaml`; AMCL particles (`max_particles: 2000`,
   `config/localization.yaml`); supervisor work per tick.
3. Only if the nodes are fast enough and messages are still a bit late: the limits in `config/coverage.yaml`
   `health` (`gate_age_s`, `scan_age_s`, `odom_age_s`, `lease_age_s`, all 0.5 s) may be too strict for the Pi.
   They are safety limits: raise them only with a reason, and keep the robot stopping fast when data stops.
4. Then the rest of "First run on the robot" in the README (a short plan, an obstacle, the return).

Other points seen during the tests:
- Turning margin is small: plans keep the centre 0.33 m from walls, the robot corner is 0.273 m from the centre,
  so a turn on the spot leaves ~6 cm. With AMCL error of 5-10 cm the controller may see a collision. A larger plan
  robot radius (e.g. 0.38 m) gives 11 cm but less coverage in narrow places (not changed; ask the user).
- AMCL put the robot ~3-10 cm from the tapped base station. Fine (limit 10 cm), but tap the base station carefully.
- The battery check stays off (`battery.enabled: false`): the INA260 does not answer on I2C.
- Shutdown of app 3 prints tracebacks (base_node "publisher's context is invalid", supervisor/meter
  KeyboardInterrupt during destroy). They were there before the fixes; harmless.

## Debug tools and data

- `tools/debug/record_drive.py`: records Nav2 command, gate output, odom, EKF, AMCL pose, trust state and all
  warnings to CSV while the panel runs app 3 (only listens).
- `tools/debug/scan_on_map.py`: PNG of the live scan on the map at the AMCL pose (broken scan / wrong pose).
- `tools/debug/tf_lag.py`: age of TF odom -> base_footprint, `/odom`, `/imu` when they arrive.
- Run them on the Pi after `source ~/NALA/config/ros_env.sh` (the panel's ROS network), from a separate SSH
  session; start long ones with `setsid nohup ... &` and check afterwards that they ended.
- Run logs: `~/NALA/maps/Test/runs/*/`. Recordings: `~/nala_debug/` on the Pi (not in git).

## Lessons from this session (for Claude on this project)

- Never run ROS tests in domain 0 (`config/ros_env.sh`): a test velocity gate could drive a running robot.
  Use `source tools/sim/env.sh` (domain 77, localhost). The app 3 test files also set their own domains.
- `pgrep -f`/`pkill -f` over SSH match the SSH command itself. `pkill -x` fails for names > 15 characters
  (`rplidar_composition`). Background jobs in a non-interactive shell ignore SIGINT: stop them with SIGTERM by
  PID or process group, then check with `ps` that they are gone. **Never leave a second LiDAR driver running.**
- In a test, never start `base_node` without the user's permission; joy/xbox tests run without base_node.
- sudo on the Pi over SSH: the user gives the password in the chat; send it on stdin to a temporary
  `SUDO_ASKPASS` script (see the old handover in git), remove the script afterwards. Never write it to a file in git.
- The panel starts app 3 with `ros2 launch nala_bringup app3_coverage_pi.launch.py ...` (`nala_panel/drive.py`);
  after a code change the user must tap **Finish** and **Drive** again to load the new code.
- `git push git@github.com:philipfei/NALA.git main` (HTTPS has no saved login on the PC).

## Prompt for the next conversation

Copy this into a new Claude Code session in the repo folder (PC, `/home/philip/Documents/NALA` or your clone):

```
Read CLAUDE.md and docs/handover.md first, then `git log --oneline c2417cf..HEAD`.

Context: app 3 (coverage) was merged into main on 2026-10-16 and tested on the robot. After several fixes in
src/nala_coverage/nala_coverage/supervisor.py the robot starts to drive, but pauses after about 30 cm with
root cause SAFETY_GATE_STALE (and later SCAN_STALE / ODOM_STALE): on the Pi 4 the app 3 Python nodes handle
messages too late. Details, data and suggested steps: docs/handover.md, "The open problem".

Task: find which node or message is late and why, and fix the cause (prefer less CPU / better threading over
looser safety limits). Start by measuring on the Pi with tools/debug/ while I run app 3 from the robot screen.
The Pi is reachable with `ssh nala@nala.local`; I give the sudo password in the chat if needed.
Rules: change code on the PC and push, the Pi only pulls; run ROS tests only in the isolated environment
(source tools/sim/env.sh); ask me before anything moves; after each fix run the app 3 tests and the simulation
before pushing, then tell me exactly what to do on the robot screen.
```
