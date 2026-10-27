# Handover (2026-10-16, late evening): app 3 covers 98.9 % on the robot; next problems

For the next person and their Claude Code session. Read `CLAUDE.md` first (rules, hardware facts, decisions,
open questions), then this file. It says what the session of 2026-10-16 did, where the work stopped, and what
to do next. Older handover text (Pi reinstall, phase A/B, firmware tuning) is in the git history:
`git log -p -- docs/handover.md`. Everything in it is done or copied into `CLAUDE.md`.

## Where we are

- Apps 1 and 2 work on the robot (driving test and a first SLAM map of the room "Test", 2026-10-16).
- App 3 (coverage) is merged into `main` and set up on the Pi. **Last robot run (2026-10-16 15:44, run 8 below):
  98.9 % of the room covered** (12.03 of 12.16 m^2). The stale-data pauses are fixed (CPU fixes, run 8).
- Open problems now (details in "Open problems after run 8"):
  1. **The robot dimensions changed** (user, 2026-10-16): longer body and a 235 mm rod in front; the turning centre
     is no longer the centre of the outline. Config, planner and Nav2 still use the old centred 410 x 360 mm box.
  2. **Long stops at every corner** (user): root cause found in the recording, not fixed yet: Nav2's controller
     limits the turn-in-place command from the *measured* turn rate, so from standstill it only asks for
     0.0175 rad/s, which is below what the wheels can start with -> deadlock until the progress checker gives up.
  3. The last plan target `(1.34, -1.33)` was not reachable for the Nav2 planner -> `BLOCKED_IN_PLACE`, user cancelled.
- Current stage: 3 (user decision).

## Motor stalls (2026-10-26) - robot test to do

User: while driving, most often turning on the spot, one or more motors sometimes stand still. Cause: the firmware
gives a stalled wheel at most FF(target) + kp x target + i_limit (10 %) PWM, ~25-34 % for slow commands.
Changed on `main`: `min_wheel_speed` 0.08 m/s (base.yaml, all apps), teleop turn 0.6 rad/s. Coverage 0.15 m/s /
0.5 rad/s was tried in one robot run (rdg_nm_petroom, run 2026-10-26_143137): the robot weaved while driving
(|turn rate| > 0.3 rad/s in 18 % of the driving time, 2-6 % before), so coverage is back at 0.12 / 0.4.
Robot test (with the user): app 1 small stick and 360 deg turns; app 3 room Test run, count stalls at corner turns,
check stop accuracy at goals (a stop from 0.08 m/s coasts ~8 mm) and corner turn overshoot. Still stalls -> firmware
stall boost (CLAUDE.md open questions, "Motors stalling").

## "Recalculating" pauses and weaving (analysis 2026-10-27) - robot test to do

Runs `maps/RDG_NM_v4/runs/2026-10-26_162946` and `_164108` (plan `2026-10-22_152140`, 0.12 m/s / 0.4 rad/s), copied
to the PC. `tools/debug/analyze_run.py <run_dir>` prints the numbers below for any run folder.
"Recalculating..." on the screen = a pause with auto-resume. Run 164108 (15 min): 53 of them (40 POSITION_UNCERTAIN,
12 SCAN_STALE, 1 ODOM_STALE), 2-7 s each, each also cancels and restarts the Nav2 goal; plus 11 SPUR_OFF_LINE.

1. **POSITION_UNCERTAIN: AMCL position sigma jumps in slow turns.** Every pause started at < 0.075 m/s while
   turning, at corners and piece starts, often 4-7 times at the same spot. Recording `~/nala_debug/drive_14_stepfix.csv`
   (2026-10-20): sigma > 0.1 m in 15-17 % of the turn samples, 0.8 % when driving straight; all 11 jumps in turns,
   e.g. 0.083 -> 0.218 m in one update at |vx| 0.014 m/s. Likely mechanism (vy was not recorded, so not proven):
   the EKF odometry (wheel vx and vy) moves sideways or diagonally when a wheel slips or stalls in a turn; the AMCL
   DifferentialMotionModel takes a move of >= 1 cm off the heading as a rotation and adds ~0.2-0.4 m of position
   noise in one update; sigma > 0.2 m stops the robot even when the scan fits. **Changed:** AMCL
   `OmniMotionModel` (config/localization.yaml), alphas unchanged; `record_drive.py` now also records vy.
2. **SCAN_STALE / ODOM_STALE: CPU.** RDG_NM_v4 (414 x 254 cells, 4x rdg_nm_petroom): load while driving 9.5
   (petroom runs the same day 5.5), supervisor callback gap median 475 ms (petroom 106 ms; scan_age_s 0.5 s), ticks
   up to 0.94 s, gaps up to 1.35 s also with short ticks; EKF and controller missed their rates. Which process uses
   the CPU is not known. **Not changed yet:** log `top -b -d 5` during the next run, then decide.
3. **Weaving: path follower limit cycle.** The plan pieces are straight, the robot stays near them (cross-track p90
   4 cm), but the heading swings +-8-10 deg with a ~6.5 s period (~0.75 m) and does not decay. Same at 0.12 m/s on
   petroom (p90 13.7 deg, low CPU); at 0.15 m/s (run 143137) median 14.8 deg, p90 32 deg. A model (pure pursuit,
   gate limits, first-order wheel lag, loop delay) gives the same amplitude and period with lookahead 0.2 m and
   ~0.8-1 s of total delay, at both speeds; the gate's 0.35 rad/s^2 changes nothing there; >= 0.35 m stays stable
   up to ~1.2 s. The +-5 cm sideways weave also explains SPUR_OFF_LINE (spur.max_offset_m 0.05).
   **Changed:** RPP speed-scaled lookahead (lookahead_time 3.0 s, 0.2-0.45 m; 0.36 m at 0.12 m/s).

Robot test (RDG_NM_v4, same plan, with the user): `record_drive.py` and `top -b -d 5 > ~/nala_debug/top.log`, then
`analyze_run.py`. Targets vs run 164108: POSITION_UNCERTAIN <= 5 per 15 min (40); heading error p90 < 6 deg (12.9);
|w| > 0.1 while driving < 15 % (36 %); no SPUR_OFF_LINE (11). Watch tight arcs and corners near walls (a longer
lookahead cuts arcs with a radius below ~0.2-0.3 m more; RPP slows down there, which shortens the lookahead again).

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
| `2038ffa` | First version of this handover; `tools/debug/` (diagnosis scripts) |
| `5571c5d` | CPU fix 1: the velocity gate creates no TF listener (`Inputs(node, tf=False)`); it never looks up a transform |
| `cc584a7` | CPU fix 2: the supervisor's TF listener runs on its own small node and thread (tf2 `node=None, spin_thread=True`); replaces the 2-thread executor of fix 2; `Inputs.close()` stops that thread at exit |
| (this commit) | This handover update (run 8, the new problems, the dimension change) |

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

8. **Run 8 (`runs/2026-10-16_154430`, recording `~/nala_debug/drive_5_after_cc584a7.csv`), after the CPU fixes:
   98.9 % covered.** See the next section. (Note: the recording covers only the first 360 s; the run lasted 474 s.)

## CPU fixes before run 8 (how the stale-data problem was solved)

Measured on the Pi with app 3 running but not started (robot standing):
- Messages were on time when standing (`/safety/state` 20 Hz, `/coverage/trust` 10 Hz, no gap > 250 ms), but the
  supervisor used 77-80 % and the gate 53 % of a core: no headroom once Nav2 drives.
- `cProfile` on the Pi (75 s, launch file copy with `prefix='python3 -m cProfile ...'`): our own code was cheap
  (the whole trust check 0.2 s, the gate logic ~1 s). Almost all time was in the rclpy executor: on every wake-up
  it rebuilds the wait set of all subscriptions, timers, services, action clients and QoS event handlers, and it
  decodes the message. `/tf` alone is ~45 messages/s. (`builtins.compile` in the profile is module import at start.)
- After `5571c5d` + `cc584a7`: supervisor 56 %, gate 32 % (idle); `/safety/state`, `/coverage/trust` still exact;
  trust check warnings only at start-up. In run 8 the trust stayed stable (only 4 untrusted states, all at start).
- `rclpy.experimental.EventsExecutor` exists in Jazzy (PC and Pi) and would cut the executor cost much more, but it
  is experimental and single-threaded; not used. Keep it in mind if CPU gets tight again.

## Run 8 in detail (`maps/Test/runs/2026-10-16_154430/` on the Pi)

- Plan `plans/2026-10-16_131806.yaml`, whole map. Result: `fraction` 0.989, covered 12.03 m^2 of 12.16 m^2
  coverable (`coverage_*.json`: also `actual_trajectory`, `covered_indices`). Task ended `CANCELED / OPERATOR_CANCEL`
  after 474 s; `root_cause` `POSITION_UNCERTAIN` (the pause right after start, t = 0 s, before AMCL settled).
- Task events (t from the first event): 0 s pause `POSITION_UNCERTAIN` (start-up); 117-209 s target
  `plan:-2.33,-1.34`: `FOLLOW_PATH_ERROR_106` (collision ahead) x2, `_105` (no progress), `NO_MOVEMENT` x5;
  224 s `OPERATOR_PAUSE` (user); 231 s 0.05 m skipped; 286-327 s target `plan:1.34,-1.33`: `_105`, `NO_MOVEMENT` x2,
  `_106` x2; 328 s pause `SCAN_MAP_MISMATCH`; 338 s 12 x `NAVIGATION_FAILED` -> `BLOCKED_IN_PLACE`; resumed,
  431 s again 12 x `NAVIGATION_FAILED` -> `BLOCKED_IN_PLACE`; 442 s cancelled by the user. `target_failures`:
  `plan:-2.33,-1.34`: 9, `plan:1.34,-1.33`: 29.
- `launch.log`: planner "failed to plan from (0.74, -1.29) / (1.20, -1.16) to (1.34, -1.33): Failed to create plan
  with tolerance of 0.05" (26x aborted); controller "Failed to make progress" 9x, "collision ahead" 5x. At shutdown
  a supervisor `RuntimeError: Unable to convert call argument '0'` in `take_message` and `InvalidHandle`: this is
  after Finish (Nav2 already cleaning up), shutdown noise like before.
- Recording (141-301 s of driving): 11 standstills > 1 s, together 61 s of 160 s. In **every** standstill Nav2
  sends `vx = 0, wz = +-0.0175` (sometimes 0.0203 / 0.026) rad/s, the gate passes it, the wheels do not move
  (odom/EKF 0.000), AMCL yaw stays constant (e.g. 11.8 deg for 24 s at 163-187 s). 90 % of all rotate-only
  commands (2351 of 2617) are below 0.03 rad/s.

## Open problems after run 8 (suggested order)

### A. Robot dimensions changed (user, 2026-10-16) - needs answers first

**Status 2026-10-19 (done in code, not driven yet):** answers (user): the rod is as wide as the body (360 mm) and
below the LiDAR (no scan filter); whether it carries the scintillator is not known yet (sensor values stay
placeholders). `config/robot.yaml` has `outline` (front 0.4425, back 0.2075, left/right 0.18 m), read by
`params.py`, `coverage_tool/settings.py`, the URDF and `tools/sim/fake_nala.py`. Farthest corner 0.478 m ->
collision radius 0.498 m, plan robot radius 0.53 m (was 0.293 / 0.33). New plan for room Test:
`maps/Test/plans/2026-10-19_104154` (100 % of the reachable floor in the plan, 10 stops); the old plan is refused
(`PLAN_TOO_CLOSE_TO_OBSTACLES`). Wheelbase: measured again by the user on 2026-10-19: **340 mm**, so
`half_wheelbase: 0.170` (k = 0.330 m) in `config/base.yaml` and firmware v2.3.3, **flashed by the user**
(v2.3.2 had 335 mm / 0.1675 and was never flashed; the 0.335 / 0.1675 values below are the state of 2026-10-16).
Next: the 360 deg turn test with the wheels and the IMU. The app 3 tests use a fixed test robot (the old centred box).
The text below is the state of 2026-10-16.

What the user said (verbatim facts):
- **Wheelbase (front-rear axle distance) is 335 mm** (user, 2026-10-16; half 0.1675 m). The config still has
  0.260 m: `config/base.yaml` `half_wheelbase: 0.130`, firmware `config.h` `HALF_WHEELBASE_M 0.130f`
  (`ROT_ARM_M` = half track + half wheelbase), `docs/mcu_protocol.md` and `CLAUDE.md` text. **Not changed yet.**
- Track width (left-right) is still **320 mm**, body width still **360 mm** (user).
- The odometry scale check (1.053 m = 1.053 m) is still valid: the user measured the travel of the rear wheels,
  and forward/strafe odometry does not depend on the wheelbase.
- **But turning does:** the rotation arm k = half track + half wheelbase is really 0.160 + 0.1675 = **0.3275 m**,
  the code uses 0.290 m. So (a) the wheel odometry reports 0.3275 / 0.290 = 1.129x the real turn (+12.9 %), and
  (b) a commanded turn rate is driven ~11 % too slow (0.290 / 0.3275). The floor test of 2026-10-16 measured
  "wheel turn rate 10-14 % too high" (360 deg by the wheels, 315-330 deg real) and blamed roller slip: **the old
  wheelbase very likely explains most of it.** To do (ask the user, firmware owner is the teammate): set
  `half_wheelbase: 0.1675` in `config/base.yaml` and `HALF_WHEELBASE_M 0.1675f` in `config.h` (both together:
  "firmware and Pi code must match"), flash, then repeat the 360 deg turn test with the wheels and the IMU. The EKF
  takes the turn rate only from the IMU, so apps 2/3 heading was not affected; Nav2 turn commands were (11 % slow).
- Body length is now **415 mm** (was 410 mm).
- A **rod sticks out 235 mm in front**, so the total length is **650 mm**.
- **The turning centre (chassis centre, `base_link`) is no longer the centre of the outline.** The 415 x 360 mm body
  (without the rod) **is centred on the wheel centre** (user, 2026-10-16). So in `base_link` (x forward, y left):
  body x -0.2075 .. +0.2075 m, y -0.18 .. +0.18 m; the rod reaches x = +0.2075 + 0.235 = **+0.4425 m**.
  Farthest body corner from the turning centre: hypot(0.2075, 0.18) = 0.275 m; rod tip: ~0.4425 m (more if the rod
  is wide or not on the centre line) -> a turn on the spot sweeps a circle of ~0.44 m radius.

Why it matters (everything assumes a centred box today):
- `config/robot.yaml` `body_length`/`body_width` -> `nala_coverage/params.py robot_footprint()` makes a box
  **centred** on `base_link`; the collision radius = farthest corner + padding (now 0.293 m). With the rod the
  farthest point is about 0.44-0.48 m from the turning centre (depends on the rod width and the body width): a turn
  on the spot sweeps a circle of that radius. Plans that keep the centre 0.33 m from walls would hit with the rod.
- `coverage_tool` also derives the robot from `robot.yaml` (`settings.robot_config`) and assumes a centred box;
  the plan robot radius 0.33 m is hard-coded as "0.273 + 0.02 + 0.03". It must be re-derived and plans re-made.
- Nav2 local costmap footprint (`params.render_nav2`: `footprint` from the box) and global `robot_radius`.
- The URDF (`nala_description`, RViz picture) uses `body_length`/`body_width`.
- Is the rod the coverage sensor (scintillator)? `coverage.yaml` has `sensor_offset_m: 0.26`, `coverage_disk_radius_m:
  0.6` as placeholders. If the sensor sits at the rod tip, the offset becomes ~0.44 m. Ask.
- The LiDAR is 95 mm in front of the chassis centre since 2026-10-23 (was at the centre; robot.yaml had 0.115 until
  2026-10-26, wrong: maps made with it, e.g. rdg_nm_petroom, may be slightly distorted), 172.5 mm above the
  floor (measured 2026-10-26): check that it does not see the rod (now only ~0.35 m in front of the LiDAR)
  (`tools/debug/scan_on_map.py`, or the scan check in this file's history: no points < 0.45 m before the rod).
- Questions still to ask: rod height, width and lateral position (on the centre line?), rigid or not, what it
  carries (the coverage sensor?). Answered: the body is centred on the wheel centre.
- Then: support a non-centred outline (front/back extents, not only length) in `robot.yaml`, `params.py`
  (`robot_footprint`, collision radius), `coverage_tool`, the URDF; new plans. Keep "the outline is configured once".

### B. Long stops at corners (turn on the spot) - root cause known

**Status 2026-10-19 (done in code, not driven yet):** the user chose the fix idea below: RPP gets its own
`motion.controller_angular_accel_rad_s2: 3.0` in `config/coverage.yaml` (rendered into `max_angular_accel`; first
step from standstill 0.15 rad/s), the velocity gate keeps `angular_accel_rad_s2: 0.35`. Risk: RPP brakes later
before the target heading, the gate's slower ramp can then overshoot by up to ~13 deg. Simulation in room Test
(rod outline, new plan): 99.9 % covered, `RETURNED_TO_BASE`, 0 contacts, no more turn reversals than without the
fix; the simulated robot has no stiction, so the deadlock itself can only be checked on the robot.

**Robot run 2026-10-19 11:06 (`runs/2026-10-19_110654`, recording `~/nala_debug/drive_6_rod_fixA.csv`):** the robot
turned at the corners (fix A works), but paused 4 times in ~60 s with `POSITION_UNCERTAIN`: AMCL position sigma median
0.093 m (run 8: 0.042), above the 0.10 m limit 42 % of the time. It grew ~1 cm per second of turning on the spot
(4x faster than driving). Not the rod (no LiDAR points < 0.65 m), not CPU (AMCL 19 %, 4 dropped scans at start).
Cause: AMCL `alpha4` (position noise from turning) 0.05. Fix (user decision): `alpha4: 0.01` in
`config/localization.yaml`; simulation: sigma median 0.038 m, AMCL error 0.039 m (was 0.047), 99.8 %, 0 contacts.

**Robot run 2026-10-19 11:21 (`runs/2026-10-19_112103`, `~/nala_debug/drive_7_alpha4.csv`):** position sigma median
0.032 m (fixed), 14 turns on the spot. New: `HEADING_UNCERTAIN` right after turns; one turn stalled 14 s at
0.15 rad/s (RPP 3.0 x 0.05 s is still below the breakaway there); one turn overshot ~19 deg (the gate's 0.35 rad/s^2
slow-down after RPP hands over to driving at 20 deg); Nav2 "Control loop missed its desired rate" 32x and EKF
"Failed to meet update rate" 27x (run 8: 0x) = CPU. Fixes (user decisions): the velocity gate lifts a Nav2 turn on
the spot to `motion.turn_start_rad_s: 0.3` (ramped), RPP back to 0.35; AMCL `alpha1` 0.05 -> 0.02; map changes
node `scan_period_s` 2.0, `beam_step` 4. Scan screen side panel fits 600 px again without scrolling (576 px on the
Pi). Simulation: 99.9 %, `RETURNED_TO_BASE`, 0 contacts, heading sigma max 8 deg. Overshoot not changed yet.

**Robot run 2026-10-19 ~11:55 (`~/nala_debug/drive_8_turnstart.csv`):** turns started without stalls (0 stalls > 1 s),
but `SCAN_MAP_MISMATCH` 244x. A search of the whole map with a live scan put the robot at x 1.79 (fit 94 %) while
AMCL said x 2.40 with sigma 0.017 m: AMCL was too sure to correct itself (alpha1 0.02 / alpha4 0.01). Heading was
right (the 180 deg mirrored pose fits only 80 %). The user had placed the robot the other way round than in the run
before; this run's placement matches the base station, so the earlier run(s) likely started backwards, which the
almost symmetric room hides. Fix (user decision): alpha1 and alpha4 0.03. Simulation: sigma median 0.042 m, AMCL
error median 0.039 m, heading sigma max 8.1 deg, 99.8 %, `RETURNED_TO_BASE`, 0 contacts.

**Robot 2026-10-19 ~12:20:** the user put the robot in a corner; AMCL (started at the base station) still placed
it near the middle (scan fit 14 %), so `SCAN_MAP_MISMATCH`. User requirement: app 3 must not assume the robot is
on the base station, must show the real position from the LiDAR, say "Robot is not on the base station", and let
the user put it there before Start. Done: `locate.py` + `locator_node` (whole-map scan search every 2 s while no
task drives, base station check incl. backwards), supervisor refuses Start unless ON_BASE and re-sends AMCL the
base station pose when the robot arrives there, panel draws the located pose and the messages. Checked on room
Test: the real corner scan -> NOT_ON_BASE at the right pose (fit 99 %); ray-cast scans: on base / 12 cm 10 deg off
-> ON_BASE, backwards -> BACKWARDS (fit 1.00 vs 0.29), 0.25 / 0.5 m off -> NOT_ON_BASE. Simulation: on base ->
RETURNED_TO_BASE 99.9 %; 1 m off -> Start refused (NOT_ON_BASE_STATION).
Speed: the first settings took 3 s per search on the Pi 4; now 0.15 m / 6 deg / 60 beams, 6 candidates refined,
and no whole-map search when the base station fit is >= 0.95. Room Test, 20 ray-cast poses: base station verdict
20/20 right; pose right 20/20 when the robot is carried (ties go to the pose nearest the last one), 19/20 right
after a fresh start: one corner looks exactly like the opposite corner turned 180 deg (both fit 100 %), so right
after app 3 starts with the robot there it can be drawn in the mirrored corner (status still NOT_ON_BASE).
Robot 13:00: the robot stood on the base station position but turned 132 deg: Start stayed off with only "not on
the base station". New status TURNED with turn_deg ("Turn it about 132 degrees to the left"); the map draws the
robot outline (dashed) and a clear arrow at the base station (user: show the required heading).
Robot ~13:25 (`runs/2026-10-19_132156`, 92 % covered, no recording): straight driving good; 13 POSITION_UNCERTAIN
pauses (in turns) and turns not in the plan near the base station. Simulation: since the gate lift and RPP 0.35 the
robot turned 1500-2700 deg on the spot per run instead of ~900 (overshoot, extra rounds). Fixes (user decisions):
the lift only from standstill, RPP max_angular_accel back to 3.0, and the trust check lets the scan decide (AMCL
sigma over the limit pauses only with a poorly fitting scan or over 2x the limit). Not simulated again (user: test
on the robot).

**Robot run 2026-10-19 13:56 (`runs/2026-10-19_135600`, `~/nala_debug/drive_10_scantrust.csv`): 98 % covered, 19
turns, 1 reversal, no stalls.** The missed floor south of the base station: at Start the user stood at the screen
(within the 0.5 m collision radius): "collision ahead" and "failed to plan" from the base station, and the first 3
plan pieces (1.6 m) were skipped within 0.8 s. User: step back after Start (no code change asked). 2
POSITION_UNCERTAIN in the north-east corner turn (sigma 0.21-0.23 m, just over the 2x hard limit). Plan order: the
west end is covered with a diagonal back and a second visit to the corner; a planner search 8x longer gives the
identical route (so it is the planner's route, not the search time). New: auto-resume after localization pauses.

**Robot run 2026-10-19 14:37 (box test, `runs/2026-10-19_143659`, `~/nala_debug/drive_11_autoresume.csv`):** driving
around a box worked; 2 POSITION_UNCERTAIN pauses resumed by themselves after ~2.5 s. Then EXECUTION_CHECK: after the
detour the robot stood 2-3 cm closer to a wall than the collision radius; the start check needed a safe straight
line from an unsafe start, and NavFn cannot plan from inside its band (14x NAVIGATION_FAILED -> BLOCKED_IN_PLACE).
Fixed: `Grid.escape_start` (plan from the nearest safe point, drive the short way out). Also: "Recalculating..." on
the driving screen during auto-resume pauses; zoom buttons removed; planner covers corners at lane ends (new plan
`maps/Test/plans/2026-10-19_151921`, 77 s planned instead of 87 s). The teammate set the wheelbase to 340 mm and
flashed firmware v2.3.3 (commit 2415f32).

- Regulated Pure Pursuit (`nav2_regulated_pure_pursuit_controller`, Jazzy) clamps the turn-in-place command to
  `measured turn rate +- max_angular_accel * control period` = `0 +- 0.35 * 0.05 = +-0.0175 rad/s` when the robot
  stands. The measured rate comes from `/odom` (`odom_topic` in `nav2_params.yaml`). 0.0175 rad/s is ~5 mm/s at the
  wheels, far below the breakaway (dead zone / stick-slip, floor test: low speeds 0.05-0.10 m/s already stick-slip),
  so the robot never starts turning, the measured rate stays 0 -> deadlock until the progress checker (15 s) or the
  supervisor's `no_motion_s` (15 s) gives up and a new goal is sent. That is the user's "waits very long at corners".
- The velocity gate is fine: it ramps from its own last output (0.35 rad/s^2), not from the measurement.
- `params.render_nav2` writes `motion.angular_accel_rad_s2` (0.35) into RPP `max_angular_accel`; the test
  `test_controller_turn_acceleration_is_the_coverage_motion_value` (test_revision.py) checks that they are equal.
- Fix idea (not done, decide with the user): give RPP a much higher `max_angular_accel` (e.g. >= 3 rad/s^2, so one
  control step allows >= 0.15 rad/s) and leave the smooth 0.35 rad/s^2 ramp to the velocity gate; change
  `render_nav2` and that test accordingly (a separate config value, or keep the file value). Check the stop
  accuracy of turns afterwards (floor test 2026-10-16: 0.3 deg with a planned 0.35 rad/s^2 deceleration).
  Also check how RPP slows the turn near the target heading so it does not end in the same dead zone.
  Alternative: feed RPP the EKF odometry (`/odometry/filtered`) - does not help, it is also 0 at standstill.

### C. Unreachable plan target and other run 8 events

**2026-10-19:** the old plan is refused for the new outline anyway. With the new plan the simulation reached every
target. The margin is small, though: the plan keeps 0.53 m, the global costmap needs 0.498 + 0.025 = 0.523 m, and
coverage_tool's per-cell check can be up to half a cell (0.025 m) more lenient than NavFn. See the robot run.

- `plan:1.34,-1.33`: NavFn could not plan to it from (0.74, -1.29) / (1.20, -1.16) ("tolerance 0.05"): the target is
  probably inside the global costmap's inflated area (robot_radius = 0.293 + half a cell) near a wall, or it was
  blocked by a costmap obstacle. Check it on the map; the plan keeps 0.33 m, the planner needs < 0.3175 m free.
- `plan:-2.33,-1.34`: collision ahead / no movement near the left end of the room (turning margin ~6 cm, see "Other points" further down).
- Start-up pause `POSITION_UNCERTAIN` is normal (AMCL settles in ~20 s); the panel enables Start only when trusted.

## The old open problem: late messages on the Pi 4 (solved by the CPU fixes, kept for reference)

All remaining pauses are "data too old" checks of the app 3 nodes (supervisor, velocity gate, meter: Python,
`nala_coverage`). They were tuned in simulation on a fast PC. CPU on the Pi during app 3 (top, 4 cores):
panel ~57 % (map redraw while driving), supervisor ~52-80 %, AMCL ~33 %, velocity gate ~33 %, base_node ~14 %,
controller ~14 %, EKF ~10 %. `ekf_filter_node` once "Failed to meet update rate" (36 ms).

Suggested order that was used (measure first with `tools/debug/`, change one thing at a time, ask the user before the robot moves):
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
- The battery check stays off (`battery.enabled: false`): the battery monitor was changed to an INA219 at 0x40 on 2026-10-21 (voltage only, see `docs/battery.md`) and is not tested on the robot yet.
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
Read CLAUDE.md and docs/handover.md first (especially "Open problems after run 8"), then
`git log --oneline c2417cf..HEAD`.

State: app 3 (coverage) runs on the robot and covered 98.9 % of the room "Test" on 2026-10-16
(run maps/Test/runs/2026-10-16_154430 on the Pi, recording ~/nala_debug/drive_5_after_cc584a7.csv).

Two problems to solve now, in this order:
1. The robot dimensions changed (handover section A): body 415 mm long, a 235 mm rod in front (650 mm total),
   the turning centre is no longer the centre of the outline; the wheelbase is now 335 mm (config/firmware still
   260 mm: fix base.yaml + firmware config.h together, then repeat the 360 deg turn test). Ask me the open questions
   of section A first (rod height/width/position, whether the rod carries the coverage sensor; the body is centred
   on the wheel centre), then make
   robot.yaml, nala_coverage (params.py footprint and collision radius), coverage_tool and the URDF support a
   non-centred outline, configured once. New plans are needed afterwards.
2. Long stops at every corner (section B): Nav2's Regulated Pure Pursuit limits turn-in-place to
   measured rate + max_angular_accel * 0.05 s = 0.0175 rad/s from standstill, below the wheels' breakaway.
   Propose a fix (e.g. a high RPP max_angular_accel, the velocity gate keeps the 0.35 rad/s^2 ramp), discuss
   it with me, implement, test in the simulation, then on the robot.
Also look at section C (unreachable last plan target).

Rules: change code on the PC and push, the Pi only pulls; run ROS tests only after `source tools/sim/env.sh`;
ask me before anything moves; after each fix run the app 3 tests and the simulation before pushing; use
tools/debug/record_drive.py on the Pi to record the next drive, then tell me exactly what to do on the screen.
The Pi is `ssh nala@nala.local`; I give the sudo password in the chat if needed.
```
