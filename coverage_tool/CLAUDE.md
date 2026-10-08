# Coverage path planner — project context for Claude Code

Owner: Dielis (TU Delft, JIP project). Python desktop tool that plans full-area coverage paths for
**NALA** (self-built square mecanum robot, driven like a differential-drive robot for now, forward only;
ROS 2 / Nav2) on a ROS lidar occupancy map (`.pgm` + `.yaml`) and exports Nav2 waypoints (YAML/JSON).
Copied from SIMBA's coverage_tool (iRobot Create 3). Runs in WSL (Ubuntu) on Dielis's laptop, and on the
robot's Pi: the touchscreen panel (`src/nala_panel`) runs `plan_cli.py` as a child process.

## NALA geometry (settings.py `geometry`)

- Path = robot centre (`base_link` = LiDAR = chassis centre). Robot radius 0.33 = turning circle of the
  410 x 360 mm box (corner 0.273) + 0.02 padding + 0.03 AMCL margin, rounded up to 1 cm, so all collision checks
  stay circles. The outline comes from the repo's `config/robot.yaml` and padding/sensor from
  `config/coverage.yaml` (`settings.robot_config`); the sensor values (0.26 / 0.6) are placeholders.
- Coverage is at the scintillator, `sensor_offset_m` 0.26 ahead, disk 0.6 (`coverage/sensor.py`):
  `stroke_union` shifts every target forwards and adds the swing (arc r=0.26) at each corner.
  Every coverage computation goes through `stroke_union`/`piece_cells` — keep it that way.
- Driving direction matters (forward only). `localsearch.Piece.covs` = (cells forwards, cells backwards);
  `count` always matches `seq` directions — every flip goes through `_flips`/`_cov_delta`/`_swap_counts`
  (flip, move, 2-opt, block move, perturb, rotate). Test: tests/test_sensor.py. (An earlier version used
  the intersection of both directions; that added many useless aimed goals and fragmented the route.)
- Final `covered` = `pipeline.driven_coverage` (each piece with its transit, real arrival heading).
  `pipeline._prune_redundant` drops gap/aim/fill pieces whose cells the rest covers.
- Candidate "Boustrophedon cells, no wall loop" (room-wide lane grid along the wall directions) wins on
  corridors for NALA. Tried and rejected: lanes spread evenly per cell (lanes close to bumpy walls get chopped,
  cost 1538 s vs 1021 s on RDG).
- Single goals of unknown heading count only `r - offset`; aimed goals (kind `aim`, `localsearch.aimed`)
  are a 5 cm forward move ending facing a missed cell — they never flip.
- `pipeline.sensor_reachable` = reachable centres dilated by the offset; coverable floor is measured from it.
- `footprint.py`: real box swept along the drivable path (forward + turn on the spot), `footprint_hits` == 0.
- `drivable.py` counts the corner swing (`pivot`) as coverage: rounding a corner loses that swing.
- `sensor_path` always emits both shifted end points at a corner (no shortcut at gentle bends), so the
  path checked in parts (drivable) and as a whole gives exactly the same cells.
- `planning.visible_many` has an exact shortcut (no blocked cell in the ray's box ⇒ visible), verified
  against the exact test.

## Run

```bash
python3 -m pip install -r requirements.txt      # numpy scipy matplotlib contourpy pyyaml pillow (NO scikit-image)
python3 app.py [map.yaml]                          # Tkinter GUI (needs python3-tk)
python3 plan_cli.py map.yaml -o plan.yaml --png plan.png    # start = base station next to the map, or --start X Y
python3 tests/regression.py [--quick]              # acceptance test, run after every planner change
```

## Architecture (`coverage/`)

| File | Role |
|---|---|
| `pipeline.py` | `plan()`: reachability → build candidates → `_finish` (centre-line fill, local search, sanitize, connect, exact cost) → cheapest wins → extra polish. `brush_reachable()` = exact reachable-cell definition. |
| `cost.py` | The cost function (seconds): drive, bends (`bend_s_per_rad`), stop+rotate above `smooth_turn_deg`, hairpins, overlap (FLOOR swept twice only; brush over obstacles is free), missed (heavy), missed edge cells (light). |
| `localsearch.py` | `RouteOptimizer`: exact-Δcost local search over pieces — drop, trim, straighten, flip, move, block-move, 2-opt, detour; then perturb-and-reoptimise until the time budget ends. |
| `optimizer.py` | Wall loops, boustrophedon region decomposition, per-region lanes vs spiral vs clipped rings (`plan_region`), `plan_regions` (rims on/off, ring_targets), `plan_hybrid`, lane generation with inward shift along bumpy edges. |
| `spiral.py` | Distance-field rings (contourpy), spiral tree/ordering, centre-line fill (hand-written Zhang–Suen skeleton), obstacle rims. `WALL_LEVEL=0.49`, `SIMPLIFY=0.2` (x cell). |
| `grid.py` | Map loader (map_server conventions), clearance map, `segment_safe`, `reachable_from` (4-connected on purpose), A* (8-dir, no corner cutting). |
| `planning.py`, `planner.py` | Dielis's original modules; still used for `stroke_union`, `visible_many`, `connect`, `split_path`, `poly_target`. Only change: 2-D cross product fix for NumPy 2. |
| `export.py`, `render.py`, `settings.py` | Nav2 export, drawing, defaults (`DEFAULTS`) + JSON load/save. |

Candidates in Auto mode: Spiral around obstacles, Spiral outer walls only, Mixed regions (2 split angles),
Straight lanes one direction, Wall loop + boustrophedon cells (2 angles), Rings or lanes per region.
Turning cost is curvature-based (cost.vertex_cost / corner_radius), resolution independent. Export = pose at
every path vertex (<= 0.5 m apart), sent with NavigateThroughPoses.

## Design decisions Dielis asked for (keep them)

- **Everything is decided by the cost function, nothing hard-coded.** Full coverage emerges from a very heavy
  `missed_weight` (10000 ≈ 167 s per 5 cm cell), not from a forced rule. Do not add "always cover" flags.
- **Robot radius = collision/safety distance; coverage radius = brush/spacing.** Spacing = 2·r_cov·(1−overlap),
  overlap 0–50 %, computed automatically.
- **Reachable cells are exact:** robot poses with clearance ≥ robot radius, 4-connected to start; a cell is reachable
  if some pose is within r_cov with line of sight. Verified against brute force — keep it that way.
- **Straight lines as much as possible.** Brush overlapping walls/obstacles is fine.
- **Missed cells must be very expensive** (Dielis checks for red pixels). `edge_missed_weight` defaults to 10000
  (strict); lowering it trades cells within `edge_band_m` of a wall for straighter lines.
- Path shape is free (spiral, lanes, mixed) — whatever is cheapest. Wall loop first by default.
- Obstacle handling (loop around vs outer walls only) is a user option.

## Gotchas

- Single-goal pieces (detours) have a heading `Piece.h` set by `orient()`/`_slot_cost()`: the turn at a point
  is charged at its junctions. Without it, U-turns at detour points look free and the order gets bad.

- `grid.astar`: `gr, gc = g; sq2 = math.sqrt(2)` must come BEFORE the `moves = [...]` list (UnboundLocalError twice).
- `reachable_from` must stay 4-connected to match A* (no diagonal squeezing).
- NumPy 2: `np.cross` rejects 2-D vectors — use explicit `a[0]*b[1]-a[1]*b[0]`.
- Don't use scikit-image (not installed on Dielis's machine).
- Planning time is budget-driven (4 s/candidate + 15 s polish), results vary slightly run to run.

## Acceptance criteria (tests/regression.py)

Every case: 0 open-floor reachable cells missed (independent brute-force check against every SENSOR position),
0 unsafe segments, 0 drivable lost cells, 0 outline (footprint) hits. Cases: 5 with NALA geometry, 2 with a
centred brush (offset 0). Also `python3 -m pytest tests/test_sensor.py`.

## Status / next steps

- Only tested on synthetic maps in `examples/` — **not yet on the real map or robot.** First task: run on the real map.
- On the NALA robot the export is driven by `app3_coverage_pi.launch.py plan:=FILE` (coverage_supervisor loads it,
  `src/nala_coverage/nala_coverage/plan_file.py`). NALA refuses plans closer than 0.293 m to walls with an exact
  capsule check (plan with 0.33), plans made on another map (`map_image_sha256`), plans made for another
  sensor offset/radius (`PLAN_ROBOT_MISMATCH`) and for another base station (`PLAN_BASE_MISMATCH`).
  The plan starts at the base station (`base_station.yaml` next to the map, exported as `base_station`). Examples: `examples/RDG_NM-DEP_V4_nala.yaml`, `map_ME_room1v4_nala.yaml`.
- **Drivable path** (`coverage/drivable.py`, run at the end of `pipeline.plan`, exported as `drive_path`): jogs of a
  few cm that lose no covered cell are straightened, then every corner (or group of same-direction corners) gets the
  widest arc >= `drive.min_arc_radius_m` that keeps every covered cell and passes `segment_safe`; the rest are stop
  corners. Straight stretches are exported as end points only (sampling a segment in pieces can differ at a cell
  corner). Regression requires `drive_lost_cells == 0` and no unsafe drivable segment.
- NALA follows `drive_path` exactly with FollowPath, piece by piece between stop corners, through its own supervisor
  (velocity gate and lease apply); a blocked piece is skipped by 0.5 m and rejoined with the Nav2 planner.
- `send_to_nav2.py` only fits a plain Nav2 stack, not NALA (its velocity gate only passes Nav2 output while the
  supervisor holds the lease). Untested. Before trusting it:
  same map in Nav2 as in the planner, set initial pose (`nav.setInitialPose`), match Nav2 `robot_radius`/inflation
  to the planner's robot radius, check goal tolerances vs waypoint spacing, consistent `use_sim_time`.
- Calibrate `stop_penalty_s`, `bend_s_per_rad`, `angular_rad_s` from real NALA logs.
- Possible next step: heading-aware box check, so passages narrower than 0.66 m (box driving straight) can be used.
- Planning speed (8 candidates ≈ 1–2 min).
