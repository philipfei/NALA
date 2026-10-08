# Coverage Path Planner

A desktop tool that plans a full-coverage path on a ROS lidar map and exports it as Nav2 waypoints.

This copy is set up for **NALA**:
- **Robot:** a 410 × 360 mm box that only drives forwards and turns on the spot. The outline comes from `config/robot.yaml` (`body_length` × `body_width`).
- **Sensor:** a scintillator 0.26 m ahead of its centre, covering a 0.6 m radius around that point. These two values are **placeholders** (the real position and size are not known yet); they come from `config/coverage.yaml`.

`coverage_tool/` sits next to the repo's `config/` folder; the tool reads the robot geometry from there (`plan_cli.py --config-dir` for another folder). Without it, the built-in values above are used.

On the robot, the touchscreen panel (`src/nala_panel`) runs `plan_cli.py` for you: see *App 3* in the main README.

See *NALA: a sensor ahead of the robot* below. With `sensor_offset_m: 0` the tool behaves as for a robot with its brush at the centre (SIMBA).

The planner generates several candidate paths: inward spirals, straight lanes, and a mix of lane directions per region. It scores each one with a **cost function** in seconds and keeps the cheapest.

## Install and run (Windows, macOS or Linux, Python 3.9 or newer)

```
pip install -r requirements.txt
python app.py                         # or: python app.py path/to/map.yaml
```

Tkinter comes with the normal python.org installers. On Linux you may need `sudo apt install python3-tk`.

Two example maps are in `examples/`.

## Workflow

1. **Open map...** Pick the `.yaml` file saved by `map_saver` or `slam_toolbox`. The `.pgm` file must be next to it.
2. **Set start** Coverage starts at the base station when `base_station.yaml` is next to the map (set on the robot screen, or with `app3_base_station_pc.launch.py`). The export records the base station: the robot starts and ends there. Click **Set start** to begin coverage somewhere else; the robot then drives from the base station to that point first.
3. **Draw area** (optional) Left-click the corners, then right-click to close the polygon. If you don't draw an area, all free space reachable from the start is covered.
4. Set the robot radius, coverage radius, overlap and cost weights, then click **Plan path**.
5. The right-hand panel shows the cost breakdown of the chosen path and the cost of every candidate.
6. **Export Nav2 waypoints...** Saves a `.yaml` or `.json` file.

## NALA: a sensor ahead of the robot

The planned path is the path of the robot's **centre** (`base_link`): that is what Nav2 follows. Coverage is computed at the **sensor**, `sensor_offset_m` (0.26 m) ahead of it (`coverage/sensor.py`):
- **Straight segments:** on every straight segment the sensor runs the same line, shifted forward.
- **Corners:** at a corner the robot turns on the spot, and the sensor swings along an arc of 0.26 m around the corner.
- **Gentle bends:** a rounded corner (a dense arc) is followed the same way, point by point.

**The robot only drives forwards**, so the direction of a path matters. At the end of a lane the sensor leads by 0.26 m and reaches closer to the wall ahead. At the start of a lane it trails by 0.26 m. A cell is either covered or not, but *which* cells a piece covers depends on the way it is driven.
- **Route optimiser:** it keeps each piece's cells for both directions. Every move that flips a piece (flip, move, 2-opt, block move) updates the coverage count exactly.
- **Final plan:** its coverage is computed along the route as driven, transit included. A single goal then gets the heading the robot really arrives with.
- **Clean-up:** at the end, detours, aimed goals and centre lines whose floor the rest of the route already covers are removed.

**Single goals** (detours to missed spots) come in two kinds:
- **Plain goal:** the heading is unknown in advance, so only `coverage radius − sensor offset` (0.34 m) around it counts.
- **Aimed goal** (kind `aim`): a 5 cm straight forward move that ends facing the missed spot, so the sensor reaches it. The optimiser uses an aimed goal where only the sensor, pointed that way, can reach a cell.

**Reachable floor:** the sensor can be anywhere within 0.26 m of a position the robot centre can reach, because the robot can turn on the spot there. Floor within the coverage radius of such a point, with a clear line of sight, counts as reachable (`pipeline.sensor_reachable`).

**Robot outline.** The robot radius (0.33 m) is the circle the 410 × 360 mm box turns in (its farthest corner is 0.273 m from the centre), plus 2 cm padding and a 3 cm localisation margin, rounded up to 1 cm (`settings.robot_config`). So every collision check stays a simple circle check.
- **Independent check:** at the end, `coverage/footprint.py` moves the real box along the drivable path, forwards along every segment and turning on the spot at every corner. It counts the poses where the box overlaps anything that is not known-free (`footprint_hits`, must be 0).
- **Narrowest passage:** the planner can only use passages of at least 2 × 0.33 m.

## The cost function

All terms are in seconds, so they can be added up. The code is in `coverage/cost.py`.

| Term | How it is computed | Setting |
|---|---|---|
| Driving | path length ÷ speed (transit between pieces included) | Speed |
| Bending | Every bend that isn't a stop costs a little per radian (*gentle bend cost*). So a truly straight line is always the cheapest way to drive, and a line that wiggles along every bump of a wall is not "free". | Gentle bend cost |
| Curves | A differential-drive robot drives an arc of radius R at speed min(v, turn rate × R). So a curve only costs extra where it is tighter than speed ÷ turn rate (0.375 m by default). The radius at each vertex is taken from the drawn path, and a rounded corner may pass at most 5 cm from the planned vertex. A curve therefore costs the same whether it is drawn with 8 or 64 points. | Speed, Turn rate |
| Corners | A real corner (no room to round it) means stop, rotate in place (angle ÷ turn rate), then accelerate again (*stop per sharp turn*). Bends up to the *no-stop* angle never stop. | Turn rate, No stop for bends up to, Stop per sharp turn |
| Reversing | The robot never drives backwards (NALA's autodrive is forward only). Every path is driven forwards, and a hairpin (direction change ≥ 150°) is a stop with a long turn on the spot, so each hairpin costs extra. | Reversal / hairpin |
| Double coverage | **floor** swept more than once × weight × time to sweep 1 m². The brush passing over a wall or obstacle costs nothing, so a straight line may run past a bumpy wall with the brush overlapping the bumps. | Double coverage weight |
| Missed area | reachable floor left uncovered × weight × time to sweep 1 m² | Missed area weight |
| Missed edge | the part of the missed floor that lies within the *edge band* of a wall or obstacle (the bottom of a recess between two lidar bumps) uses the lower *missed edge* weight | Edge band, Missed edge weight |

**Missed cells are penalised very heavily, not forbidden.** The default missed-area weight is 10000: one missed 5 cm cell of open floor costs about 167 s, as much as almost three minutes of driving. So the optimiser practically always goes back for every reachable cell, because that is the cheapest option under the cost function. There is no hard rule. With a low weight, tiny specks are left out whenever a detour costs more than they are worth. At weight 1, driving nothing at all becomes "cheapest".

**Straight lines versus the last centimetres along a bumpy wall.** A line that follows a lidar wall at the robot's safety distance has to weave around every bump. A straight line has to stay clear of the biggest bump, so it can't reach the bottom of the recesses in between. Floor within the *edge band* (default 0.10 m) of a wall or obstacle can be charged with a separate *missed edge* weight. By default it is 10000, the same as open floor, so every reachable cell is covered. Lower it (e.g. 100, about 1.4 s per 5 cm cell) to let lines stay straight past bumpy walls at the price of the bottoms of the recesses.

**Which cells count as reachable** (`pipeline.brush_reachable`) is worked out in two steps:

1. **Where the robot can go.** These are all cells whose centre is at least one robot radius from any obstacle or unknown cell, and that are connected to the start without squeezing diagonally past two blocked cells.
2. **What the brush can reach from there.** A cell counts as reachable when at least one of those robot positions lies within one coverage radius of it and has a clear line of sight to it. Walls block the brush.

Everything else in the area shows yellow: the robot can't get close enough, or the cell is hidden behind an obstacle. This uses exactly the same test as the coverage check. It was verified against a brute-force check of every cell against every robot position.

The wall loop runs on the centres of the outermost safe cells, which is exactly the robot's safety limit. That way the brush reaches as close to the walls as the robot radius allows.

## How the route is optimised

Candidate patterns (spirals, straight lanes, mixed regions) only give a starting route. Each one is then improved by a local search on the full cost (`coverage/localsearch.py`). The route is treated as a sequence of pieces: lanes, loops, rings, centre lines and single goals. Any move that lowers the total cost is applied:

| Move | Effect |
|---|---|
| Drop | Remove a piece that only double-covers. This saves driving; it would cost the missed cells only that piece covers. |
| Trim | Shorten a lane or line from either end while its end only double-covers. |
| Flip | Drive a piece in the other direction. |
| Move | Take a piece or a block of 2–3 pieces and insert it elsewhere, in either direction. |
| 2-opt | Reverse a stretch of the route. |
| Detour | Cover still-missed cells where the penalty removed is larger than the extra driving and turning. A detour is a single goal at the position that covers the most missed cells, an aimed goal (see *NALA* above), or a line along the centre of a missed strip (one line instead of a row of stop-and-go points); the cheapest is used. |
| Rotate | Enter a closed loop (wall loop, loop around an obstacle, ring) at the point that is cheapest to reach and to leave. |
| Straighten | Replace a wiggly piece (a loop along a bumpy wall or around a cluttered obstacle, a centre line) by fewer, longer straight lines. It tries several tolerances, from "almost the same line" to "one straight line", and only uses straight chords the robot can drive safely. It keeps the version with the lowest total cost, so it only happens where the saved stops and bends outweigh any floor the straighter line no longer reaches. |

Every move is scored with exact cost differences: connector time (straight, or A* around obstacles), stop-and-turn costs at every junction, hairpins, double coverage and missed cells. When no single move helps any more, the optimiser shakes the route up a little (a random reversal and relocation), optimises again, and keeps the result only if it is cheaper. It does this until the time budget runs out.

Each candidate gets *Time per candidate* seconds. The cheapest one then gets *Extra time for best* seconds more.

Cells that no safe robot position can see are shown yellow (unreachable), not red.

Because corners cost a stop plus a rotation, long straight lanes and smooth curves are the cheapest way to drive. Raise the stop penalty for a robot that turns slowly; the planner will then prefer fewer, longer lanes.

## Candidate paths

| Candidate | What it is |
|---|---|
| Spiral, around obstacles / outer walls only | The inward spiral: wall loop first, then rings inward. |
| Straight lanes, one direction | Wall loop and obstacle loops first. Then the whole interior gets parallel lanes, with the lane angle chosen by cost. |
| Mixed regions (split at X deg) | Wall loop and obstacle loops first. The interior is then split into regions with a boustrophedon cell decomposition, and small regions are merged. Every region tries lanes at 0°, 15°, … 165° (plus the wall directions) and a spiral, and keeps whichever is cheapest. Regions are visited in a greedy order, entering each one at its nearest corner and in its best lane direction. |
| Boustrophedon cells, no wall loop (X deg) | Boustrophedon cells swept with straight lanes along the main wall directions, without a wall loop first. With a sensor much wider than the wall clearance (NALA: 0.6 m vs 0.33 m), a corridor is covered by a few parallel lanes, and a wall loop would only add a fragmented, bumpy strip. On the RDG department this candidate wins clearly. |
| Wall loop + boustrophedon cells (X deg) | Classic boustrophedon coverage. After the wall loop, the inside is split into boustrophedon cells (see the *Boustrophedon cells* view). Every cell is swept with one continuous zigzag of straight lanes. The short hop between two lanes is driven with the brush on, so it covers the strip along the edge. All cells use one room-wide lane grid, so neighbouring cells don't double up. A cell edge along an obstacle gets an extra lane, moved slightly inward on bumpy edges so it stays one straight line. |
| Rings or lanes per region | Regional instead of all-or-nothing: after the wall loop, the room is split into regions and **each region** keeps whichever is cheaper: the part of the spiral inside it (its rings, including the loops around obstacles), or straight lanes up to the obstacles, parallel to the walls or along the region's long axis. So a strip where lanes are better gets lanes, even when rings win in the rest of the room. |

Lanes along a bumpy edge: the outermost lane of a region may move inward by up to (coverage radius − robot radius + edge band) if that keeps it one long straight line instead of being chopped into pieces at every bump.

For every candidate the planner then:

1. Adds centre lines for thin uncovered strips.
2. Connects the pieces with collision-checked straight lines, or A* where a straight line isn't safe.
3. Scores the result with the same cost function.

*Pattern: Auto* picks the cheapest candidate. *Lanes / mixed regions* and *Spiral only* limit the search to those patterns.

In the test runs, different candidates win on different maps: straight lanes on the example office, wall loop + boustrophedon cells on the rotated hall and on cluttered rooms, the spiral on the bumpy test room. With 8 candidates, planning takes about 1–2 minutes.

## Settings

| Setting | What it does |
|---|---|
| Robot radius | Keeps the robot's centre at least this far from walls and obstacles. Every path segment and connection is collision-checked with it. For NALA at least the turning circle of its box (0.273 m) + 0.02 m; the default 0.33 m adds a localisation margin. |
| Coverage radius | Radius of the area the sensor covers (0.6 m for NALA's scintillator). |
| Sensor ahead of centre | How far the centre of the covered disk lies straight ahead of the robot centre (0.26 m for NALA; 0 for a centred brush). |
| Overlap (0–50 %) | Loop spacing is computed automatically: **spacing = 2 × coverage radius × (1 − overlap)**. The computed value is shown under the slider. |
| Pattern | *Auto (cheapest)*, *Lanes / mixed regions* or *Spiral only*. |
| Follow the outer walls first | Starts with the wall loop. When this is off, the wall loop is ordered like any other piece. |
| Spiral obstacles | Used for the spiral candidates when Pattern is *Spiral only*. In *Auto* both variants are tried. *Loop around them*: every ring follows the walls **and** the free-standing obstacles. *Outer walls only*: rings keep the shape of the room. Where a ring meets an obstacle it is cut and the robot drives around it. Each obstacle also gets one loop around it, so its edges are still covered. |
| Wall on robot's | *Right* drives counter-clockwise along the outer walls. *Left* drives clockwise. |
| Max goal spacing | How often a Nav2 goal is placed along the path. This does not change the path itself. |
| Gentle bend cost | Seconds per radian for bends that don't need a stop. Raise it for straighter lines. |
| Edge band / Missed edge weight | See "Straight lines versus the last centimetres along a bumpy wall" above. |
| Time per candidate / Extra time for best | Optimisation time budget. More time can give a cheaper route. |

## How the spiral is made

1. **Reachable space.** Cells where the robot fits (clearance ≥ robot radius) form the reachable space. Unknown cells count as obstacles.
2. **Distance field.** A distance field measures how far each point lies inside that space.
   - **Ring 0** (orange) lies at the safety limit, so it runs exactly one robot radius from the walls.
   - **Ring k** (blue) lies k × spacing further inward. The rings are the level lines of the distance field, so they keep the shape of the room.
3. **Splitting.** Deeper inside, the room can split into separate parts, for example the two arms of an L-shape. Each part is then spiralled to its own centre before the robot moves on to the next part.
4. **Centre lines** (teal). Where two rings meet from opposite sides, for example in the middle of a corridor, a thin strip can stay uncovered. The planner drives along the centre line of that strip. Very small centre regions get a single goal.
5. **Connecting and splitting into goals.** Consecutive pieces are connected by straight lines where that is safe, otherwise by A*. The path is then split into Nav2 goals.

## Views (dropdown "View" above the map)

| View | What it shows |
|---|---|
| Plan | The planned pieces by type (wall loops, lanes, rings, centre lines, detours) and the transit between them. |
| Robot path & coverage | The route in driving order as one line, coloured from start (dark) to end (yellow). The sensor's own path (dotted orange, with its swing at every turn on the spot) is shown here, not in the Plan view. Transit is dashed, arrows show the driving direction, black dots are the Nav2 goals. The background shows how often the brush passes each floor cell: red = missed, green = once, yellow = twice, orange = 3 or more. |
| Boustrophedon cells | The boustrophedon cell decomposition of the area where the robot centre can go, with the lanes along the plan's split angle (or the main wall direction). Every cell is a region that parallel lanes can sweep without being cut by an obstacle. |
| Regions used by the plan | For region-based plans: the regions the plan actually used, with the pattern chosen for each (lanes and angle, spiral or rings). |

In code: `render.draw_cells(ax, grid, result['cells'])`, `render.draw_robot_path(ax, grid, result)`, `render.coverage_count(grid, result)` (passes per cell) and `render.robot_route(result)` (the route as `(points, is_transit)` pieces). Command line: `--cells-png cells.png --path-png path.png`.

## Reading the map view

| On the map | Meaning |
|---|---|
| Orange lines | Wall-following loops (and obstacle loops) |
| Purple lines | Straight lanes |
| Blue lines | Spiral rings |
| Teal lines | Centre lines |
| Grey dashed lines | Transit between pieces |
| Green shading | Covered area |
| Red shading | Reachable area that was missed |
| Yellow shading | Too close to a wall for the brush to reach at this robot radius |

Two coverage numbers are shown:

- **Coverage of area** counts the yellow strip as missed.
- **Coverage of reachable** only counts what the robot can physically reach.

If the coverage radius equals the robot radius, the thin strip against the walls can never be reached, so coverage of area stays around 96 %.

## Export format

The YAML/JSON file contains `poses`: the whole driven path in order, transit included. There is a pose at every vertex of the planned path and at least every 0.5 m. This makes Nav2 follow the planned shape instead of planning its own shortcut between far-apart goals. Each pose has `x`, `y`, `yaw`, a quaternion and a `kind` (wall, lane, spiral, fill, gap, transit). `segments` holds the planned pieces, and `stats` and `settings` are included for reference. `map_image_sha256` fingerprints the map image, so the robot can refuse a plan made on a different map.

`drive_path` is the path the robot actually drives (made by `coverage/drivable.py` at the end of planning; the black line and red circles in the preview):

1. **Straighten jogs.** A corner is dropped if a straight line between its neighbours still covers every cell the plan covers and stays clear of walls. Lines that follow LiDAR bumps with jogs of a few centimetres become straight.
2. **Round corners.** Every remaining corner, or group of corners that turn the same way (a bevelled corner, a curve drawn with short segments), gets the widest circular arc that:
   - is tangent to the straight parts before and after it;
   - covers every cell the plan covers (same brush and line-of-sight test as the planner);
   - stays at least the robot radius from walls and obstacles;
   - is at least `drive.min_arc_radius_m` (0.2 m), so the robot's controller can follow it exactly.
3. **Stop corners.** Corners where no such arc exists, and turnarounds of 150° or more, stay sharp. The robot stops there and turns on the spot. So it only stops where that is needed to cover every cell.

`drive_path` holds:
- `points`: straight stretches as their end points, arcs every `drive.spacing_m` (0.05 m);
- `stops`: the indices of the stop corners.

`drive_path` also records `driving: forward_only`: every stretch is driven forwards.

The planner's stats report:
- `drive_arcs` and `drive_stops`;
- `drive_lost_cells` and `drive_unsafe_segments`, which must be 0;
- `footprint_hits`, the real outline check, which must be 0.

The export's `settings.geometry` holds `robot_radius_m`, `coverage_disk_radius_m`, `sensor_offset_m` and `footprint_m`. The robot uses them to refuse a plan made for another sensor.

**On the NALA robot**, run the plan with `app3_coverage_pi.launch.py plan:=FILE`; see *App 3* in the repository README. Plan with **robot radius 0.33 m** (the default):
- NALA refuses a path that comes within 0.293 m of a wall, and this tool's per-cell clearance check is up to half a cell more lenient than NALA's exact one;
- NALA refuses a plan made for another base station (`PLAN_BASE_MISMATCH`);
- NALA refuses a plan made with another `sensor_offset_m` or `coverage_disk_radius_m` (`PLAN_ROBOT_MISMATCH`).

`send_to_nav2.py` is for a plain Nav2 stack only. It does not work with NALA's coverage launch: NALA's velocity gate only passes Nav2 commands while its own supervisor holds the lease, and that supervisor already follows the plan's `drive_path` exactly. This script sends the poses with **NavigateThroughPoses**: the robot drives through them without stopping. Use `--waypoints` for FollowWaypoints (the robot stops at every pose, so it is much slower). `--set-initial-pose` also tells AMCL that the robot is at the plan start. This script has not been tested on a robot yet.

## Command line

```
python plan_cli.py ../maps/house1/map.yaml -o ../maps/house1/plan.yaml --png plan.png   # start = base station next to the map
python plan_cli.py examples/RDG_NM-DEP_V4.yaml --base-station examples/RDG_NM-DEP_V4_base_station.yaml -o out.yaml --png out.png
python plan_cli.py examples/office.yaml --start 2 2 --mode auto --overlap 20 \
       --robot-radius 0.4 --coverage-radius 0.6 --sensor-offset 0.26 -o out.yaml --png out.png
```

## Files

| File | Contents |
|---|---|
| `app.py` | The GUI |
| `plan_cli.py` | Command-line version |
| `coverage/cost.py` | The cost function |
| `coverage/localsearch.py` | Cost-driven route optimiser (drop, trim, straighten, flip, move, 2-opt, detours) |
| `coverage/optimizer.py` | Wall loops, region decomposition, lanes vs spiral (or rings) per region, region ordering |
| `coverage/spiral.py` | Distance field, rings, spiral ordering, centre lines, obstacle loops |
| `coverage/pipeline.py` | Builds all candidates, scores them and returns the cheapest |
| `coverage/sensor.py` | The sensor's path for a path of the robot centre (offset ahead, swing at corners) |
| `coverage/footprint.py` | Independent check of the drivable path against the robot's real outline |
| `coverage/drivable.py` | The drivable path: straightened jogs, rounded corners, stop corners |
| `coverage/grid.py` | Map loader, clearance map, collision checks, A* |
| `coverage/planning.py`, `coverage/planner.py` | Your original modules, reused for coverage calculation, connections and splitting the path into goals |
| `coverage/export.py`, `coverage/render.py`, `coverage/settings.py` | Nav2 export, drawing and settings |
