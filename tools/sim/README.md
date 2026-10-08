# Closed-loop simulation

Drives an exported coverage_tool plan through the **real** NALA stack on a PC, without the robot:
- `app3_coverage_pi.launch.py plan:=… hardware:=false` with Nav2 (map server, AMCL, planner, controller) and the
  the robot's URDF (`robot_state_publisher`, LiDAR facing backward);
- the coverage supervisor, coverage meter and velocity gate;
- a fake NALA (`nala_base`, MCU and LiDAR) in place of the hardware.

Build the workspace first (`colcon build --symlink-install --base-paths src`, main README).

Use it to check a plan, a config change or a code change before taking it to the robot.

| File | Purpose |
|---|---|
| `fake_nala.py` | Kinematic NALA, like `nala_base` + MCU. Drives on `/cmd_vel` (forward, sideways, turn) in the MCU protocol's 0.02 steps, scaled down to `max_wheel_speed`, and stops after `cmd_vel_timeout` without a command (`nala_base`'s own functions and `config/base.yaml`). Publishes `/odom` at 20 Hz, TF `odom → base_footprint` and a ray-cast 300-beam `/scan` in the `laser` frame (mount from `robot.yaml`: facing backward). Checks NALA's 410 × 360 mm box (`robot.yaml`) against walls and obstacles. Logs the true pose and velocity. |
| `run_sim.py` | Starts the launch and the fake robot at the base station (AMCL starts there too), and acts as the operator: preview, start, resume after listed pauses. A finished task ends back at the base station (`RETURNED_TO_BASE`). Writes everything to `runs/<name>/`. |
| `analyze.py` | `summary.json` and `plot.png` for one run. |
| `coverage_map.py` | Covered vs. missed floor per run, next to the plan followed exactly. Uses NALA's coverable area and its sensor disk, 0.26 m ahead of the robot. |
| `amcl_error.py` | AMCL position error against the true pose. |
| `env.sh` | ROS environment for the simulation (own domain 77, localhost only, FastDDS). |

**What is simulated, and what isn't**
- Walls come from the map image. `--obstacle X,Y,R` adds round obstacles that exist in the world but not in the map Nav2 loads.
- A move that would make NALA's box touch a wall or obstacle is not made and counts as a **contact**.
- **Not simulated:** odometry drift and wheel slip, LiDAR motion blur, and the real MCU's speed controller (dead band at low speed, coasting stop). Odometry is perfect, so AMCL does better than on the robot.
- Everything runs in real time.

## Run it

The simulation also needs `python3-scipy`.

```bash
cd /path/to/NALA
source tools/sim/env.sh
export SIM_RUNS_DIR=/tmp/nala-sim      # outside the editor's workspace: its file watcher slows the PC
cd tools/sim
# The room plan (coverage_tool/examples/map_ME_room1v4_nala.yaml), robot on its base station
python3 run_sim.py room --max-resumes 40 --pause-timeout 60
python3 analyze.py $SIM_RUNS_DIR/room
python3 coverage_map.py $SIM_RUNS_DIR/room.png room
# A box that is not in the map, on the path
python3 run_sim.py box --obstacle 1.6,-2.6,0.15 --max-resumes 40 --pause-timeout 60
```

The department plan takes longer than the default mission deadline (`deadlines.mission_s`, 1800 s) allows on a busy PC. Use a config copy with a longer deadline for it:

```bash
cp -r ../../config $SIM_RUNS_DIR/config_rdg
sed -i 's/      mission_s: 1800.0/      mission_s: 3600.0/' $SIM_RUNS_DIR/config_rdg/coverage.yaml
python3 run_sim.py rdg --map ../../coverage_tool/examples/RDG_NM-DEP_V4.yaml \
    --plan ../../coverage_tool/examples/RDG_NM-DEP_V4_nala.yaml \
    --config $SIM_RUNS_DIR/config_rdg --timeout 4500 --max-resumes 100 --pause-timeout 180
```

**Options of `run_sim.py`** (`--help` for all):

| Option | What it does |
|---|---|
| `--plan`, `--map` | Other plan and map. |
| `--base-station FILE` | Base station (default: `<map>_base_station.yaml` in `examples/`, else `base_station.yaml` next to the map). The fake robot boots there. |
| `--start-offset DX DY DYAW` | Place the robot this far from the base station, to test a robot not put down exactly. |
| `--config DIR` | Another config directory. Copy `config/` and edit the copy to try parameters. |
| `--resume-on` | Pause reasons after which it calls `/coverage/resume`. |
| `--max-resumes N`, `--pause-timeout S` | How often it resumes (default 5) and how long a pause may last before it stops (default 15 s). On a busy PC, timing pauses (`TF_UNAVAILABLE`, `SCAN_STALE`, `AMCL_STALE`) come more often: the fake robot is one Python process and lags when other programs take the CPU. |

Only one simulation can run per ROS domain. `run_sim.py` refuses to start while another one is running in the same domain.

## Reading the results

`runs/<name>/` (or `$SIM_RUNS_DIR/<name>/`) contains:

| File | Contents |
|---|---|
| `result.json` | Final supervisor state, timeline, resumes, errors |
| `summary.json` | See the fields below |
| `plot.png` | Plan, driven path, obstacles, skipped stretches |
| `truth.csv` | True pose, velocity and contact flag at 10 Hz |
| `amcl.json` | AMCL pose and sigma |
| `tf.json` | `map→odom` timing, trust reasons, gate faults |
| `launch.log` | Output of all launched nodes |
| `robot.log` | Output of the fake robot |
| `output/task_*.json` | The supervisor's task report |

Key fields in `summary.json`:
- `meter_fraction`: coverage as the robot's meter measured it, at the sensor.
- `box_contacts`: times NALA's box would have touched something. **Must be 0.**
- `reverse_distance_m`: distance driven backwards. **Must be about 0**, because autodrive is forward only.
- `distance_to_path_median_m`, `_p95_m`, `_max_m`: how closely the robot followed the drivable path.
- `skipped_poses`: `PLAN_STRETCH_SKIPPED` stretches, each with its cause.
- `min_clearance_m`: closest approach of the robot centre to a wall.

In the coverage map, the colours mean:
- **green:** covered;
- **red:** coverable but missed;
- **grey:** free floor that no sensor position can reach.

## Troubleshooting

| Symptom | Cause and fix |
|---|---|
| `AMCL never published a pose`, or lifecycle errors in `launch.log` | A process from an earlier run is still alive in the same domain, or stale FastDDS shared memory. Check with `pgrep -af "app3_coverage_pi.launch\|fake_nala\|run_sim"`, stop those processes, then run `fastdds shm clean`. |
| Occasional `TF_UNAVAILABLE` / `SCAN_STALE` pauses | Timing hiccups of a loaded PC; `run_sim.py` resumes after them. |
| `resume refused: …` | The reason is printed. For example, a pose too close to a wall: the robot then stays paused. |
