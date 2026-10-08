# NALA

A 4-wheel mecanum robot that maps a house with a 2D LiDAR and then covers every room on its own.
Built on ROS 2 Jazzy. The Raspberry Pi on the robot runs the robot software.
The PC runs the keyboard teleop, RViz and the map tools. An Xbox controller (paired with the Pi) can drive too.
For coverage (app 3), a 7 inch touchscreen on the robot shows the panel: choose a map, set the base station,
plan the path, choose a plan and drive it.

## Status

| App | What it does | Status |
|---|---|---|
| 1. Teleop | Drive the chassis with the PC keyboard or an Xbox controller | Code done. Hardware test pending (motors not connected yet) |
| 2. Mapping | App 1 + real-time LiDAR SLAM on the Pi + RViz on the PC + save the map and copy it to the PC | Code done. Tested without driving (motors not connected). SLAM tuning needs driving |
| 3. Coverage | On the robot screen: choose a map from app 2, set the base station, plan a coverage path (from the base station or another start point, optionally only an area), choose a plan and drive it. The robot starts at the base station, covers, and returns | Code done. Tested in simulation only (`tools/sim`) |

## System

| | PC | Pi |
|---|---|---|
| Hardware | - | Raspberry Pi 4B, 8 GB, hostname `NALA` |
| OS | Ubuntu 24.04 | Ubuntu 24.04 |
| Architecture | amd64 | arm64 |
| ROS 2 | Jazzy (ros-base + rviz2) | Jazzy (ros-base) |
| DDS | Cyclone DDS | Cyclone DDS |
| Python | 3.12 | 3.12 |

Network: the PC shares its connection on USB Ethernet (10.42.0.1/24). The Pi is on WiFi in that network.
ROS uses only this network (`config/cyclonedds.xml`), `ROS_DOMAIN_ID=0`.

MCU firmware: ATmega328PB, owned by a teammate, built with Microchip Studio on Windows and flashed with avrdude.
All dependencies: [requirements.yaml](requirements.yaml).

## Hardware

- Chassis: 4 mecanum wheels. Outer size 410 x 360 mm (length x width).
  Track width 320 mm (half: 160 mm). Wheelbase 260 mm (half: 130 mm). Wheel radius 40 mm (diameter: 80 mm).
- Motors: M1 front-left, M2 rear-left, M3 rear-right, M4 front-right. Encoders: 1536 counts per wheel revolution.
- LiDAR: RPLIDAR A2M8 (USB, CP2102 adapter, 115200 baud), at the chassis center, facing backward. Height not measured yet.
- Motor driver: ATmega328PB on the Pi GPIO UART (`/dev/ttyS0`). It runs a speed controller per wheel and sends back
  the cumulative encoder counts every 50 ms at 38400 baud (control loop 20 Hz, stops after 200 ms without a command).
  Firmware v1.1.0 or newer is needed. Protocol: [docs/mcu_protocol.md](docs/mcu_protocol.md).
- IMU: none for now (may be added later).
- Xbox Wireless Controller (`C8:3F:26:93:1B:B2`), paired with the Pi over Bluetooth.
- Screen (app 3): Waveshare 7 inch HDMI LCD (C), 1024 x 600, capacitive touch over USB. On the Pi HDMI port.

## How it fits together

```
PC                                   Pi (on the robot)                         MCU
nala_teleop keyboard --/cmd_vel-->   nala_base --UART velocity frame, 20 Hz-->  motor_driver
Xbox --Bluetooth--> joy_node -> teleop_twist_joy --/cmd_vel--> nala_base
                                     nala_base <--UART encoder counts, 20 Hz-   (PI per wheel)
                                     nala_base: wheel odometry (/odom, odom->base_footprint)
RViz <--/map /scan /tf--             rplidar_ros  (/scan)                       [stage 2]
                                     robot_state_publisher (base_footprint->base_link->laser)
                                     slam_toolbox (/map, map->odom)             [stage 2]

App 3 (all on the Pi; the PC is optional):
Robot screen: nala_panel --starts--> app3_coverage_pi.launch.py, --runs--> coverage_tool/plan_cli.py
  map_server + AMCL (starts at the base station): map->odom
  Nav2 planner + controller --/cmd_vel_nav--> velocity gate (nala_coverage) --/cmd_vel--> nala_base
  coverage supervisor (follows the plan, detours, returns to the base station) + coverage meter
RViz on the PC (optional) <--/map /scan /tf /coverage/*--
```

## Settings: the `config/` folder

All settings you may want to change are in [config/](config/): speeds, motor limit, timeouts, serial port,
teleop speeds (keyboard and Xbox controller), network, robot model, LiDAR, SLAM, localization, Nav2,
coverage and the robot screen.
Edit them on the PC, push, then `git pull` on the Pi (see below) and restart the app.
An edited file works without a rebuild. A new file needs a rebuild.

## File tree

`(empty)` = placeholder with no content yet.
`[1]` `[2]` `[3]` = the first app (stage) that needs the file.

```
NALA/
├── CLAUDE.md                              Rules and project facts for Claude Code
├── README.md                              This file
├── requirements.yaml                      System versions and all dependencies (PC, Pi, firmware)
├── .gitignore                             What git ignores: build output, ref/, map files
├── .gitattributes                         Force LF line endings (the code runs on Linux)
│
├── config/                                All settings the user may change (installed by nala_bringup)
│   ├── base.yaml                          [1] Pi base driver: serial port, motor limit, /cmd_vel timeout, send rate.
│   │                                      [2] Wheel geometry for odometry
│   ├── teleop.yaml                        [1] Fixed teleop speeds: PC keyboard, Pi Xbox controller (buttons, sticks)
│   ├── ros_env.sh                         [1] ROS environment for PC and Pi: domain ID, Cyclone DDS
│   ├── cyclonedds.xml                     [1] Cyclone DDS: use only the robot network 10.42.0.0/24
│   ├── robot.yaml                         [2] Robot model: body size, LiDAR mount pose (read by the URDF)
│   ├── rplidar.yaml                       [2] RPLIDAR A2M8 driver params: port, baud rate, frame, scan mode
│   ├── slam_mapping.yaml                  [2] slam_toolbox mapping params, tuned against pose jumps
│   ├── localization.yaml                  [3] AMCL (localization in the saved map) and the map server
│   ├── nav2_params.yaml                   [3] Nav2 planner (NavFn), controller (Regulated Pure Pursuit, forward only), costmaps
│   ├── coverage.yaml                      [3] Coverage sensor (placeholder), coverage speeds, frames, mission, health, recovery
│   └── panel.yaml                         [3] Robot screen (nala_panel): map folder, window and button size, update rates
│
├── docs/
│   └── mcu_protocol.md                    Pi-side summary of the Pi <-> MCU protocol, first hardware test
│
├── firmware/                              MCU motor driver (ATmega328PB), Microchip Studio project.
│   │                                      Owned by a teammate, who builds and flashes it. v1.1.1 (by Claude).
│   ├── README.md                          Protocol (source of truth), parameters, build/flash, changes per version
│   ├── docs/
│   │   └── state_machine.svg              Firmware state machine (labels in Chinese)
│   ├── tests/                             Model tests and simulations (Python, run on a PC)
│   │   ├── rx_parser_model_test.py        UART frame parser
│   │   ├── timeout_model_test.py          200 ms command timeout
│   │   ├── motor_polarity_model_test.py   Motor direction pins
│   │   ├── pid_sim.py                     Speed controller simulation
│   │   └── tx_budget.py                   Feedback line load at 38400 baud
│   ├── motor_driver_C.atsln               Studio solution file (open this one)
│   └── motor_driver_C/
│       ├── motor_driver_C.cproj           Studio project: device, compiler and linker settings
│       ├── motor_driver_C.componentinfo.xml   Studio device pack info
│       ├── config.h                       All firmware parameters and the wiring map
│       ├── main.c                         Command handling, kinematics, PID loop, feedback, timeout
│       ├── encoder.c / encoder.h          Encoder counting in pin-change interrupts
│       ├── USART.c / Usart.h              UART driver (8N1, TX ring buffer; baud rate in config.h)
│       ├── motor_functions.c / .h         Set PWM duty and direction of each motor
│       ├── pwm.c / pwm.h                  PWM pin and timer setup
│       ├── timer.c / timer.h              Timer helpers (Timer3 is the 50 ms control tick)
│       ├── ADC.c / ADC.h                  ADC setup and read
│       └── notused.c                      Old code, all commented out
│
├── coverage_tool/                         [3] Coverage path planner (Python, no ROS). Used by the robot screen
│   │                                      (plan_cli.py) and on the PC (app.py). Details: coverage_tool/README.md
│   ├── README.md                          How the planner works, its cost function, settings and commands
│   ├── CLAUDE.md                          Planner facts for Claude Code
│   ├── requirements.txt                   Python libraries of the planner (for pip on other machines; apt on the Pi)
│   ├── .gitignore                         Python cache files
│   ├── app.py                             PC: interactive planner window (Tkinter)
│   ├── plan_cli.py                        Plan from the command line: map + base station (+ start, area) -> plan .yaml/.png
│   ├── send_to_nav2.py                    Only for a plain Nav2 stack, not used by NALA (see coverage_tool/README.md)
│   ├── coverage/                          The planner: grid, sensor model, patterns, cost, optimiser, drivable path, export
│   │   └── *.py                           (one module per step; list in coverage_tool/README.md)
│   ├── examples/                          Example maps (*.yaml + *.pgm), map generators (make_*.py), NALA example plans
│   │                                      (*_nala.yaml) and their base stations (*_base_station.yaml)
│   └── tests/
│       ├── test_sensor.py                 Unit tests: sensor ahead of the robot, outline check, geometry from config/
│       └── regression.py                  Acceptance test on the example maps (about 15 minutes; --quick for 2 cases)
│
├── maps/                                  Saved maps and base station files. The map files are not in git.
│   │                                      Per map: map.yaml/.pgm (app 2), base_station.yaml, plans/, runs/ (app 3)
│   └── .gitkeep                           (empty) Keeps the folder in git. Stays empty
│
├── scripts/
│   ├── setup_pi.sh                        [1] One-time Pi setup: ROS, Cyclone DDS, UART, serial permissions, ~/.bashrc
│   ├── save_map.sh                        [2] Pi: save the current SLAM map into maps/<name>/
│   ├── setup_pi_screen.sh                 [3] One-time Pi setup for the robot screen: desktop, autologin, Nav2, panel start
│   └── start_panel.sh                     [3] Start the robot screen panel (at login, or by hand)
│
├── tools/
│   └── sim/                               [3] PC: closed-loop simulation of app 3 with a fake robot (see tools/sim/README.md)
│       ├── README.md                      How to run it, options, what is simulated
│       ├── env.sh                         ROS environment for the simulation (own domain, localhost only)
│       ├── fake_nala.py                   Fake nala_base + MCU + LiDAR (uses nala_base's speed limit and config/base.yaml)
│       ├── run_sim.py                     Runs app 3 against the fake robot like an operator, records the run
│       ├── analyze.py                     Summary and plot of a run
│       ├── coverage_map.py                Covered and missed floor of a run
│       └── amcl_error.py                  AMCL error against the true pose
│
└── src/                                   ROS 2 packages. The repo root is the colcon workspace
    │
    ├── nala_description/                  Robot model: frames base_footprint, base_link, laser
    │   ├── CMakeLists.txt                 [2] Install launch/ and urdf/
    │   ├── package.xml                    [2] Package manifest
    │   ├── launch/
    │   │   └── description.launch.py      [2] Start robot_state_publisher with the URDF (arg robot_config)
    │   └── urdf/
    │       └── nala.urdf.xacro            [2] Robot frames and LiDAR mount pose (numbers from config/robot.yaml)
    │
    ├── nala_base/                         Pi <-> MCU driver: /cmd_vel to the MCU, wheel odometry
    │   ├── package.xml                    [1] Package manifest
    │   ├── setup.py                       [1] Python package setup and node entry points
    │   ├── setup.cfg                      [1] Install paths for ros2 run
    │   ├── resource/
    │   │   └── nala_base                  (empty) [1] ament index marker. Stays empty
    │   ├── nala_base/
    │   │   ├── __init__.py                (empty) [1] Python package marker. Stays empty
    │   │   ├── mcu_protocol.py            [1] Build velocity frames, parse feedback lines (no ROS)
    │   │   ├── kinematics.py              [1] Wheel speed limit. [2] Wheel speeds -> body speed, pose integration (no ROS)
    │   │   └── base_node.py               [1] ROS node: serial I/O, wheel speed limit, /cmd_vel timeout. [2] /odom and TF
    │   └── test/
    │       ├── test_mcu_protocol.py       [1] Unit tests for mcu_protocol.py
    │       └── test_kinematics.py         [1] Unit tests for kinematics.py
    │
    ├── nala_teleop/                       PC keyboard teleop with fixed speeds (no speed keys)
    │   ├── package.xml                    [1] Package manifest
    │   ├── setup.py                       [1] Python package setup and node entry points
    │   ├── setup.cfg                      [1] Install paths for ros2 run
    │   ├── resource/
    │   │   └── nala_teleop                (empty) [1] ament index marker. Stays empty
    │   ├── nala_teleop/
    │   │   ├── __init__.py                (empty) [1] Python package marker. Stays empty
    │   │   ├── keys.py                    [1] Key layout: key -> drive direction, help text (no ROS)
    │   │   └── keyboard_node.py           [1] ROS node: read keys in the terminal, publish /cmd_vel
    │   └── test/
    │       └── test_keys.py               [1] Unit tests for keys.py
    │
    ├── nala_bringup/                      Launch files and RViz layouts for the 3 apps. Installs config/
    │   ├── CMakeLists.txt                 [1] Install launch/, rviz/ and the repo config/ folder
    │   ├── package.xml                    [1] Package manifest
    │   ├── launch/
    │   │   ├── app1_teleop_pi.launch.py   [1] Pi: base driver, Xbox controller (joy + teleop_twist_joy)
    │   │   ├── app2_slam_pi.launch.py     [2] Pi: app 1 + robot model + LiDAR + SLAM
    │   │   ├── app2_slam_pc.launch.py     [2] PC: RViz with slam.rviz
    │   │   ├── app3_base_station_pc.launch.py   [3] PC: show a saved map, click the base station in RViz (the robot screen can do this too)
    │   │   ├── app3_coverage_pi.launch.py [3] Pi: base driver + robot model + LiDAR + AMCL + Nav2 + coverage mission + velocity gate
    │   │   └── app3_coverage_pc.launch.py [3] PC: RViz with coverage.rviz to watch the mission
    │   └── rviz/
    │       ├── slam.rviz                  [2] RViz layout for mapping: map, scan, robot, frames
    │       └── coverage.rviz              [3] RViz layout for base station picking and coverage
    │
    ├── nala_coverage/                     The base -> coverage -> base mission with Nav2, velocity gate, coverage meter
    │   ├── package.xml                    [3] Package manifest
    │   ├── setup.py                       [3] Python package setup and node entry points
    │   ├── setup.cfg                      [3] Install paths for ros2 run
    │   ├── resource/
    │   │   └── nala_coverage              (empty) [3] ament index marker. Stays empty
    │   ├── nala_coverage/
    │   │   ├── __init__.py                [3] Python package marker
    │   │   ├── supervisor.py              [3] Mission: preview/start/pause/resume/cancel, Nav2 actions, detours, return to base
    │   │   ├── plan_file.py               [3] Load and check a coverage_tool plan (map, wall clearance, sensor, base station)
    │   │   ├── planning.py                [3] Plan pieces and path helpers (no ROS)
    │   │   ├── base_station.py            [3] Load, save and check base_station.yaml (no ROS)
    │   │   ├── base_station_picker_node.py   [3] PC: RViz "2D Goal Pose" -> maps/<name>/base_station.yaml
    │   │   ├── gate_node.py               [3] Velocity gate: the only /cmd_vel publisher in app 3; forward only for Nav2
    │   │   ├── health.py                  [3] Ownership and localization trust checks (no ROS)
    │   │   ├── meter.py                   [3] Coverage meter node: covered floor from trusted poses, at the sensor
    │   │   ├── measurement.py             [3] Coverage painting (no ROS)
    │   │   ├── geometry.py                [3] Occupancy grid, reachability, coverable floor (no ROS)
    │   │   ├── params.py                  [3] Load and check config/ (outline from robot.yaml), render the Nav2 params
    │   │   ├── keyboard_teleop.py         [3] coverage_teleop: drive by hand through the gate while a task is paused
    │   │   ├── mode_guard.py              [3] Refuse to start app 3 next to SLAM or another /cmd_vel publisher
    │   │   ├── ros_common.py              [3] Shared ROS helpers (QoS, messages, sensor freshness)
    │   │   └── state.py                   [3] Mission bookkeeping (no ROS)
    │   └── test/
    │       ├── test_core.py, test_plan_file.py, test_base_station.py, test_revision.py   [3] Unit tests (no ROS)
    │       ├── test_launch_contract.py    [3] Static checks of the app 3 launch files
    │       └── test_ros.py                [3] Loopback ROS tests: gate, supervisor, meter, plan driving, return to base
    │
    └── nala_panel/                        Robot screen for app 3 (PyQt5): maps, base station, planning, driving
        ├── package.xml                    [3] Package manifest
        ├── setup.py                       [3] Python package setup and node entry points
        ├── setup.cfg                      [3] Install paths for ros2 run
        ├── resource/
        │   └── nala_panel                 (empty) [3] ament index marker. Stays empty
        ├── nala_panel/
        │   ├── __init__.py                (empty) [3] Python package marker. Stays empty
        │   ├── library.py                 [3] Map library: maps, base station, plans, can a plan be driven (no ROS, no Qt)
        │   ├── planning.py                [3] Run coverage_tool/plan_cli.py, check start and area, cancel (no Qt)
        │   ├── drive.py                   [3] Start/stop app 3, follow its state, Start/Pause/Resume/Cancel/STOP, hand control (no Qt)
        │   └── ui.py                      [3] The screens: Maps, Map (plans, new plan), Planning, Drive
        └── test/
            ├── conftest.py                [3] Test map library (the example room map)
            ├── test_library.py            [3] Unit tests for library.py
            ├── test_planning.py           [3] Planning tests (one real plan on a small map, about 1 minute)
            ├── test_drive.py              [3] Loopback ROS tests of the drive session with fake app 3 services
            └── test_ui.py                 [3] Screen smoke test without a display (NALA_PANEL_SHOTS=<dir> saves screenshots)
```

`ref/` (the previous team's code) exists only on the development machine. It is not in git.

## Setup (one time)

### PC

ROS 2 Jazzy is already installed. In a PC terminal:

```bash
sudo ufw status                      # is 10.42.0.0/24 allowed? If not:
sudo ufw allow from 10.42.0.0/24     # DDS traffic from the Pi
```

Build the workspace on the PC (again after every pull that adds or changes a package), in the repo folder:

```bash
source config/ros_env.sh
colcon build --symlink-install --base-paths src
source config/ros_env.sh             # now also loads install/setup.bash
```

`--base-paths src`: build only the packages in `src/` (`ref/` has old ROS 1 packages).

### Pi

The Pi needs internet (through the PC). In a PC terminal:

```bash
ssh-copy-id nala@nala.local          # optional: log in without a password from now on
ssh nala@nala.local
```

Then in the Pi SSH terminal:

```bash
git clone https://github.com/philipfei/NALA.git ~/NALA
bash ~/NALA/scripts/setup_pi.sh      # asks for the sudo password, takes about 15 minutes
sudo reboot
```

`setup_pi.sh` installs ROS 2 and the tools from [requirements.yaml](requirements.yaml), gives `nala` access to
the UART and the game controller, removes the Linux serial console from `/dev/ttyS0`,
and adds `source ~/NALA/config/ros_env.sh` to `~/.bashrc`.
When a pull changes `setup_pi.sh` (new packages), run it again and log in again (new groups need a new login).

Robot screen for app 3 (one time, after `setup_pi.sh`), in the Pi SSH terminal:

```bash
bash ~/NALA/scripts/setup_pi_screen.sh   # desktop with autologin, Nav2, PyQt5, planner libraries, panel autostart
cd ~/NALA && colcon build --symlink-install --base-paths src
sudo reboot
```

After the reboot the panel starts full-screen on the 7 inch screen. First check that the picture fills the
screen and that a tap lands where you touch. The Waveshare 7 inch HDMI LCD (C) normally works without a driver
(HDMI + USB touch); if the picture or the touch is wrong, see the screen's manual before changing
`/boot/firmware/config.txt`.

Xbox controller (one time): pair it with the Pi. In the Pi SSH terminal run `bluetoothctl`, then
`scan on`, hold the pair button on the controller until the Xbox logo blinks fast, then
`pair C8:3F:26:93:1B:B2`, `trust C8:3F:26:93:1B:B2`, `connect C8:3F:26:93:1B:B2`, `exit`.
After that it connects by itself when it is switched on.

## Get new code onto the Pi (git pull over SSH)

Code is never edited on the Pi. Changes are pushed to GitHub from the development machine,
and the Pi pulls them. In the Pi SSH terminal (`ssh nala@nala.local`):

```bash
cd ~/NALA
git pull
colcon build --symlink-install --base-paths src
source ~/NALA/config/ros_env.sh
```

- `.local` needs mDNS (`avahi-daemon` on the Pi) and both machines on the same network.
  If it does not work, use the Pi IP address instead (run `hostname -I` on the Pi).
- `--base-paths src`: build only the packages in `src/`.
- Changed only a file in `config/`? Then `git pull` is enough, no build.

## How to run the applications

### App 1: teleop (keyboard or Xbox controller)

**Pi terminal** (`ssh nala@nala.local`):

```bash
ros2 launch nala_bringup app1_teleop_pi.launch.py
```

Add `log_level:=debug` to see every frame sent to the MCU and the wheel speeds (from the encoder counts).

**PC terminal** (in the repo folder, for the keyboard; not needed for the Xbox controller):

```bash
source config/ros_env.sh
ros2 run nala_teleop keyboard_teleop --ros-args --params-file config/teleop.yaml
```

Keys (keep the teleop terminal focused):

```
u  i  o        i = forward, , = backward, j / l = turn left / right
j  k  l        u o m . = drive and turn at the same time
m  ,  .        k (or any other key) = stop
Shift + U I O J L M < > = drive without turning (J / L = move sideways)
```

- **Hold** a key to drive. When you release it, the robot stops after 0.6 s (`cmd_vel_timeout`).
- Speeds are fixed: 0.45 m/s and 1.0 rad/s (`config/teleop.yaml`). There are no speed keys.

Xbox controller (switch it on; it connects to the Pi by itself):

- **Hold LB** to drive. Release LB = stop at once.
- Left stick: up / down = forward / backward, left / right = move sideways.
- Right stick left / right = turn.
- Full stick = 0.45 m/s and 1.0 rad/s (`config/teleop.yaml`).

Both:

- Strafe and turn work at the same time. If a command needs a wheel faster than the motors can go
  (`max_wheel_speed` in `config/base.yaml`), the Pi slows the whole command down by the same factor.
- Stop: Ctrl+C in either terminal. The Pi sends a stop frame when it exits.

### App 2: mapping

**Pi terminal 1** (`ssh nala@nala.local`): app 1 + robot model + LiDAR + SLAM

```bash
ros2 launch nala_bringup app2_slam_pi.launch.py
```

**PC terminal 1** (in the repo folder): RViz shows the map, the LiDAR scan (red) and the robot

```bash
source config/ros_env.sh
ros2 launch nala_bringup app2_slam_pc.launch.py
```

**PC terminal 2** (only for the keyboard): the same keyboard teleop as app 1

```bash
source config/ros_env.sh
ros2 run nala_teleop keyboard_teleop --ros-args --params-file config/teleop.yaml
```

Drive slowly through every room (the Xbox controller with half stick is easiest).
Turn slowly, and come back through rooms you have already seen. The map grows in RViz.

**Save the map** while app 2 still runs. **Pi terminal 2** (`ssh nala@nala.local`):

```bash
bash ~/NALA/scripts/save_map.sh house1        # any new name; existing maps are never overwritten
```

This writes `~/NALA/maps/house1/`: `map.pgm` + `map.yaml` (the map image) and
`map.posegraph` + `map.data` (the SLAM graph, for localization in app 3).

**Copy the map to the PC**, PC terminal (in the repo folder):

```bash
scp -r nala@nala.local:NALA/maps/house1 maps/
```

Then stop app 2 with Ctrl+C in the Pi terminal 1.

### App 3: coverage

Uses a map saved by app 2 (`maps/<name>/` on the Pi). Everything runs on the Pi; the robot screen is the
normal way to use it. The PC is optional (RViz, or planning on the PC).

#### On the robot screen

The panel starts by itself after the Pi boots (`scripts/setup_pi_screen.sh`). By hand, in a terminal on the Pi:
`bash ~/NALA/scripts/start_panel.sh`.

1. **Maps:** tap the map (every folder in `maps/` with a `map.yaml` from app 2).
2. **New plan** tab:
   - **Set base station:** tap where the robot stands, then tap the direction it faces. It is saved as
     `maps/<name>/base_station.yaml`. The robot needs room to turn there (0.293 m from walls), else the panel refuses.
   - **Coverage starts at:** *Base station*, or *Start point* + **Set start point** (tap). With a start point the
     robot still stands on the base station: Nav2 first drives it to the start point.
   - **Area** (optional): **Draw area**, tap the corners, **Close area**. Only the floor inside is covered.
     **Undo corner** and **Clear area** to change it. No area = all floor the robot can reach.
   - **Plan.** Planning runs on the Pi and can take several minutes. The result shows the coverage and four
     checks (all reachable floor covered, robot outline touches nothing, no unsafe segment, no lost floor).
     **Keep plan** saves it in `maps/<name>/plans/`; **Discard** deletes it.
3. **Plans** tab: every saved plan, newest first. A plan the robot would refuse is grey, with the reason
   (made on another map image, for another base station, or for other sensor values). **Delete** removes one.
4. **Drive:** put the robot on the base station (within about 10 cm), facing the base station direction
   (within about 10 deg). Tap **Drive** on a plan. The panel starts app 3 and shows:
   - *Finding the robot on the map*, then *Checking the plan* (the preview);
   - **Start** becomes active when the plan is checked. **The robot only moves after Start.**
   - while driving: the robot, the plan and the covered floor on the map, the state and the % covered;
   - **STOP** (red, always there): the robot stops at once and the task pauses. **Resume** continues.
   - **Pause / Resume / Cancel task.** When paused (for example `BLOCKED_IN_PLACE`), **Drive by hand** and the
     arrow buttons move the robot through the velocity gate (hold to drive). Tap **Hand control off**, then **Resume**.
   - At the end the robot drives back to the base station: *Finished*, `RETURNED_TO_BASE`.
   - **Finish** stops app 3 and shows the task report (also saved in `maps/<name>/runs/<time>/`).

Only one app at a time: stop app 1 and app 2 first. App 3 does not start the Xbox controller.

#### Pi terminal (without the screen)

The same as the panel's **Drive**, over ssh (`ssh nala@nala.local`). The plan must already exist
(`maps/<name>/plans/`, from the screen or the PC):

```bash
ros2 launch nala_bringup app3_coverage_pi.launch.py map:=$HOME/NALA/maps/house1/map.yaml \
    plan:=$HOME/NALA/maps/house1/plans/2026-10-08_143005.yaml output_dir:=$HOME/NALA/maps/house1/runs/manual
```

Pi terminal 2: wait until `/coverage/state` shows `"trusted": true`, then:

```bash
ros2 service call /coverage/preview std_srvs/srv/Trigger '{}'
ros2 topic echo /coverage/state --once --full-length     # preview_ready must be true
ros2 service call /coverage/start std_srvs/srv/Trigger '{}'
ros2 service call /coverage/pause std_srvs/srv/Trigger '{}'    # resume / cancel likewise
```

Drive by hand while paused (W/S/A/D, `Q` to leave): `ros2 run nala_coverage coverage_teleop`.
Do not use `nala_teleop`'s keyboard or the Xbox controller during app 3: they publish `/cmd_vel` directly, the
gate sees a second publisher and stops the robot (`VELOCITY_GRAPH_CONFLICT`).

#### PC terminal (optional)

Watch the mission in RViz (in the repo folder):

```bash
source config/ros_env.sh
ros2 launch nala_bringup app3_coverage_pc.launch.py
```

Plan on the PC instead of the robot (faster): copy the map folder from the Pi, plan, copy it back.

```bash
scp -r nala@nala.local:NALA/maps/house1 maps/
ros2 launch nala_bringup app3_base_station_pc.launch.py map:=maps/house1/map.yaml   # if the base station is not set yet
python3 coverage_tool/plan_cli.py maps/house1/map.yaml -o maps/house1/plans/pc_plan.yaml --png maps/house1/plans/pc_plan.png
scp -r maps/house1 nala@nala.local:NALA/maps/
```

`plan_cli.py` options: `--start X Y` (coverage start, default the base station), `--area x1 y1 x2 y2 ...`.
The interactive planner on the PC: `python3 coverage_tool/app.py maps/house1/map.yaml`.

#### Robot geometry for coverage

All values in `base_footprint`/`base_link` (on the floor under the chassis centre), x forward, y left.

| What | Value | From |
|---|---|---|
| Outer box (must never touch anything) | x +-0.205 m, y +-0.18 m | `config/robot.yaml` (`body_length` x `body_width`) |
| Farthest corner from the centre | 0.273 m | derived |
| Collision radius | 0.273 + 0.02 padding = **0.293 m** | `planning_padding_m` in `config/coverage.yaml` |
| Plan robot radius (coverage_tool) | **0.33 m** | + 3 cm for localization, rounded up to 1 cm |
| Coverage sensor (scintillator) point | (+0.26, 0) m, static TF `scintillator_link` | `config/coverage.yaml` (**placeholder**) |
| Sensor coverage | disk of 0.6 m radius around that point | `config/coverage.yaml` (**placeholder**) |

The outline is configured once, in `robot.yaml`; `nala_coverage/params.py` and `coverage_tool` derive the rest.
The robot turns on the spot anywhere on its path, so plans keep its centre 0.33 m from walls: the narrowest
passage it uses is about 0.66 m. When the sensor position and size are known, change them in
`config/coverage.yaml` and plan again: the robot refuses plans made for other sensor values.

#### How the plan is driven

- **Start:** AMCL starts at the base station (the supervisor sends it as the initial pose, spread 0.1 m and
  10 deg). Nav2's planner (`ComputePathToPose`) drives the robot to the plan start if it is not there.
- **Following:** the plan's drivable path is split at its stop corners. Each piece is followed forwards with
  Nav2's controller (`FollowPath`, Regulated Pure Pursuit, no reversing); the robot turns on the spot between pieces.
- **Blocked path:** the controller stops in front of an obstacle. The supervisor skips 0.5 m ahead (more after
  repeated failures), drives around it with the planner and continues on the path. Skipped stretches are in the
  task report (`PLAN_STRETCH_SKIPPED`); that floor is not covered.
- **No way past:** after 12 attempts or 2 m of skipped plan without moving, the task pauses
  (`BLOCKED_IN_PLACE`). Remove the obstacle or drive by hand, then resume.
- **Return:** after the plan the planner drives the robot back to the base station and turns it to the base
  station direction. If that fails, the task pauses (`RETURN_FAILED`); resume tries again.
- **Forward only:** the plan only drives forwards, the controller cannot reverse (no behavior server), and the
  velocity gate passes no negative forward speed from Nav2.
- **Speed steps:** the MCU protocol has steps of 0.02 m/s; the controller's slowest speed is 0.04 m/s, so a
  command never rounds to 0 near a goal.
- **Refused:** plans closer than 0.293 m to a wall, made on another map image, for other sensor values or for
  another base station; a base station closer than 0.293 m to a wall.
- **Obstacles the LiDAR cannot see** (lower or higher than its scan plane) are not avoided. NALA has no bumper.

#### First run on the robot

Keep a hand on the robot's emergency stop.

1. **LiDAR:** the scan must line up with the map when the robot stands still and when it turns.
2. **Localization:** put the robot on the base station and tap **Drive** on a plan. Before **Start**, drive by
   hand is not possible, so check the map on the screen: the robot arrow must be on the base station.
   AMCL values in `config/localization.yaml` were tuned in simulation only.
3. **Costmaps:** with RViz on the PC, put a box in front of the robot: it must appear in `/local_costmap/costmap`.
4. **Plan driving:** a short plan in open space first. Then a box on the path: the robot must stop before it,
   skip past it, drive around and continue. At the end it must drive back to the base station.
5. **Speeds:** `motion` in `config/coverage.yaml` holds placeholders (0.12 m/s, 0.4 rad/s). Raise them only
   after the controller is tuned on the floor.

#### Simulation on the PC

`tools/sim` runs app 3 with Nav2 against a fake robot, in its own ROS domain (see `tools/sim/README.md`):

```bash
source tools/sim/env.sh && export SIM_RUNS_DIR=/tmp/nala-sim && cd tools/sim
python3 run_sim.py room --max-resumes 40 --pause-timeout 60
python3 analyze.py $SIM_RUNS_DIR/room
```

The panel against the simulation (in the repo folder; not `start_panel.sh`, which loads the robot network settings):

```bash
source tools/sim/env.sh
python3 tools/sim/fake_nala.py --map maps/house1/map.yaml --start <base station x y yaw> --log /tmp/truth.csv &
ros2 run nala_panel panel --ros-args --params-file config/panel.yaml -p repo_dir:=$PWD \
    -p fullscreen:=false -p hardware:=false
```

#### Tests (PC)

```bash
source config/ros_env.sh
python3 -m pytest -q -p no:cacheprovider src/nala_coverage/test src/nala_panel/test   # about 3 minutes
cd coverage_tool && python3 -m pytest -q -p no:cacheprovider tests/test_sensor.py
```

`test_ros.py` runs real ROS nodes on loopback in one Python process. On a busy PC a plan-driving test sometimes
pauses with `TF_UNAVAILABLE`; run it again on its own (`-k <name>`) before treating it as a real failure.

## Build and flash the MCU firmware

The firmware is owned by a teammate. See [firmware/README.md](firmware/README.md):
build with Microchip Studio on Windows (`firmware/motor_driver_C.atsln`, F7), flash with `avrdude` and an ArduinoISP.
The exact commands are in section 5 of the firmware README.
