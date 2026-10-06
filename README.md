# NALA

A 4-wheel mecanum robot that maps a house with a 2D LiDAR and then covers every room on its own.
Built on ROS 2 Jazzy. The Raspberry Pi on the robot runs the robot software.
The PC runs the keyboard teleop, RViz and the map tools.

## Status

**Stage 0: folder structure and docs only. There is no code yet.** Most files are empty placeholders.

| App | What it does | Status |
|---|---|---|
| 1. Teleop | Drive the chassis with the PC keyboard | Not started |
| 2. Mapping | App 1 + real-time LiDAR SLAM on the Pi + RViz on the PC + save the map and copy it to the PC | Not started |
| 3. Coverage | Pick a base station in the saved map on the PC. The robot starts there, covers the whole house, and returns | Not started |

## System

| | PC | Pi |
|---|---|---|
| Hardware | - | Raspberry Pi 4B, 8 GB |
| OS | Ubuntu 24.04 | Ubuntu 24.04 |
| Architecture | amd64 | arm64 |
| ROS 2 | Jazzy | Jazzy |
| Python | 3.12 | 3.12 |

MCU firmware: ATmega328PB, built and flashed with Microchip Studio on Windows.
All dependencies: [requirements.yaml](requirements.yaml).

## Hardware

- Chassis: 4 mecanum wheels.
  Track width 320 mm (half: 160 mm). Wheelbase 260 mm (half: 130 mm). Wheel radius 40 mm (diameter: 80 mm).
- LiDAR: RPLIDAR A2M8 (USB).
- Motor driver: ATmega328PB, on the Pi GPIO UART. It runs a speed PID per wheel and sends back
  the measured wheel speeds. Protocol: [docs/mcu_protocol.md](docs/mcu_protocol.md).
- IMU: none for now (may be added later).

## How it fits together (plan)

```
PC                                   Pi (on the robot)                         MCU
teleop_twist_keyboard --/cmd_vel-->  nala_base --UART velocity command-->       motor_driver
                                     nala_base <--UART measured wheel speeds--  (PID per wheel)
RViz <--/map /scan /tf /odom--       rplidar_ros  (/scan)
                                     slam_toolbox (/map, map->odom)
                                     Nav2 + nala_coverage (stage 3)
```

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
├── docs/
│   └── mcu_protocol.md                    Pi <-> MCU UART protocol and known firmware issues
│
├── firmware/                              MCU motor driver (ATmega328PB), Microchip Studio project.
│   │                                      Unchanged copy of ref/motor_driver (the baseline). [1] fixes it.
│   ├── motor_driver_C.atsln               Studio solution file (open this one)
│   └── motor_driver_C/
│       ├── motor_driver_C.cproj           Studio project: device, compiler and linker settings
│       ├── motor_driver_C.componentinfo.xml   Studio device pack info
│       ├── main.c                         Command parser, kinematics, wheel speed measuring, PID loop, feedback
│       ├── USART.c                        UART driver (9600 baud, 8N1)
│       ├── Usart.h                        UART driver header
│       ├── motor_functions.c              Set PWM duty and direction of each motor
│       ├── pwm.c                          PWM pin and timer setup
│       ├── timer.c                        Timer 3/4 helpers (not used by the main loop)
│       ├── ADC.c                          ADC setup and read (only the setup is called)
│       └── notused.c                      Old code, all commented out
│
├── maps/                                  Saved maps and base station files. The map files are not in git.
│   └── .gitkeep                           (empty) Keeps the folder in git. Stays empty
│
├── scripts/
│   ├── setup_pi.sh                        (empty) [1] One-time Pi setup: UART, serial permissions, dependencies
│   └── save_map.sh                        (empty) [2] Pi: save the current SLAM map into maps/<name>/
│
└── src/                                   ROS 2 packages. The repo root is the colcon workspace
    │
    ├── nala_description/                  Robot model: frames base_footprint, base_link, laser
    │   ├── CMakeLists.txt                 (empty) [2] Build and install rules
    │   ├── package.xml                    (empty) [2] Package manifest
    │   ├── launch/
    │   │   └── description.launch.py      (empty) [2] Start robot_state_publisher with the URDF
    │   └── urdf/
    │       └── nala.urdf.xacro            (empty) [2] Robot frames and LiDAR mount pose
    │
    ├── nala_base/                         Pi <-> MCU driver: /cmd_vel to the MCU, measured wheel speeds to /odom
    │   ├── package.xml                    (empty) [1] Package manifest
    │   ├── setup.py                       (empty) [1] Python package setup and node entry points
    │   ├── setup.cfg                      (empty) [1] Install paths for ros2 run
    │   ├── resource/
    │   │   └── nala_base                  (empty) [1] ament index marker. Stays empty
    │   ├── config/
    │   │   └── base.yaml                  (empty) [1] Serial port, geometry, speed limits, command timeout
    │   ├── launch/
    │   │   └── base.launch.py             (empty) [1] Start base_node with base.yaml
    │   ├── nala_base/
    │   │   ├── __init__.py                (empty) [1] Python package marker. Stays empty
    │   │   ├── mcu_protocol.py            (empty) [1] Build command frames, parse feedback lines (no ROS)
    │   │   ├── kinematics.py              (empty) [1] Mecanum inverse and forward kinematics (no ROS)
    │   │   └── base_node.py               (empty) [1] ROS node: serial I/O, cmd timeout, /odom and TF [2]
    │   └── test/
    │       ├── test_mcu_protocol.py       (empty) [1] Unit tests for mcu_protocol.py
    │       └── test_kinematics.py         (empty) [1] Unit tests for kinematics.py
    │
    ├── nala_bringup/                      Launch files, configs and RViz layouts for the 3 apps
    │   ├── CMakeLists.txt                 (empty) [1] Build and install rules
    │   ├── package.xml                    (empty) [1] Package manifest
    │   ├── config/
    │   │   ├── rplidar.yaml               (empty) [2] RPLIDAR A2M8 driver params: port, frame, scan mode
    │   │   ├── slam_mapping.yaml          (empty) [2] slam_toolbox mapping params, tuned against pose jumps
    │   │   ├── localization.yaml          (empty) [3] Localization in the saved map
    │   │   └── nav2_params.yaml           (empty) [3] Nav2 params: holonomic controller, costmaps
    │   ├── launch/
    │   │   ├── app1_teleop_pi.launch.py   (empty) [1] Pi: base driver
    │   │   ├── app2_slam_pi.launch.py     (empty) [2] Pi: base driver + LiDAR + robot model + SLAM
    │   │   ├── app2_slam_pc.launch.py     (empty) [2] PC: RViz with slam.rviz
    │   │   ├── app3_base_station_pc.launch.py   (empty) [3] PC: show the saved map, click to set the base station
    │   │   ├── app3_coverage_pi.launch.py (empty) [3] Pi: base driver + LiDAR + localization + Nav2 + coverage mission
    │   │   └── app3_coverage_pc.launch.py (empty) [3] PC: RViz with coverage.rviz to watch the mission
    │   └── rviz/
    │       ├── slam.rviz                  (empty) [2] RViz layout for mapping
    │       └── coverage.rviz              (empty) [3] RViz layout for base station picking and coverage
    │
    └── nala_coverage/                     Coverage path planning and the base -> coverage -> base mission
        ├── package.xml                    (empty) [3] Package manifest
        ├── setup.py                       (empty) [3] Python package setup and node entry points
        ├── setup.cfg                      (empty) [3] Install paths for ros2 run
        ├── resource/
        │   └── nala_coverage              (empty) [3] ament index marker. Stays empty
        ├── config/
        │   └── coverage.yaml              (empty) [3] Coverage width, overlap, wall margin
        ├── nala_coverage/
        │   ├── __init__.py                (empty) [3] Python package marker. Stays empty
        │   ├── coverage_planner.py        (empty) [3] Map -> back-and-forth coverage path (no ROS)
        │   ├── coverage_mission_node.py   (empty) [3] Pi: leave the base, follow the coverage path with Nav2, return
        │   └── base_station_picker_node.py   (empty) [3] PC: click in RViz -> maps/<name>/base_station.yaml
        └── test/
            └── test_coverage_planner.py   (empty) [3] Unit tests for coverage_planner.py
```

`ref/` (the previous team's code) exists only on the development machine. It is not in git.

## Setup (one time, on both PC and Pi)

1. Install ROS 2 Jazzy: <https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html>
   (PC: `ros-jazzy-desktop`, Pi: `ros-jazzy-ros-base`).
2. Install the apt packages for your machine from [requirements.yaml](requirements.yaml).
3. Get the code: see the next section. The PC uses the same steps (without ssh).
4. Pi only: run `scripts/setup_pi.sh` (written in stage 1).

## Get new code onto the Pi (git pull over SSH)

Code is never edited on the Pi. Changes are pushed to GitHub from the development machine,
and the Pi pulls them.

Open an SSH terminal to the Pi from the PC:

```bash
ssh nala@<PI_HOSTNAME>.local
```

- `<PI_HOSTNAME>`: run `hostname` on the Pi to see it.
- `.local` needs mDNS (`avahi-daemon` on the Pi) and both machines on the same network.
  If it does not work, use the Pi IP address instead (run `hostname -I` on the Pi).

First time only, in the Pi SSH terminal:

```bash
git clone https://github.com/philipfei/NALA.git ~/NALA
```

Every time there is new code, in the Pi SSH terminal:

```bash
cd ~/NALA
git pull
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Note: in stage 0 the packages are empty, so there is nothing to build yet. `colcon build` works from stage 1 on.

## How to run the applications

Will be written when each app is finished: the exact commands for the **PC terminal** and the **Pi terminal**.

- App 1 (teleop): not ready yet.
- App 2 (mapping): not ready yet.
- App 3 (coverage): not ready yet.

## Build and flash the MCU firmware

Will be written in stage 1 (Microchip Studio on Windows, open `firmware/motor_driver_C.atsln`).
