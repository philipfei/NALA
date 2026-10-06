# NALA

A 4-wheel mecanum robot that maps a house with a 2D LiDAR and then covers every room on its own.
Built on ROS 2 Jazzy. The Raspberry Pi on the robot runs the robot software.
The PC runs the keyboard teleop, RViz and the map tools.

## Status

| App | What it does | Status |
|---|---|---|
| 1. Teleop | Drive the chassis with the PC keyboard | Code done. Hardware test pending (MCU not connected yet) |
| 2. Mapping | App 1 + real-time LiDAR SLAM on the Pi + RViz on the PC + save the map and copy it to the PC | Not started |
| 3. Coverage | Pick a base station in the saved map on the PC. The robot starts there, covers the whole house, and returns | Not started |

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

MCU firmware: ATmega328PB, written by a teammate, built and flashed with Microchip Studio on Windows.
All dependencies: [requirements.yaml](requirements.yaml).

## Hardware

- Chassis: 4 mecanum wheels. Outer size 410 x 360 mm (length x width).
  Track width 320 mm (half: 160 mm). Wheelbase 260 mm (half: 130 mm). Wheel radius 40 mm (diameter: 80 mm).
- Motors: M1 front-left, M2 rear-left, M3 rear-right, M4 front-right. Encoders: 1536 counts per wheel revolution.
- LiDAR: RPLIDAR A2M8 (USB), at the chassis center, facing backward.
- Motor driver: ATmega328PB on the Pi GPIO UART (`/dev/ttyS0`). It runs a speed PID per wheel and sends back
  the measured wheel speeds every 100 ms. Protocol: [docs/mcu_protocol.md](docs/mcu_protocol.md).
- IMU: none for now (may be added later).

## How it fits together

```
PC                                   Pi (on the robot)                         MCU
teleop_twist_keyboard --/cmd_vel-->  nala_base --UART velocity frame, 10 Hz-->  motor_driver
                                     nala_base <--UART wheel speeds, 10 Hz---   (PID per wheel)
RViz <--/map /scan /tf /odom--       rplidar_ros  (/scan)                       [stage 2]
                                     slam_toolbox (/map, map->odom)             [stage 2]
                                     Nav2 + nala_coverage                       [stage 3]
```

## Settings: the `config/` folder

All settings you may want to change are in [config/](config/): speed limits, timeouts, serial port,
teleop start speeds, network, and later LiDAR, SLAM, Nav2 and coverage.
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
│   ├── base.yaml                          [1] Pi base driver: serial port, speed limits, /cmd_vel timeout, send rate
│   ├── teleop.yaml                        [1] PC keyboard teleop start speeds
│   ├── ros_env.sh                         [1] ROS environment for PC and Pi: domain ID, Cyclone DDS
│   ├── cyclonedds.xml                     [1] Cyclone DDS: use only the robot network 10.42.0.0/24
│   ├── rplidar.yaml                       (empty) [2] RPLIDAR A2M8 driver params: port, frame, scan mode
│   ├── slam_mapping.yaml                  (empty) [2] slam_toolbox mapping params, tuned against pose jumps
│   ├── localization.yaml                  (empty) [3] Localization in the saved map
│   ├── nav2_params.yaml                   (empty) [3] Nav2 params: holonomic controller, costmaps
│   └── coverage.yaml                      (empty) [3] Coverage width, overlap, wall margin
│
├── docs/
│   └── mcu_protocol.md                    Pi-side summary of the Pi <-> MCU protocol, first hardware test
│
├── firmware/                              MCU motor driver (ATmega328PB), Microchip Studio project.
│   │                                      Written and owned by a teammate (v0.1.0). Claude does not edit it.
│   ├── CHANGES.md                         What changed from the old firmware (English)
│   ├── 变化.md                            Same as CHANGES.md (Chinese)
│   ├── docs/
│   │   ├── protocol.md                    The protocol, source of truth (Chinese)
│   │   ├── state_machine.svg              UART frame parser state machine
│   │   └── 笔记.md                        Refactoring notes (Chinese)
│   ├── motor_driver_C.atsln               Studio solution file (open this one)
│   └── motor_driver_C/
│       ├── motor_driver_C.cproj           Studio project: device, compiler and linker settings
│       ├── motor_driver_C.componentinfo.xml   Studio device pack info
│       ├── config.h                       All firmware parameters and the wiring map
│       ├── main.c                         Command handling, kinematics, PID loop, feedback, timeout
│       ├── encoder.c / encoder.h          Encoder counting in pin-change interrupts
│       ├── USART.c / Usart.h              UART driver (9600 baud, 8N1, TX ring buffer)
│       ├── motor_functions.c / .h         Set PWM duty and direction of each motor
│       ├── pwm.c / pwm.h                  PWM pin and timer setup
│       ├── timer.c / timer.h              Timer helpers (Timer3 is the 100 ms control tick)
│       ├── ADC.c / ADC.h                  ADC setup and read
│       └── notused.c                      Old code, all commented out
│
├── maps/                                  Saved maps and base station files. The map files are not in git.
│   └── .gitkeep                           (empty) Keeps the folder in git. Stays empty
│
├── scripts/
│   ├── setup_pi.sh                        [1] One-time Pi setup: ROS, Cyclone DDS, UART, serial permissions, ~/.bashrc
│   └── save_map.sh                        (empty) [2] Pi: save the current SLAM map into maps/<name>/
│
└── src/                                   ROS 2 packages. The repo root is the colcon workspace
    │
    ├── nala_description/                  Robot model: frames base_footprint, base_link, laser
    │   ├── COLCON_IGNORE                  (empty) Skip this package until stage 2 fills it
    │   ├── CMakeLists.txt                 (empty) [2] Build and install rules
    │   ├── package.xml                    (empty) [2] Package manifest
    │   ├── launch/
    │   │   └── description.launch.py      (empty) [2] Start robot_state_publisher with the URDF
    │   └── urdf/
    │       └── nala.urdf.xacro            (empty) [2] Robot frames and LiDAR mount pose
    │
    ├── nala_base/                         Pi <-> MCU driver: /cmd_vel to the MCU, measured wheel speeds
    │   ├── package.xml                    [1] Package manifest
    │   ├── setup.py                       [1] Python package setup and node entry points
    │   ├── setup.cfg                      [1] Install paths for ros2 run
    │   ├── resource/
    │   │   └── nala_base                  (empty) [1] ament index marker. Stays empty
    │   ├── nala_base/
    │   │   ├── __init__.py                (empty) [1] Python package marker. Stays empty
    │   │   ├── mcu_protocol.py            [1] Build velocity frames, parse feedback lines (no ROS)
    │   │   ├── kinematics.py              [1] Speed limits. [2] Forward kinematics for odometry (no ROS)
    │   │   └── base_node.py               [1] ROS node: serial I/O, limits, /cmd_vel timeout. [2] /odom and TF
    │   └── test/
    │       ├── test_mcu_protocol.py       [1] Unit tests for mcu_protocol.py
    │       └── test_kinematics.py         [1] Unit tests for kinematics.py
    │
    ├── nala_bringup/                      Launch files and RViz layouts for the 3 apps. Installs config/
    │   ├── CMakeLists.txt                 [1] Install launch/ and the repo config/ folder
    │   ├── package.xml                    [1] Package manifest
    │   ├── launch/
    │   │   ├── app1_teleop_pi.launch.py   [1] Pi: base driver
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
        ├── COLCON_IGNORE                  (empty) Skip this package until stage 3 fills it
        ├── package.xml                    (empty) [3] Package manifest
        ├── setup.py                       (empty) [3] Python package setup and node entry points
        ├── setup.cfg                      (empty) [3] Install paths for ros2 run
        ├── resource/
        │   └── nala_coverage              (empty) [3] ament index marker. Stays empty
        ├── nala_coverage/
        │   ├── __init__.py                (empty) [3] Python package marker. Stays empty
        │   ├── coverage_planner.py        (empty) [3] Map -> back-and-forth coverage path (no ROS)
        │   ├── coverage_mission_node.py   (empty) [3] Pi: leave the base, follow the coverage path with Nav2, return
        │   └── base_station_picker_node.py   (empty) [3] PC: click in RViz -> maps/<name>/base_station.yaml
        └── test/
            └── test_coverage_planner.py   (empty) [3] Unit tests for coverage_planner.py
```

`ref/` (the previous team's code) exists only on the development machine. It is not in git.

## Setup (one time)

### PC

ROS 2 Jazzy is already installed. Missing for app 1 (run in a PC terminal):

```bash
sudo apt install ros-jazzy-teleop-twist-keyboard
sudo ufw status                      # is 10.42.0.0/24 allowed? If not:
sudo ufw allow from 10.42.0.0/24     # DDS traffic from the Pi
```

The PC does not need to build anything for app 1.
Do not run `colcon build` in the repo on the PC: the repo drive is NTFS, and a build there crashed the ntfs3 kernel driver.

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
the UART, removes the Linux serial console from `/dev/ttyS0`, and adds `source ~/NALA/config/ros_env.sh` to `~/.bashrc`.

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

### App 1: keyboard teleop

**Pi terminal** (`ssh nala@nala.local`):

```bash
ros2 launch nala_bringup app1_teleop_pi.launch.py
```

Add `log_level:=debug` to see every frame sent to the MCU and the measured wheel speeds.

**PC terminal** (in the repo folder):

```bash
source config/ros_env.sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --params-file config/teleop.yaml
```

Keys (keep the teleop terminal focused):

```
u  i  o        i = forward, , = backward, j / l = turn left / right
j  k  l        u o m . = drive and turn at the same time
m  ,  .        k (or any other key) = stop
Shift + U I O J L M < > = drive without turning (J / L = move sideways)
q / z = all speeds +10 % / -10 %,  w / x = linear only,  e / c = turn only
```

- **Hold** a key to drive. When you release it, the robot stops after 0.6 s (`cmd_vel_timeout`).
- Limits: 2.0 m/s and 1.0 rad/s (`config/base.yaml`), even if the teleop shows higher values.
- Stop: Ctrl+C in either terminal. The Pi sends a stop frame when it exits.

App 2 (mapping) and app 3 (coverage): not ready yet.

## Build and flash the MCU firmware

The firmware is written by a teammate. See [firmware/CHANGES.md](firmware/CHANGES.md):
open `firmware/motor_driver_C.atsln` in Microchip Studio on Windows (device pack ATmega_DFP 1.7.374), build, flash.
