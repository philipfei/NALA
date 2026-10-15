# NALA

A 4-wheel mecanum robot that maps a house with a 2D LiDAR and then covers every room on its own.
Built on ROS 2 Jazzy. The Raspberry Pi on the robot runs the robot software.
The PC runs the keyboard teleop, RViz and the map tools. An Xbox controller (paired with the Pi) can drive too.

## Status

| App | What it does | Status |
|---|---|---|
| 1. Teleop | Drive the chassis with the PC keyboard or an Xbox controller | Driven 2026-10-07. Firmware v2.1.0: MCU link, motors and encoders checked in the air (2026-10-15). Speed control tuning with v2.2.0 pending |
| 2. Mapping | App 1 + real-time LiDAR SLAM on the Pi (wheel odometry + IMU in an EKF) + RViz on the PC + save the map and copy it to the PC | Code done. IMU and EKF checked on the Pi (2026-10-09). The new MCU link (firmware v2.1.0) is not tested on the robot yet: see [docs/handover.md](docs/handover.md). SLAM tuning needs driving |
| 3. Coverage | Pick a base station in the saved map on the PC. The robot starts there, covers the whole house, and returns | Not started |

## System

| | PC | Pi |
|---|---|---|
| Hardware | - | Raspberry Pi 4B, 8 GB, hostname `NALA` |
| OS | Ubuntu 24.04 | Ubuntu 24.04 |
| Architecture | amd64 | arm64 |
| ROS 2 | Jazzy (ros-base + rviz2) | Jazzy (ros-base + rviz2) |
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
- Motor driver: ATmega328PB (5 V). UART PD0/PD1 <-> level shifter <-> Pi GPIO14/15 (`/dev/ttyAMA0`), 38400 baud,
  binary frames with CRC. It runs a speed controller per wheel (control loop 20 Hz, stops after 200 ms without
  a command) and sends the cumulative encoder counts every 50 ms. Firmware v2.1.0 or newer is needed.
  Protocol: [docs/mcu_protocol.md](docs/mcu_protocol.md).
- Level shifter on the board: a TXB type (the schematic says TXS0104E). Fine for UART, cannot do I2C.
- IMU: WitMotion WT901C-TTL (3.3 V) on a CH340G USB-to-TTL adapter (`/dev/ttyIMU`), flat, chip side up.
  Only its turn rate is used (EKF in app 2): [docs/imu_ekf.md](docs/imu_ekf.md).
- Battery: 4S2P Li-ion 18650 (3300 mAh per cell: 16.8 V full, 6.6 Ah). Battery monitor: TI INA260 on the Pi I2C
  bus 1 (GPIO2/3), read by `battery_node` (`/battery_state`, `/battery_low` below 14.0 V).
- Xbox Wireless Controller (`C8:3F:26:93:1B:B2`), paired with the Pi over Bluetooth.

## How it fits together

```
PC                                   Pi (on the robot)                         MCU
nala_teleop keyboard --/cmd_vel-->   nala_base --UART command frame, 20 Hz-->  motor_driver
Xbox --Bluetooth--> joy_node -> teleop_twist_joy --/cmd_vel--> nala_base
                                     nala_base <--UART state frame, 20 Hz----  (PI per wheel)
                                     nala_base: wheel odometry (/odom; TF odom->base_footprint in app 1)
INA260 --I2C--> battery_node (/battery_state, /battery_low)
WT901 IMU --USB serial--> imu_node (/imu)                                      [stage 2]
                                     ekf_node: /odom vx,vy + /imu turn rate -> TF odom->base_footprint
RViz <--/map /scan /tf--             rplidar_ros  (/scan)                       [stage 2]
                                     robot_state_publisher (base_footprint->base_link->laser, imu_link)
                                     slam_toolbox (/map, map->odom)             [stage 2]
                                     Nav2 + nala_coverage                       [stage 3]
```

## Settings: the `config/` folder

All settings you may want to change are in [config/](config/): speeds, motor limit, timeouts, serial port,
teleop speeds (keyboard and Xbox controller), network, robot model, LiDAR, SLAM, and later Nav2 and coverage.
Edit them on the PC, push, then `git pull` on the Pi (see below) and restart the app.
An edited file works without a rebuild. A new file needs a rebuild.

## File tree

`(empty)` = placeholder with no content yet.
`[1]` `[2]` `[3]` = the first app (stage) that needs the file.

```
NALA/
├── CLAUDE.md                              Shared Claude Code rules (tracked, edited by others). The maintainer's own rules are in CLAUDE.local.md (in .gitignore, not in git)
├── README.md                              This file
├── requirements.yaml                      System versions and all dependencies (PC, Pi, firmware)
├── .gitignore                             What git ignores: build output, ref/, map files
├── .gitattributes                         Force LF line endings (the code runs on Linux)
│
├── config/                                All settings the user may change (installed by nala_bringup)
│   ├── base.yaml                          [1] Pi base driver: serial port, motor limit, /cmd_vel timeout, send rate.
│   │                                      [2] Wheel geometry for odometry, odometry TF on/off, odometry variances
│   ├── teleop.yaml                        [1] Fixed teleop speeds: PC keyboard, Pi Xbox controller (buttons, sticks)
│   ├── ros_env.sh                         [1] ROS environment for PC and Pi: domain ID, Cyclone DDS
│   ├── cyclonedds.xml                     [1] Cyclone DDS: use only the robot network 10.42.0.0/24
│   ├── robot.yaml                         [2] Robot model: body size, LiDAR and IMU mount poses (read by the URDF)
│   ├── battery.yaml                       [1] INA260 battery monitor: I2C address, full / low / empty voltage
│   ├── tune_motors.yaml                   [1] Test settings of scripts/tune_motors.py (step times, sweep speeds, timeouts).
│   │                                      The controller parameters themselves are in the MCU EEPROM
│   ├── imu.yaml                           [2] WT901 IMU: serial port, baud rate, gyro variance; rate written by setup_imu.py
│   ├── ekf.yaml                           [2] robot_localization EKF: wheel vx, vy + IMU turn rate -> odom TF
│   ├── rplidar.yaml                       [2] RPLIDAR A2M8 driver params: port, baud rate, frame, scan mode
│   ├── slam_mapping.yaml                  [2] slam_toolbox mapping params, tuned against pose jumps
│   ├── localization.yaml                  (empty) [3] Localization in the saved map
│   ├── nav2_params.yaml                   (empty) [3] Nav2 params: holonomic controller, costmaps
│   └── coverage.yaml                      (empty) [3] Coverage width, overlap, wall margin
│
├── docs/
│   ├── handover.md                        State on 2026-10-12, steps after the Pi reinstall, prompt for the next Claude session
│   ├── mcu_protocol.md                    Pi-side summary of the Pi <-> MCU UART protocol, first hardware test
│   └── imu_ekf.md                         [2] IMU wiring and setup, EKF design: why only the gyro turn rate
│
├── firmware/                              MCU motor driver (ATmega328PB), Microchip Studio project.
│   │                                      Owned by a teammate, who builds and flashes it. v2.2.0 (by Claude).
│   ├── README.md                          Protocol (source of truth), parameters, build/flash, changes per version
│   ├── prayer.jpg                         The team's prayer picture (JPEG). Also the Pi boot splash (scripts/setup_splash.sh)
│   ├── docs/
│   │   └── state_machine.svg              Firmware state machine (labels in Chinese)
│   ├── tests/                             Model tests and simulations (Python, run on a PC)
│   │   ├── uart_frame_model_test.py       UART frames: receive state machine, parameter frames, CRC-8 (same bytes as the Pi)
│   │   ├── timeout_model_test.py          200 ms command timeout (also after a pause, e.g. an EEPROM save)
│   │   ├── param_model_test.py            Setting mode: parameter ops, busy rule, NaN/Inf, FF table check, EEPROM CRC-16, ramp
│   │   ├── motor_polarity_model_test.py   Motor direction pins
│   │   ├── pid_sim.py                     Speed controller simulation
│   ├── arduino_uart_test/
│   │   ├── README.md                      What the two bench tests are
│   │   ├── arduino_uart_test.ino          Arduino Uno bench test: listens on SoftwareSerial (D13, D12 in turn), decodes the MCU state frames, counts CRC errors and lost frames
│   │   └── mcu_uart_tx_test/              Bench test firmware for the MCU (Studio project, not the robot firmware): sends a fake state frame every 50 ms
│   │       ├── mcu_uart_tx_test.atsln     Studio solution file (open this one)
│   │       └── mcu_uart_tx_test/
│   │           ├── mcu_uart_tx_test.cproj Studio project (ATmega328PB, 16 MHz)
│   │           ├── mcu_uart_tx_test.componentinfo.xml   Studio device pack info (copy of the motor_driver_C one)
│   │           └── main.c                 UART0 38400 8N1, TX only, fake v2.1.0 state frames
│   ├── motor_driver_C.atsln               Studio solution file (open this one)
│   └── motor_driver_C/
│       ├── motor_driver_C.cproj           Studio project: device, compiler and linker settings
│       ├── motor_driver_C.componentinfo.xml   Studio device pack info
│       ├── config.h                       All firmware parameters and the wiring map
│       ├── main.c                         Command handling, kinematics, PI loop + ramp, feedback, timeout, setting mode + EEPROM
│       ├── encoder.c / encoder.h          Encoder counting in pin-change interrupts
│       ├── USART.c / Usart.h              UART driver (TX ring buffer; the frame parser is in main.c)
│       ├── motor_functions.c / .h         Set PWM duty and direction of each motor
│       ├── pwm.c / pwm.h                  PWM pin and timer setup
│       ├── timer.c / timer.h              Timer helpers (Timer3 is the 50 ms control tick)
│       ├── ADC.c / ADC.h                  ADC setup and read
│       └── notused.c                      Old code, all commented out
│
├── maps/                                  Saved maps and base station files. The map files are not in git.
│   └── .gitkeep                           (empty) Keeps the folder in git. Stays empty
│
├── scripts/
│   ├── setup_pi.sh                        [1] One-time Pi setup: ROS, Cyclone DDS, UART, I2C, USB serial rules, permissions, ~/.bashrc
│   ├── setup_imu.py                       [2] Pi, once: IMU output (acc + gyro), 50 Hz, 115200 baud
│   ├── tune_motors.py                     [1] Pi: MCU speed controller tuning (show/set/save parameters, step, sweep, ff test)
│   ├── setup_splash.sh                    Pi, once: boot splash = firmware/prayer.jpg (Plymouth theme, rebuilds the initramfs)
│   ├── splash/
│   │   ├── nala.plymouth                  Plymouth theme description (script plugin)
│   │   └── nala.script                    Plymouth script: the picture on black, scaled to the screen, centered
│   └── save_map.sh                        [2] Pi: save the current SLAM map into maps/<name>/
│
└── src/                                   ROS 2 packages. The repo root is the colcon workspace
    │
    ├── nala_description/                  Robot model: frames base_footprint, base_link, laser
    │   ├── CMakeLists.txt                 [2] Install launch/ and urdf/
    │   ├── package.xml                    [2] Package manifest
    │   ├── launch/
    │   │   └── description.launch.py      [2] Start robot_state_publisher with the URDF (arg robot_config)
    │   └── urdf/
    │       └── nala.urdf.xacro            [2] Robot frames, LiDAR and IMU mount poses (numbers from config/robot.yaml)
    │
    ├── nala_base/                         Pi <-> MCU driver (UART): /cmd_vel to the MCU, wheel odometry; IMU; battery
    │   ├── package.xml                    [1] Package manifest
    │   ├── setup.py                       [1] Python package setup and node entry points
    │   ├── setup.cfg                      [1] Install paths for ros2 run
    │   ├── resource/
    │   │   └── nala_base                  (empty) [1] ament index marker. Stays empty
    │   ├── nala_base/
    │   │   ├── __init__.py                (empty) [1] Python package marker. Stays empty
    │   │   ├── mcu_protocol.py            [1] UART command, state and parameter frames, CRC-8 (no ROS)
    │   │   ├── step_metrics.py            [1] Step-response metrics of the wheel speeds, for tune_motors.py (no ROS)
    │   │   ├── kinematics.py              [1] Wheel speed limit. [2] Wheel speeds -> body speed, pose integration (no ROS)
    │   │   ├── base_node.py               [1] ROS node: UART, wheel speed limit, /cmd_vel timeout. [2] /odom (and TF)
    │   │   ├── ina260.py                  [1] INA260 register decoding, voltage -> charge (no ROS)
    │   │   ├── battery_node.py            [1] ROS node: INA260 -> /battery_state, /battery_low
    │   │   ├── wt901.py                   [2] WT901 IMU serial frames (no ROS)
    │   │   └── imu_node.py                [2] ROS node: IMU serial port -> /imu
    │   └── test/
    │       ├── test_mcu_protocol.py       [1] Unit tests for mcu_protocol.py
    │       ├── test_ina260.py             [1] Unit tests for ina260.py
    │       ├── test_kinematics.py         [1] Unit tests for kinematics.py
    │       ├── test_step_metrics.py       [1] Unit tests for step_metrics.py
    │       └── test_wt901.py              [2] Unit tests for wt901.py
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
    │   │   ├── app1_teleop_pi.launch.py   [1] Pi: base driver, battery monitor, Xbox controller (joy + teleop_twist_joy)
    │   │   ├── app2_slam_pi.launch.py     [2] Pi: app 1 + robot model + LiDAR + SLAM
    │   │   ├── app2_slam_pc.launch.py     [2] PC: RViz with slam.rviz
    │   │   ├── app3_base_station_pc.launch.py   (empty) [3] PC: show the saved map, click to set the base station
    │   │   ├── app3_coverage_pi.launch.py (empty) [3] Pi: base driver + LiDAR + localization + Nav2 + coverage mission
    │   │   └── app3_coverage_pc.launch.py (empty) [3] PC: RViz with coverage.rviz to watch the mission
    │   └── rviz/
    │       ├── slam.rviz                  [2] RViz layout for mapping: map, scan, robot, frames
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

The Pi runs Ubuntu 24.04 (Desktop or Server). At the installation: user name `nala`, computer name `NALA`,
then join the WiFi of the PC's shared network. On Ubuntu Desktop install the SSH server first, in a terminal
on the Pi: `sudo apt install openssh-server`.
The Pi needs internet (through the PC). In a PC terminal:

```bash
ssh-keygen -R nala.local             # only after a reinstall of the Pi: forget its old SSH key
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
the UART, I2C and the game controller, removes the Linux serial console from the GPIO UART, puts the PL011 UART
on GPIO14/15 for the MCU (`dtoverlay=miniuart-bt`, `core_freq=500` + `core_freq_min=500`, Bluetooth moves to the mini UART),
makes the fixed name `/dev/ttyIMU` for the IMU's CH340G adapter (udev rule), removes `brltty` and stops
ModemManager (on Ubuntu Desktop they disturb USB serial adapters), and adds `source ~/NALA/config/ros_env.sh` to `~/.bashrc`.

IMU (only for a new IMU; ours is already set since 2026-10-09 and keeps it): with nothing running,
`python3 ~/NALA/scripts/setup_imu.py`. It sets the IMU to 115200 baud, 50 Hz, acceleration + angular velocity only
(see [docs/imu_ekf.md](docs/imu_ekf.md)).

Battery monitor: `i2cdetect -y 1` must show the INA260 (expected at `40`); if it is at another address,
change `i2c_address` in `config/battery.yaml`.
When a pull changes `setup_pi.sh` (new packages), run it again and log in again (new groups need a new login).

Boot splash (one time, optional): `bash ~/NALA/scripts/setup_splash.sh` (asks for the sudo password, about 1 minute).
It shows `firmware/prayer.jpg` while the Pi boots, on a connected display (after the Pi's rainbow screen).
To go back to the Ubuntu splash: `sudo update-alternatives --config default.plymouth`, then `sudo update-initramfs -u`.
Without a display the boot is the same: the desktop is ready about 50 s after power-on (`plymouth-quit-wait` waits
for the GNOME autologin session, measured 2026-10-15; SSH works earlier).

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

### Tune the wheel speed controller (MCU firmware v2.2.0)

The PI gains, the target ramp and the feed-forward table live in the MCU (RAM, saved in its EEPROM).
Stop app 1 / app 2 first (same UART). **Pi terminal** (`ssh nala@nala.local`):

```bash
python3 ~/NALA/scripts/tune_motors.py show                  # what the MCU uses now
python3 ~/NALA/scripts/tune_motors.py ff                    # wheels in the air: measure the feed-forward table
python3 ~/NALA/scripts/tune_motors.py sweep                 # wheels in the air: steps in 6 directions, 4 speeds
python3 ~/NALA/scripts/tune_motors.py step --vx 0.15        # one step, metrics per wheel (--csv FILE for the data)
python3 ~/NALA/scripts/tune_motors.py set kp 2 accel_limit 20   # change in RAM (only while stopped)
python3 ~/NALA/scripts/tune_motors.py save                  # keep it after power-off (EEPROM)
python3 ~/NALA/scripts/tune_motors.py reload                # undo unsaved changes
python3 ~/NALA/scripts/tune_motors.py defaults              # the firmware defaults (config.h); then save
python3 ~/NALA/scripts/tune_motors.py sweep --floor         # on the floor: short steps, each pair comes back
```

Every motion asks first ("Wheels lifted or floor clear?"). Ctrl+C stops at once; `ff` puts the original gains back.
Parameter names: `kp`, `ki`, `i_limit`, `accel_limit`, `ff_speed0`..`ff_speed9`, `ff_pwm0`..`ff_pwm9`.

### App 2: mapping

**Pi terminal 1** (`ssh nala@nala.local`): app 1 + robot model + LiDAR + IMU + EKF + SLAM

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

App 3 (coverage): not ready yet.

## Build and flash the MCU firmware

The firmware is owned by a teammate. See [firmware/README.md](firmware/README.md):
build with Microchip Studio on Windows (`firmware/motor_driver_C.atsln`, F7), flash with `avrdude` and an ArduinoISP.
The exact commands are in section 5 of the firmware README.
