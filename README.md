# NALA

A 4-wheel mecanum robot that maps a house with a 2D LiDAR and then covers every room on its own.
Built on ROS 2 Jazzy. The Raspberry Pi on the robot runs the robot software.
The PC runs the keyboard teleop, RViz and the map tools. An Xbox controller (paired with the Pi) can drive too.
For coverage (app 3), a 7 inch touchscreen on the robot shows the panel: choose a map, set the base station,
plan the path, choose a plan and drive it.

## Status

| App | What it does | Status |
|---|---|---|
| 1. Teleop | Drive the chassis with the PC keyboard or an Xbox controller | Driven 2026-10-07. Firmware v2.3.x: speed control tuned on the floor (2026-10-15). Driving test 2026-10-16: forward and strafe odometry exact |
| 2. Mapping | App 1 + real-time LiDAR SLAM on the Pi (wheel odometry + IMU in an EKF) + RViz on the PC + save the map and copy it to the PC | First map of one room 2026-10-16: clean. IMU yaw checked in a 360 deg turn. Next: the real house (similar rooms), see [docs/handover.md](docs/handover.md) |
| 3. Coverage | On the robot screen: choose a map from app 2, set the base station, plan a coverage path (from the base station or another start point, optionally only an area), choose a plan and drive it. The robot starts at the base station, covers, and returns | Code done. Tested in simulation only (`tools/sim`). Not tested on the robot |

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
ROS uses only this network (`config/cyclonedds.xml`), `ROS_DOMAIN_ID=0`. Without it (no WiFi), the Pi uses the loopback (`config/cyclonedds_local.xml`): the robot screen and the apps still work, the PC does not see the robot.

MCU firmware: ATmega328PB, owned by a teammate, built with Microchip Studio on Windows and flashed with avrdude.
All dependencies: [requirements.yaml](requirements.yaml).

## Hardware

- Chassis: 4 mecanum wheels. Body 415 x 360 mm (length x width) centred on the wheel centre, plus a rod 235 mm in front
  as wide as the body: outer size 650 x 360 mm, not centred on the turning centre.
  Track width 320 mm (half: 160 mm). Wheelbase 335 mm (half: 167.5 mm). Wheel radius 40 mm (diameter: 80 mm).
- Motors: M1 front-left, M2 rear-left, M3 rear-right, M4 front-right. Encoders: 1536 counts per wheel revolution.
- LiDAR: RPLIDAR A2M8 (USB, CP2102 adapter, 115200 baud), 95 mm in front of the chassis center (since 2026-10-23; 0.115 in the config until 2026-10-26 was wrong), facing backward, 172.5 mm above the floor.
- Motor driver: ATmega328PB (5 V). UART PD0/PD1 <-> level shifter <-> Pi GPIO14/15 (`/dev/ttyAMA0`), 38400 baud,
  binary frames with CRC. It runs a speed controller per wheel (control loop 20 Hz, stops after 200 ms without
  a command) and sends the cumulative encoder counts every 50 ms. Firmware v2.1.0 or newer is needed.
  Protocol: [docs/mcu_protocol.md](docs/mcu_protocol.md).
- Level shifter on the board: a TXB type (the schematic says TXS0104E). Fine for UART, cannot do I2C.
- IMU: WitMotion WT901C-TTL (3.3 V) on a CH340G USB-to-TTL adapter (`/dev/ttyIMU`), flat, chip side up.
  Only its turn rate is used (EKF in apps 2 and 3): [docs/imu_ekf.md](docs/imu_ekf.md).
- Battery: 4S2P Li-ion 18650 (3300 mAh per cell: 16.8 V full, 6.6 Ah). Battery monitor: TI INA219 (address 0x40, IN+ and IN- shorted: voltage only, no current)
  on the Pi I2C bus 1 (GPIO2/3), read by `battery_node` (`/battery_state`, `/battery_low` below 14.4 V): [docs/battery.md](docs/battery.md).
- Scintillator (app 3): USB, Prolific USB-to-serial adapter, 19200 baud 8N1 RTS/CTS, one text line per second,
  read by `scintillator_node` (`/scintillator/count_rate`): [docs/scintillator.md](docs/scintillator.md).
- Xbox Wireless Controller (`C8:3F:26:93:1B:B2`), paired with the Pi over Bluetooth.
- Screen (app 3): Waveshare 7 inch HDMI LCD (C), 1024 x 600, capacitive touch over USB. On the Pi HDMI port.

## How it fits together

```
PC                                   Pi (on the robot)                         MCU
nala_teleop keyboard --/cmd_vel-->   nala_base --UART command frame, 20 Hz-->  motor_driver
Xbox --Bluetooth--> joy_node -> nala_teleop xbox_teleop --/cmd_vel--> nala_base
                                     nala_base <--UART state frame, 20 Hz----  (PI per wheel)
                                     nala_base: wheel odometry (/odom; TF odom->base_footprint in app 1)
INA219 --I2C--> battery_node (/battery_state, /battery_low)
WT901 IMU --USB serial--> imu_node (/imu)                                      [stage 2]
                                     ekf_node: /odom vx,vy + /imu turn rate -> TF odom->base_footprint
RViz <--/map /scan /tf--             rplidar_ros  (/scan)                       [stage 2]
                                     robot_state_publisher (base_footprint->base_link->laser, imu_link)
                                     slam_toolbox (/map, map->odom)             [stage 2]

App 3 (all on the Pi; the PC is optional):
Robot screen: nala_panel --starts--> app3_coverage_pi.launch.py, --runs--> coverage_tool/plan_cli.py
  nala_base (base_node, battery_node, imu_node) + EKF + rplidar_ros + robot_state_publisher, as in app 2
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
│   ├── cyclonedds.xml                     [1] Cyclone DDS: use only the robot network 10.42.0.0/24 or 192.168.137.0/24
│   ├── cyclonedds_local.xml               [3] Loopback only: used when no robot network is up (ros_env.sh picks it; robot screen works without WiFi)
│   ├── robot.yaml                         [2] Robot model: body size, LiDAR and IMU mount poses (read by the URDF)
│   ├── battery.yaml                       [1] INA219 battery monitor: I2C address, full / low / empty voltage
│   ├── tune_motors.yaml                   [1] Test settings of scripts/tune_motors.py (step times, sweep speeds, timeouts).
│   │                                      The controller parameters themselves are in the MCU EEPROM
│   ├── imu.yaml                           [2] WT901 IMU: serial port, baud rate, gyro variance; rate written by setup_imu.py
│   ├── scintillator.yaml                  [3] Scintillator: serial port, baud rate, flow control, counting interval, timeout
│   ├── ekf.yaml                           [2] robot_localization EKF: wheel vx, vy + IMU turn rate -> odom TF
│   ├── rplidar.yaml                       [2] RPLIDAR A2M8 driver params: port, baud rate, frame, scan mode
│   ├── slam_mapping.yaml                  [2] slam_toolbox mapping params, tuned against pose jumps
│   ├── localization.yaml                  [3] AMCL (localization in the saved map) and the map server
│   ├── nav2_params.yaml                   [3] Nav2 planner (NavFn), controller (Regulated Pure Pursuit, forward only), costmaps
│   ├── coverage.yaml                      [3] Coverage sensor (placeholder), coverage speeds, frames, mission, health, recovery,
│   │                                      radiation, map changes
│   └── panel.yaml                         [3] Robot screen (nala_panel): map folder, window and button size, update rates
│
├── docs/
│   ├── handover.md                        State on 2026-10-12, steps after the Pi reinstall, prompt for the next Claude session
│   ├── mcu_protocol.md                    Pi-side summary of the Pi <-> MCU UART protocol, first hardware test
│   ├── imu_ekf.md                         [2] IMU wiring and setup, EKF design: why only the gyro turn rate
│   ├── battery.md                         [1] INA219 battery voltage monitor: wiring, code, /battery_state data format
│   └── scintillator.md                    [3] Scintillator input (/scintillator/count_rate), heatmap, hotspots, contamination stop
│
├── firmware/                              MCU motor driver (ATmega328PB), Microchip Studio project.
│   │                                      Owned by a teammate, who builds and flashes it. v2.3.0 (by Claude).
│   ├── README.md                          Protocol (source of truth), parameters, build/flash, changes per version
│   ├── prayer.jpeg                        The team's prayer picture (JPEG, 1672x941). Also the Pi boot splash (scripts/setup_splash.sh)
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
├── coverage_tool/                         [3] Coverage path planner (Python, no ROS). Used by the robot screen
│   │                                      (plan_cli.py) and on the PC (app.py). Details: coverage_tool/README.md
│   ├── README.md                          How the planner works, its cost function, settings and commands
│   ├── CLAUDE.md                          Planner facts for Claude Code
│   ├── requirements.txt                   Python libraries of the planner (for pip on other machines; apt on the Pi)
│   ├── .gitignore                         Python cache files
│   ├── app.py                             PC: interactive planner window (Tkinter)
│   ├── plan_cli.py                        Plan from the command line: map + base station (+ start, area) -> plan .yaml/.png
│   ├── send_to_nav2.py                    Only for a plain Nav2 stack, not used by NALA (see coverage_tool/README.md)
│   ├── coverage/                          The planner: grid, sensor model, patterns, cost, optimiser, drivable path,
│   │                                      drive-in candidates for narrow pockets (spurs.py), passages through
│   │                                      narrow entrances (passages.py), export
│   │   └── *.py                           (one module per step; list in coverage_tool/README.md)
│   ├── examples/                          Example maps (*.yaml + *.pgm), map generators (make_*.py), NALA example plans
│   │                                      (*_nala.yaml) and their base stations (*_base_station.yaml)
│   └── tests/
│       ├── test_sensor.py                 Unit tests: sensor ahead of the robot, outline check, geometry from config/
│       ├── test_spurs.py                  Unit tests: drive-ins into a narrow pocket, planned with the route
│       ├── test_passages.py               Unit tests: a room behind a door narrower than the turning circle
│       └── regression.py                  Acceptance test on the example maps (about 15 minutes; --quick for 2 cases)
│
├── maps/                                  Saved maps and base station files. The map files are not in git.
│   │                                      Per map: map.yaml/.pgm (app 2), base_station.yaml, plans/, runs/ (app 3)
│   └── .gitkeep                           (empty) Keeps the folder in git. Stays empty
│
├── scripts/
│   ├── setup_pi.sh                        [1] One-time Pi setup: ROS, Cyclone DDS, UART, I2C, USB serial rules, permissions, ~/.bashrc
│   ├── setup_imu.py                       [2] Pi, once: IMU output (acc + gyro), 50 Hz, 115200 baud
│   ├── tune_motors.py                     [1] Pi: MCU speed controller tuning (show/set/save parameters, step, sweep, ff test)
│   ├── setup_splash.sh                    Pi, once: boot splash = firmware/prayer.jpeg (Plymouth theme, rebuilds the initramfs)
│   ├── splash/
│   │   ├── nala.plymouth                  Plymouth theme description (script plugin)
│   │   └── nala.script                    Plymouth script: the picture on black, scaled to the screen, centered
│   ├── save_map.sh                        [2] Pi: save the current SLAM map into maps/<name>/
│   ├── setup_pi_screen.sh                 [3] One-time Pi setup for the robot screen: Nav2, panel libraries, GNOME autostart entries
│   ├── screen_always_on.sh                [3] Pi, at every login (GNOME autostart): no screen blanking, dimming, lock or suspend
│   └── start_panel.sh                     [3] Start the robot screen panel (at login, or by hand); waits for the XWayland display
│
├── tools/
│   ├── debug/                             [3] Pi: diagnosis while app 3 runs (only listen). See docs/handover.md
│   │   ├── analyze_run.py                 PC or Pi: summary of an app 3 run folder (pauses, weaving against the plan, supervisor CPU timing); reads files only
│   │   ├── record_drive.py                Record Nav2 command, gate output, odom, EKF (vx, wz, vy), AMCL, trust state, warnings to CSV
│   │   ├── scan_on_map.py                 PNG of the live LiDAR scan on the map at the AMCL pose
│   │   └── tf_lag.py                      Age of the odom TF, /odom and /imu when they arrive
│   └── sim/                               [3] PC: closed-loop simulation of app 3 with a fake robot (see tools/sim/README.md)
│       ├── README.md                      How to run it, options, what is simulated
│       ├── env.sh                         ROS environment for the simulation and the app 3 tests (own domain, localhost, Cyclone)
│       ├── cyclonedds_sim.xml             Cyclone DDS for the simulation: loopback only, big messages and receive buffer
│       ├── fake_nala.py                   Fake nala_base + MCU + LiDAR (uses nala_base's speed limit and config/base.yaml)
│       ├── run_sim.py                     Runs app 3 against the fake robot like an operator, records the run
│       ├── analyze.py                     Summary and plot of a run
│       ├── coverage_map.py                Covered and missed floor of a run
│       ├── amcl_error.py                  AMCL error against the true pose
│       └── sources_room.yaml              Simulated radiation sources for the room example (fake_nala.py --sources)
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
    │   │   ├── ina260.py                  [1] INA219 register decoding (bus voltage only; the file name is old), voltage -> charge (no ROS)
    │   │   ├── battery_node.py            [1] ROS node: INA219 -> /battery_state, /battery_low
    │   │   ├── wt901.py                   [2] WT901 IMU serial frames (no ROS)
    │   │   ├── imu_node.py                [2] ROS node: IMU serial port -> /imu
    │   │   ├── scintillator.py            [3] Scintillator serial text lines -> counts per second (no ROS)
    │   │   └── scintillator_node.py       [3] ROS node: scintillator serial port -> /scintillator/count_rate
    │   └── test/
    │       ├── test_mcu_protocol.py       [1] Unit tests for mcu_protocol.py
    │       ├── test_ina260.py             [1] Unit tests for ina260.py
    │       ├── test_kinematics.py         [1] Unit tests for kinematics.py
    │       ├── test_step_metrics.py       [1] Unit tests for step_metrics.py
    │       ├── test_wt901.py              [2] Unit tests for wt901.py
    │       └── test_scintillator.py       [3] Unit tests for scintillator.py (real lines from the robot)
    │
    ├── nala_teleop/                       Teleop with fixed speeds: PC keyboard (no speed keys), Pi Xbox controller
    │   ├── package.xml                    [1] Package manifest
    │   ├── setup.py                       [1] Python package setup and node entry points
    │   ├── setup.cfg                      [1] Install paths for ros2 run
    │   ├── resource/
    │   │   └── nala_teleop                (empty) [1] ament index marker. Stays empty
    │   ├── nala_teleop/
    │   │   ├── __init__.py                (empty) [1] Python package marker. Stays empty
    │   │   ├── keys.py                    [1] Key layout: key -> drive direction, help text (no ROS)
    │   │   ├── keyboard_node.py           [1] ROS node: read keys in the terminal, publish /cmd_vel
    │   │   ├── xbox.py                    [1] Xbox rules: hold LB, centre the sticks first, /joy watchdog (no ROS)
    │   │   └── xbox_node.py               [1] Pi ROS node xbox_teleop: /joy -> /cmd_vel
    │   └── test/
    │       ├── test_keys.py               [1] Unit tests for keys.py
    │       └── test_xbox.py               [1] Unit tests for xbox.py
    │
    ├── nala_bringup/                      Launch files and RViz layouts for the 3 apps. Installs config/
    │   ├── CMakeLists.txt                 [1] Install launch/, rviz/ and the repo config/ folder
    │   ├── package.xml                    [1] Package manifest
    │   ├── launch/
    │   │   ├── app1_teleop_pi.launch.py   [1] Pi: base driver, battery monitor, Xbox controller (joy + xbox_teleop)
    │   │   ├── app2_slam_pi.launch.py     [2] Pi: app 1 + robot model + LiDAR + SLAM
    │   │   ├── app2_slam_pc.launch.py     [2] PC: RViz with slam.rviz
    │   │   ├── app3_base_station_pc.launch.py   [3] PC: show a saved map, click the base station in RViz (the robot screen can do this too)
    │   │   ├── app3_coverage_pi.launch.py [3] Pi: base driver, battery, IMU + EKF, robot model, LiDAR, AMCL, Nav2, coverage mission, velocity gate, Xbox B = pause
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
    │   │   ├── supervisor.py              [3] Mission: preview/start/pause/resume/cancel/go home, background measurement,
    │   │   │                                  Nav2 actions, detours, contamination stop (ends the task), return to base,
    │   │   │                                  drive-ins into pockets (SPUR: in forwards, out backwards over measured floor)
    │   │   ├── radiation.py               [3] Radiation map: background, readings at trusted poses, hotspots; contamination
    │   │   │                                  threshold (counts/s or x background) and alarm (2 of the last 3 readings) (no ROS)
    │   │   ├── radiation_node.py          [3] Scintillator readings -> heatmap, hotspots, run folder files
    │   │   ├── mapchange.py               [3] Map changes: new objects and removed objects from LiDAR scans (no ROS)
    │   │   ├── mapchange_node.py          [3] LiDAR scans at trusted poses -> /map_changes, changes.npz in the run folder
    │   │   ├── locate.py                  [3] Find the robot on the map from one scan; on the base station / backwards (no ROS)
    │   │   ├── locator_node.py            [3] /scan -> /coverage/located while no task drives (Start needs ON_BASE)
    │   │   ├── plan_file.py               [3] Load and check a coverage_tool plan (map, wall clearance, outline on
    │   │   │                                  drive-ins, sensor, base station)
    │   │   ├── planning.py                [3] Plan pieces and path helpers (no ROS)
    │   │   ├── base_station.py            [3] Load, save and check base_station.yaml (no ROS)
    │   │   ├── base_station_picker_node.py   [3] PC: RViz "2D Goal Pose" -> maps/<name>/base_station.yaml
    │   │   ├── gate_node.py               [3] Velocity gate: the only /cmd_vel publisher in app 3; forward only for Nav2,
    │   │   │                                  slow reverse only from the supervisor's /cmd_vel_reverse (backing out)
    │   │   ├── health.py                  [3] Ownership and localization trust checks (no ROS)
    │   │   ├── meter.py                   [3] Coverage meter node: covered floor from trusted poses, at the sensor (avoided floor not counted)
    │   │   ├── measurement.py             [3] Coverage painting (no ROS)
    │   │   ├── geometry.py                [3] Occupancy grid, reachability, coverable floor (no ROS)
    │   │   ├── params.py                  [3] Load and check config/ (outline from robot.yaml), render the Nav2 params
    │   │   ├── keyboard_teleop.py         [3] coverage_teleop: drive by hand through the gate while a task is paused
    │   │   ├── buttons.py                 [3] Game controller button press detection (no ROS)
    │   │   ├── xbox_pause_node.py         [3] xbox_pause: Xbox B -> /coverage/pause (never /cmd_vel)
    │   │   ├── mode_guard.py              [3] Refuse to start app 3 next to SLAM or another /cmd_vel publisher
    │   │   ├── ros_common.py              [3] Shared ROS helpers (QoS, messages, sensor freshness)
    │   │   └── state.py                   [3] Mission bookkeeping (no ROS)
    │   └── test/
    │       ├── test_core.py, test_plan_file.py, test_base_station.py, test_revision.py, test_radiation.py   [3] Unit tests (no ROS)
    │       ├── test_radiation_node.py     [3] Loopback ROS test of radiation_node
    │       ├── test_mapchange.py          [3] Map changes: a moved cabinet, a small pose error, a person walking by
    │       ├── test_locate.py             [3] Locator: found anywhere, on the base station, backwards, beside it
    │       ├── test_launch_contract.py    [3] Static checks of the app 3 launch files
    │       ├── test_buttons.py            [3] Unit tests for buttons.py
    │       ├── test_xbox_pause.py         [3] Loopback ROS tests of xbox_pause with a fake /coverage/pause service
    │       └── test_ros.py                [3] Loopback ROS tests: gate, supervisor, meter, plan driving, return to base,
    │                                      background, contamination stop, go home
    │
    ├── nala_interfaces/                   [3] Message types (ament_cmake, rosidl)
    │   ├── CMakeLists.txt                 [3] Build the messages
    │   ├── package.xml                    [3] Package manifest
    │   └── msg/
    │       └── CountRate.msg              [3] Scintillator reading: counts per second over an interval (docs/scintillator.md)
    │
    └── nala_panel/                        Robot screen (PyQt5): app 3 (maps, base station, planning, driving), apps 1 and 2
        ├── package.xml                    [3] Package manifest
        ├── setup.py                       [3] Python package setup and node entry points
        ├── setup.cfg                      [3] Install paths for ros2 run
        ├── resource/
        │   └── nala_panel                 (empty) [3] ament index marker. Stays empty
        ├── nala_panel/
        │   ├── __init__.py                (empty) [3] Python package marker. Stays empty
        │   ├── apps.py                    [1][2] Start/stop app 1 / app 2 from the screen, app 2's live map, save_map.sh,
        │   │                                  map name check (no Qt)
        │   ├── library.py                 [3] Map library: maps, base station, plans, can a plan be driven, runs, the saved
        │   │                                  contamination threshold (no ROS, no Qt)
        │   ├── planning.py                [3] Run coverage_tool/plan_cli.py, check start and area, cancel (no Qt)
        │   ├── drive.py                   [3] Start/stop app 3, follow its state and the radiation data; Start/Pause/Resume/
        │   │                                  Go home/STOP, mark checked, hand control, send the threshold (no Qt)
        │   ├── mapwidget.py               [3] Map drawing (QPainter): coverage, heatmap, actual trail, hotspots, contamination
        │   │                                  spot, robot; touch
        │   ├── battery.py                 [3] Battery voltage for the screen: reads the INA219, moving average, text (no Qt)
        │   ├── style.py                   [3] Colours and stylesheet after the scan screen design
        │   └── ui.py                      [3] The screens: Maps, Map (plans, new plan, results), Planning, Scan (red
        │                                      contamination warning); App (apps 1 and 2) with an on-screen keyboard for the
        │                                      map name; Settings (contamination threshold, keypad)
        └── test/
            ├── conftest.py                [3] Test map library (the example room map)
            ├── test_battery.py            [3] Unit tests for battery.py
            ├── test_library.py            [3] Unit tests for library.py
            ├── test_planning.py           [3] Planning tests (one real plan on a small map, about 1 minute)
            ├── test_drive.py              [3] Loopback ROS tests of the drive session with fake app 3 services
            └── test_ui.py                 [3] Screen tests without a display: every screen fits 1024 x 600, taps, hotspot sheet,
                                               go home, apps 1 and 2, map names (NALA_PANEL_SHOTS=<dir> saves screenshots)
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

Battery monitor: `i2cdetect -y 1` must show the INA219 at `40` (see [docs/battery.md](docs/battery.md)); if it is at another address,
change `i2c_address` in `config/battery.yaml`.
Read the battery voltage in the Pi SSH terminal (app 1 and app 3 already start `battery_node`; otherwise start it alone):

```bash
ros2 run nala_base battery_node --ros-args --params-file ~/NALA/config/battery.yaml   # only if no app runs
ros2 topic echo /battery_state --field voltage                                        # second terminal, V every 1 s
```
When a pull changes `setup_pi.sh` (new packages), run it again and log in again (new groups need a new login).

Boot splash (one time, optional): `bash ~/NALA/scripts/setup_splash.sh` (asks for the sudo password, about 1 minute).
It shows `firmware/prayer.jpeg` while the Pi boots, on a connected display (after the Pi's rainbow screen).
To go back to the Ubuntu splash: `sudo update-alternatives --config default.plymouth`, then `sudo update-initramfs -u`.
Without a display the boot is the same: the desktop is ready about 50 s after power-on (`plymouth-quit-wait` waits
for the GNOME autologin session, measured 2026-10-15; SSH works earlier).

Robot screen for app 3 (one time, after `setup_pi.sh`), in the Pi SSH terminal:

```bash
bash ~/NALA/scripts/setup_pi_screen.sh   # Nav2, PyQt5, planner libraries, GNOME autostart: panel + screen always on
cd ~/NALA && colcon build --symlink-install --base-paths src
sudo reboot
```

The Pi keeps its Ubuntu Desktop (GNOME, Wayland, automatic login of `nala`); nothing else is installed for the
screen. After the reboot and the GNOME login the panel starts full-screen on the 7 inch screen (on XWayland,
`QT_QPA_PLATFORM=xcb` in `start_panel.sh`), and the screen no longer blanks, dims or locks. First check that the picture fills the
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

**On the robot screen** (no terminal needed): tap **Drive (app 1)** on the map list. App 1 starts at once; drive
with the Xbox controller (below). **STOP** stops app 1 (the motors stop), **Start** starts it again, **Back** stops it
and returns to the map list. Only one app runs at a time: if app 1, 2 or 3 is already running (e.g. started over
ssh), the screen says so and does not start another one.

**Or over ssh:**

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
- Speeds are fixed: 0.45 m/s and 0.5 rad/s (`config/teleop.yaml`). There are no speed keys.

Xbox controller (switch it on; it connects to the Pi by itself):

- **Hold LB** to drive. Release LB = stop at once.
- After pressing LB, let both sticks go to the centre once: until then the robot does not move, even if a
  stick is pushed (so it cannot jump off when LB is pressed with a stick held).
- If the controller is lost while LB is held (no `/joy` for 0.2 s), the robot stops. Release LB and press it again.
- Left stick: up / down = forward / backward, left / right = move sideways.
- Right stick left / right = turn.
- Full stick = 0.45 m/s and 0.5 rad/s (`config/teleop.yaml`).

Both:

- Strafe and turn work at the same time. If a command needs a wheel faster than the motors can go
  (`max_wheel_speed` in `config/base.yaml`), the Pi slows the whole command down by the same factor.
- Stop: Ctrl+C in either terminal. The Pi sends a stop frame when it exits.

### Tune the wheel speed controller (MCU firmware v2.3.0)

The PI gains, the target ramp and the feed-forward table (one PWM row per motor) live in the MCU (RAM, saved in its EEPROM).
Stop app 1 / app 2 first (same UART). **Pi terminal** (`ssh nala@nala.local`):

```bash
python3 ~/NALA/scripts/tune_motors.py show                  # what the MCU uses now
python3 ~/NALA/scripts/tune_motors.py ff --apply            # wheels in the air: measure each motor (3 s settle, 2 s
                                                            # measured per point), write the new PWM rows into RAM
python3 ~/NALA/scripts/tune_motors.py ff                    # check: measured = table speed (about 60 s)
python3 ~/NALA/scripts/tune_motors.py sweep                 # wheels in the air: steps in 6 directions, 4 speeds
python3 ~/NALA/scripts/tune_motors.py step --vx 0.15        # one step, metrics per wheel (--csv FILE for the data)
python3 ~/NALA/scripts/tune_motors.py set kp 2 accel_limit 20   # change in RAM (only while stopped)
python3 ~/NALA/scripts/tune_motors.py save                  # keep it after power-off (EEPROM)
python3 ~/NALA/scripts/tune_motors.py reload                # undo unsaved changes
python3 ~/NALA/scripts/tune_motors.py defaults              # the firmware defaults (config.h); then save
python3 ~/NALA/scripts/tune_motors.py ff --floor --apply    # on the floor: each speed forward + back (legs <= 1.5 m)
python3 ~/NALA/scripts/tune_motors.py sweep --floor         # on the floor: short steps, each pair comes back
```

Every motion asks first ("Wheels lifted or floor clear?"). Ctrl+C stops at once; `ff` puts the original gains back.
Parameter names: `kp`, `ki`, `i_limit`, `accel_limit`, `ff_speed0`..`ff_speed9` (shared),
`ff_m1_pwm0`..`ff_m1_pwm9` .. `ff_m4_pwm0`..`ff_m4_pwm9` (one row per motor).

### App 2: mapping

**On the robot screen** (no terminal needed): tap **Make a map (app 2)** on the map list. App 2 starts at once and
the map grows on the screen, with the robot on it; drive with the Xbox controller (as in app 1). Tap **Save map**,
type the name on the on-screen keyboard (letters, digits, - and _; a date/time name is filled in) and **Save**:
the map is saved like `save_map.sh` does and appears in the map list for app 3. **STOP** / **Back** as in app 1.

**Or over ssh:**

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

### App 3: coverage

Uses a map saved by app 2 (`maps/<name>/` on the Pi). Everything runs on the Pi; the robot screen is the
normal way to use it. The PC is optional (RViz, or planning on the PC).

#### On the robot screen

The panel starts by itself after the Pi boots (`scripts/setup_pi_screen.sh`). By hand, in a terminal on the Pi:
`bash ~/NALA/scripts/start_panel.sh`.

1. **Maps:** tap the map (every folder in `maps/` with a `map.yaml` from app 2).
2. **New plan** tab:
   - **Set base station:** tap where the robot stands, then tap the direction it faces. It is saved as
     `maps/<name>/base_station.yaml`. The robot needs room to turn there (0.498 m from walls), else the panel refuses.
   - **Coverage starts at:** *Base station*, or *Start point* + **Set start point** (tap). With a start point the
     robot still stands on the base station: Nav2 first drives it to the start point.
   - **Area** (optional): **Draw area**, tap the corners, **Close area**. Only the floor inside is covered.
     **Undo corner** and **Clear area** to change it. No area = all floor the robot can reach.
   - **Plan.** Planning runs on the Pi and can take several minutes. The result shows the coverage and four
     checks (all reachable floor covered, robot outline touches nothing, no unsafe segment, no lost floor).
     **Keep plan** saves it in `maps/<name>/plans/`; **Discard** deletes it.
3. **Plans** tab: every saved plan, newest first. A plan the robot would refuse is grey, with the reason
   (made on another map image, for another base station, or for other sensor values). **Delete** removes one.
   **Results** tab: every previous scan of this map, newest first (drag to scroll); tap one to see its
   heatmap, hotspots, the floor it covered and the path it drove.
4. **Scan:** put the robot on the base station (within about 10 cm), facing the base station direction
   (within about 10 deg). Tap **Scan** on a plan. The panel starts app 3 and shows the scan screen
   (the design of `Scan_screen_visualiser.html`):
   - the map shows where the robot really is (found from the LiDAR scan, wherever it stands). If it is not on
     the base station, the screen says *Robot is not on the base station*, or *... the wrong way round* when
     it stands there backwards, or *... turned. Turn it about N degrees to the left/right*: put it on the base
     station the right way round (the map shows the robot's outline there, dashed, with an arrow for the direction
     it must face); Start stays off until then;
   - *Finding the robot on the map*, then *Checking the plan* (the preview);
   - **Start** becomes active when the plan is checked. **The robot only moves after Start.** It first measures
     the radiation background on the base station (*Measuring the background*), then drives.
   - the map shows what really happened, from the measured poses only: the trail (blue), the floor the sensor
     covered (teal), the scintillator readings (heatmap, contaminated floor magenta), the numbered hotspots, the
     robot and the sensor reach;
   - **contamination found** (2 of the last 3 readings at the threshold set under **Settings** on the map list:
     counts per second or times the background): the robot stops, the scan ends, and the whole screen turns red
     with the readings, the threshold, the time and the sensor position, until **Acknowledge**. Then the map shows
     the spot (red rings). The robot stays where it is: no resume, no driving home; move it by hand;
   - while driving the screen shows the large progress view (%, state, bar, hotspot alert); **Show map** shows the
     map, **Big view** goes back;
   - tap a hotspot: highest reading, background, position, first seen, **Mark as checked**;
   - map changes: floor that is free on the map but where the LiDAR keeps seeing an object (**new object**,
     orange-red), and map obstacles the LiDAR keeps seeing through (**removed object**, light blue), for example a moved
     cabinet. Only from scans at trusted poses; small pose errors along walls are ignored. If much has changed,
     make a new map with app 2;
   - layer buttons Coverage / Radiation / Path / Changes switch the map layers;
   - **STOP** (red, always there): the robot stops at once, the task pauses and a STOP banner shows.
     **Resume** continues;
   - **Xbox controller B** (if the controller is on): pauses the task like **Pause** (Nav2 stops, the rest of
     the plan is kept). Resume on the screen. The controller cannot drive in app 3;
   - **Pause / Resume.** When paused (for example `BLOCKED_IN_PLACE`), **Drive by hand** and the arrow buttons
     move the robot through the velocity gate (hold to drive). Tap **Hand control off**, then **Resume**;
   - **Stop and go home** (**Go home** in the side panel) asks first, then stops the plan and drives back to the
     base station;
   - at the end the robot drives back to the base station: *Scan complete*;
   - **Back** stops app 3 and shows the task report (also saved in `maps/<name>/runs/<time>/` with the readings,
     hotspots, heatmap and map changes).

Only one app at a time: stop app 1 and app 2 first. App 3 starts `joy_node` only for the B button (pause);
it does not start `xbox_teleop`, so the controller cannot drive.

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
ros2 service call /coverage/pause std_srvs/srv/Trigger '{}'    # resume / cancel / go_home likewise
```

Drive by hand while paused (W/S/A/D, `Q` to leave): `ros2 run nala_coverage coverage_teleop`.
Do not run `nala_teleop`'s keyboard or `xbox_teleop` (app 1) during app 3: they publish `/cmd_vel` directly, the
gate sees a second publisher and stops the robot (`VELOCITY_GRAPH_CONFLICT`). Xbox B (pause) is fine.

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

#### Scintillator readings in a terminal

App 3 starts the scintillator driver itself. To watch the readings (counts per second, one per second):

PC terminal (in the repo folder) or Pi terminal (`ssh nala@nala.local`), while app 3 runs:

```bash
source config/ros_env.sh       # Pi: source ~/NALA/config/ros_env.sh
ros2 topic echo /scintillator/count_rate --field counts_per_second
```

Without app 3, start the driver by hand in a Pi terminal first (Ctrl-C stops it):

```bash
ros2 run nala_base scintillator_node --ros-args --params-file ~/NALA/config/scintillator.yaml
```

The raw lines (Pi terminal, only while no driver runs: the driver locks the port; `Ctrl+]` to leave):

```bash
python3 -m serial.tools.miniterm --rtscts /dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_ESCDb147612-if00-port0 19200
```

`radiation.enabled` in `config/coverage.yaml` is false for now: the readings are recorded (heatmap, hotspots,
`readings.csv`), but there is no background measurement and no contamination stop.

#### Robot geometry for coverage

All values in `base_footprint`/`base_link` (on the floor under the chassis centre), x forward, y left.

| What | Value | From |
|---|---|---|
| Outer box (must never touch anything) | x -0.2075 .. +0.4425 m (rod in front), y +-0.18 m | `config/robot.yaml` (`outline`: front, back, left, right) |
| Farthest corner from the centre | 0.478 m (front corners, rod) | derived |
| Collision radius | 0.478 + 0.02 padding = **0.498 m** | `planning_padding_m` in `config/coverage.yaml` |
| Plan robot radius (coverage_tool) | **0.53 m** | + 3 cm for localization, rounded up to 1 cm |
| Coverage sensor (scintillator) point | (+0.26, 0) m, static TF `scintillator_link` | `config/coverage.yaml` (**placeholder**) |
| Sensor coverage | disk of 0.6 m radius around that point | `config/coverage.yaml` (**placeholder**) |

The outline is configured once, in `robot.yaml`; `nala_coverage/params.py` and `coverage_tool` derive the rest.
The robot turns on the spot anywhere on its path, so plans keep its centre 0.53 m from walls: the narrowest
passage it uses is about 1.06 m. When the sensor position and size are known, change them in
`config/coverage.yaml` and plan again: the robot refuses plans made for other sensor values.

#### How the plan is driven

- **Start:** AMCL starts at the base station (the supervisor sends it as the initial pose, spread 0.1 m and
  10 deg). The radiation background is measured there first (`docs/scintillator.md`; without scintillator
  readings the task pauses with `NO_SCINTILLATOR`, or set `radiation.enabled: false`). Then Nav2's planner
  (`ComputePathToPose`) drives the robot to the plan start if it is not there.
- **Contamination:** 2 of the last 3 readings at the operator's threshold (robot screen Settings: counts/s or x
  background; default in `config/coverage.yaml` `radiation`) stop the robot and end the task (`FAILED`,
  `CONTAMINATION_FOUND`, details under `contamination` in the task report). The robot never drives around it
  (`docs/scintillator.md`).
- **Go home:** `/coverage/go_home` (the panel's Stop and go home) stops the plan and drives back to the base station.
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
- **Speeds and stopping:** cruise 0.12 m/s, turns 0.4 rad/s, slowest 0.04 m/s near a goal (the MCU protocol has
  1 mm/s steps). Acceleration limits from the floor test 2026-10-16: 0.1 m/s^2 (velocity gate) and 0.35 rad/s^2
  (gate and controller), so the robot can follow the slow-down before a goal (it cannot brake harder while the
  command is above 0). A stop command makes the wheels coast (about 0.38 m/s^2).
- **Heading:** like app 2, the EKF takes the turn rate only from the IMU gyro (the wheels show 10-14 % too much
  turn). AMCL corrects the remaining drift against the map.
- **Refused:** plans closer than 0.498 m to a wall, made on another map image, for other sensor values or for
  another base station; a base station closer than 0.498 m to a wall.
- **Obstacles the LiDAR cannot see** (lower or higher than its scan plane) are not avoided. NALA has no bumper.

#### First run on the robot

Keep a hand on the robot's emergency stop.

1. **LiDAR and EKF:** the scan must line up with the map when the robot stands still and when it turns
   (if it rotates against the map while turning, check `/imu` and the EKF first, like in app 2).
2. **Localization:** put the robot on the base station and tap **Drive** on a plan. Before **Start**, drive by
   hand is not possible, so check the map on the screen: the robot arrow must be on the base station.
   AMCL values in `config/localization.yaml` were tuned in simulation only.
3. **Costmaps:** with RViz on the PC, put a box in front of the robot: it must appear in `/local_costmap/costmap`.
4. **Plan driving:** a short plan in open space first. Then a box on the path: the robot must stop before it,
   skip past it, drive around and continue. At the end it must drive back to the base station.
5. **Speeds:** `motion` in `config/coverage.yaml`: 0.12 m/s and 0.4 rad/s are start values (not tested with Nav2);
   the acceleration limits are measured. Raise the speeds only after the controller is tuned on the floor.

#### Simulation on the PC

`tools/sim` runs app 3 with Nav2 against a fake robot, in its own ROS domain (see `tools/sim/README.md`):

```bash
source tools/sim/env.sh && export SIM_RUNS_DIR=/tmp/nala-sim && cd tools/sim
python3 run_sim.py room --sources sources_room.yaml --max-resumes 40 --pause-timeout 60   # with radiation sources
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

In the simulation environment (own domain 77, this PC only): with `config/ros_env.sh` (domain 0, robot network)
a test velocity gate could send `/cmd_vel` to a robot that is running.

```bash
source tools/sim/env.sh
python3 -m pytest -q -p no:cacheprovider src/nala_coverage/test src/nala_panel/test   # about 3 minutes
cd coverage_tool && python3 -m pytest -q -p no:cacheprovider tests/test_sensor.py
```

`test_ros.py` runs real ROS nodes on loopback in one Python process. On a busy PC a plan-driving test sometimes
pauses with `TF_UNAVAILABLE`; run it again on its own (`-k <name>`) before treating it as a real failure.

## Build and flash the MCU firmware

The firmware is owned by a teammate. See [firmware/README.md](firmware/README.md):
build with Microchip Studio on Windows (`firmware/motor_driver_C.atsln`, F7), flash with `avrdude` and an ArduinoISP.
The exact commands are in section 5 of the firmware README.
