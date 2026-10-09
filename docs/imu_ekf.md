# IMU and EKF (app 2)

Why: mecanum wheels slip, most of all when the robot turns or strafes. The heading (yaw) error of the
wheel odometry grows into position error, and slam_toolbox only searches a small window around the
odometry pose (`config/slam_mapping.yaml`, against jumps between similar rooms). A gyro measures the
turn rate directly and does not care about wheel slip.

## Hardware

| Item | Value |
|---|---|
| IMU | WitMotion WT901C-TTL, 3.3 V, mounted flat with the chip side up |
| Wiring | IMU TX -> Pi GPIO15 (RX), IMU RX <- Pi GPIO14 (TX), GND |
| Pi UART | PL011 on GPIO14/15 = `/dev/ttyAMA0`. `/boot/firmware/config.txt`: `enable_uart=1`, `dtoverlay=miniuart-bt` (Bluetooth moves to the mini UART), `core_freq=250` (fixed VPU clock: stable mini UART baud rate for Bluetooth). Set by `scripts/setup_pi.sh`, needs a reboot |
| IMU settings | 115200 baud, 50 Hz, only acceleration (0x51) and angular velocity (0x52) frames. Written once by `scripts/setup_imu.py` (factory: 9600 baud) |
| Frame | `imu_link` in the URDF (`config/robot.yaml: imu`), published by robot_state_publisher on `/tf_static` |

WT901 frame: `0x55, type, 4 x int16 LE, checksum` (11 bytes, checksum = sum of the first 10 bytes).
Acceleration = value / 32768 x 16 g, angular velocity = value / 32768 x 2000 deg/s (`nala_base/wt901.py`).
If `setup_imu.py` fails, reset the IMU with the WitMotion PC software and run it again.

## Nodes

- `imu_node` (`nala_base`, `config/imu.yaml`): reads the UART, keeps the newest acceleration (starts at
  0, 0, 9.81 m/s^2) and publishes one `sensor_msgs/Imu` on `/imu` per angular velocity frame.
  `orientation_covariance[0] = -1` (no orientation) and `linear_acceleration_covariance[0] = -1`
  (acceleration only for viewing). Error message if no gyro data comes for `imu_timeout`.
- `base_node`: wheel odometry on `/odom` (vx, vy with `odom_twist_variance`). In app 2 it does not publish TF.
- `ekf_filter_node` (robot_localization, `config/ekf.yaml`): fuses them and publishes TF
  `odom -> base_footprint` (and `/odometry/filtered`). slam_toolbox uses this TF.

## What the EKF uses, and what not

| Input | Used | Why |
|---|---|---|
| Wheel vx, vy (`/odom`) | yes | the only translation sensor. vy has a larger variance: strafing slips more |
| IMU turn rate z (`/imu`) | yes, the **only** yaw source | not affected by wheel slip |
| Wheel turn rate | **no** | mecanum turning slips, and the rotation term k = 0.29 m is not checked |
| IMU orientation (roll, pitch, yaw) | **no** | the WT901 yaw uses the magnetometer (9-axis): indoor magnetic fields (steel, motors, cables) distort it badly. Roll and pitch are not needed on a flat floor (two_d_mode) |
| IMU acceleration | **no** | integrating it twice for position is far too noisy |

Consequence: if the IMU fails, the EKF has no turn rate. `imu_node` then reports an error.
Translation slip (for example strafing on carpet) is not corrected by the EKF: slam_toolbox scan matching corrects it.

## Start values to tune on the robot

- `angular_velocity_variance` (imu.yaml) 0.02^2, `odom_twist_variance` (base.yaml) 0.05^2 / 0.1^2.
- Checks: turn the robot by hand 360 deg: `/odometry/filtered` yaw changes by 360 deg. Gyro sign: a
  counter-clockwise turn gives a positive `/imu` angular_velocity.z. Gyro bias: standing still, the yaw must not drift
  noticeably within a minute.
