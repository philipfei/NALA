"""Measure for 15 s how old TF odom -> base_footprint, /odom and /imu are when they arrive. Only listens.

    source ~/NALA/config/ros_env.sh
    python3 ~/NALA/tools/debug/tf_lag.py

2026-10-16 on the Pi: EKF TF alone median 8 ms (max 25 ms); during app 3 median 22 ms, max 106 ms.
The app 3 supervisor looks up the pose at now - tf_delay_s (0.1 s) and falls back to the newest TF.
"""
import statistics
import time

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage

SECONDS = 15


def main():
    rclpy.init()
    node = rclpy.create_node('nala_tf_lag')
    lag = {'tf odom->base_footprint': [], 'odom': [], 'imu': []}

    def now():
        return node.get_clock().now().nanoseconds * 1e-9

    def age(header):
        return now() - (header.stamp.sec + header.stamp.nanosec * 1e-9)

    def tf(m):
        for t in m.transforms:
            if t.header.frame_id == 'odom' and t.child_frame_id == 'base_footprint':
                lag['tf odom->base_footprint'].append(age(t.header))
    node.create_subscription(TFMessage, '/tf', tf, 50)
    node.create_subscription(Odometry, '/odom', lambda m: lag['odom'].append(age(m.header)), 50)
    node.create_subscription(Imu, '/imu', lambda m: lag['imu'].append(age(m.header)), 50)
    end = time.monotonic() + SECONDS
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    for name, values in lag.items():
        if not values:
            print(name, ': no messages')
            continue
        values.sort()
        print(f'{name}: {len(values) / SECONDS:.1f} Hz, age median {statistics.median(values) * 1000:.0f} ms, '
              f'p95 {values[int(0.95 * len(values))] * 1000:.0f} ms, max {values[-1] * 1000:.0f} ms')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
