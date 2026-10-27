"""Record an app 3 drive on the Pi to a CSV file, for diagnosis. Only listens, never publishes.

    source ~/NALA/config/ros_env.sh    # the same ROS network as the panel / app 3
    python3 ~/NALA/tools/debug/record_drive.py --out ~/nala_debug/drive.csv --seconds 360

CSV rows: time_s,kind,values...
  nav     Nav2 controller command (/cmd_vel_nav): vx, wz
  gate    velocity gate output (/cmd_vel, what base_node gets): vx, wz
  odom    wheel odometry speed (/odom): vx, wz, vy (vy since 2026-10-27, at the end: older files have only vx, wz)
  ekf     EKF speed (/odometry/filtered): vx, wz, vy
  amcl    AMCL pose: x, y, yaw_deg, position sigma (m)
  trust   supervisor trust state (/coverage/trust), only when it changes
  log     warnings and errors of all nodes (/rosout), except battery_node
"""
import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from std_msgs.msg import String

from nala_coverage.ros_common import LATEST


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', required=True)
    parser.add_argument('--seconds', type=float, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('nala_drive_recorder')
    t0 = time.time()
    out = open(args.out, 'w', buffering=1)

    def write(kind, *values):
        out.write(f'{time.time() - t0:.3f},{kind},' + ','.join(str(v) for v in values) + '\n')

    def twist(kind):
        return lambda m: write(kind, round(m.linear.x, 4), round(m.angular.z, 4))

    def odom(kind):
        return lambda m: write(kind, round(m.twist.twist.linear.x, 4), round(m.twist.twist.angular.z, 4),
                               round(m.twist.twist.linear.y, 4))

    def amcl(m):
        c = m.pose.covariance
        q = m.pose.pose.orientation
        yaw = math.degrees(math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z))
        write('amcl', round(m.pose.pose.position.x, 3), round(m.pose.pose.position.y, 3), round(yaw, 1),
              round(math.sqrt(max(c[0], c[7])), 3))

    last_trust = [None]

    def trust(m):
        if m.data != last_trust[0]:
            last_trust[0] = m.data
            write('trust', json.dumps(m.data))

    def log(m):
        if m.level >= 30 and m.name != 'battery_node':
            write('log', m.name, json.dumps(m.msg[:200]))

    node.create_subscription(Twist, '/cmd_vel_nav', twist('nav'), 10)
    node.create_subscription(Twist, '/cmd_vel', twist('gate'), 10)
    node.create_subscription(Odometry, '/odom', odom('odom'), 10)
    node.create_subscription(Odometry, '/odometry/filtered', odom('ekf'), 10)
    node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', amcl, 10)
    node.create_subscription(String, '/coverage/trust', trust, LATEST)
    node.create_subscription(Log, '/rosout', log, 100)
    end = time.monotonic() + args.seconds
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    out.close()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
