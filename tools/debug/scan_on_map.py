"""Draw the live LiDAR scan on the saved map at the AMCL pose (PNG), while app 3 runs. Only listens.

    source ~/NALA/config/ros_env.sh
    python3 ~/NALA/tools/debug/scan_on_map.py --map ~/NALA/maps/Test/map.yaml --out ~/nala_debug/scan_on_map.png

Straight walls must give straight scan lines on the map walls. A curved or doubled scan means a broken
scan (2026-10-16: two LiDAR drivers on one serial port), a shifted one a wrong AMCL pose.
Laser mount: at the robot centre, yaw 180 deg (config/robot.yaml).
"""
import argparse
import math
import time
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
import yaml  # noqa: E402
from geometry_msgs.msg import PoseWithCovarianceStamped  # noqa: E402
from PIL import Image  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402

LASER_YAW = math.pi   # keep equal to laser yaw in config/robot.yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', required=True)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    folder = Path(args.map).parent
    meta = yaml.safe_load(open(args.map))
    img = np.array(Image.open(folder / meta['image']))
    res = meta['resolution']
    ox, oy = meta['origin'][:2]
    h, w = img.shape[:2]
    rclpy.init()
    node = rclpy.create_node('nala_scan_on_map')
    scans, pose = [], [None]
    node.create_subscription(LaserScan, '/scan', scans.append, qos_profile_sensor_data)

    def amcl(m):
        q = m.pose.pose.orientation
        pose[0] = (m.pose.pose.position.x, m.pose.pose.position.y, math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z))
    node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', amcl, 10)
    end = time.monotonic() + 15
    while time.monotonic() < end and (pose[0] is None or len(scans) < 5):
        rclpy.spin_once(node, timeout_sec=0.1)
    if pose[0] is None or not scans:
        raise SystemExit('no /amcl_pose or no /scan within 15 s (is app 3 running?)')
    x, y, yaw = pose[0]
    s = scans[-1]
    r = np.asarray(s.ranges)
    a = s.angle_min + np.arange(len(r)) * s.angle_increment
    ok = np.isfinite(r) & (r >= s.range_min)
    ang = a[ok] + LASER_YAW + yaw
    px, py = x + r[ok] * np.cos(ang), y + r[ok] * np.sin(ang)
    fig, ax = plt.subplots(figsize=(12, 6))
    ax.imshow(img, cmap='gray', extent=[ox, ox + w * res, oy, oy + h * res], origin='upper')
    ax.scatter(px, py, s=4, c='red', label='live scan at the AMCL pose')
    ax.plot([x, x + 0.3 * math.cos(yaw)], [y, y + 0.3 * math.sin(yaw)], 'b-', lw=3)
    ax.plot(x, y, 'bo', label='AMCL pose (line = front)')
    ax.set_xlim(px.min() - 0.5, px.max() + 0.5)
    ax.set_ylim(py.min() - 0.5, py.max() + 0.5)
    ax.set_aspect('equal')
    ax.legend(loc='upper right')
    fig.savefig(args.out, dpi=90, bbox_inches='tight')
    print('saved', args.out, 'pose', pose[0])
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
