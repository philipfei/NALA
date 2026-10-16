"""Kinematic NALA stand-in for the robot repo's nala_base + MCU + LiDAR, for closed-loop tests of stage 3.

Like nala_base (its own functions and config/base.yaml): /cmd_vel is scaled down to max_wheel_speed,
sent in the MCU protocol's 0.02 steps (docs/mcu_protocol.md), and replaced by zero after cmd_vel_timeout.
Like the MCU: forward (linear.x), sideways (linear.y, mecanum; the gate sends 0 for now) and turning
(angular.z), acceleration limited.
Publishes /odom at 20 Hz (the MCU's counts rate) with TF odom->base_footprint, and a ray-cast /scan in
the 'laser' frame. base_footprint->base_link->laser comes from the robot repo's URDF (robot_state_publisher,
started by app3_coverage_pi.launch.py): the LiDAR faces backward, so the scan is cast with the laser yaw
from robot.yaml.
Walls come from the map image; extra round obstacles (--obstacle x,y,r) exist only in the simulated world,
not in the map that Nav2 loads. NALA's outer box (body_length x body_width in robot.yaml) is checked against
walls and obstacles at every step: a move that would make it touch something is not made and counts as a
contact. The true pose and velocity are logged to a CSV file.

Not simulated: odometry drift, wheel slip, LiDAR motion blur, the MCU's speed controller.
"""
import argparse
import csv
import math
import time
from pathlib import Path
import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt
import rclpy
import rclpy.duration
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from nala_base.kinematics import limit_wheel_speed
from nala_base.mcu_protocol import SCALE, to_count

BEAMS = 300   # Fits one DDS fragment; see env.sh.
CONFIG = Path(__file__).resolve().parents[2] / 'config/robot.yaml'


def mcu_command(vx, vy, wz, base):
    """What reaches the wheels: nala_base's wheel speed limit, then the MCU protocol's int8 steps."""
    k = base['half_track_width'] + base['half_wheelbase']
    return tuple(to_count(v) * SCALE for v in limit_wheel_speed(vx, vy, wz, k, base['max_wheel_speed']))


def box_points(footprint, spacing=.01):
    """Points every `spacing` m filling NALA's outer box (base_footprint frame)."""
    f = np.asarray(footprint, float)
    xs = np.arange(f[:, 0].min(), f[:, 0].max() + 1e-9, spacing)
    ys = np.arange(f[:, 1].min(), f[:, 1].max() + 1e-9, spacing)
    return np.array([[x, y] for x in xs for y in ys])


class World:
    """Occupancy at 1 cm with a distance field for collision checks and ray casting."""
    def __init__(self, map_yaml, obstacles, res=0.01):
        m = yaml.safe_load(open(map_yaml))
        img = np.array(Image.open(Path(map_yaml).parent / m['image']))
        self.res = res
        self.ox, self.oy = m['origin'][:2]
        k = int(round(m['resolution'] / res))
        occ = np.kron((img == 0).astype(np.uint8), np.ones((k, k), np.uint8)).astype(bool)[::-1]  # row 0 = origin y
        yy, xx = np.mgrid[0:occ.shape[0], 0:occ.shape[1]]
        cx, cy = self.ox + (xx + .5) * res, self.oy + (yy + .5) * res
        for x, y, r in obstacles:
            occ |= (cx - x) ** 2 + (cy - y) ** 2 <= r * r
        self.occ = occ
        self.dist = distance_transform_edt(~occ) * res

    def touches(self, x, y, yaw, body):
        c, s = math.cos(yaw), math.sin(yaw)
        px = x + c * body[:, 0] - s * body[:, 1]
        py = y + s * body[:, 0] + c * body[:, 1]
        r = ((py - self.oy) / self.res).astype(int)
        k = ((px - self.ox) / self.res).astype(int)
        inside = (r >= 0) & (r < self.occ.shape[0]) & (k >= 0) & (k < self.occ.shape[1])
        return bool((~inside).any() or self.occ[r[inside], k[inside]].any())

    def clearance(self, x, y):
        r, c = int((y - self.oy) / self.res), int((x - self.ox) / self.res)
        if not (0 <= r < self.occ.shape[0] and 0 <= c < self.occ.shape[1]):
            return 0.
        return float(self.dist[r, c])

    def scan(self, x, y, angles, max_range=12.):
        # Sphere tracing on the distance field: exact to the grid resolution and fast in numpy.
        t = np.full(len(angles), .05)
        c, s = np.cos(angles), np.sin(angles)
        for _ in range(80):
            px, py = x + t * c, y + t * s
            r = ((py - self.oy) / self.res).astype(int)
            k = ((px - self.ox) / self.res).astype(int)
            inside = (r >= 0) & (r < self.occ.shape[0]) & (k >= 0) & (k < self.occ.shape[1])
            d = np.zeros_like(t)
            d[inside] = self.dist[r[inside], k[inside]]
            d[~inside] = max_range
            t = np.minimum(t + np.maximum(d, self.res * .5) * (d > self.res * .5), max_range)
        return t


class FakeNala(Node):
    def __init__(self, a):
        super().__init__('fake_nala')
        self.world = World(a.map, a.obstacle)
        robot = yaml.safe_load(open(a.config))
        hx, hy = robot['body_length'] / 2, robot['body_width'] / 2
        self.body = box_points([[hx, hy], [hx, -hy], [-hx, -hy], [-hx, hy]])
        self.laser = (robot['laser']['x'], robot['laser']['y'], robot['laser']['yaw'])
        # nala_base settings next to robot.yaml (config/base.yaml).
        self.base = yaml.safe_load(open(Path(a.config).with_name('base.yaml')))['base_node']['ros__parameters']
        self.x0, self.y0, self.yaw0 = a.start
        self.x, self.y, self.yaw = a.start
        self.v = self.vy = self.w = 0.
        self.cmd = (0., 0., 0.)
        self.cmd_time = -1.
        self.history = [(time.monotonic(), *a.start)]
        self.contact_until = 0.
        self.contacts = 0
        self.rng = np.random.default_rng(1)
        self.log_file = open(a.log, 'w', newline='', buffering=1)  # Line-buffered: complete even if killed.
        self.log = csv.writer(self.log_file)
        self.log.writerow(['t', 'x', 'y', 'yaw', 'v', 'vy', 'w', 'clearance', 'contact'])
        self.t0 = time.monotonic()
        self.create_subscription(Twist, '/cmd_vel', self.command, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)  # Reliable: Nav2 subscribes with SystemDefaultsQoS.
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 100)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.dt = .02
        self.create_timer(self.dt, self.step)
        self.create_timer(self.base['feedback_period'], self.publish_odom)
        self.create_timer(.1, self.publish_scan)

    def header(self, frame=''):
        h = Odometry().header
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = frame
        return h

    def command(self, msg):
        self.cmd = mcu_command(msg.linear.x, msg.linear.y, msg.angular.z, self.base)
        self.cmd_time = time.monotonic()

    def step(self):
        now = time.monotonic()
        tv, tvy, tw = self.cmd if now - self.cmd_time < self.base['cmd_vel_timeout'] else (0., 0., 0.)
        self.v += float(np.clip(tv - self.v, -1. * self.dt, 1. * self.dt))
        self.vy += float(np.clip(tvy - self.vy, -1. * self.dt, 1. * self.dt))
        self.w += float(np.clip(tw - self.w, -4. * self.dt, 4. * self.dt))
        yaw = self.yaw + self.w * self.dt
        x = self.x + (self.v * math.cos(yaw) - self.vy * math.sin(yaw)) * self.dt
        y = self.y + (self.v * math.sin(yaw) + self.vy * math.cos(yaw)) * self.dt
        if self.world.touches(x, y, yaw, self.body):
            if now > self.contact_until:
                self.contacts += 1
                self.get_logger().warn(f'CONTACT: the box would touch something at ({self.x:.2f}, {self.y:.2f})')
            self.contact_until = now + .3
            self.v = self.vy = self.w = 0.
            x, y, yaw = self.x, self.y, self.yaw
        self.x, self.y, self.yaw = x, y, math.atan2(math.sin(yaw), math.cos(yaw))
        self.history.append((now, self.x, self.y, self.yaw))
        del self.history[:-20]
        if int(now / .1) != int((now - self.dt) / .1):
            self.log.writerow([f'{now - self.t0:.2f}', f'{self.x:.4f}', f'{self.y:.4f}', f'{self.yaw:.4f}',
                               f'{self.v:.3f}', f'{self.vy:.3f}', f'{self.w:.3f}',
                               f'{self.world.clearance(self.x, self.y):.3f}', int(now < self.contact_until)])

    def odom_pose(self):
        # Odometry starts at (0, 0, 0) where the robot booted.
        dx, dy = self.x - self.x0, self.y - self.y0
        c, s = math.cos(-self.yaw0), math.sin(-self.yaw0)
        return c * dx - s * dy, s * dx + c * dy, self.yaw - self.yaw0

    def publish_odom(self):
        x, y, yaw = self.odom_pose()
        od = Odometry()
        od.header = self.header('odom')
        od.child_frame_id = 'base_footprint'
        od.pose.pose.position.x, od.pose.pose.position.y = x, y
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        od.twist.twist.linear.x, od.twist.twist.linear.y, od.twist.twist.angular.z = self.v, self.vy, self.w
        self.odom_pub.publish(od)
        t = TransformStamped()
        t.header = od.header
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.rotation = od.pose.pose.orientation
        self.tf_pub.publish(TFMessage(transforms=[t]))

    def publish_scan(self):
        scan = LaserScan()
        scan.header = self.header('laser')
        # rplidar_ros stamps a scan with its start time, one scan period before publishing.
        scan.header.stamp = (self.get_clock().now() - rclpy.duration.Duration(seconds=.1)).to_msg()
        scan.angle_min, scan.angle_increment = -math.pi, 2 * math.pi / BEAMS
        scan.angle_max = scan.angle_min + (BEAMS - 1) * scan.angle_increment
        scan.range_min, scan.range_max, scan.scan_time = .15, 12., .1
        # Ray-cast from the pose at the stamp time, so scan and odometry agree while turning.
        _, x, y, yaw = min(self.history, key=lambda h: abs(h[0] - (time.monotonic() - .1)))
        lx, ly, lyaw = self.laser
        x, y = x + math.cos(yaw) * lx - math.sin(yaw) * ly, y + math.sin(yaw) * lx + math.cos(yaw) * ly
        angles = scan.angle_min + np.arange(BEAMS) * scan.angle_increment + yaw + lyaw   # backward LiDAR: lyaw = pi
        ranges = self.world.scan(x, y, angles) + self.rng.normal(0, .01, BEAMS)
        ranges[ranges >= 11.9] = np.inf
        scan.ranges = ranges.astype(float).tolist()
        self.scan_pub.publish(scan)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True)
    ap.add_argument('--start', type=float, nargs=3, required=True, metavar=('X', 'Y', 'YAW'))
    ap.add_argument('--obstacle', type=lambda s: tuple(map(float, s.split(','))), action='append', default=[],
                    metavar='X,Y,R')
    ap.add_argument('--log', required=True)
    ap.add_argument('--config', default=str(CONFIG), help="the robot repo's robot.yaml (outline and LiDAR mount)")
    a = ap.parse_args()
    rclpy.init()
    node = FakeNala(a)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.get_logger().info(f'contacts={node.contacts}')
        node.log_file.close()


if __name__ == '__main__':
    main()
