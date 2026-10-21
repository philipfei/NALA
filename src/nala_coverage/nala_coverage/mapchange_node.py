"""Map change node: LiDAR scans at trusted poses -> cells that changed against the saved map.

In:  /scan, TF map -> laser at the scan time, /coverage/trust (the supervisor's localization trust),
     /coverage/definition (a new task starts a new comparison).
Out: /map_changes (OccupancyGrid on the map grid: 100 = new object, 50 = removed object, 0 = no change), latched;
     changes.npz in the run folder (appeared, vanished).
Settings: map_changes in config/coverage.yaml.
"""
import math
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from .geometry import Grid
from .mapchange import ChangeMap
from .params import node_settings
from .ros_common import run_until_stopped, LATCHED, LATEST, decode, transform3

NEW, REMOVED = 100, 50


class MapChangeNode(Node):
    def __init__(self):
        super().__init__('map_change_node')
        self.settings = node_settings(self)
        self.cfg = self.settings['map_changes']
        self.frame = self.settings['runtime']['map_frame']
        self.grid = Grid.load(self.settings['runtime']['map_yaml'])
        self.changes = ChangeMap(self.grid, self.cfg)
        self.output = Path(self.settings['runtime']['output_dir'])
        self.trusted = False
        self.generation = None
        self.last_scan = 0.
        self.tf = Buffer(cache_time=Duration(seconds=10.))
        # Own small TF node and thread, like the supervisor (ros_common.Inputs): /tf (~45/s) does not wake this node.
        self.listener = TransformListener(self.tf, None, spin_thread=True)
        self.pub = self.create_publisher(OccupancyGrid, '/map_changes', LATCHED)
        self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data)
        self.create_subscription(String, '/coverage/trust', self.trust, LATEST)
        self.create_subscription(String, '/coverage/definition', self.definition, LATCHED)
        self.create_timer(self.cfg['publish_period_s'], self.publish)

    def trust(self, msg):
        try:
            self.trusted = bool(decode(msg)['trusted'])
        except (ValueError, KeyError, TypeError):
            self.trusted = False

    def definition(self, msg):
        try:
            generation = decode(msg)['generation']
        except (ValueError, KeyError, TypeError):
            return
        if generation != self.generation:
            self.generation = generation
            self.changes.reset()

    def scan(self, msg):
        now = time.monotonic()
        if not self.trusted or now - self.last_scan < self.cfg['scan_period_s']:
            return
        try:
            t = self.tf.lookup_transform(self.frame, msg.header.frame_id, Time.from_msg(msg.header.stamp),
                                         timeout=Duration(seconds=0.))
        except TransformException:
            return
        self.last_scan = now
        x, y, yaw = transform3(t)
        step = self.cfg['beam_step']
        ranges = np.asarray(msg.ranges, float)[::step]
        angles = yaw + msg.angle_min + np.arange(len(msg.ranges))[::step] * msg.angle_increment
        ranges = np.where(ranges < msg.range_min, np.nan, ranges)
        self.changes.add_scan((x, y), angles, ranges, msg.range_max)

    def publish(self):
        appeared, vanished = self.changes.appeared(), self.changes.vanished()
        g = self.grid
        m = OccupancyGrid()
        m.header.frame_id = self.frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.info.resolution = g.resolution
        m.info.height, m.info.width = g.cells.shape
        m.info.origin.position.x, m.info.origin.position.y = g.origin[0], g.origin[1]
        m.info.origin.orientation.z = math.sin(g.origin[2] / 2)
        m.info.origin.orientation.w = math.cos(g.origin[2] / 2)
        m.data = np.where(appeared, NEW, np.where(vanished, REMOVED, 0)).astype(np.int8).ravel().tolist()
        self.pub.publish(m)
        if self.generation is not None:
            self.output.mkdir(parents=True, exist_ok=True)
            np.savez_compressed(self.output / 'changes.npz', appeared=appeared, vanished=vanished)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MapChangeNode()
    try:
        run_until_stopped(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the TF listener's thread: it is not a daemon thread, so the process cannot exit while it runs.
        node.listener.executor.shutdown()
        node.listener.dedicated_listener_thread.join()
        node.destroy_node()
        rclpy.try_shutdown()
