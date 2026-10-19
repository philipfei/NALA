"""Locator node: finds the robot on the map from the LiDAR scan while no task drives (nala_coverage.locate).

In:  /scan, TF base_link -> laser (the LiDAR mount), /coverage/state (the supervisor's task state).
Out: /coverage/located (JSON: status ON_BASE / BACKWARDS / NOT_ON_BASE, pose, fit, ambiguous).
Settings: locator in config/coverage.yaml. Idle while a task runs or is paused (CPU on the Pi; AMCL tracks it then).
"""
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from . import base_station
from .geometry import Grid
from .locate import Locator, endpoints
from .params import node_settings
from .ros_common import LATEST, decode, json_msg, transform3

# Supervisor states in which no task holds the robot: the locator works only then.
IDLE_STATES = ('', 'IDLE', 'FINISHED', 'CANCELED', 'FAILED')


class LocatorNode(Node):
    def __init__(self):
        super().__init__('locator_node')
        self.settings = node_settings(self)
        self.cfg = self.settings['locator']
        runtime = self.settings['runtime']
        self.base_frame = runtime['base_frame']
        grid = Grid.load(runtime['map_yaml'])
        station_file = runtime['base_station_yaml'] or str(base_station.default_path(runtime['map_yaml']))
        self.station = base_station.load(station_file, runtime['map_frame'])
        self.locator = Locator(grid, self.cfg)
        self.scan = None
        self.state = ''
        self.tf = Buffer(cache_time=Duration(seconds=10.))
        # Own small TF node and thread, like the supervisor (ros_common.Inputs): /tf (~45/s) does not wake this node.
        self.listener = TransformListener(self.tf, None, spin_thread=True)
        self.pub = self.create_publisher(String, '/coverage/located', LATEST)
        self.create_subscription(LaserScan, '/scan', self.scan_received, qos_profile_sensor_data)
        self.create_subscription(String, '/coverage/state', self.state_received, LATEST)
        self.create_timer(self.cfg['period_s'], self.tick)

    def scan_received(self, msg):
        self.scan = msg

    def state_received(self, msg):
        try:
            self.state = decode(msg).get('state', '')
        except (ValueError, AttributeError):
            pass

    def tick(self):
        scan = self.scan
        if self.state not in IDLE_STATES or scan is None:
            return
        try:
            laser = transform3(self.tf.lookup_transform(self.base_frame, scan.header.frame_id, Time()))
        except TransformException as e:
            self.get_logger().warning('No LiDAR mount TF: ' + str(e).split('\n')[0], throttle_duration_sec=5.)
            return
        points = endpoints(scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max,
                           laser, self.cfg['max_beams'])
        self.pub.publish(json_msg(self.locator.locate(points, self.station)))


def main(args=None):
    rclpy.init(args=args)
    node = LocatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the TF listener's thread: it is not a daemon thread, so the process cannot exit while it runs.
        node.listener.executor.shutdown()
        node.listener.dedicated_listener_thread.join()
        node.destroy_node()
        rclpy.try_shutdown()
