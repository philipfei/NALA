"""PC: click the base station in RViz ("2D Goal Pose") and save it as base_station.yaml next to the map.

The pose must leave room to turn on the spot (collision radius from robot.yaml + padding).
"""
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from . import base_station
from .geometry import Grid
from .params import node_settings
from .ros_common import pose3


class BaseStationPicker(Node):
    def __init__(self):
        super().__init__('base_station_picker')
        self.settings = node_settings(self)
        runtime = self.settings['runtime']
        if not runtime['map_yaml']:
            raise ValueError('base_station_picker needs the map_yaml parameter')
        self.grid = Grid.load(runtime['map_yaml'])
        self.path = runtime['base_station_yaml'] or base_station.default_path(runtime['map_yaml'])
        self.create_subscription(PoseStamped, '/goal_pose', self.picked, 10)
        self.get_logger().info(f'Click the base station with "2D Goal Pose" in RViz. It is saved to {self.path}')

    def picked(self, msg):
        map_frame = self.settings['runtime']['map_frame']
        if msg.header.frame_id != map_frame:
            self.get_logger().error(f'Pose is in {msg.header.frame_id}, expected {map_frame}')
            return
        pose = pose3(msg.pose)
        try:
            base_station.check(self.grid, pose, self.settings.collision)
        except ValueError as error:
            self.get_logger().error(f'{error}. Pick another place.')
            return
        base_station.save(self.path, pose, map_frame)
        self.get_logger().info(f'Saved base station x={pose[0]:.2f} y={pose[1]:.2f} yaw={pose[2]:.2f} to {self.path}')


def main(args=None):
    rclpy.init(args=args)
    node = BaseStationPicker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
