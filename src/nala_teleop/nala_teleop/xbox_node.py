"""Pi: Xbox controller teleop, /joy (joy_node) -> /cmd_vel. The rules are in xbox.py.

ros2 run nala_teleop xbox_teleop --ros-args --params-file config/teleop.yaml
All parameters come from config/teleop.yaml (no defaults here).
Hold LB; after pressing it, let both sticks be centred once, then drive. Release LB = stop.
"""

import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Joy

from nala_teleop.keyboard_node import make_twist
from nala_teleop.xbox import XboxTeleop


class XboxTeleopNode(Node):

    def __init__(self):
        super().__init__('xbox_teleop')
        self.teleop = XboxTeleop(
            enable_button=self.param('enable_button', Parameter.Type.INTEGER),
            axes=(self.param('axis_linear_x', Parameter.Type.INTEGER),
                  self.param('axis_linear_y', Parameter.Type.INTEGER),
                  self.param('axis_angular_yaw', Parameter.Type.INTEGER)),
            scales=(self.param('scale_linear_x', Parameter.Type.DOUBLE),
                    self.param('scale_linear_y', Parameter.Type.DOUBLE),
                    self.param('scale_angular_yaw', Parameter.Type.DOUBLE)),
            joy_timeout=self.param('joy_timeout', Parameter.Type.DOUBLE))
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Joy, 'joy', self.on_joy, 10)
        self.create_timer(self.teleop.joy_timeout / 4, self.check_timeout)

    def param(self, name, param_type):
        self.declare_parameter(name, param_type)
        return self.get_parameter(name).value

    def send(self, command):
        if command is not None:
            self.pub.publish(make_twist(*command))

    def on_joy(self, msg):
        self.send(self.teleop.update(msg.buttons, msg.axes, time.monotonic()))

    def check_timeout(self):
        command = self.teleop.timeout(time.monotonic())
        if command is not None:
            self.get_logger().warn(
                f'No /joy for {self.teleop.joy_timeout} s while LB is held: stop. Release and press LB again.')
        self.send(command)


def main():
    rclpy.init()
    node = XboxTeleopNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
