"""PC keyboard teleop: keys -> /cmd_vel with the fixed speeds from config/teleop.yaml.

ros2 run nala_teleop keyboard_teleop --ros-args --params-file config/teleop.yaml

A terminal cannot see a key release. The Pi stops the robot when no /cmd_vel came for
cmd_vel_timeout (config/base.yaml), so a held key (key repeat) keeps it driving.
"""

import sys
import termios
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.parameter import Parameter

from nala_teleop.keys import HELP, twist_for_key

CTRL_C = '\x03'   # raw mode turns Ctrl+C into a normal character


def read_key():
    """Wait for one key press. Raw mode: no Enter needed, no echo."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def make_twist(vx, vy, wz):
    msg = Twist()
    msg.linear.x = float(vx)
    msg.linear.y = float(vy)
    msg.angular.z = float(wz)
    return msg


def main():
    rclpy.init()
    node = rclpy.create_node('keyboard_teleop')
    node.declare_parameter('linear_speed', Parameter.Type.DOUBLE)
    node.declare_parameter('angular_speed', Parameter.Type.DOUBLE)
    linear = node.get_parameter('linear_speed').value
    angular = node.get_parameter('angular_speed').value
    pub = node.create_publisher(Twist, 'cmd_vel', 10)

    print(HELP)
    print(f'Speed {linear} m/s, turn {angular} rad/s')
    try:
        while True:
            key = read_key()
            if key == CTRL_C:
                break
            pub.publish(make_twist(*twist_for_key(key, linear, angular)))
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            pub.publish(make_twist(0.0, 0.0, 0.0))
        node.destroy_node()
        rclpy.try_shutdown()
