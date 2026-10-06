"""ROS node: /cmd_vel -> MCU velocity frames over the UART, and read the MCU feedback lines.

All parameters come from config/base.yaml (no defaults here).
Stage 2 adds /odom and TF from the measured wheel speeds.
"""

import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
import serial

from nala_base.kinematics import limit_twist
from nala_base.mcu_protocol import encode_command, parse_line


class BaseNode(Node):

    def __init__(self):
        super().__init__('base_node')
        port = self.param('serial_port', Parameter.Type.STRING)
        baud = self.param('baud_rate', Parameter.Type.INTEGER)
        self.max_linear = self.param('max_linear_speed', Parameter.Type.DOUBLE)
        self.max_angular = self.param('max_angular_speed', Parameter.Type.DOUBLE)
        self.cmd_timeout = self.param('cmd_vel_timeout', Parameter.Type.DOUBLE)
        rate = self.param('command_rate', Parameter.Type.DOUBLE)

        self.serial = serial.Serial(port, baud, timeout=0)
        self.rx_buffer = b''
        self.cmd = (0.0, 0.0, 0.0)
        self.cmd_time = None      # time.monotonic() of the last /cmd_vel
        self.stopped = True       # True while sending zero because /cmd_vel is too old

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info(
            f'{port} at {baud} baud, sending at {rate} Hz. '
            f'Limits: {self.max_linear} m/s, {self.max_angular} rad/s. '
            f'/cmd_vel timeout: {self.cmd_timeout} s.')

    def param(self, name, param_type):
        self.declare_parameter(name, param_type)
        return self.get_parameter(name).value

    def on_cmd_vel(self, msg):
        self.cmd = limit_twist(msg.linear.x, msg.linear.y, msg.angular.z,
                               self.max_linear, self.max_angular)
        self.cmd_time = time.monotonic()

    def on_timer(self):
        self.send_command()
        self.read_feedback()

    def send_command(self):
        fresh = (self.cmd_time is not None
                 and time.monotonic() - self.cmd_time <= self.cmd_timeout)
        if fresh:
            self.stopped = False
        elif not self.stopped:
            self.stopped = True
            self.get_logger().info(f'No /cmd_vel for {self.cmd_timeout} s: stop.')
        vx, vy, wz = self.cmd if fresh else (0.0, 0.0, 0.0)
        frame = encode_command(vx, vy, wz)
        self.serial.write(frame)
        self.get_logger().debug(f'sent {frame.hex(" ")}')

    def read_feedback(self):
        self.rx_buffer += self.serial.read(self.serial.in_waiting)
        *lines, self.rx_buffer = self.rx_buffer.split(b'\n')
        if len(self.rx_buffer) > 1000:   # noise without any newline
            self.rx_buffer = b''
        for raw in lines:
            result = parse_line(raw.decode('ascii', errors='replace'))
            if result is None:
                continue
            kind, value = result
            if kind == 'wheels':
                self.get_logger().debug(f'wheels M1..M4 [rad/s]: {value}')
            elif kind == 'echo':
                self.get_logger().debug(f'echo vx vy wz: {value}')
            elif value == 'cmd timeout':
                self.get_logger().warn('MCU: cmd timeout (no velocity frame for 500 ms)')
            else:
                self.get_logger().info(f'MCU: {value}')

    def stop(self):
        """Send a zero command and close the port."""
        self.serial.write(encode_command(0.0, 0.0, 0.0))
        self.serial.flush()
        self.serial.close()


def main():
    rclpy.init()
    node = BaseNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()
