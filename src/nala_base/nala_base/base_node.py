"""ROS node: /cmd_vel -> MCU velocity frames over the UART, MCU wheel speeds -> /odom and TF.

All parameters come from config/base.yaml (no defaults here).
Odometry: each measured wheel speed line is integrated over the time since the line before.
"""

import math
import time

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
import serial
from tf2_ros import TransformBroadcaster

from nala_base.kinematics import integrate_pose, limit_twist, wheels_to_body
from nala_base.mcu_protocol import encode_command, parse_line

ODOM_FRAME = 'odom'
BASE_FRAME = 'base_footprint'


class BaseNode(Node):

    def __init__(self):
        super().__init__('base_node')
        port = self.param('serial_port', Parameter.Type.STRING)
        baud = self.param('baud_rate', Parameter.Type.INTEGER)
        self.max_linear = self.param('max_linear_speed', Parameter.Type.DOUBLE)
        self.max_angular = self.param('max_angular_speed', Parameter.Type.DOUBLE)
        self.cmd_timeout = self.param('cmd_vel_timeout', Parameter.Type.DOUBLE)
        rate = self.param('command_rate', Parameter.Type.DOUBLE)
        self.radius = self.param('wheel_radius', Parameter.Type.DOUBLE)
        self.k = (self.param('half_track_width', Parameter.Type.DOUBLE)
                  + self.param('half_wheelbase', Parameter.Type.DOUBLE))
        poll_rate = self.param('feedback_poll_rate', Parameter.Type.DOUBLE)
        self.feedback_timeout = self.param('feedback_timeout', Parameter.Type.DOUBLE)

        self.serial = serial.Serial(port, baud, timeout=0)
        self.serial.reset_input_buffer()   # drop old lines from before the start
        self.rx_buffer = b''
        self.cmd = (0.0, 0.0, 0.0)
        self.cmd_time = None      # time.monotonic() of the last /cmd_vel
        self.stopped = True       # True while sending zero because /cmd_vel is too old

        self.pose = (0.0, 0.0, 0.0)            # x, y, yaw in the odom frame
        self.wheels_time = time.monotonic()    # time of the last wheel speed line (or the start)
        self.feedback_lost = False             # True after feedback_timeout without wheel speeds

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.create_timer(1.0 / rate, self.on_command_timer)
        self.create_timer(1.0 / poll_rate, self.read_feedback)
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

    def on_command_timer(self):
        self.send_command()
        if not self.feedback_lost and time.monotonic() - self.wheels_time > self.feedback_timeout:
            self.feedback_lost = True
            self.get_logger().warn(
                f'No wheel speeds from the MCU for {self.feedback_timeout} s: odometry stops.')

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
                self.on_wheels(value)
            elif kind == 'echo':
                self.get_logger().debug(f'echo vx vy wz: {value}')
            elif value == 'cmd timeout':
                self.get_logger().warn('MCU: cmd timeout (no velocity frame for 500 ms)')
            else:
                self.get_logger().info(f'MCU: {value}')

    def on_wheels(self, wheels):
        """Integrate one measured wheel speed line and publish /odom and TF."""
        now = time.monotonic()
        dt = now - self.wheels_time
        self.wheels_time = now
        vx, vy, wz = wheels_to_body(*wheels, self.radius, self.k)
        if self.feedback_lost:
            # Do not integrate over the gap: the speed during it is unknown.
            self.feedback_lost = False
            self.get_logger().info('Wheel speeds from the MCU again.')
        else:
            self.pose = integrate_pose(*self.pose, vx, vy, wz, dt)
        self.publish_odom(vx, vy, wz)

    def publish_odom(self, vx, vy, wz):
        x, y, yaw = self.pose
        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = ODOM_FRAME
        odom.child_frame_id = BASE_FRAME
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = math.sin(yaw / 2)
        odom.pose.pose.orientation.w = math.cos(yaw / 2)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header = odom.header
        tf.child_frame_id = BASE_FRAME
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf)

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
