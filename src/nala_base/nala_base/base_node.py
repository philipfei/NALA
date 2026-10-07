"""ROS node: /cmd_vel -> MCU velocity frames over the UART, MCU wheel speeds -> /odom and TF.

All parameters come from config/base.yaml (no defaults here).
Odometry: each measured wheel speed line is integrated over the time since the line before.
A reader thread blocks on the UART, so each line gets its arrival time and no CPU is used while waiting.
"""

import math
import threading
import time

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
import serial
from tf2_ros import TransformBroadcaster

from nala_base.kinematics import integrate_pose, limit_wheel_speed, wheels_to_body
from nala_base.mcu_protocol import encode_command, parse_line

ODOM_FRAME = 'odom'
BASE_FRAME = 'base_footprint'
READ_TIMEOUT = 0.1   # s, a blocking read returns after this, so the reader thread can stop


class BaseNode(Node):

    def __init__(self):
        super().__init__('base_node')
        port = self.param('serial_port', Parameter.Type.STRING)
        baud = self.param('baud_rate', Parameter.Type.INTEGER)
        self.max_wheel_speed = self.param('max_wheel_speed', Parameter.Type.DOUBLE)
        self.cmd_timeout = self.param('cmd_vel_timeout', Parameter.Type.DOUBLE)
        rate = self.param('command_rate', Parameter.Type.DOUBLE)
        self.radius = self.param('wheel_radius', Parameter.Type.DOUBLE)
        self.k = (self.param('half_track_width', Parameter.Type.DOUBLE)
                  + self.param('half_wheelbase', Parameter.Type.DOUBLE))
        self.feedback_timeout = self.param('feedback_timeout', Parameter.Type.DOUBLE)

        self.serial = serial.Serial(port, baud, timeout=READ_TIMEOUT)
        self.serial.reset_input_buffer()   # drop old lines from before the start
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
        self.reading = True
        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()
        self.get_logger().info(
            f'{port} at {baud} baud, sending at {rate} Hz. '
            f'Max wheel speed: {self.max_wheel_speed} m/s. '
            f'/cmd_vel timeout: {self.cmd_timeout} s.')

    def param(self, name, param_type):
        self.declare_parameter(name, param_type)
        return self.get_parameter(name).value

    def on_cmd_vel(self, msg):
        self.cmd = limit_wheel_speed(msg.linear.x, msg.linear.y, msg.angular.z,
                                     self.k, self.max_wheel_speed)
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

    def read_loop(self):
        """Reader thread: wait for MCU bytes and handle each complete line."""
        rx_buffer = b''
        while self.reading:
            rx_buffer += self.serial.read(self.serial.in_waiting or 1)
            *lines, rx_buffer = rx_buffer.split(b'\n')
            if len(rx_buffer) > 1000:   # noise without any newline
                rx_buffer = b''
            for raw in lines:
                self.handle_line(raw.decode('ascii', errors='replace'))

    def handle_line(self, line):
        result = parse_line(line)
        if result is None:
            return
        kind, value = result
        if kind == 'wheels':
            self.get_logger().debug(f'wheels M1..M4 [rad/s]: {value}')
            self.on_wheels(value)
        elif kind == 'echo':
            self.get_logger().debug(f'echo vx vy wz: {value}')
        elif value == 'cmd timeout':
            self.get_logger().warn('MCU: cmd timeout (no velocity frame for 200 ms)')
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
        """Stop the reader thread, send a zero command and close the port."""
        self.reading = False
        self.reader.join()
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
