"""ROS node: /cmd_vel -> MCU over I2C, MCU encoder counts -> /odom (and TF).

All parameters come from config/base.yaml (no defaults here).
Every timer tick: one I2C write (the command), then one I2C read (the state frame),
as two separate transfers with a STOP between them.
Odometry: the change of the cumulative encoder counts is the exact wheel movement.
The speed divides it by the exact MCU time (periods x feedback_period, from the MCU
period counter); the message stamp is the Pi time of the read.
"""

import fcntl
import math
import time

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from smbus2 import i2c_msg, SMBus
from tf2_ros import TransformBroadcaster

from nala_base.kinematics import integrate_pose, limit_wheel_speed, wheels_to_body
from nala_base.mcu_protocol import (
    count_delta, encode_command, FLAG_BOOT, FLAG_TIMEOUT, parse_state, STATE_LEN)

ODOM_FRAME = 'odom'
BASE_FRAME = 'base_footprint'
I2C_RETRIES = 0x0701      # ioctl numbers from linux/i2c-dev.h
I2C_TIMEOUT = 0x0702      # adapter timeout, in units of 10 ms
LOG_THROTTLE = 5.0        # s, repeat I2C error warnings at most this often


class BaseNode(Node):

    def __init__(self):
        super().__init__('base_node')
        bus = self.param('i2c_bus', Parameter.Type.INTEGER)
        self.address = self.param('i2c_address', Parameter.Type.INTEGER)
        i2c_timeout = self.param('i2c_timeout', Parameter.Type.DOUBLE)
        self.max_wheel_speed = self.param('max_wheel_speed', Parameter.Type.DOUBLE)
        self.cmd_timeout = self.param('cmd_vel_timeout', Parameter.Type.DOUBLE)
        rate = self.param('command_rate', Parameter.Type.DOUBLE)
        self.radius = self.param('wheel_radius', Parameter.Type.DOUBLE)
        self.k = (self.param('half_track_width', Parameter.Type.DOUBLE)
                  + self.param('half_wheelbase', Parameter.Type.DOUBLE))
        self.rad_per_count = 2 * math.pi / self.param('encoder_counts_per_rev', Parameter.Type.INTEGER)
        self.feedback_period = self.param('feedback_period', Parameter.Type.DOUBLE)
        self.feedback_timeout = self.param('feedback_timeout', Parameter.Type.DOUBLE)
        self.publish_tf = self.param('publish_tf', Parameter.Type.BOOL)
        self.twist_variance = self.param('odom_twist_variance', Parameter.Type.DOUBLE_ARRAY)

        self.bus = SMBus(bus)
        # Give up a hanging transfer quickly instead of blocking the executor.
        fcntl.ioctl(self.bus.fd, I2C_TIMEOUT, max(1, round(i2c_timeout / 0.01)))
        fcntl.ioctl(self.bus.fd, I2C_RETRIES, 0)
        self.i2c_errors = 0       # failed transfers (NACK, timeout)
        self.crc_errors = 0       # state frames with a wrong CRC

        self.cmd = (0.0, 0.0, 0.0)
        self.cmd_time = None      # time.monotonic() of the last /cmd_vel
        self.stopped = True       # True while sending zero because /cmd_vel is too old

        self.pose = (0.0, 0.0, 0.0)            # x, y, yaw in the odom frame
        self.counts = None                     # last counts (None: the next frame is only the reference)
        self.seq = 0                           # MCU period counter of the last frame
        self.feedback_time = time.monotonic()  # time of the last valid state frame (or the start)
        self.feedback_lost = False             # True after feedback_timeout without a valid frame
        self.mcu_timed_out = False             # MCU reported its command timeout

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info(
            f'I2C bus {bus}, MCU address 0x{self.address:02x}, {rate} Hz. '
            f'Max wheel speed: {self.max_wheel_speed} m/s. /cmd_vel timeout: {self.cmd_timeout} s. '
            f'Odometry TF: {"on" if self.publish_tf else "off (published by the EKF)"}.')

    def param(self, name, param_type):
        self.declare_parameter(name, param_type)
        return self.get_parameter(name).value

    def on_cmd_vel(self, msg):
        self.cmd = limit_wheel_speed(msg.linear.x, msg.linear.y, msg.angular.z,
                                     self.k, self.max_wheel_speed)
        self.cmd_time = time.monotonic()

    def on_timer(self):
        self.send_command()
        self.read_state()
        if not self.feedback_lost and time.monotonic() - self.feedback_time > self.feedback_timeout:
            self.feedback_lost = True
            self.get_logger().warn(
                f'No valid state from the MCU for {self.feedback_timeout} s '
                f'(I2C errors {self.i2c_errors}, CRC errors {self.crc_errors}): odometry stops.')

    def transfer(self, msg):
        """One I2C transfer. On NACK or timeout: count it, warn (throttled), return False."""
        try:
            self.bus.i2c_rdwr(msg)
            return True
        except OSError as error:
            self.i2c_errors += 1
            self.get_logger().warn(f'I2C error ({self.i2c_errors} so far): {error}',
                                   throttle_duration_sec=LOG_THROTTLE)
            return False

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
        if self.transfer(i2c_msg.write(self.address, frame)):
            self.get_logger().debug(f'sent {frame.hex(" ")}')

    def read_state(self):
        msg = i2c_msg.read(self.address, STATE_LEN)
        if not self.transfer(msg):
            return
        state = parse_state(bytes(list(msg)))
        if state is None:
            self.crc_errors += 1
            self.get_logger().warn(f'State frame with a wrong CRC ({self.crc_errors} so far).',
                                   throttle_duration_sec=LOG_THROTTLE)
            return
        self.on_state(*state)

    def on_state(self, seq, flags, counts):
        """Integrate the count change since the last frame and publish /odom (and TF)."""
        self.feedback_time = time.monotonic()
        if self.feedback_lost:
            self.feedback_lost = False
            self.get_logger().info('Valid state from the MCU again.')
        timed_out = bool(flags & FLAG_TIMEOUT)
        if timed_out and not self.mcu_timed_out:
            self.get_logger().warn('MCU: cmd timeout (no valid command for 200 ms), stopped.')
        self.mcu_timed_out = timed_out

        if self.counts is None or flags & FLAG_BOOT:
            if self.counts is not None:
                self.get_logger().info('MCU restarted: its counters start again at 0.')
            self.counts, self.seq = counts, seq
            return
        periods = (seq - self.seq) % 256
        if periods == 0:          # no new MCU control period since the last read
            return
        angles = [count_delta(new, old) * self.rad_per_count for new, old in zip(counts, self.counts)]
        self.counts, self.seq = counts, seq
        dx, dy, dyaw = wheels_to_body(*angles, self.radius, self.k)   # movement in the robot frame
        self.pose = integrate_pose(*self.pose, dx, dy, dyaw, 1.0)
        dt = periods * self.feedback_period
        self.get_logger().debug(f'wheels M1..M4 [rad/s]: {[round(a / dt, 2) for a in angles]}')
        self.publish_odom(dx / dt, dy / dt, dyaw / dt)

    def publish_odom(self, vx, vy, wz):
        x, y, yaw = self.pose
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = ODOM_FRAME
        odom.child_frame_id = BASE_FRAME
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = math.sin(yaw / 2)
        odom.pose.pose.orientation.w = math.cos(yaw / 2)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        odom.twist.covariance[0] = self.twist_variance[0]    # vx
        odom.twist.covariance[7] = self.twist_variance[1]    # vy
        self.odom_pub.publish(odom)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header = odom.header
            tf.child_frame_id = BASE_FRAME
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf)

    def stop(self):
        """Send a zero command and close the bus."""
        self.transfer(i2c_msg.write(self.address, encode_command(0.0, 0.0, 0.0)))
        self.bus.close()


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
