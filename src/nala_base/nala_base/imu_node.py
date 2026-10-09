"""ROS node: WitMotion WT901 IMU on a UART -> sensor_msgs/Imu on /imu.

All parameters come from config/imu.yaml (no defaults here).
The IMU sends acceleration (0x51) and angular velocity (0x52) frames (scripts/setup_imu.py).
Every gyro frame makes one Imu message with the newest acceleration.
Only the angular velocity is meant to be used (the EKF fuses its z part). The message
has no orientation (orientation_covariance[0] = -1: the magnetometer is not used) and
the acceleration is only for viewing (linear_acceleration_covariance[0] = -1).
A reader thread blocks on the UART, so each frame gets its arrival time.
"""

import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu
import serial

from nala_base.wt901 import split_frames

READ_TIMEOUT = 0.1   # s, a blocking read returns after this, so the reader thread can stop


class ImuNode(Node):

    def __init__(self):
        super().__init__('imu_node')
        port = self.param('serial_port', Parameter.Type.STRING)
        baud = self.param('baud_rate', Parameter.Type.INTEGER)
        self.frame_id = self.param('frame_id', Parameter.Type.STRING)
        self.gyro_variance = self.param('angular_velocity_variance', Parameter.Type.DOUBLE)
        self.imu_timeout = self.param('imu_timeout', Parameter.Type.DOUBLE)

        self.serial = serial.Serial(port, baud, timeout=READ_TIMEOUT)
        self.serial.reset_input_buffer()
        self.acc = [0.0, 0.0, 9.81]           # newest acceleration (m/s^2); flat until the first 0x51
        self.gyro_time = time.monotonic()     # time of the last gyro frame (or the start)
        self.lost = False                     # True after imu_timeout without gyro frames

        self.pub = self.create_publisher(Imu, 'imu', 50)
        self.create_timer(self.imu_timeout / 2, self.check_timeout)
        self.reading = True
        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()
        self.get_logger().info(f'{port} at {baud} baud, frame {self.frame_id}.')

    def param(self, name, param_type):
        self.declare_parameter(name, param_type)
        return self.get_parameter(name).value

    def read_loop(self):
        """Reader thread: wait for IMU bytes and publish one Imu message per gyro frame."""
        buffer = b''
        while self.reading:
            buffer += self.serial.read(self.serial.in_waiting or 1)
            frames, buffer = split_frames(buffer)
            for kind, values in frames:
                if kind == 'acc':
                    self.acc = values
                else:
                    self.publish(values)

    def publish(self, gyro):
        self.gyro_time = time.monotonic()
        if self.lost:
            self.lost = False
            self.get_logger().info('IMU data again.')
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.orientation_covariance[0] = -1.0             # no orientation
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyro
        for i in (0, 4, 8):
            msg.angular_velocity_covariance[i] = self.gyro_variance
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = self.acc
        msg.linear_acceleration_covariance[0] = -1.0     # acceleration not for use
        self.pub.publish(msg)

    def check_timeout(self):
        if not self.lost and time.monotonic() - self.gyro_time > self.imu_timeout:
            self.lost = True
            self.get_logger().error(
                f'No IMU data for {self.imu_timeout} s: the EKF has no turn rate now '
                '(it does not use the wheel turn rate). Check the IMU and its cable.')

    def stop(self):
        self.reading = False
        self.reader.join()
        self.serial.close()


def main():
    rclpy.init()
    node = ImuNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()
