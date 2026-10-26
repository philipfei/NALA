"""ROS node: scintillator on USB serial -> nala_interfaces/CountRate on /scintillator/count_rate.

All parameters come from config/scintillator.yaml (no defaults here).
The scintillator sends one text line per second (nala_base/scintillator.py, docs/scintillator.md).
Every line with a count rate makes one message. Its stamp is the Pi time of arrival (= end of the counting
interval; the scintillator's own clock is not used).
A reader thread blocks on the serial port, so each line gets its arrival time.
"""

import threading
import time

from nala_interfaces.msg import CountRate
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
import serial

from nala_base.scintillator import parse_line, split_lines

READ_TIMEOUT = 0.1          # s, a blocking read returns after this, so the reader thread can stop
BAD_LINE_LOG_PERIOD = 10.0  # s, at most one warning about unreadable lines per this time


class ScintillatorNode(Node):

    def __init__(self):
        super().__init__('scintillator_node')
        port = self.param('serial_port', Parameter.Type.STRING)
        baud = self.param('baud_rate', Parameter.Type.INTEGER)
        rtscts = self.param('rtscts', Parameter.Type.BOOL)
        self.interval_s = self.param('interval_s', Parameter.Type.DOUBLE)
        self.reading_timeout = self.param('reading_timeout', Parameter.Type.DOUBLE)

        # exclusive: a second reader (e.g. miniterm) would take part of the lines.
        self.serial = serial.Serial(port, baud, rtscts=rtscts, exclusive=True, timeout=READ_TIMEOUT)
        self.serial.reset_input_buffer()
        self.reading_time = time.monotonic()  # time of the last reading (or the start)
        self.lost = False                     # True after reading_timeout without readings
        self.bad_lines = 0                    # lines without a count rate since the start

        self.pub = self.create_publisher(CountRate, '/scintillator/count_rate', 10)
        self.create_timer(self.reading_timeout / 2, self.check_timeout)
        self.reading = True
        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()
        self.get_logger().info(f'{port} at {baud} baud, RTS/CTS {rtscts}.')

    def param(self, name, param_type):
        self.declare_parameter(name, param_type)
        return self.get_parameter(name).value

    def read_loop(self):
        """Reader thread: wait for scintillator bytes and publish one CountRate per line."""
        buffer = b''
        while self.reading:
            buffer += self.serial.read(self.serial.in_waiting or 1)
            lines, buffer = split_lines(buffer)
            for line in lines:
                rate = parse_line(line)
                if rate is None:
                    self.bad_line(line)
                else:
                    self.publish(rate)

    def bad_line(self, line):
        if not line.strip():
            return
        self.bad_lines += 1
        self.get_logger().warn(f'Line without a count rate ({self.bad_lines} so far): {line!r}',
                               throttle_duration_sec=BAD_LINE_LOG_PERIOD)

    def publish(self, rate):
        self.reading_time = time.monotonic()
        if self.lost:
            self.lost = False
            self.get_logger().info('Scintillator readings again.')
        msg = CountRate()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.counts_per_second = rate
        msg.interval_s = self.interval_s
        self.pub.publish(msg)

    def check_timeout(self):
        if not self.lost and time.monotonic() - self.reading_time > self.reading_timeout:
            self.lost = True
            self.get_logger().error(
                f'No scintillator reading for {self.reading_timeout} s. Check the scintillator and its USB cable.')

    def stop(self):
        self.reading = False
        self.reader.join()
        self.serial.close()


def main():
    rclpy.init()
    node = ScintillatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()
