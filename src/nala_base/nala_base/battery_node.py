"""ROS node: INA260 power monitor on the Pi I2C bus -> /battery_state and /battery_low.

All parameters come from config/battery.yaml (no defaults here).
The charge percentage is only a rough value: it is linear between empty_voltage and
full_voltage, but the discharge curve of 18650 Li-ion cells is not linear (flat in the
middle, steep at both ends), and the voltage sags under load.
/battery_low (Bool) is true while the voltage is below low_voltage (for a later
"drive home on low battery").
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import BatteryState
from smbus2 import SMBus
from std_msgs.msg import Bool

from nala_base.ina260 import (
    charge_ratio, decode_current, decode_voltage, REG_CURRENT, REG_VOLTAGE)

LOG_THROTTLE = 30.0   # s, repeat warnings at most this often


class BatteryNode(Node):

    def __init__(self):
        super().__init__('battery_node')
        bus = self.param('i2c_bus', Parameter.Type.INTEGER)
        self.address = self.param('i2c_address', Parameter.Type.INTEGER)
        rate = self.param('publish_rate', Parameter.Type.DOUBLE)
        self.full = self.param('full_voltage', Parameter.Type.DOUBLE)
        self.low = self.param('low_voltage', Parameter.Type.DOUBLE)
        self.empty = self.param('empty_voltage', Parameter.Type.DOUBLE)
        self.capacity = self.param('design_capacity', Parameter.Type.DOUBLE)
        self.current_sign = self.param('current_sign', Parameter.Type.DOUBLE)

        self.bus = SMBus(bus)
        self.state_pub = self.create_publisher(BatteryState, 'battery_state', 10)
        self.low_pub = self.create_publisher(Bool, 'battery_low', 10)
        self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info(
            f'INA260 at 0x{self.address:02x} on I2C bus {bus}. '
            f'Full {self.full} V, low {self.low} V, empty {self.empty} V.')

    def param(self, name, param_type):
        self.declare_parameter(name, param_type)
        return self.get_parameter(name).value

    def on_timer(self):
        try:
            voltage = decode_voltage(self.bus.read_i2c_block_data(self.address, REG_VOLTAGE, 2))
            current = decode_current(self.bus.read_i2c_block_data(self.address, REG_CURRENT, 2))
        except OSError as error:
            self.get_logger().warn(f'Cannot read the INA260: {error}', throttle_duration_sec=LOG_THROTTLE)
            return

        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = voltage
        msg.current = self.current_sign * current     # ROS: negative when discharging
        msg.charge = float('nan')
        msg.capacity = float('nan')
        msg.design_capacity = self.capacity
        msg.percentage = charge_ratio(voltage, self.empty, self.full)
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present = True
        self.state_pub.publish(msg)

        low = voltage < self.low
        self.low_pub.publish(Bool(data=low))
        if low:
            self.get_logger().warn(f'Battery low: {voltage:.2f} V (below {self.low} V).',
                                   throttle_duration_sec=LOG_THROTTLE)


def main():
    rclpy.init()
    node = BatteryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.bus.close()
        node.destroy_node()
        rclpy.try_shutdown()
