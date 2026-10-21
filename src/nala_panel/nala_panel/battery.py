"""Battery voltage for the panel (no Qt, no ROS): the panel reads the INA219 itself, so the value is there on every
screen, also when no app (and so no battery_node) runs. A moving average keeps the shown value from jumping.
"""
from collections import deque

from nala_base.ina260 import REG_VOLTAGE, charge_ratio, decode_voltage


class BatteryMeter:
    """Reads the INA219 bus voltage and averages the last `samples` readings."""

    def __init__(self, bus, address, samples):
        self.bus_number, self.address = bus, address
        self.values = deque(maxlen=samples)
        self.bus = None

    def read(self):
        """One reading. No smbus2 (PC) or no answer: the old readings are dropped, so the screen shows '--'."""
        try:
            if self.bus is None:
                from smbus2 import SMBus           # not installed on the PC
                self.bus = SMBus(self.bus_number)
            self.values.append(decode_voltage(self.bus.read_i2c_block_data(self.address, REG_VOLTAGE, 2)))
        except (ImportError, OSError):
            self.values.clear()

    def voltage(self):
        """Mean voltage in V, or None without readings."""
        return sum(self.values) / len(self.values) if self.values else None


def battery_text(voltage, empty_voltage, full_voltage):
    """'16.4 V  92%' (0.1 V, 1 %), or '-- V  --%' without a value."""
    if voltage is None:
        return '-- V  --%'
    return f'{voltage:.1f} V  {charge_ratio(voltage, empty_voltage, full_voltage) * 100:.0f}%'
