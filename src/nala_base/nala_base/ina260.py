"""TI INA260 power monitor (I2C) register decoding (no ROS).

Every register is 2 bytes, big endian (high byte first):
  0x01 current: signed int16, 1.25 mA per bit
  0x02 bus voltage: unsigned uint16, 1.25 mV per bit
"""

import struct

REG_CURRENT = 0x01
REG_VOLTAGE = 0x02
CURRENT_LSB = 0.00125   # A per bit
VOLTAGE_LSB = 0.00125   # V per bit


def decode_current(data):
    """2 register bytes -> current in A (signed)."""
    return struct.unpack('>h', bytes(data))[0] * CURRENT_LSB


def decode_voltage(data):
    """2 register bytes -> bus voltage in V."""
    return struct.unpack('>H', bytes(data))[0] * VOLTAGE_LSB


def charge_ratio(voltage, empty_voltage, full_voltage):
    """Battery charge 0.0 .. 1.0 from the voltage, linear between empty and full, clamped."""
    ratio = (voltage - empty_voltage) / (full_voltage - empty_voltage)
    return max(0.0, min(1.0, ratio))
