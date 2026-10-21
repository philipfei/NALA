"""TI INA219 power monitor (I2C) register decoding (no ROS).

The file keeps its old name ina260.py: the first monitor was an INA260 (it never answered on the bus),
the chip on the robot is an INA219. Only the bus voltage is read: IN+ and IN- are shorted, so the
chip measures the battery voltage and no current.

The bus voltage register (0x02) is 2 bytes, big endian (high byte first):
  bits 15..3  voltage, 4 mV per bit (unsigned)
  bit 1       CNVR, a conversion is ready (not used)
  bit 0       OVF, math overflow of the current / power registers (not used: they are not read)
The chip starts with the 32 V range, enough for the 16.8 V battery.
"""

import struct

REG_VOLTAGE = 0x02
VOLTAGE_LSB = 0.004   # V per bit, after dropping the 3 low flag bits


def decode_voltage(data):
    """2 register bytes -> bus voltage in V."""
    return (struct.unpack('>H', bytes(data))[0] >> 3) * VOLTAGE_LSB


def charge_ratio(voltage, empty_voltage, full_voltage):
    """Battery charge 0.0 .. 1.0 from the voltage, linear between empty and full, clamped."""
    ratio = (voltage - empty_voltage) / (full_voltage - empty_voltage)
    return max(0.0, min(1.0, ratio))
