"""Pi <-> MCU I2C protocol, firmware v2.0.0 (no ROS). See docs/mcu_protocol.md.

Axes are the ROS axes: vx forward, vy left (m/s), wz counter-clockwise (rad/s).
"""

import struct

CMD_ID = 0x01
CMD_LEN = 8           # id, vx, vy, w (int16 LE), crc
STATE_LEN = 19        # seq, flags, 4 x int32 LE counts, crc
SCALE = 1000          # mm/s and mrad/s per m/s and rad/s
INT16_MAX = 32767

FLAG_BOOT = 0x01      # first frame after the MCU started: its counters start at 0
FLAG_TIMEOUT = 0x02   # the MCU stopped because no valid command came for 200 ms


def crc8(data):
    """CRC-8, polynomial 0x07, initial value 0 (same as crc8() in the firmware)."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def to_int16(value):
    """Speed -> int16 in mm/s or mrad/s, rounded and clamped."""
    return max(-INT16_MAX, min(INT16_MAX, round(value * SCALE)))


def encode_command(vx, vy, wz):
    """Return the 8-byte command: 0x01, vx, vy, w (int16 LE), crc."""
    body = struct.pack('<Bhhh', CMD_ID, to_int16(vx), to_int16(vy), to_int16(wz))
    return body + bytes([crc8(body)])


def parse_state(data):
    """Parse the 19-byte state frame.

    Returns (seq, flags, [m1, m2, m3, m4]) with the cumulative encoder counts
    (+ = forward), or None if the length or the CRC is wrong.
    """
    data = bytes(data)
    if len(data) != STATE_LEN or crc8(data[:-1]) != data[-1]:
        return None
    seq, flags, *counts = struct.unpack('<BBiiii', data[:-1])
    return seq, flags, counts


def count_delta(new, old):
    """Encoder counts from old to new. Also correct when the MCU int32 counter wraps around."""
    return (new - old + 2**31) % 2**32 - 2**31
