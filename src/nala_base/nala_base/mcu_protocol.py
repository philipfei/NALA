"""Pi <-> MCU UART protocol, firmware v2.1.0: binary frames (no ROS). See docs/mcu_protocol.md.

Axes are the ROS axes: vx forward, vy left (m/s), wz counter-clockwise (rad/s).
Every frame: 0xA5 (sync), id, data, CRC-8 of id + data.
"""

import struct

SYNC = 0xA5
ID_CMD = 0x01         # Pi -> MCU: vx, vy, w (int16 LE)
ID_STATE = 0x02       # MCU -> Pi: seq, flags, 4 x int32 LE counts
STATE_LEN = 21        # sync, id, seq, flags, 16 count bytes, crc
SCALE = 1000          # mm/s and mrad/s per m/s and rad/s
INT16_MAX = 32767

FLAG_BOOT = 0x01      # no valid command since the MCU started: its counters start at 0
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
    """Return the 9-byte command frame: 0xA5, 0x01, vx, vy, w (int16 LE), crc."""
    body = struct.pack('<Bhhh', ID_CMD, to_int16(vx), to_int16(vy), to_int16(wz))
    return bytes([SYNC]) + body + bytes([crc8(body)])


def split_frames(buffer):
    """Cut a byte buffer from the MCU into state frames.

    Returns (states, rest, bad): states is a list of (seq, flags, [m1, m2, m3, m4]) with the
    cumulative encoder counts (+ = forward); rest is the unused tail (an incomplete frame);
    bad counts the places that looked like a frame start but had a wrong CRC.
    Other bytes are skipped one by one until the next valid frame.
    """
    states = []
    bad = 0
    i = 0
    while len(buffer) - i >= STATE_LEN:
        frame = buffer[i:i + STATE_LEN]
        if frame[0] != SYNC or frame[1] != ID_STATE:
            i += 1
            continue
        if crc8(frame[1:-1]) != frame[-1]:
            bad += 1
            i += 1
            continue
        seq, flags, *counts = struct.unpack('<BBiiii', frame[2:-1])
        states.append((seq, flags, counts))
        i += STATE_LEN
    return states, buffer[i:], bad


def count_delta(new, old):
    """Encoder counts from old to new. Also correct when the MCU int32 counter wraps around."""
    return (new - old + 2**31) % 2**32 - 2**31
