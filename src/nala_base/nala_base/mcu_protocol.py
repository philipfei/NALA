"""Pi <-> MCU UART protocol, firmware v2.3.0: binary frames (no ROS). See docs/mcu_protocol.md.

Axes are the ROS axes: vx forward, vy left (m/s), wz counter-clockwise (rad/s).
Every frame: 0xA5 (sync), id, data, CRC-8 of id + data.
"""

import struct

SYNC = 0xA5
ID_CMD = 0x01         # Pi -> MCU: vx, vy, w (int16 LE)
ID_STATE = 0x02       # MCU -> Pi: seq, flags, 4 x int32 LE counts
ID_PARAM = 0x03       # Pi -> MCU: op, idx, value (float32 LE)            (setting mode, v2.2.0)
ID_PARAM_REPLY = 0x04 # MCU -> Pi: op, idx, status, value (float32 LE)    (setting mode, v2.2.0)
STATE_LEN = 21        # sync, id, seq, flags, 16 count bytes, crc
REPLY_LEN = 10        # sync, id, op, idx, status, 4 value bytes, crc
SCALE = 1000          # mm/s and mrad/s per m/s and rad/s
INT16_MAX = 32767

FLAG_BOOT = 0x01        # no valid command since the MCU started: its counters start at 0
FLAG_TIMEOUT = 0x02     # the MCU stopped because no valid command came for 200 ms
FLAG_EEPROM = 0x04      # the MCU parameters are the EEPROM block (loaded or saved, unchanged since)
FLAG_FF_INVALID = 0x08  # the MCU feed-forward table is invalid: it ignores non-zero drive commands

# Setting mode: parameter request ops and reply status (firmware main.c, same numbers).
OP_GET, OP_SET, OP_SAVE, OP_DEFAULTS, OP_RELOAD = range(5)
STATUS_OK = 0
STATUS_TEXT = {0: 'ok', 1: 'bad index', 2: 'out of range or not finite', 3: 'busy (the robot is not stopped)',
               4: 'bad feed-forward table', 5: 'bad op', 6: 'no valid EEPROM block'}

# Parameter index -> name. Same order as param[] in the firmware (main.c, config.h "Setting mode").
# v2.3.0: one feed-forward PWM row per motor (ff_m1_pwm0..9 .. ff_m4_pwm0..9), shared speed points.
FF_TABLE_N = 10
PARAM_NAMES = (['kp', 'ki', 'i_limit', 'accel_limit']
               + [f'ff_speed{i}' for i in range(FF_TABLE_N)]
               + [f'ff_m{m}_pwm{i}' for m in range(1, 5) for i in range(FF_TABLE_N)])


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


def encode_param(op, idx=0, value=0.0):
    """Return the 9-byte parameter request: 0xA5, 0x03, op, idx, value (float32 LE), crc.

    Same length as a command frame (6 data bytes + crc), so the MCU parses both alike.
    """
    body = struct.pack('<BBBf', ID_PARAM, op, idx, value)
    return bytes([SYNC]) + body + bytes([crc8(body)])


FRAME_LEN = {ID_STATE: STATE_LEN, ID_PARAM_REPLY: REPLY_LEN}


def split_frames(buffer):
    """Cut a byte buffer from the MCU into state frames and parameter replies.

    Returns (states, replies, rest, bad): states is a list of (seq, flags, [m1, m2, m3, m4]) with
    the cumulative encoder counts (+ = forward); replies is a list of (op, idx, status, value);
    rest is the unused tail (an incomplete frame); bad counts the places that looked like a
    frame start but had a wrong CRC. Other bytes are skipped one by one until the next valid frame.
    """
    states = []
    replies = []
    bad = 0
    i = 0
    while len(buffer) - i >= 2:
        length = FRAME_LEN.get(buffer[i + 1]) if buffer[i] == SYNC else None
        if length is None:
            i += 1
            continue
        if len(buffer) - i < length:
            break                       # incomplete: wait for the rest
        frame = buffer[i:i + length]
        if crc8(frame[1:-1]) != frame[-1]:
            bad += 1
            i += 1
            continue
        if frame[1] == ID_STATE:
            seq, flags, *counts = struct.unpack('<BBiiii', frame[2:-1])
            states.append((seq, flags, counts))
        else:
            replies.append(struct.unpack('<BBBf', frame[2:-1]))
        i += length
    return states, replies, buffer[i:], bad


def count_delta(new, old):
    """Encoder counts from old to new. Also correct when the MCU int32 counter wraps around."""
    return (new - old + 2**31) % 2**32 - 2**31
