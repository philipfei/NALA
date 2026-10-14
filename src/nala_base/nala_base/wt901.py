"""WitMotion WT901 (JY901) serial protocol (no ROS). See docs/imu_ekf.md.

Every data frame is 11 bytes: 0x55, type, 4 x int16 little endian, checksum.
checksum = sum of the first 10 bytes & 0xFF.
  0x51 acceleration: x, y, z / 32768 * 16 g
  0x52 angular velocity: x, y, z / 32768 * 2000 deg/s
Other frame types are skipped.
"""

import math
import struct

HEADER = 0x55
FRAME_LEN = 11
ACC = 0x51
GYRO = 0x52
G = 9.80665   # m/s^2 per g


def checksum_ok(frame):
    return sum(frame[:10]) & 0xFF == frame[10]


def split_frames(buffer):
    """Cut a byte buffer into frames.

    Returns (frames, rest): frames is a list of ('acc', [ax, ay, az]) in m/s^2 and
    ('gyro', [wx, wy, wz]) in rad/s; rest is the unused tail (an incomplete frame).
    Bytes before a header and frames with a wrong checksum are dropped.
    """
    frames = []
    i = 0
    while len(buffer) - i >= FRAME_LEN:
        if buffer[i] != HEADER or not checksum_ok(buffer[i:i + FRAME_LEN]):
            i += 1                  # not at a frame start: resync byte by byte
            continue
        kind = buffer[i + 1]
        x, y, z, _ = struct.unpack('<hhhh', buffer[i + 2:i + 10])
        if kind == ACC:
            frames.append(('acc', [v / 32768 * 16 * G for v in (x, y, z)]))
        elif kind == GYRO:
            frames.append(('gyro', [math.radians(v / 32768 * 2000) for v in (x, y, z)]))
        i += FRAME_LEN
    return frames, buffer[i:]
