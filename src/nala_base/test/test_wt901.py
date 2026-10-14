import math
import struct

import pytest

from nala_base.wt901 import G, split_frames


def frame(kind, x, y, z, t=0):
    body = bytes([0x55, kind]) + struct.pack('<hhhh', x, y, z, t)
    return body + bytes([sum(body) & 0xFF])


def test_gyro_frame_in_rad_per_s():
    frames, rest = split_frames(frame(0x52, 0, 0, 16384))   # z = 1000 deg/s
    assert rest == b''
    assert frames[0][0] == 'gyro'
    assert frames[0][1] == pytest.approx([0.0, 0.0, math.radians(1000.0)])


def test_acc_frame_in_m_per_s2():
    frames, _ = split_frames(frame(0x51, 0, -2048, 2048))    # 2048 = 1 g
    assert frames == [('acc', pytest.approx([0.0, -G, G]))]


def test_resync_after_garbage_and_bad_checksum():
    bad = bytearray(frame(0x52, 1, 2, 3))
    bad[10] ^= 0xFF
    data = b'\x00\x55\x12' + bytes(bad) + frame(0x52, 0, 0, 100)
    frames, _ = split_frames(data)
    assert len(frames) == 1 and frames[0][0] == 'gyro'


def test_other_types_skipped_and_tail_kept():
    data = frame(0x53, 1, 2, 3) + frame(0x51, 0, 0, 2048) + frame(0x52, 0, 0, 0)[:6]
    frames, rest = split_frames(data)
    assert [f[0] for f in frames] == ['acc']
    assert rest == frame(0x52, 0, 0, 0)[:6]
