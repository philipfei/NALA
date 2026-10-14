import struct

from nala_base.mcu_protocol import (
    count_delta, crc8, encode_command, FLAG_BOOT, split_frames, STATE_LEN)


def test_crc8_standard_vector():
    assert crc8(b'123456789') == 0xF4   # CRC-8 (poly 0x07, init 0); also in firmware/tests


def test_stop_command():
    frame = encode_command(0.0, 0.0, 0.0)
    assert frame[:8] == bytes([0xA5, 0x01, 0, 0, 0, 0, 0, 0])
    assert frame[8] == crc8(frame[1:8])


def test_command_units_and_signs():
    # 0.3 m/s forward = 300 mm/s = 0x012C, 0.1 m/s right = -100 = 0xFF9C, 1.0 rad/s = 1000 = 0x03E8
    assert encode_command(0.3, -0.1, 1.0)[2:8] == bytes([0x2C, 0x01, 0x9C, 0xFF, 0xE8, 0x03])


def test_command_rounding_and_clamp():
    vx, vy, wz = struct.unpack('<hhh', encode_command(0.0004, 0.0006, 40.0)[2:8])
    assert (vx, vy, wz) == (0, 1, 32767)


def state_frame(seq, flags, counts):
    body = struct.pack('<BBBiiii', 0x02, seq, flags, *counts)
    return bytes([0xA5]) + body + bytes([crc8(body)])


def test_one_state_frame():
    frame = state_frame(200, FLAG_BOOT, [1536, -20, 2**31 - 1, -2**31])
    assert len(frame) == STATE_LEN
    assert split_frames(frame) == ([(200, FLAG_BOOT, [1536, -20, 2**31 - 1, -2**31])], b'', 0)


def test_frames_with_garbage_and_incomplete_tail():
    tail = state_frame(3, 0, [9, 9, 9, 9])[:10]
    data = b'\x00test\n' + state_frame(1, 0, [1, 2, 3, 4]) + b'\xa5' + state_frame(2, 0, [5, 6, 7, 8]) + tail
    states, rest, bad = split_frames(data)
    assert states == [(1, 0, [1, 2, 3, 4]), (2, 0, [5, 6, 7, 8])]
    assert rest == tail
    assert bad == 0


def test_sync_byte_inside_the_counts_is_data():
    frame = state_frame(5, 0, [0x02A5, 0xA5, -1, 0x02A502A5])
    assert split_frames(frame + frame)[0] == [(5, 0, [0x02A5, 0xA5, -1, 0x02A502A5])] * 2


def test_bad_crc_is_counted_and_skipped():
    broken = bytearray(state_frame(1, 0, [1, 2, 3, 4]))
    broken[6] ^= 0x10
    states, rest, bad = split_frames(bytes(broken) + state_frame(2, 0, [5, 6, 7, 8]))
    assert states == [(2, 0, [5, 6, 7, 8])]
    assert bad == 1


def test_count_delta():
    assert count_delta(1536, 0) == 1536
    assert count_delta(-100, 50) == -150
    assert count_delta(-2147483648, 2147483647) == 1     # int32 wrap forward
    assert count_delta(2147483647, -2147483648) == -1    # int32 wrap backward
