import struct

from nala_base.mcu_protocol import (
    count_delta, crc8, encode_command, FLAG_BOOT, parse_state, STATE_LEN)


def test_crc8_standard_vector():
    assert crc8(b'123456789') == 0xF4   # CRC-8 (poly 0x07, init 0); also in firmware/tests


def test_stop_command():
    frame = encode_command(0.0, 0.0, 0.0)
    assert frame[:7] == bytes([0x01, 0, 0, 0, 0, 0, 0])
    assert frame[7] == crc8(frame[:7])


def test_command_units_and_signs():
    # 0.3 m/s forward = 300 mm/s = 0x012C, 0.1 m/s right = -100 = 0xFF9C, 1.0 rad/s = 1000 = 0x03E8
    assert encode_command(0.3, -0.1, 1.0)[1:7] == bytes([0x2C, 0x01, 0x9C, 0xFF, 0xE8, 0x03])


def test_command_rounding_and_clamp():
    _, vx, vy, wz = struct.unpack('<Bhhh', encode_command(0.0004, 0.0006, 40.0)[:7])
    assert (vx, vy, wz) == (0, 1, 32767)


def state_frame(seq, flags, counts):
    body = struct.pack('<BBiiii', seq, flags, *counts)
    return body + bytes([crc8(body)])


def test_parse_state():
    frame = state_frame(200, FLAG_BOOT, [1536, -20, 2**31 - 1, -2**31])
    assert len(frame) == STATE_LEN
    assert parse_state(frame) == (200, FLAG_BOOT, [1536, -20, 2**31 - 1, -2**31])


def test_parse_state_rejects_bad_crc_and_length():
    frame = bytearray(state_frame(1, 0, [0, 0, 0, 0]))
    frame[5] ^= 0x10
    assert parse_state(frame) is None
    assert parse_state(state_frame(1, 0, [0, 0, 0, 0])[:-1]) is None


def test_count_delta():
    assert count_delta(1536, 0) == 1536
    assert count_delta(-100, 50) == -150
    assert count_delta(-2147483648, 2147483647) == 1     # int32 wrap forward
    assert count_delta(2147483647, -2147483648) == -1    # int32 wrap backward
