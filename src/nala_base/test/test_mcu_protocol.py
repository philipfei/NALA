import struct

import math

from nala_base.mcu_protocol import (
    count_delta, crc8, encode_command, encode_param, FLAG_BOOT, OP_GET, OP_SAVE, OP_SET, PARAM_NAMES,
    REPLY_LEN, split_frames, STATE_LEN)


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
    assert split_frames(frame) == ([(200, FLAG_BOOT, [1536, -20, 2**31 - 1, -2**31])], [], b'', 0)


def test_frames_with_garbage_and_incomplete_tail():
    tail = state_frame(3, 0, [9, 9, 9, 9])[:10]
    data = b'\x00test\n' + state_frame(1, 0, [1, 2, 3, 4]) + b'\xa5' + state_frame(2, 0, [5, 6, 7, 8]) + tail
    states, _, rest, bad = split_frames(data)
    assert states == [(1, 0, [1, 2, 3, 4]), (2, 0, [5, 6, 7, 8])]
    assert rest == tail
    assert bad == 0


def test_sync_byte_inside_the_counts_is_data():
    frame = state_frame(5, 0, [0x02A5, 0xA5, -1, 0x02A502A5])
    assert split_frames(frame + frame)[0] == [(5, 0, [0x02A5, 0xA5, -1, 0x02A502A5])] * 2


def test_bad_crc_is_counted_and_skipped():
    broken = bytearray(state_frame(1, 0, [1, 2, 3, 4]))
    broken[6] ^= 0x10
    states, _, rest, bad = split_frames(bytes(broken) + state_frame(2, 0, [5, 6, 7, 8]))
    assert states == [(2, 0, [5, 6, 7, 8])]
    assert bad == 1


def test_count_delta():
    assert count_delta(1536, 0) == 1536
    assert count_delta(-100, 50) == -150
    assert count_delta(-2147483648, 2147483647) == 1     # int32 wrap forward
    assert count_delta(2147483647, -2147483648) == -1    # int32 wrap backward


def reply_frame(op, idx, status, value):          # like send_param_reply() in the firmware
    body = struct.pack('<BBBBf', 0x04, op, idx, status, value)
    return bytes([0xA5]) + body + bytes([crc8(body)])


def test_param_request_has_the_command_length():
    frame = encode_param(OP_SET, 1, 8.0)
    assert len(frame) == len(encode_command(0.0, 0.0, 0.0)) == 9
    assert frame[:4] == bytes([0xA5, 0x03, OP_SET, 1])
    assert struct.unpack('<f', frame[4:8])[0] == 8.0
    assert frame[8] == crc8(frame[1:8])


def test_param_names_match_the_firmware_order():
    assert len(PARAM_NAMES) == 24
    assert PARAM_NAMES[:4] == ['kp', 'ki', 'i_limit', 'accel_limit']
    assert PARAM_NAMES[4] == 'ff_speed0' and PARAM_NAMES[14] == 'ff_pwm0' and PARAM_NAMES[23] == 'ff_pwm9'


def test_replies_and_states_mixed():
    reply = reply_frame(OP_GET, 0, 0, 1.5)
    assert len(reply) == REPLY_LEN
    data = state_frame(1, 0, [1, 2, 3, 4]) + reply + b'\x00' + state_frame(2, 0, [5, 6, 7, 8])
    states, replies, rest, bad = split_frames(data)
    assert [s[0] for s in states] == [1, 2]
    assert replies == [(OP_GET, 0, 0, 1.5)]
    assert (rest, bad) == (b'', 0)


def test_reply_alone_is_not_held_back():
    # During an EEPROM save no state frames come: a reply at the end of the buffer must be returned now.
    states, replies, rest, bad = split_frames(reply_frame(OP_SAVE, 0, 0, 0.0))
    assert (states, replies, rest, bad) == ([], [(OP_SAVE, 0, 0, 0.0)], b'', 0)


def test_incomplete_reply_waits():
    reply = reply_frame(OP_GET, 3, 0, 2.0)
    states, replies, rest, bad = split_frames(reply[:6])
    assert (states, replies, rest, bad) == ([], [], reply[:6], 0)
    assert split_frames(rest + reply[6:])[1] == [(OP_GET, 3, 0, 2.0)]


def test_nan_is_encoded_as_nan():
    # The MCU must reject it (status 2); the Pi does not filter it.
    assert math.isnan(struct.unpack('<f', encode_param(OP_SET, 0, float('nan'))[4:8])[0])
