import pytest

from nala_base.mcu_protocol import count_delta, encode_command, parse_line


def test_stop_frame():
    assert encode_command(0.0, 0.0, 0.0) == bytes([0x80, 0x86, 0x00, 0x00, 0x00])


# Examples from firmware/README.md
@pytest.mark.parametrize('vx, vy, wz, frame', [
    (0.10, 0.0, 0.0, [0x80, 0x86, 0x05, 0x00, 0x00]),    # forward 0.10 m/s
    (-0.10, 0.0, 0.0, [0x80, 0x86, 0xFB, 0x00, 0x00]),   # backward 0.10 m/s
    (0.0, 0.10, 0.0, [0x80, 0x86, 0x00, 0x05, 0x00]),    # left 0.10 m/s
    (0.0, 0.0, 0.50, [0x80, 0x86, 0x00, 0x00, 0x19]),    # counter-clockwise 0.50 rad/s
])
def test_protocol_examples(vx, vy, wz, frame):
    assert encode_command(vx, vy, wz) == bytes(frame)


def test_rounding():
    # 0.029 / 0.02 = 1.45 -> 1, 0.031 / 0.02 = 1.55 -> 2
    assert encode_command(0.029, 0.031, -0.031)[2:] == bytes([1, 2, 0xFE])


def test_clamp_to_int8():
    # 3.0 m/s would be 150 counts -> 127; -3.0 -> -127 (0x81)
    assert encode_command(3.0, -3.0, 0.0)[2:] == bytes([0x7F, 0x81, 0x00])


def test_parse_counts():
    assert parse_line('c 0 0 0 0') == ('counts', [0, 0, 0, 0])
    assert parse_line('c 1536 -20 2147483647 -2147483648\r') == ('counts', [1536, -20, 2147483647, -2147483648])


def test_count_delta():
    assert count_delta(1536, 0) == 1536
    assert count_delta(-100, 50) == -150
    assert count_delta(-2147483648, 2147483647) == 1     # int32 wrap forward
    assert count_delta(2147483647, -2147483648) == -1    # int32 wrap backward


def test_parse_echo():
    assert parse_line('0.100000 0.000000 -0.500000') == ('echo', [0.1, 0.0, -0.5])


@pytest.mark.parametrize('line', ['cmd timeout', 'test', 'a', 'echo on', '1.0 2.0', '1.0 x 2.0 3.0',
                                  '0.0 1.0 2.0 3.0', 'c 1 2 3', 'c 1 2 x 4'])
def test_parse_text(line):
    assert parse_line(line) == ('text', line)


def test_parse_empty():
    assert parse_line('') is None
    assert parse_line(' \r') is None
