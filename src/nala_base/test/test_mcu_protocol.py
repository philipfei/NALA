import pytest

from nala_base.mcu_protocol import encode_command, parse_line


def test_stop_frame():
    assert encode_command(0.0, 0.0, 0.0) == bytes([0x80, 0x86, 0x00, 0x00, 0x00])


# Examples from firmware/docs/protocol.md
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


def test_parse_wheels():
    line = '0.000000 \t 2.451000 \t -2.451000 \t 1.000000'
    assert parse_line(line) == ('wheels', [0.0, 2.451, -2.451, 1.0])


def test_parse_echo():
    assert parse_line('0.100000 0.000000 -0.500000') == ('echo', [0.1, 0.0, -0.5])


@pytest.mark.parametrize('line', ['cmd timeout', 'test', 'a', 'echo on', '1.0 2.0', '1.0 x 2.0 3.0'])
def test_parse_text(line):
    assert parse_line(line) == ('text', line)


def test_parse_empty():
    assert parse_line('') is None
    assert parse_line(' \r') is None
