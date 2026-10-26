import pytest

from nala_base.scintillator import parse_line, split_lines

# Real lines from the robot's scintillator (2026-10-26).
REAL = (b'0\t\t21-10-2026 13:57:39\t0.000\tcps\t100\t%\t4\tcps\t47\t%\r\n'
        b'0\t\t21-10-2026 13:57:40\t0.000\tcps\t100\t%\t10\tcps\t32\t%\r\n'
        b'0\t\t21-10-2026 13:57:41\t0.000\tcps\t100\t%\t7\tcps\t39\t%\r\n')


def test_real_lines_give_the_second_cps_value():
    lines, rest = split_lines(REAL)
    assert rest == b''
    assert [parse_line(line) for line in lines] == [4.0, 10.0, 7.0]


def test_decimal_value():
    assert parse_line(b'0\t\tx\t0.000\tcps\t100\t%\t12.5\tcps\t20\t%') == pytest.approx(12.5)


def test_line_with_one_cps_value_is_skipped():
    assert parse_line(b'0\t\t21-10-2026 13:57:39\t0.000\tcps\t100\t%') is None


def test_garbage_and_not_a_number_are_skipped():
    assert parse_line(b'') is None
    assert parse_line(b'\xff\x00garbage') is None
    assert parse_line(b'0\t1.0\tcps\tx\tcps') is None


def test_incomplete_line_kept_as_rest():
    lines, rest = split_lines(REAL + b'0\t\t21-10')
    assert len(lines) == 3
    assert rest == b'0\t\t21-10'
