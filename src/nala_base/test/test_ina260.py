import pytest

from nala_base.ina260 import charge_ratio, decode_current, decode_voltage


def test_voltage_is_unsigned_big_endian():
    assert decode_voltage([0x34, 0x80]) == pytest.approx(16.8)      # 0x3480 = 13440 x 1.25 mV
    assert decode_voltage([0xFF, 0xFF]) == pytest.approx(81.91875)  # not negative


def test_current_is_signed_big_endian():
    assert decode_current([0x03, 0x20]) == pytest.approx(1.0)       # 0x0320 = 800 x 1.25 mA
    assert decode_current([0xFC, 0xE0]) == pytest.approx(-1.0)      # two's complement


def test_charge_ratio_linear_and_clamped():
    assert charge_ratio(16.8, 12.0, 16.8) == 1.0
    assert charge_ratio(12.0, 12.0, 16.8) == 0.0
    assert charge_ratio(14.4, 12.0, 16.8) == pytest.approx(0.5)
    assert charge_ratio(17.5, 12.0, 16.8) == 1.0
    assert charge_ratio(10.0, 12.0, 16.8) == 0.0
