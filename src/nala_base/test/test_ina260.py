import pytest

from nala_base.ina260 import charge_ratio, decode_voltage


def test_voltage_is_unsigned_big_endian_in_4_mv_after_dropping_3_flag_bits():
    assert decode_voltage([0x83, 0x40]) == pytest.approx(16.8)      # 0x8340 >> 3 = 4200 x 4 mV
    assert decode_voltage([0xFF, 0xF8]) == pytest.approx(32.764)    # full scale, not negative


def test_voltage_ignores_the_cnvr_and_ovf_flags():
    assert decode_voltage([0x83, 0x43]) == decode_voltage([0x83, 0x40])


def test_charge_ratio_linear_and_clamped():
    assert charge_ratio(16.8, 12.0, 16.8) == 1.0
    assert charge_ratio(12.0, 12.0, 16.8) == 0.0
    assert charge_ratio(14.4, 12.0, 16.8) == pytest.approx(0.5)
    assert charge_ratio(17.5, 12.0, 16.8) == 1.0
    assert charge_ratio(10.0, 12.0, 16.8) == 0.0
