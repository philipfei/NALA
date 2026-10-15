import math

import pytest

from nala_base.step_metrics import body_to_wheels, step_metrics, wheel_speeds


def test_body_to_wheels_matches_the_firmware_signs():
    r, k = 0.04, 0.29
    assert body_to_wheels(0.04, 0, 0, r, k) == pytest.approx([1, 1, 1, 1])
    assert body_to_wheels(0, 0.04, 0, r, k) == pytest.approx([-1, 1, -1, 1])      # strafe left
    assert body_to_wheels(0, 0, 0.04 / k, r, k) == pytest.approx([-1, -1, 1, 1])  # turn counter-clockwise


def test_wheel_speeds_use_the_mcu_periods():
    rad = 2 * math.pi / 1536
    frames = [(1.00, 10, 0, [0, 0, 0, 0]), (1.05, 11, 0, [10, -10, 0, 5]), (1.20, 14, 0, [40, -10, 0, 5])]
    samples = wheel_speeds(frames, rad, 0.05)
    assert samples[0][0] == pytest.approx(1.025)
    assert samples[0][1] == pytest.approx([10 * rad / 0.05, -10 * rad / 0.05, 0, 5 * rad / 0.05])
    assert samples[1][0] == pytest.approx(1.125)                  # 3 periods: middle of 0.15 s
    assert samples[1][1][0] == pytest.approx(30 * rad / 0.15)


def test_wheel_speeds_seq_wrap_and_duplicate():
    frames = [(0.0, 255, 0, [0] * 4), (0.05, 0, 0, [1] * 4), (0.06, 0, 0, [1] * 4)]
    samples = wheel_speeds(frames, 1.0, 0.05)
    assert len(samples) == 1 and samples[0][1] == pytest.approx([20.0] * 4)


def first_order(target, delay, tau, overshoot=0.0, t_end=2.0, dt=0.05):
    times, speeds = [], []
    t = 0.0
    while t <= t_end + 1e-9:
        if t < delay:
            y = 0.0
        else:
            x = t - delay
            y = 1 - math.exp(-x / tau) + overshoot * math.exp(-x / tau) * math.sin(x / tau)
        times.append(t)
        speeds.append(target * y)
        t += dt
    return times, speeds


def test_clean_first_order_step():
    times, speeds = first_order(4.0, delay=0.1, tau=0.1)
    m = step_metrics(times, speeds, 4.0, 0.0, 2.0)
    assert m['dead'] == pytest.approx(0.15)        # first sample >= 10 %
    assert 0.15 <= m['rise'] <= 0.3
    assert m['overshoot'] == 0.0
    assert m['settle'] == pytest.approx(0.35, abs=0.06)
    assert m['mean'] == pytest.approx(4.0, rel=1e-3)
    assert m['std'] < 1e-3


def test_overshoot_on_a_negative_target():
    times, speeds = first_order(-3.0, delay=0.0, tau=0.1, overshoot=1.5)
    m = step_metrics(times, speeds, -3.0, 0.0, 2.0)
    assert m['overshoot'] > 10
    assert m['mean'] == pytest.approx(-3.0, rel=1e-2)


def test_never_reaches_the_target():
    times = [i * 0.05 for i in range(41)]
    m = step_metrics(times, [0.0] * 41, 2.0, 0.0, 2.0)
    assert m['dead'] is None and m['rise'] is None and m['settle'] is None
    assert m['overshoot'] == 0.0 and m['mean'] == 0.0


def test_still_outside_the_band_at_the_end_is_not_settled():
    times = [i * 0.05 for i in range(41)]
    speeds = [2.0] * 40 + [3.0]
    assert step_metrics(times, speeds, 2.0, 0.0, 2.0)['settle'] is None
