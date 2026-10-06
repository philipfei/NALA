import math

import pytest

from nala_base.kinematics import limit_twist


def test_under_limit_unchanged():
    assert limit_twist(0.5, -0.3, 0.2, 2.0, 1.0) == (0.5, -0.3, 0.2)


def test_linear_limit_keeps_direction():
    vx, vy, wz = limit_twist(3.0, 4.0, 0.0, 2.0, 1.0)   # speed 5.0 -> 2.0
    assert math.hypot(vx, vy) == pytest.approx(2.0)
    assert (vx, vy) == pytest.approx((1.2, 1.6))
    assert wz == 0.0


def test_angular_clamp():
    assert limit_twist(0.0, 0.0, 1.5, 2.0, 1.0)[2] == 1.0
    assert limit_twist(0.0, 0.0, -1.5, 2.0, 1.0)[2] == -1.0
