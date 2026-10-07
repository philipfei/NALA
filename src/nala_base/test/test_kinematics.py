import math

import pytest

from nala_base.kinematics import integrate_pose, limit_twist, wheels_to_body


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


R = 0.040
K = 0.290


def firmware_wheels(vx, vy, wz):
    """Firmware inverse kinematics (docs/mcu_protocol.md), M1..M4 in rad/s."""
    return ((vx - vy - K * wz) / R,
            (vx + vy - K * wz) / R,
            (vx - vy + K * wz) / R,
            (vx + vy + K * wz) / R)


@pytest.mark.parametrize('body', [(0.5, 0.0, 0.0), (0.0, 0.3, 0.0), (0.0, 0.0, 1.0), (0.4, -0.2, -0.7)])
def test_wheels_to_body_inverts_firmware(body):
    assert wheels_to_body(*firmware_wheels(*body), R, K) == pytest.approx(body)


def test_all_wheels_forward_is_forward():
    vx, vy, wz = wheels_to_body(2.5, 2.5, 2.5, 2.5, R, K)
    assert (vx, vy, wz) == pytest.approx((0.1, 0.0, 0.0))


def test_straight_line_in_rotated_frame():
    # Facing +y (yaw 90 deg), forward 1 m/s for 2 s -> moves to (0, 2).
    x, y, yaw = integrate_pose(0.0, 0.0, math.pi / 2, 1.0, 0.0, 0.0, 2.0)
    assert (x, y, yaw) == pytest.approx((0.0, 2.0, math.pi / 2))


def test_strafe_left():
    assert integrate_pose(0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 2.0) == pytest.approx((0.0, 1.0, 0.0))


def test_pure_turn_wraps_yaw():
    x, y, yaw = integrate_pose(1.0, 2.0, 3.0, 0.0, 0.0, 1.0, 0.5)
    assert (x, y) == pytest.approx((1.0, 2.0))
    assert yaw == pytest.approx(3.5 - 2 * math.pi)


def test_circle_returns_to_start():
    # Forward 1 m/s while turning 1 rad/s: a full circle of radius 1 m in 2*pi s.
    x, y, yaw = 0.0, 0.0, 0.0
    steps = 1000
    dt = 2 * math.pi / steps
    for _ in range(steps):
        x, y, yaw = integrate_pose(x, y, yaw, 1.0, 0.0, 1.0, dt)
    assert (x, y) == pytest.approx((0.0, 0.0), abs=1e-6)
    assert math.cos(yaw) == pytest.approx(1.0)
