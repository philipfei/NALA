import math

import pytest

from nala_base.kinematics import integrate_pose, limit_wheel_speed, wheels_to_body


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


def fastest_rim_speed(vx, vy, wz):
    return max(abs(m) for m in firmware_wheels(vx, vy, wz)) * R


def test_under_wheel_limit_unchanged():
    assert limit_wheel_speed(0.5, -0.3, 0.2, K, 1.0) == (0.5, -0.3, 0.2)
    assert limit_wheel_speed(1.0, 0.0, 0.0, K, 1.0) == (1.0, 0.0, 0.0)   # exactly at the limit


def test_strafe_and_turn_scaled_together():
    # Full strafe + full turn needs 1.29 m/s on M1 and M4.
    assert fastest_rim_speed(0.0, 1.0, 1.0) == pytest.approx(1.29)
    vx, vy, wz = limit_wheel_speed(0.0, 1.0, 1.0, K, 1.0)
    assert fastest_rim_speed(vx, vy, wz) == pytest.approx(1.0)
    assert wz / vy == pytest.approx(1.0)   # same ratio: the turn is kept
    assert vx == 0.0


@pytest.mark.parametrize('body', [(-2.0, 0.0, 0.0), (0.0, 0.0, -5.0), (0.7, -0.6, 0.9)])
def test_limit_is_the_fastest_wheel(body):
    limited = limit_wheel_speed(*body, K, 1.0)
    assert fastest_rim_speed(*limited) == pytest.approx(1.0)
    assert limited[0] * body[1] == pytest.approx(limited[1] * body[0])   # direction kept


def test_one_wheel_turn_is_one_circumference():
    # Wheel angle changes (rad) give the body movement: one turn of all 4 wheels = 2*pi*R forward.
    turn = 2 * math.pi
    assert wheels_to_body(turn, turn, turn, turn, R, K) == pytest.approx((2 * math.pi * R, 0.0, 0.0))


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
