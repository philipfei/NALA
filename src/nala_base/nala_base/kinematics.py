"""Mecanum chassis math (no ROS): wheel speed limit and odometry."""

import math


def limit_wheel_speed(vx, vy, wz, k, max_wheel_speed):
    """Limit a body speed command so that no wheel is faster than max_wheel_speed.

    The wheel rim speeds of a mecanum base are vx +- vy +- k*wz (all 4 sign pairs),
    so the fastest wheel is |vx| + |vy| + k*|wz|. If that is too fast, vx, vy and wz are
    scaled down by the same factor: the path keeps its shape, only slower.
    k: half track width + half wheelbase (m). max_wheel_speed: wheel rim speed (m/s).
    """
    peak = abs(vx) + abs(vy) + k * abs(wz)
    if peak > max_wheel_speed:
        scale = max_wheel_speed / peak
        vx *= scale
        vy *= scale
        wz *= scale
    return vx, vy, wz


def wheels_to_body(m1, m2, m3, m4, radius, k):
    """Measured wheel speeds (rad/s) -> body speed (vx, vy, wz).

    M1 front-left, M2 rear-left, M3 rear-right, M4 front-right. Positive = the wheel pushes forward.
    radius: wheel radius (m). k: half track width + half wheelbase (m).
    This is the inverse of the firmware kinematics (docs/mcu_protocol.md).
    """
    vx = radius / 4 * (m1 + m2 + m3 + m4)
    vy = radius / 4 * (-m1 + m2 - m3 + m4)
    wz = radius / (4 * k) * (-m1 - m2 + m3 + m4)
    return vx, vy, wz


def integrate_pose(x, y, yaw, vx, vy, wz, dt):
    """Move the pose (x, y, yaw) by the body speed (vx, vy, wz) for dt seconds.

    Uses the heading in the middle of the step, which is exact enough for small steps.
    """
    mid = yaw + wz * dt / 2
    x += (vx * math.cos(mid) - vy * math.sin(mid)) * dt
    y += (vx * math.sin(mid) + vy * math.cos(mid)) * dt
    yaw = math.atan2(math.sin(yaw + wz * dt), math.cos(yaw + wz * dt))
    return x, y, yaw
