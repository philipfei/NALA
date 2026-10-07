"""Mecanum chassis math (no ROS): speed limits and odometry."""

import math


def limit_twist(vx, vy, wz, max_linear, max_angular):
    """Limit a body speed command.

    The (vx, vy) vector is scaled down to max_linear, so the driving direction stays the same.
    wz is clamped to +-max_angular.
    """
    speed = math.hypot(vx, vy)
    if speed > max_linear:
        scale = max_linear / speed
        vx *= scale
        vy *= scale
    wz = max(-max_angular, min(max_angular, wz))
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
