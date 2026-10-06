"""Mecanum chassis math (no ROS). Stage 2 adds forward kinematics for odometry."""

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
