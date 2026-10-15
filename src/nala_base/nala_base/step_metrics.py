"""Step-response metrics of the wheel speeds (no ROS). Used by scripts/tune_motors.py."""

import math

BAND = 0.10          # settled = within +-10 % of the target


def body_to_wheels(vx, vy, wz, radius, k):
    """Firmware inverse kinematics: target wheel speeds M1..M4 in rad/s (k = half track + half wheelbase)."""
    return [(vx - vy - k * wz) / radius, (vx + vy - k * wz) / radius,
            (vx - vy + k * wz) / radius, (vx + vy + k * wz) / radius]


def wheel_speeds(frames, rad_per_count, period):
    """State frames (t, seq, flags, counts) -> samples (t_mid, [M1..M4 rad/s]).

    The speed of a frame is the count change since the previous frame over the exact MCU time
    (seq difference x period). Its time is the middle of that interval (t = arrival of the frame).
    """
    samples = []
    for (_, seq0, _, c0), (t1, seq1, _, c1) in zip(frames, frames[1:]):
        periods = (seq1 - seq0) % 256
        if periods == 0:
            continue
        dt = periods * period
        speeds = [((n - o + 2**31) % 2**32 - 2**31) * rad_per_count / dt for n, o in zip(c1, c0)]
        samples.append((t1 - dt / 2, speeds))
    return samples


def step_metrics(times, speeds, target, t_step, t_end):
    """Metrics of one wheel's response to a step to `target` (rad/s, not 0) from t_step to t_end (s).

    Returns a dict (None where a value was not reached):
      dead      s from the step until the speed first reaches 10 % of the target
      rise      s from 10 % to 90 % of the target
      overshoot % of the target above the target (0 if it never goes above)
      settle    s from the step until the speed stays within +-10 % of the target
      mean, std rad/s over the last 40 % of the step (steady state)
    """
    pts = [(t, v / target) for t, v in zip(times, speeds) if t_step <= t <= t_end]
    result = {'dead': None, 'rise': None, 'overshoot': None, 'settle': None, 'mean': None, 'std': None}
    if not pts:
        return result
    t10 = next((t for t, y in pts if y >= 0.1), None)
    t90 = next((t for t, y in pts if y >= 0.9), None)
    result['dead'] = None if t10 is None else t10 - t_step
    result['rise'] = None if t10 is None or t90 is None else t90 - t10
    result['overshoot'] = max(0.0, (max(y for _, y in pts) - 1.0) * 100.0)
    outside = [i for i, (_, y) in enumerate(pts) if abs(y - 1.0) > BAND]
    if not outside:
        result['settle'] = pts[0][0] - t_step
    elif outside[-1] < len(pts) - 1:
        result['settle'] = pts[outside[-1] + 1][0] - t_step
    steady = [y * target for t, y in pts if t >= t_step + 0.6 * (t_end - t_step)]
    if steady:
        mean = sum(steady) / len(steady)
        result['mean'] = mean
        result['std'] = math.sqrt(sum((v - mean) ** 2 for v in steady) / len(steady))
    return result
