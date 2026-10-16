"""Where the coverage sensor (NALA's scintillator) is while the robot drives a path.

The planned path is the path of the robot's rotation centre (base_link, which Nav2 follows).
The sensor sits `sensor_offset_m` straight ahead of it, so on every straight segment the
sensor runs the same line shifted forward, and at a corner, where the robot rotates in
place, the sensor swings along an arc of radius `sensor_offset_m` around the corner point.
The robot only drives forward, so the direction of a path matters: the sensor leads at
the end of a path and trails at its start.

A single pose (no heading known) covers only `radius - offset` for certain, whatever the
heading: see `point_radius`.
"""
import math

import numpy as np

from .settings import load_settings

EPS = 1e-9
ARC_STEP = math.radians(30.0)   # chords of a 30 deg step stay within 1 cm of a 0.26 m arc


def offset(settings=None):
    s = load_settings() if settings is None else settings
    return float(s['geometry'].get('sensor_offset_m', 0.0))


def point_radius(radius, off):
    """Radius covered for certain around a pose whose heading is unknown."""
    return max(0.0, radius - off)


def sensor_path(points, off, arc_step=ARC_STEP):
    """The sensor's polyline for a base_link polyline driven forwards (in point order).
    Returns the input (as lists) when `off` is 0 or the path has no length."""
    pts = [np.asarray(p, float) for p in points]
    clean = pts[:1]
    for p in pts[1:]:
        if np.linalg.norm(p - clean[-1]) > EPS:
            clean.append(p)
    if off <= 0.0 or len(clean) < 2:
        return [p.tolist() for p in pts]
    units = [(b - a) / np.linalg.norm(b - a) for a, b in zip(clean, clean[1:])]
    out = [clean[0] + off * units[0]]
    for k in range(1, len(clean) - 1):
        p, u0, u1 = clean[k], units[k - 1], units[k]
        h0 = math.atan2(u0[1], u0[0])
        dh = math.atan2(u0[0] * u1[1] - u0[1] * u1[0], float(u0 @ u1))   # shortest rotation in place
        # Always both shifted end points: a path checked in parts (coverage/drivable.py) and as a whole then
        # gives exactly the same segments, so the same cells.
        out.append(p + off * u0)
        n = int(math.ceil(abs(dh) / arc_step))
        for i in range(1, n):
            h = h0 + dh * i / n
            out.append(p + off * np.array([math.cos(h), math.sin(h)]))
        if abs(dh) > EPS:
            out.append(p + off * u1)
    out.append(clean[-1] + off * units[-1])
    return [q.tolist() for q in out]
