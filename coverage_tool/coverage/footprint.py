"""Independent check of a drivable path against the robot's real outline.

The planner keeps the robot centre `robot_radius_m` from walls, which is enough for any outline
inside that circle. This check does not rely on that: it moves the real footprint polygon
(NALA's 480 x 360 mm box, plus padding) along the path, driving forwards along every segment
and turning on the spot at every corner, and counts the poses where it overlaps a cell that
is not known-free (occupied, unknown or outside the map).
"""
import math

import numpy as np

STEP_M = 0.02
TURN_STEP = math.radians(5.0)


def _body_points(footprint, padding, spacing):
    """Points filling the (padded) footprint's bounding box that lie inside the polygon."""
    poly = np.asarray(footprint, float)
    lo, hi = poly.min(axis=0) - padding, poly.max(axis=0) + padding
    xs = np.linspace(lo[0], hi[0], max(2, int(math.ceil((hi[0] - lo[0]) / spacing)) + 1))
    ys = np.linspace(lo[1], hi[1], max(2, int(math.ceil((hi[1] - lo[1]) / spacing)) + 1))
    P = np.array([[x, y] for x in xs for y in ys])
    if padding > 0:      # a padded polygon: inside, or within `padding` of the outline
        return P[_inside(poly, P) | (_edge_distance(poly, P) <= padding + 1e-9)]
    return P[_inside(poly, P) | (_edge_distance(poly, P) <= 1e-9)]


def _inside(poly, P):
    x, y = P[:, 0], P[:, 1]
    inside = np.zeros(len(P), bool)
    for (x0, y0), (x1, y1) in zip(poly, np.roll(poly, -1, axis=0)):
        cross = ((y0 > y) != (y1 > y)) & (x < (x1 - x0) * (y - y0) / np.where(y1 == y0, 1e-12, y1 - y0) + x0)
        inside ^= cross
    return inside


def _edge_distance(poly, P):
    d = np.full(len(P), np.inf)
    for a, b in zip(poly, np.roll(poly, -1, axis=0)):
        v = b - a
        t = np.clip((P - a) @ v / float(v @ v), 0, 1)
        d = np.minimum(d, np.linalg.norm(P - (a + t[:, None] * v), axis=1))
    return d


def poses(points, step=STEP_M, turn_step=TURN_STEP):
    """(x, y, yaw) poses of the robot driving `points` forwards, turning on the spot at corners."""
    pts = [np.asarray(p, float) for p in points]
    out, yaw = [], None
    for a, b in zip(pts, pts[1:]):
        d = float(np.linalg.norm(b - a))
        if d < 1e-9:
            continue
        h = math.atan2(b[1] - a[1], b[0] - a[0])
        if yaw is not None:
            dh = math.atan2(math.sin(h - yaw), math.cos(h - yaw))
            n = int(math.ceil(abs(dh) / turn_step))
            out += [(a[0], a[1], yaw + dh * i / n) for i in range(1, n)]
        n = max(1, int(math.ceil(d / step)))
        out += [(*(a + (b - a) * i / n), h) for i in range(n + (1 if b is pts[-1] else 0))]
        yaw = h
    return out


def hits(grid, points, footprint, padding=0.0, chunk=2000):
    """Poses where the footprint overlaps a cell that is not known-free: (count, [(x, y, yaw), ...])."""
    body = _body_points(footprint, padding, grid.resolution / 2)
    P = np.asarray(poses(points), float)
    if not len(P):
        return 0, []
    bad = np.zeros(len(P), bool)
    h, w = grid.cells.shape
    for s in range(0, len(P), chunk):
        q = P[s:s + chunk]
        c, si = np.cos(q[:, 2])[:, None], np.sin(q[:, 2])[:, None]
        X = q[:, 0:1] + c * body[None, :, 0] - si * body[None, :, 1]
        Y = q[:, 1:2] + si * body[None, :, 0] + c * body[None, :, 1]
        rc = grid.local(np.stack([X.ravel(), Y.ravel()], axis=1)) / grid.resolution
        col, row = np.floor(rc[:, 0]).astype(int), np.floor(rc[:, 1]).astype(int)
        inside = (row >= 0) & (row < h) & (col >= 0) & (col < w)
        free = np.zeros(len(row), bool)
        free[inside] = grid.free[row[inside], col[inside]]
        bad[s:s + chunk] = ~free.reshape(X.shape).all(axis=1)
    return int(bad.sum()), [tuple(map(float, p)) for p in P[bad][:20]]
