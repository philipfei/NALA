"""In-and-out stretches ("spurs") into floor the normal route cannot reach (user, 2026-10-20).

The route keeps the robot centre `robot_radius_m` from walls everywhere, because the robot may turn on the spot
anywhere (with the rod its turning circle is ~0.96 m across). A pocket narrower than that is missed although the
robot fits into it driving straight. A spur leaves the route at a point P (on the route, so there is turning room),
drives straight forwards into the pocket to Q, and reverses straight back out to P along the same line. Forwards the
sensor (ahead of the robot) measures the floor; backwards the robot only passes over floor it has just measured, so
it never reverses into unmeasured (possibly contaminated) floor. The straight line is checked with the robot's real
outline (footprint_m) plus a margin, never with the turning circle.

Runs after the drivable path is made (drivable.make_drivable) and inserts the spurs into it:
..., A, P, Q, P, B, ... with kinds 'spur' (arriving at Q) and 'reverse' (arriving back at P), and stops at P, Q, P.
"""
import math

import numpy as np
from scipy import ndimage

from .cost import piece_cells
from .footprint import _body_points
from .planning import stroke_union, poly_target
from .settings import LOCALISATION_MARGIN_M

STEP_M = 0.05            # depth search and points along the route tried as P
MAX_DEPTH_M = 2.0        # longest spur
MIN_GAIN_CELLS = 4       # a spur must cover at least this many missed cells
ANGLE_STEPS_DEG = range(-30, 31, 5)


def _pose_free(grid, body, x, y, yaw):
    """The outline (body points) at (x, y, yaw) lies on known-free cells only."""
    c, s = math.cos(yaw), math.sin(yaw)
    X = x + c * body[:, 0] - s * body[:, 1]
    Y = y + s * body[:, 0] + c * body[:, 1]
    rc = grid.local(np.stack([X, Y], axis=1)) / grid.resolution
    col, row = np.floor(rc[:, 0]).astype(int), np.floor(rc[:, 1]).astype(int)
    h, w = grid.cells.shape
    inside = (row >= 0) & (row < h) & (col >= 0) & (col < w)
    return bool(inside.all() and grid.free[row, col].all())


def _depth(grid, body, p, yaw, step=STEP_M, max_depth=MAX_DEPTH_M):
    """How far the outline can drive straight from p with heading yaw, checked every step (0 if not at all)."""
    d, best = step, 0.0
    while d <= max_depth + 1e-9:
        x, y = p[0] + d * math.cos(yaw), p[1] + d * math.sin(yaw)
        if not _pose_free(grid, body, x, y, yaw):
            break
        best = d
        d += step
    return best


def _route_points(points, step=STEP_M):
    """Points along the drive path every `step`: (xy, index of the segment they lie on)."""
    out = []
    for i, (a, b) in enumerate(zip(points, points[1:])):
        a, b = np.asarray(a, float), np.asarray(b, float)
        n = max(1, int(math.ceil(np.linalg.norm(b - a) / step)))
        for k in range(n + (1 if i == len(points) - 2 else 0)):
            out.append(((a + (b - a) * k / n).tolist(), i))
    return out


def spur_time(settings, depth):
    """Seconds for one spur: in and out, the turn there and back, and its three stops."""
    v = float(settings['motion']['linear_m_s'])
    w = float(settings['motion']['angular_rad_s'])
    return 2 * depth / v + 2 * (math.pi / 2) / w + 3 * float(settings['cost']['stop_penalty_s'])


def find_spur(grid, region, missed, points, kinds, settings, body):
    """The best spur for the missed cells: (gain cells, P, Q, segment index) or None."""
    radius = float(settings['geometry']['coverage_disk_radius_m'])
    off = float(settings['geometry'].get('sensor_offset_m', 0.0))
    lab, n = ndimage.label(missed.reshape(grid.cells.shape), structure=np.ones((3, 3)))
    if not n:
        return None
    sizes = ndimage.sum(np.ones_like(lab), lab, range(1, n + 1))
    best = None
    for k in np.argsort(sizes)[::-1]:
        if sizes[k] < MIN_GAIN_CELLS:
            break
        cells = np.argwhere(lab == k + 1)
        centre = grid.world(cells).mean(axis=0)
        reach = MAX_DEPTH_M + off + radius
        for p, seg in _route_points(points):
            if kinds and kinds[seg + 1] in ('spur', 'reverse'):
                continue
            v = centre - np.asarray(p)
            dist = float(np.linalg.norm(v))
            if dist > reach or dist < 1e-6:
                continue
            for da in ANGLE_STEPS_DEG:
                yaw = math.atan2(v[1], v[0]) + math.radians(da)
                depth = _depth(grid, body, p, yaw)
                if depth < STEP_M:
                    continue
                q = [p[0] + depth * math.cos(yaw), p[1] + depth * math.sin(yaw)]
                gain = int(missed[piece_cells(grid, region, [p, q], radius)].sum())
                if gain >= MIN_GAIN_CELLS:
                    score = gain / spur_time(settings, depth)   # cells per second
                    if best is None or score > best[0]:
                        best = (score, gain, p, q, seg)
        if best is not None:
            return best[1:]
    return None


def add_spurs(grid, result, settings, progress=None, max_spurs=8):
    """Insert spurs into result['drive'] for floor the route misses. Returns how many were added."""
    drive = result.get('drive')
    g = settings['geometry']
    if not drive or len(drive['points']) < 2 or not g.get('footprint_m'):
        return 0
    margin = float(g.get('footprint_padding_m', 0.0)) + LOCALISATION_MARGIN_M   # like the robot radius
    body = _body_points(g['footprint_m'], margin, grid.resolution / 2)
    region = result['region']
    radius = float(g['coverage_disk_radius_m'])
    points, kinds, stops = [list(p) for p in drive['points']], list(drive['kinds']), list(drive['stops'])
    covered = stroke_union(grid, region, [poly_target(points, 'drive')], radius).ravel() | result['covered'].ravel()
    free = (region.ravel() & grid.free.ravel())
    added = 0
    total_s = 0.0
    while added < max_spurs:
        missed = free & ~covered
        found = find_spur(grid, region, missed, points, kinds, settings, body)
        if found is None:
            break
        gain, p, q, seg = found
        if progress:
            progress(f'Spur {added + 1}: {gain} cells')
        k = seg + 1                                   # insert after point seg: ..., A, P, Q, P, B, ...
        if math.dist(points[seg], p) < 1e-6:
            ins, ins_kinds, at = [q, p], ['spur', 'reverse'], seg
        elif math.dist(points[seg + 1], p) < 1e-6:
            ins, ins_kinds, at = [q, p], ['spur', 'reverse'], seg + 1
        else:
            ins, ins_kinds, at = [p, q, p], [kinds[k], 'spur', 'reverse'], seg
        points[at + 1:at + 1] = [list(x) for x in ins]
        kinds[at + 1:at + 1] = ins_kinds
        stops = [s + len(ins) if s > at else s for s in stops]
        first = at + 1 if len(ins) == 3 else at       # index of P
        stops = sorted((set(stops) | {first, first + 1, first + 2}) - {0, len(points) - 1})
        covered[piece_cells(grid, region, [p, q], radius)] = True
        total_s += spur_time(settings, math.dist(p, q))
        added += 1
    if added:
        drive['points'], drive['kinds'], drive['stops'] = points, kinds, stops
        drive['stats']['drive_spurs'] = added
        drive['stats']['drive_stops'] = len(stops)
        drive['stats']['drive_length_m'] = float(sum(math.dist(a, b) for a, b in zip(points, points[1:])))
        drive['stats']['drive_spur_s'] = total_s
        result['covered'] = covered.reshape(result['covered'].shape)
    return added
