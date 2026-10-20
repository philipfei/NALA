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


def _lines(grid, body, points, kinds, centre, reach, turn_room):
    """Every candidate line towards centre: (P, Q, segment index, depth), Q as deep as the outline fits.
    P needs `turn_room` clearance: the robot turns on the spot there after backing out, and the back-out may stop a
    few cm off P (simulation 2026-10-20: 4 cm short of P, on the pocket side, the rod touched a wall turning)."""
    for p, seg in _route_points(points):
        if kinds and kinds[seg + 1] in ('spur', 'reverse'):
            continue
        r, c = grid.cell(p)
        if not grid.valid((r, c)) or grid.clearance[r, c] < turn_room:
            continue
        v = centre - np.asarray(p)
        dist = float(np.linalg.norm(v))
        if dist > reach or dist < 1e-6:
            continue
        for da in ANGLE_STEPS_DEG:
            yaw = math.atan2(v[1], v[0]) + math.radians(da)
            depth = _depth(grid, body, p, yaw)
            if depth >= STEP_M:
                yield p, [p[0] + depth * math.cos(yaw), p[1] + depth * math.sin(yaw)], seg, depth


def _shortest(grid, region, missed, p, q, radius, gain):
    """Q pulled back towards P in STEP_M steps as long as the line still covers `gain` missed cells: no deeper into
    the pocket (closer to its walls) than needed."""
    d = math.dist(p, q)
    u = [(q[0] - p[0]) / d, (q[1] - p[1]) / d]
    while d - STEP_M >= STEP_M:
        shorter = [p[0] + (d - STEP_M) * u[0], p[1] + (d - STEP_M) * u[1]]
        if int(missed[piece_cells(grid, region, [p, shorter], radius)].sum()) < gain:
            break
        d -= STEP_M
        q = shorter
    return q


def find_spur(grid, region, missed, points, kinds, settings, body, whole=False):
    """The best spur for the missed cells: (gain cells, P, Q, segment index) or None.
    Most missed cells per second; whole=True: only a spur covering ALL missed cells (one cluster), shortest."""
    radius = float(settings['geometry']['coverage_disk_radius_m'])
    off = float(settings['geometry'].get('sensor_offset_m', 0.0))
    lab, n = ndimage.label(missed.reshape(grid.cells.shape), structure=np.ones((3, 3)))
    if not n or (whole and n > 1):
        return None
    sizes = ndimage.sum(np.ones_like(lab), lab, range(1, n + 1))
    best = None
    for k in np.argsort(sizes)[::-1]:
        if sizes[k] < MIN_GAIN_CELLS:
            break
        centre = grid.world(np.argwhere(lab == k + 1)).mean(axis=0)
        turn_room = float(settings['geometry']['robot_radius_m']) + LOCALISATION_MARGIN_M
        for p, q, seg, depth in _lines(grid, body, points, kinds, centre, MAX_DEPTH_M + off + radius, turn_room):
            gain = int(missed[piece_cells(grid, region, [p, q], radius)].sum())
            if gain < MIN_GAIN_CELLS or (whole and gain < int(missed.sum())):
                continue
            q = _shortest(grid, region, missed, p, q, radius, gain)
            score = gain / spur_time(settings, math.dist(p, q))   # cells per second
            if best is None or score > best[0]:
                best = (score, gain, p, q, seg)
        if best is not None:
            return best[1:]
    return None


def drive_cells(grid, region, points, kinds, radius):
    """Flat bool mask of the cells the drivable path's sensor covers: the forward runs as polylines, each drive-in as
    its line (backing out passes the same floor)."""
    cov = np.zeros(grid.cells.size, bool)
    run = [points[0]]
    for i in range(1, len(points)):
        if kinds[i] in ('spur', 'reverse'):
            if len(run) > 1:
                cov[piece_cells(grid, region, run, radius)] = True
            if kinds[i] == 'spur':
                cov[piece_cells(grid, region, [points[i - 1], points[i]], radius)] = True
            run = [points[i]]
        else:
            run.append(points[i])
    if len(run) > 1:
        cov[piece_cells(grid, region, run, radius)] = True
    return cov


def _merge(grid, region, spurs_found, base_cov, points, kinds, settings, body, radius):
    """Two drive-ins whose own cells one drive-in can cover become that one (robot 2026-10-20: two drive-ins into the
    same pocket from almost the same point, the second for 5 cells)."""
    merged = True
    while merged:
        merged = False
        cells = [np.zeros(grid.cells.size, bool) for _ in spurs_found]
        for c, (_, p, q, _) in zip(cells, spurs_found):
            c[piece_cells(grid, region, [p, q], radius)] = True
        for i in range(len(spurs_found)):
            for j in range(i + 1, len(spurs_found)):
                others = base_cov.copy()
                for k, c in enumerate(cells):
                    if k not in (i, j):
                        others |= c
                own = (cells[i] | cells[j]) & ~others
                one = find_spur(grid, region, own, points, kinds, settings, body, whole=True) if own.any() else None
                if one is not None:
                    spurs_found[i] = one
                    del spurs_found[j]
                    merged = True
                    break
            if merged:
                break
    return spurs_found


def _trim_tail(grid, region, points, kinds, stops, radius):
    """Cut the end of the path back to its last stop while that end covers nothing the rest does not (the route was
    made before the drive-ins; its last stretch may only have served floor a drive-in now covers; robot 2026-10-20:
    back across the room to the pocket mouth at the end). Returns the metres cut."""
    full = drive_cells(grid, region, points, kinds, radius)
    cut_m = 0.0
    while stops and stops[-1] < len(points) - 1:
        s = stops[-1]
        if any(k in ('spur', 'reverse') for k in kinds[s + 1:]):
            break                       # never into a drive-in: the robot would stay in the pocket
        if (full & ~drive_cells(grid, region, points[:s + 1], kinds[:s + 1], radius)).any():
            break
        cut_m += sum(math.dist(a, b) for a, b in zip(points[s:], points[s + 1:]))
        del points[s + 1:], kinds[s + 1:], stops[-1]
    return cut_m


def add_spurs(grid, result, settings, progress=None, max_spurs=8):
    """Insert spurs into result['drive'] for floor the route misses, then cut a path end made redundant by them.
    Returns how many spurs were added."""
    drive = result.get('drive')
    g = settings['geometry']
    if not drive or len(drive['points']) < 2 or not g.get('footprint_m'):
        return 0
    margin = float(g.get('footprint_padding_m', 0.0)) + LOCALISATION_MARGIN_M   # like the robot radius
    body = _body_points(g['footprint_m'], margin, grid.resolution / 2)
    region = result['region']
    radius = float(g['coverage_disk_radius_m'])
    points, kinds, stops = [list(p) for p in drive['points']], list(drive['kinds']), list(drive['stops'])
    base_cov = stroke_union(grid, region, [poly_target(points, 'drive')], radius).ravel() | result['covered'].ravel()
    free = (region.ravel() & grid.free.ravel())
    found, covered = [], base_cov.copy()
    while len(found) < max_spurs:
        spur = find_spur(grid, region, free & ~covered, points, kinds, settings, body)
        if spur is None:
            break
        found.append(spur)
        covered[piece_cells(grid, region, [spur[1], spur[2]], radius)] = True
    found = _merge(grid, region, found, base_cov, points, kinds, settings, body, radius)
    # Insert along the path, last first so the earlier indices stay valid: ..., A, P, Q, P, B, ...
    total_s = 0.0
    order = sorted(found, key=lambda f: (f[3], math.dist(points[f[3]], f[1])), reverse=True)
    for n, (gain, p, q, seg) in enumerate(order):
        if progress:
            progress(f'Drive-in {n + 1}: {gain} cells, {math.dist(p, q):.2f} m')
        if math.dist(points[seg], p) < 1e-6:
            ins, ins_kinds, at = [q, p], ['spur', 'reverse'], seg
        elif math.dist(points[seg + 1], p) < 1e-6:
            ins, ins_kinds, at = [q, p], ['spur', 'reverse'], seg + 1
        else:
            ins, ins_kinds, at = [p, q, p], [kinds[seg + 1], 'spur', 'reverse'], seg
        points[at + 1:at + 1] = [list(x) for x in ins]
        kinds[at + 1:at + 1] = ins_kinds
        stops = [s + len(ins) if s > at else s for s in stops]
        first = at + 1 if len(ins) == 3 else at       # index of P
        stops = sorted((set(stops) | {first, first + 1, first + 2}) - {0, len(points) - 1})
        total_s += spur_time(settings, math.dist(p, q))
    cut_m = _trim_tail(grid, region, points, kinds, stops, radius) if found else 0.0
    if found:
        all_cov = base_cov.copy()
        for _, p, q, _ in found:
            all_cov[piece_cells(grid, region, [p, q], radius)] = True
        drive['points'], drive['kinds'], drive['stops'] = points, kinds, stops
        drive['stats']['drive_spurs'] = len(found)
        drive['stats']['drive_stops'] = len(stops)
        drive['stats']['drive_length_m'] = float(sum(math.dist(a, b) for a, b in zip(points, points[1:])))
        drive['stats']['drive_spur_s'] = total_s
        drive['stats']['drive_trimmed_m'] = cut_m
        result['covered'] = all_cov.reshape(result['covered'].shape)
        if cut_m:
            _drop_cut_targets(result, points[-1])
    return len(found)


def _drop_cut_targets(result, end):
    """The route pieces after the cut path end (picture and Nav2 poses show what the robot drives)."""
    from .planning import points_of
    for i, t in enumerate(result.get('targets') or []):
        if math.dist(points_of(t)[-1], end) < 1e-6:
            del result['targets'][i + 1:], result['connections'][i + 1:]
            return
