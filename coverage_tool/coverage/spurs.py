"""Drive-ins ("spurs") into floor the robot cannot reach with turning room (user, 2026-10-20).

The route keeps the robot centre `robot_radius_m` from walls everywhere, because the robot may turn on the spot
anywhere (with the rod its turning circle is ~0.96 m across). A pocket narrower than that is missed although the
robot fits into it driving straight. A drive-in starts at a point P with turning room, drives straight forwards into
the pocket to Q, and backs straight out to P along the same line. Forwards the sensor (ahead of the robot) measures
the floor; backwards the robot only passes over floor it has just measured, so it never backs into unmeasured
(possibly contaminated) floor. The line is checked with the robot's real outline (footprint_m) plus padding and the
localisation margin, never with the turning circle.

This module only offers candidates. The route optimiser (localsearch.RouteOptimizer) decides with the cost function
which drive-ins the route uses, from where, and where in the route, like any other piece (user, 2026-10-20: the
planner must plan the route with them, not add them to a finished route).
"""
import math

import numpy as np
from scipy import ndimage

from .cost import piece_cells
from .footprint import _body_points
from .settings import LOCALISATION_MARGIN_M


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


def _depth(grid, body, p, yaw, step, max_depth):
    """How far the outline can drive straight from p with heading yaw, checked every step (0 if not at all).
    Every step is shorter than the outline, so the poses together cover everything it passes."""
    d = np.arange(1, int(math.floor(max_depth / step + 1e-9)) + 1) * step
    if not len(d):
        return 0.0
    c, s = math.cos(yaw), math.sin(yaw)
    X = p[0] + d[:, None] * c + (c * body[:, 0] - s * body[:, 1])[None, :]
    Y = p[1] + d[:, None] * s + (s * body[:, 0] + c * body[:, 1])[None, :]
    rc = grid.local(np.stack([X.ravel(), Y.ravel()], axis=1)) / grid.resolution
    col, row = np.floor(rc[:, 0]).astype(int), np.floor(rc[:, 1]).astype(int)
    h, w = grid.cells.shape
    ok = (row >= 0) & (row < h) & (col >= 0) & (col < w)
    ok[ok] = grid.free[row[ok], col[ok]]
    ok = ok.reshape(len(d), -1).all(axis=1)
    bad = np.flatnonzero(~ok)
    n = bad[0] if len(bad) else len(d)
    return float(d[n - 1]) if n else 0.0


def body(settings, resolution):
    """Outline points for the drive-in check: footprint + padding + the larger of the localisation margin and how
    far beside its line the robot may start a drive-in (spur.start_offset_m: it steers onto the line while driving
    in; user 2026-10-20: rather than an extra turn-drive-turn onto the start)."""
    g = settings['geometry']
    margin = max(LOCALISATION_MARGIN_M, float(settings['spur'].get('start_offset_m', 0.0)))
    return _body_points(g['footprint_m'], float(g.get('footprint_padding_m', 0.0)) + margin, resolution / 2)


def turn_room(settings):
    """Clearance a drive-in's start needs: the robot turns there after backing out, and AMCL may put it a few cm
    off (simulation 2026-10-20: 4 cm short of P, the rod touched a wall turning). User 2026-10-20: keep it."""
    return float(settings['geometry']['robot_radius_m']) + LOCALISATION_MARGIN_M


def drive_in_time(settings, depth):
    """Seconds of one drive-in: in and out at the drive-in speed, and a stop at Q and back at P. The turns into it
    and out of it are charged at its junctions, like for every piece."""
    v = float(settings['motion']['linear_m_s']) * float(settings['spur']['speed_factor'])
    return 2 * depth / v + 2 * float(settings['cost']['stop_penalty_s'])


def _shortest(grid, region, target, p, q, radius, gain, step):
    """Q pulled back towards P in steps as long as the line still covers `gain` cells of `target`: no deeper into the
    pocket (closer to its walls) than the sensor needs (user 2026-10-20: only as deep as needed)."""
    d = math.dist(p, q)
    u = [(q[0] - p[0]) / d, (q[1] - p[1]) / d]
    while d - step >= step:
        shorter = [p[0] + (d - step) * u[0], p[1] + (d - step) * u[1]]
        if int(target[piece_cells(grid, region, [p, shorter], radius)].sum()) < gain:
            break
        d -= step
        q = shorter
    return q


def lines_from(grid, region, pocket, p, settings, outline=None):
    """The best drive-in from a given start p into the pocket (flat bool mask): (P, Q) covering the most of it, as
    shallow as possible, or None (no turning room at p, or nothing reachable)."""
    r, c = grid.cell(p)
    if not grid.valid((r, c)) or grid.clearance[r, c] < turn_room(settings):
        return None
    sp = settings['spur']
    radius = float(settings['geometry']['coverage_disk_radius_m'])
    step, max_depth = float(sp['step_m']), float(sp['max_depth_m'])
    outline = body(settings, grid.resolution) if outline is None else outline
    cells = np.flatnonzero(pocket)
    centre = grid.world(np.stack(np.divmod(cells, grid.cells.shape[1]), axis=1)).mean(axis=0)
    towards = math.atan2(centre[1] - p[1], centre[0] - p[0])
    best = None
    for da in np.radians(np.arange(-float(sp['angle_range_deg']), float(sp['angle_range_deg']) + 1e-9,
                                   float(sp['angle_step_deg']))):
        yaw = towards + float(da)
        depth = _depth(grid, outline, p, yaw, step, max_depth)
        if depth < step:
            continue
        q = [p[0] + depth * math.cos(yaw), p[1] + depth * math.sin(yaw)]
        gain = int(pocket[piece_cells(grid, region, [p, q], radius)].sum())
        if gain and (best is None or gain > best[0]):
            best = (gain, q)
    if best is None:
        return None
    return list(p), _shortest(grid, region, pocket, list(p), best[1], radius, best[0], step)


def candidates(grid, reach, region, coverable, settings, progress=None):
    """Drive-ins for the floor in `region` the sensor cannot reach from positions with turning room (`coverable`):
    per pocket (8-connected cluster of such cells) the best `per_pocket` lines by pocket cells per second, and the
    lines that cover the most of it. Returns [(P, Q, flat cell indices it covers in region, pocket index)] and the
    pockets (flat bool masks), as (lines, pockets)."""
    sp = settings['spur']
    g = settings['geometry']
    radius = float(g['coverage_disk_radius_m'])
    off = float(g.get('sensor_offset_m', 0.0))
    step, max_depth = float(sp['step_m']), float(sp['max_depth_m'])
    angles = np.radians(np.arange(-float(sp['angle_range_deg']), float(sp['angle_range_deg']) + 1e-9,
                                  float(sp['angle_step_deg'])))
    outline = body(settings, grid.resolution)
    starts = grid.world(np.argwhere(reach & (grid.clearance >= turn_room(settings))))
    lab, n = ndimage.label(region & ~coverable, structure=np.ones((3, 3)))
    out, seen, pockets = [], set(), []
    for k in range(1, n + 1):
        pocket = (lab == k).ravel()
        pockets.append(pocket)
        centre = grid.world(np.argwhere(lab == k)).mean(axis=0)
        near = starts[np.linalg.norm(starts - centre, axis=1) <= max_depth + off + radius]
        if len(near) > int(sp['max_starts']):
            near = near[np.linspace(0, len(near) - 1, int(sp['max_starts'])).astype(int)]
        # Rank every line by a cheap upper bound (pocket cells within reach of the sensor's straight path, walls in
        # between ignored); only the best few get the exact sensor check (piece_cells: 3 ms per line).
        pxy = grid.world(np.argwhere(lab == k))
        rough = []
        for p in near:
            p = p.tolist()
            towards = math.atan2(centre[1] - p[1], centre[0] - p[0])
            for da in angles:
                yaw = towards + float(da)
                depth = _depth(grid, outline, p, yaw, step, max_depth)
                if depth < step:
                    continue
                u = np.array([math.cos(yaw), math.sin(yaw)])
                a = np.asarray(p) + off * u                           # the sensor's path: a -> a + depth * u
                t_ = np.clip((pxy - a) @ u, 0.0, depth)
                bound = int((np.linalg.norm(pxy - (a + t_[:, None] * u), axis=1) <= radius).sum())
                if bound:
                    rough.append((bound, bound / drive_in_time(settings, depth), p,
                                  [p[0] + depth * u[0], p[1] + depth * u[1]]))
        per = int(sp['per_pocket'])
        shortlist = sorted(rough, key=lambda x: -x[1])[:3 * per] + sorted(rough, key=lambda x: -x[0])[:3 * per]
        lines, done = [], set()
        for _, _, p, q in shortlist:
            key = (round(p[0], 4), round(p[1], 4), round(q[0], 4), round(q[1], 4))
            if key in done:
                continue
            done.add(key)
            gain = int(pocket[piece_cells(grid, region, [p, q], radius)].sum())
            if not gain:
                continue
            q = _shortest(grid, region, pocket, p, q, radius, gain, step)
            lines.append((gain, gain / drive_in_time(settings, math.dist(p, q)), p, q))
        if progress and lines:
            progress(f'Drive-ins: pocket {k}/{n}, {int(pocket.sum())} cells, {len(rough)} lines')
        chosen = sorted(lines, key=lambda x: -x[1])[:per] + sorted(lines, key=lambda x: (-x[0], -x[1]))[:per]
        for gain, _, p, q in chosen:
            key = (round(p[0], 4), round(p[1], 4), round(q[0], 4), round(q[1], 4))
            if key not in seen:
                seen.add(key)
                out.append((p, q, piece_cells(grid, region, [p, q], radius), k - 1))
    return out, pockets
