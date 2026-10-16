"""Turn a planned route into a path the robot can follow exactly.

The planner works with straight lines and sharp corners. A differential-drive robot cannot drive
a sharp corner without stopping, and a controller that rounds it off on its own leaves cells
uncovered. So every corner of the route is replaced here by the widest circular arc (fillet)
that
  - is tangent to both neighbouring segments and fits in them,
  - keeps every cell the planned route covers (same brush and line-of-sight test as the planner),
  - stays at least the robot radius away from walls and obstacles,
  - is at least `drive.min_arc_radius_m`, so the controller can track it exactly.
A corner where no such arc exists (and every hairpin) stays sharp and becomes a *stop*: the
robot drives exactly to the corner, turns on the spot and continues. So the robot only stops
where that is needed to cover every cell.
"""
import math

import numpy as np

from . import sensor
from .cost import piece_cells
from .planning import points_of, poly_target, stroke_union


def route_with_kinds(result):
    """The whole driven route (connectors included) as vertices with the kind of the segment after each."""
    pts, kinds = [], []
    for t, conn in zip(result['targets'], result['connections']):
        kind = getattr(t, 'kind', 'spiral')
        tp = [list(map(float, p)) for p in points_of(t)]
        for p, k in [(list(map(float, q)), 'transit') for q in conn] + [(p, kind) for p in tp]:
            if pts and math.dist(pts[-1], p) < 1e-6:
                kinds[-1] = k
                continue
            pts.append(p)
            kinds.append(k)
    return pts, kinds


def _unit(v):
    n = math.hypot(v[0], v[1])
    return (v[0] / n, v[1] / n) if n > 1e-12 else (0.0, 0.0)


def _line(a, b):
    """A straight stretch is exported as its end point only: the same segment that was checked for
    clearance (sampling a segment in pieces can give a different answer at a cell corner)."""
    return [list(b)] if math.dist(a, b) > 1e-9 else []


def _arc(v, u, w, radius, dh, spacing):
    """Fillet of `radius` at corner v between incoming direction u and outgoing w: (A, points A..B, B)."""
    L = radius * math.tan(dh / 2)
    A = [v[0] - u[0] * L, v[1] - u[1] * L]
    B = [v[0] + w[0] * L, v[1] + w[1] * L]
    side = 1.0 if u[0] * w[1] - u[1] * w[0] > 0 else -1.0      # left turn: centre to the left of u
    c = [A[0] - side * u[1] * radius, A[1] + side * u[0] * radius]
    a0 = math.atan2(A[1] - c[1], A[0] - c[0])
    n = max(2, int(math.ceil(radius * dh / spacing)))
    pts = [[c[0] + radius * math.cos(a0 + side * dh * i / n), c[1] + radius * math.sin(a0 + side * dh * i / n)]
           for i in range(n + 1)]
    pts[0], pts[-1] = A, B
    return A, pts, B


def _safe(grid, pts, collision):
    return all(grid.segment_safe(p, q, collision) for p, q in zip(pts, pts[1:]))


def _simplify(grid, v, kinds, cells, keep_mask, collision, pivot):
    """Remove corners whose removal loses no planned cell and stays clear of walls.

    The planner follows LiDAR bumps along walls with jogs of a few centimetres and draws small
    U-shapes; the brush covers those cells from a straight line as well, and no arc fits in them.
    `pivot(v, k)` = cells the sensor sweeps while the robot turns on the spot at corner k."""
    v, kinds = [list(p) for p in v], list(kinds)
    seg = [cells([v[i], v[i + 1]]) for i in range(len(v) - 1)]
    piv = [pivot(v, k) for k in range(len(v))]
    count = np.zeros(keep_mask.size, int)
    for c in seg + piv:
        np.add.at(count, c, 1)
    changed = True
    while changed:
        changed = False
        i = 1
        while i < len(v) - 1:
            if not grid.segment_safe(v[i - 1], v[i + 1], collision):
                i += 1
                continue
            v2 = v[:i] + v[i + 1:]
            new = [cells([v[i - 1], v[i + 1]]), pivot(v2, i - 1), pivot(v2, i)]
            base = count.copy()
            for c in (seg[i - 1], seg[i], piv[i - 1], piv[i], piv[i + 1]):
                np.subtract.at(base, c, 1)
            covered_now = np.zeros(keep_mask.size, bool)
            for c in new:
                covered_now[c] = True
            if np.any(keep_mask & (count > 0) & (base <= 0) & ~covered_now):
                i += 1
                continue
            for c in new:
                np.add.at(base, c, 1)
            count = base
            seg[i - 1:i + 1] = [new[0]]
            piv[i - 1:i + 2] = [new[1], new[2]]
            v = v2
            del kinds[i]
            changed = True
    return v, kinds


def make_drivable(grid, result, settings, progress=None):
    """Drivable path for a planning result.

    Returns {'points': [[x, y], ...] (straight stretches as their end points, arcs every `spacing_m`),
    'kinds': kind per point,
    'stops': indices of the corners where the robot stops and turns, 'stats': {...}}.
    """
    d = settings.get('drive', {})
    spacing = float(d.get('spacing_m', 0.05))
    r_min = float(d.get('min_arc_radius_m', 0.2))
    straight = math.radians(float(d.get('straight_deg', 1.0)))
    max_span = int(d.get('max_corner_group', 8))
    hairpin = math.radians(float(settings['cost']['reverse_angle_deg']))
    radius = float(settings['geometry']['coverage_disk_radius_m'])
    collision = settings.collision
    region = result['region']
    keep = np.flatnonzero(result['covered'].ravel())                 # every cell the plan covers stays covered
    keep_mask = np.zeros(region.size, bool)
    keep_mask[keep] = True

    v, kinds = route_with_kinds(result)
    vertices_planned = len(v)

    off = sensor.offset(settings)

    def cells(pts):
        return piece_cells(grid, region, pts, radius, off) if len(pts) else np.zeros(0, int)

    def pivot(vl, k):
        """Cells the sensor (ahead of the robot) sweeps while the robot turns on the spot at corner k."""
        if off <= 0 or k <= 0 or k >= len(vl) - 1:
            return np.zeros(0, int)
        arc = sensor.sensor_path([vl[k - 1], vl[k], vl[k + 1]], off)[1:-1]
        return piece_cells(grid, region, arc, radius, 0.0) if arc else np.zeros(0, int)

    if len(v) > 2:
        v, kinds = _simplify(grid, v, kinds, cells, keep_mask, collision, pivot)
    if len(v) < 2:
        return {'points': [list(p) for p in v], 'kinds': kinds, 'stops': [],
                'stats': {'drive_stops': 0, 'drive_arcs': 0, 'drive_length_m': 0.0, 'drive_lost_cells': 0}}

    # Coverage multiplicity of the current path pieces: a corner may only drop cells covered elsewhere.
    count = np.zeros(region.size, int)
    seg_start = [list(p) for p in v[:-1]]          # current start of segment i (moves when vertex i is rounded)
    for i in range(len(v) - 1):
        np.add.at(count, cells([v[i], v[i + 1]]), 1)
    for i in range(1, len(v) - 1):
        np.add.at(count, pivot(v, i), 1)          # a rounded corner loses its turn on the spot

    out, out_kinds, stops, arcs, radii, reasons = [list(v[0])], [kinds[0]], [], 0, [], {}
    k = 1
    while k < len(v) - 1:
        if progress and k % 25 == 0:
            progress(f'Drivable path: corner {k}/{len(v) - 2}')
        a = seg_start[k - 1]
        u = _unit((v[k][0] - v[k - 1][0], v[k][1] - v[k - 1][1]))
        avail_in = math.dist(a, v[k])
        best, turn, prev = None, 0.0, u
        why = 'no_room'
        # One arc may replace a group of corners that turn the same way (a bevelled corner or a curve
        # drawn with short segments): from corner k up to corner j, tangent to the segment before k
        # and the segment after j.
        for j in range(k, min(len(v) - 1, k + max_span)):
            w = _unit((v[j + 1][0] - v[j][0], v[j + 1][1] - v[j][1]))
            cross = prev[0] * w[1] - prev[1] * w[0]
            step = math.atan2(cross, prev[0] * w[0] + prev[1] * w[1])
            if turn and abs(step) > straight and (step > 0) != (turn > 0):
                break                                   # turns the other way: not one arc
            turn += step
            prev = w
            dh = abs(turn)
            if dh >= hairpin:
                why = 'hairpin' if j == k else why
                break
            if dh < straight:
                continue
            uw = u[0] * w[1] - u[1] * w[0]
            dvec = (v[j][0] - v[k][0], v[j][1] - v[k][1])
            s = (dvec[0] * w[1] - dvec[1] * w[0]) / uw if abs(uw) > 1e-12 else 0.0     # corner X = v[k] + s*u
            tt = (u[0] * dvec[1] - u[1] * dvec[0]) / uw if abs(uw) > 1e-12 else 0.0     # X = v[j] - tt*w
            if s < -1e-9 or tt < -1e-9:
                continue
            X = [v[k][0] + u[0] * s, v[k][1] + u[1] * s]
            seg_out = math.dist(v[j], v[j + 1])
            avail_out = seg_out if j + 1 == len(v) - 1 else 0.5 * seg_out    # leave half for the next corner
            tan = math.tan(dh / 2)
            r_lo, r_hi = max(s, tt) / tan, min(s + avail_in, tt + avail_out) / tan
            if best is not None and r_hi <= best[0]:
                continue                                 # cannot beat the arc already found
            if r_hi < max(r_lo, r_min):
                continue
            old = [cells([a, v[k]])] + [cells([v[i], v[i + 1]]) for i in range(k, j + 1)] \
                + [pivot(v, i) for i in range(k, j + 1)]
            base = count.copy()
            for c in old:
                np.subtract.at(base, c, 1)
            at_risk = keep_mask & (count > 0) & (base <= 0)
            candidates, r = [], r_hi
            while r > max(r_lo, r_min):
                candidates.append(r)
                r *= 0.85
            candidates.append(max(r_lo, r_min))
            for r in candidates:
                if best is not None and r <= best[0]:
                    break
                A, arc, B = _arc(X, u, w, r, dh, spacing)
                if not _safe(grid, [a, A] + arc[1:] + [list(v[j + 1])], collision):
                    why = 'clearance' if why == 'no_room' else why
                    continue
                new = [cells([a, A]), cells(arc), cells([B, v[j + 1]])]
                covered_now = np.zeros(region.size, bool)
                for c in new:
                    covered_now[c] = True
                if np.any(at_risk & ~covered_now):
                    why = 'coverage'
                    continue
                best = (r, j, A, arc, B, old, new)
                break
        if best:
            r, j, A, arc, B, old, new = best
            for c in old:
                np.subtract.at(count, c, 1)
            for c in new:
                np.add.at(count, c, 1)
            seg_line = _line(out[-1], A)
            out += seg_line + [list(p) for p in arc[1:]]
            out_kinds += [kinds[k - 1]] * len(seg_line) + [kinds[j]] * (len(arc) - 1)
            seg_start[j] = list(B)
            arcs += 1
            radii.append(r)
            k = j + 1
            continue
        seg_line = _line(out[-1], v[k])
        out += seg_line
        out_kinds += [kinds[k - 1]] * len(seg_line)
        w = _unit((v[k + 1][0] - v[k][0], v[k + 1][1] - v[k][1]))
        if math.acos(max(-1.0, min(1.0, u[0] * w[0] + u[1] * w[1]))) >= straight:
            stops.append(len(out) - 1)
            reasons[why] = reasons.get(why, 0) + 1
        k += 1
    tail = _line(out[-1], v[-1])
    out += tail
    out_kinds += [kinds[-2]] * len(tail)

    # Final check on the whole drivable path, with the planner's own coverage test.
    final = stroke_union(grid, region, [poly_target(out, 'drive')], radius)
    lost = int((result['covered'] & ~final).sum())
    unsafe = sum(not grid.segment_safe(p, q, collision) for p, q in zip(out, out[1:]))
    length = float(sum(math.dist(p, q) for p, q in zip(out, out[1:])))
    stats = {'drive_stops': len(stops), 'drive_arcs': arcs, 'drive_length_m': length, 'drive_lost_cells': lost,
             'drive_unsafe_segments': unsafe,
             'drive_min_arc_radius_m': min(radii) if radii else 0.0, 'drive_stop_reasons': reasons,
             'drive_corners_planned': vertices_planned - 2, 'drive_corners_kept': len(v) - 2}
    return {'points': out, 'kinds': out_kinds, 'stops': stops, 'stats': stats}
