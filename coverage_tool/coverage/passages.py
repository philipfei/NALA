"""Passages: straight drives through an entrance too narrow for the turning circle into an area with turning room
(user, 2026-10-21: a room on the RDG map has turning room inside, but its 0.70 m entrance is narrower than the
1.06 m turning circle, so the route never got there).

The route keeps the robot centre `robot_radius_m` from walls everywhere, because the robot may turn on the spot
anywhere. An "island" is an area where it can turn, but which is only connected to the rest through a place narrower
than that. A passage is a straight line from a point with turning room outside (A) to one inside (B), which the
robot's real outline (plus padding and the drive-in margin, spurs.body) can drive forwards in both directions:
A -> B to get in, B -> A (turned around inside) to get out. No reversing.

The grid's A* (Grid.astar) routes through passages when start and goal lie in different areas, so every connection
the planner makes (pattern builders, route optimiser, final connectors) can use them; the drivable path marks them
'passage' (checked with the outline, never rounded or cut), the robot checks them with its outline too.
"""
import math

import numpy as np
from scipy import ndimage

from .spurs import body


def _line_free(grid, outline, a, b, step):
    """The outline drives straight from a to b (heading a -> b) over known-free cells only, checked every step."""
    d = math.dist(a, b)
    yaw = math.atan2(b[1] - a[1], b[0] - a[0])
    n = max(1, int(math.ceil(d / step)))
    t = np.linspace(0.0, 1.0, n + 1)
    c, s = math.cos(yaw), math.sin(yaw)
    X = (a[0] + t * (b[0] - a[0]))[:, None] + (c * outline[:, 0] - s * outline[:, 1])[None, :]
    Y = (a[1] + t * (b[1] - a[1]))[:, None] + (s * outline[:, 0] + c * outline[:, 1])[None, :]
    rc = grid.local(np.stack([X.ravel(), Y.ravel()], axis=1)) / grid.resolution
    col, row = np.floor(rc[:, 0]).astype(int), np.floor(rc[:, 1]).astype(int)
    h, w = grid.cells.shape
    inside = (row >= 0) & (row < h) & (col >= 0) & (col < w)
    return bool(inside.all() and grid.free[row, col].all())


def find(grid, reach, settings, progress=None):
    """Passages from `reach` (the area with turning room connected to the start) into the islands of the same free
    space, also from island to island. Returns (passages [(A, B)], reach plus every island linked by them)."""
    sp, step = settings['passage'], float(settings['spur']['step_m'])
    radius = float(settings['geometry']['robot_radius_m'])
    outline = body(settings, grid.resolution)
    max_len = float(sp['max_length_m'])
    # The free space the robot is in (8-connected, like coverage_region), and the turning-room areas in it.
    free_lab, _ = ndimage.label(grid.free, structure=np.ones((3, 3)))
    ids = np.unique(free_lab[reach & grid.free])
    space = np.isin(free_lab, ids[ids > 0])
    ok_lab, n = ndimage.label(grid.passable(radius) & space)   # 4-connected, as reachable_from / A*
    cell_m2 = grid.resolution ** 2
    linked, passages = reach.copy(), []
    reach_px = max_len / grid.resolution
    changed = True
    while changed:
        changed = False
        dist, (ir, ic) = ndimage.distance_transform_edt(~linked, return_indices=True)
        for k in range(1, n + 1):
            island = ok_lab == k
            if (island & linked).any() or island.sum() * cell_m2 < float(sp['min_island_m2']):
                continue
            # Inner ends: island points within reach, spread over the island; outer ends: linked points within
            # max_length_m of each, spread too (the nearest ones often see the entrance at an angle the outline
            # cannot pass; RDG room: the line that fits starts 1.9 m away).
            near = np.argwhere(island & (dist <= reach_px))
            if not len(near):
                continue
            near = near[np.linspace(0, len(near) - 1, min(len(near), int(sp['ends_per_island']))).astype(int)]
            lk = np.argwhere(linked)
            best = None
            for r, c in near:
                b = grid.world(np.array([r, c])).tolist()
                d = np.hypot(lk[:, 0] - r, lk[:, 1] - c)
                cand = np.flatnonzero(d <= reach_px)
                if best is not None:
                    cand = cand[d[cand] * grid.resolution < best[0]]
                if not len(cand):
                    continue
                cand = cand[np.argsort(d[cand])]
                cand = cand[np.linspace(0, len(cand) - 1, min(len(cand), int(sp['starts_per_end']))).astype(int)]
                for j in cand:                                   # shortest first
                    a = grid.world(lk[j]).tolist()
                    if _line_free(grid, outline, a, b, step) and _line_free(grid, outline, b, a, step):
                        best = (math.dist(a, b), a, b)
                        break
            if best is not None:
                passages.append((best[1], best[2]))
                linked |= island
                changed = True
                if progress:
                    progress(f'Passage {len(passages)}: {best[0]:.2f} m into {island.sum() * cell_m2:.2f} m2 '
                             f'with turning room')
                break                       # distances changed: start again from the larger linked area
    return passages, linked


def way_out(grid, xy, home, reach, home_area, radius):
    """The path from xy inside an island through its passage(s) to the first point of the area with turning room the
    start lies in (home_area), or None when xy is not inside an island (user, 2026-10-21: a route that would end in
    an island includes its way out; Nav2's return home cannot plan through a narrow entrance)."""
    r, c = grid.cell(xy)
    if not grid.valid((r, c)) or home_area[r, c]:
        return None
    path = grid.astar(xy, home, reach, radius)
    for k, p in enumerate(path[1:], 1):
        if home_area[grid.cell(p)]:
            return path[:k + 1]
    return path or None
