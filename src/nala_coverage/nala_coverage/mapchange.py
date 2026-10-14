"""Changes against the saved map, from LiDAR scans taken at trusted poses (no ROS).

Every beam passes through cells (seen free) and ends in a cell (seen occupied) unless it reached its range limit.
A cell counts at most once per scan, so the thresholds are numbers of scans (separate looks over time).
New object: a cell free on the map, not next to a wall of the map, that the beams keep ending in.
Removed object: a cell occupied on the map that the beams keep passing through, with no beam ending in or near it.
The margin around walls stops a few cm of localisation error from making the walls look changed.
(SLAM maps draw objects as outlines, often with unknown cells inside: the outline cells are what can be removed.)
"""
import math
import numpy as np


def _dilate(mask, cells):
    """mask grown by a disk of `cells` cells."""
    out = mask.copy()
    if cells <= 0 or not mask.any():
        return out
    rows, cols = mask.shape
    for dr in range(-cells, cells + 1):
        for dc in range(-cells, cells + 1):
            if dr * dr + dc * dc > cells * cells:
                continue
            out[max(dr, 0):rows + min(dr, 0), max(dc, 0):cols + min(dc, 0)] |= \
                mask[max(-dr, 0):rows + min(-dr, 0), max(-dc, 0):cols + min(-dc, 0)]
    return out


class ChangeMap:
    def __init__(self, grid, cfg):
        self.grid, self.cfg = grid, cfg
        self.occupied = grid.cells > 0
        self.margin = int(math.ceil(cfg['wall_margin_m'] / grid.resolution))
        self.may_appear = (grid.cells == 0) & ~_dilate(self.occupied, self.margin)
        self.reset()

    def reset(self):
        shape = self.grid.cells.shape
        self.hits = np.zeros(shape, np.int32)
        self.passes = np.zeros(shape, np.int32)

    def add_scan(self, origin, angles, ranges, range_max):
        """One scan from the laser at `origin` (x, y) with beam directions `angles` (map frame, rad)."""
        res = self.grid.resolution
        use = np.isfinite(ranges) & (ranges > 0)
        angles, ranges = np.asarray(angles)[use], np.minimum(np.asarray(ranges)[use], self.cfg['max_range_m'])
        hit = np.asarray(ranges) < min(range_max, self.cfg['max_range_m']) - 1e-6
        if not len(ranges):
            return
        # Points along each beam every half cell, stopping one cell before its end (that cell is the hit).
        steps = np.arange(0.0, ranges.max(), res / 2)
        d = steps[None, :]
        inside = d < (ranges[:, None] - res)
        xs = origin[0] + d * np.cos(angles)[:, None]
        ys = origin[1] + d * np.sin(angles)[:, None]
        self.passes += self._seen(xs[inside], ys[inside])
        self.hits += self._seen(origin[0] + ranges[hit] * np.cos(angles[hit]),
                                origin[1] + ranges[hit] * np.sin(angles[hit]))

    def _seen(self, xs, ys):
        """0/1 per cell: does any of the points fall in it (a cell counts once per scan)."""
        local = self.grid.local(np.stack([xs, ys], axis=-1))
        rc = np.floor(local[:, ::-1] / self.grid.resolution).astype(int)
        rows, cols = self.grid.cells.shape
        ok = (rc[:, 0] >= 0) & (rc[:, 0] < rows) & (rc[:, 1] >= 0) & (rc[:, 1] < cols)
        seen = np.zeros(rows * cols, np.int32)
        seen[rc[ok, 0] * cols + rc[ok, 1]] = 1
        return seen.reshape(rows, cols)

    def appeared(self):
        """New objects: free map cells that beams end in, much more often than they pass through."""
        c = self.cfg
        return self.may_appear & (self.hits >= c['min_hits']) & (self.hits >= c['hit_ratio'] * (self.hits + self.passes))

    def vanished(self):
        """Removed objects: occupied map cells that beams pass through again and again, with no beam ending in or near them."""
        near_hit = _dilate(self.hits > 0, self.margin)
        return self.occupied & (self.passes >= self.cfg['min_passes']) & ~near_hit

    def area_m2(self):
        cell = self.grid.resolution ** 2
        return float(self.appeared().sum()) * cell, float(self.vanished().sum()) * cell
