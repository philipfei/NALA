"""Find the robot on the map from one LiDAR scan (no ROS).

App 3 does not assume that the robot stands on the base station when it starts (user, 2026-10-19): the locator
searches the whole map for the pose where the scan fits best (shown on the robot screen), and checks whether the
robot stands on the base station, the right way round. Fit = share of scan points within match_distance_m of a
wall cell of the map.

Room "Test" is almost the same turned 180 deg around its base station, so the base check compares the base
station heading with the heading turned 180 deg (BACKWARDS when that fits clearly better). When both fit the
same (a fully symmetric room), the direction cannot be told and the robot counts as on the base station.
"""
import math

import numpy as np
from scipy import ndimage

ON_BASE = 'ON_BASE'
BACKWARDS = 'BACKWARDS'
NOT_ON_BASE = 'NOT_ON_BASE'


def endpoints(ranges, angle_min, angle_increment, range_min, range_max, laser, max_beams):
    """Scan points in the robot frame (x forward, y left) as an (N, 2) array; laser = (x, y, yaw) of the
    LiDAR in the robot frame. At most max_beams points, evenly spread over the scan."""
    r = np.asarray(ranges, float)
    ids = np.flatnonzero(np.isfinite(r) & (r >= range_min) & (r < range_max))
    if len(ids) > max_beams:
        ids = ids[np.linspace(0, len(ids) - 1, max_beams).astype(int)]
    a = angle_min + ids * angle_increment + laser[2]
    return np.column_stack((laser[0] + r[ids] * np.cos(a), laser[1] + r[ids] * np.sin(a)))


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Locator:
    def __init__(self, grid, cfg):
        """grid: nala_coverage.geometry.Grid of the saved map; cfg: the `locator` section of coverage.yaml."""
        self.grid = grid
        self.cfg = cfg
        self.res = grid.resolution
        # Distance (m) from every cell to the nearest wall cell; outside the map counts as far.
        self.dist = ndimage.distance_transform_edt(grid.cells != 100) * self.res
        step = max(1, round(cfg['position_step_m'] / self.res))
        rc = np.argwhere(grid.free[::step, ::step]) * step
        self.positions = grid.world(rc)

    def _fits(self, points, xy, yaw):
        """Fit of the robot-frame points for every position in xy (P, 2) at one heading: (P,) shares."""
        c, s = math.cos(yaw), math.sin(yaw)
        rotated = points @ np.array([[c, s], [-s, c]])
        world = xy[:, None, :] + rotated[None, :, :]
        local = self.grid.local(world.reshape(-1, 2)).reshape(world.shape)
        col = np.floor(local[..., 0] / self.res).astype(int)
        row = np.floor(local[..., 1] / self.res).astype(int)
        h, w = self.dist.shape
        inside = (row >= 0) & (row < h) & (col >= 0) & (col < w)
        near = np.zeros(row.shape, bool)
        near[inside] = self.dist[row[inside], col[inside]] < self.cfg['match_distance_m']
        return near.mean(axis=1)

    def fit(self, points, pose):
        return float(self._fits(points, np.array([pose[:2]], float), pose[2])[0]) if len(points) else 0.

    def refine(self, points, pose, window_m, window_deg):
        """Best pose (and its fit) within +-window around pose, on a fine grid."""
        dxy = np.arange(-window_m, window_m + 1e-9, self.res / 2)
        xy = np.array([[pose[0] + dx, pose[1] + dy] for dx in dxy for dy in dxy])
        best = (-1., tuple(pose))
        for dyaw in np.radians(np.arange(-window_deg, window_deg + 1e-9, 1.)):
            yaw = _wrap(pose[2] + dyaw)
            f = self._fits(points, xy, yaw)
            i = int(np.argmax(f))
            if f[i] > best[0]:
                best = (float(f[i]), (float(xy[i, 0]), float(xy[i, 1]), yaw))
        return best

    def search(self, points, previous=None):
        """Search the whole map. Returns (fit, pose) of the best pose and of the best clearly different one
        (more than 0.5 m or 30 deg away), or None when there are no points or no free cells. Poses that fit
        within ambiguity_margin of the best are a tie: then the one nearest to previous (x, y, yaw) wins."""
        if not len(points) or not len(self.positions):
            return None
        found = []
        for yaw in np.radians(np.arange(-180., 180., self.cfg['yaw_step_deg'])):
            f = self._fits(points, self.positions, yaw)
            for i in np.argsort(f)[-5:]:
                found.append((float(f[i]), (float(self.positions[i, 0]), float(self.positions[i, 1]), float(yaw))))
        found.sort(key=lambda item: -item[0])
        # Refine the best few clearly different candidates, then choose: on the coarse grid the true pose can
        # score a little below its mirror image in an almost symmetric room (room Test, a corner).
        distinct = []
        for item in found:
            if all(math.dist(item[1][:2], d[1][:2]) > .5 or abs(_wrap(item[1][2] - d[1][2])) > math.radians(30)
                   for d in distinct):
                distinct.append(item)
            if len(distinct) == 6:
                break
        refined = sorted((self.refine(points, d[1], self.cfg['position_step_m'], self.cfg['yaw_step_deg'])
                          for d in distinct), key=lambda item: -item[0])
        if previous is not None:
            # A robot carried by hand moves continuously: of tied poses (mirror images in a symmetric room) keep
            # the one near where it was found last.
            ties = [item for item in refined if item[0] >= refined[0][0] - self.cfg['ambiguity_margin']]
            near = min(ties, key=lambda item: math.dist(item[1][:2], previous[:2]))
            refined.remove(near)
            refined.insert(0, near)
        return refined[0], (refined[1] if len(refined) > 1 else None)

    def base_check(self, points, station):
        """ON_BASE, BACKWARDS or NOT_ON_BASE for a robot that should stand on station (x, y, yaw), with the
        fit and pose of the matching direction."""
        tol_m, tol_deg = self.cfg['base_tolerance_m'], self.cfg['base_tolerance_deg']
        forward = self.refine(points, station, tol_m, tol_deg)
        backward = self.refine(points, (station[0], station[1], _wrap(station[2] + math.pi)), tol_m, tol_deg)
        if backward[0] >= self.cfg['min_fit'] and backward[0] > forward[0] + self.cfg['direction_margin']:
            return BACKWARDS, backward
        if forward[0] >= self.cfg['min_fit']:
            return ON_BASE, forward
        return NOT_ON_BASE, forward

    def locate(self, points, station, previous=None):
        """Everything the screen and the supervisor need, as a JSON-ready dict. previous: the pose found
        last (or None), to choose between poses that fit equally well."""
        if not len(points):
            return {'status': NOT_ON_BASE, 'pose': None, 'fit': 0., 'base_fit': 0., 'ambiguous': False, 'points': 0}
        status, (base_fit, base_pose) = self.base_check(points, station)
        # On the base station with a fit no other pose can beat by direction_margin: no whole-map search
        # (it takes most of the time, ~3 s on the Pi 4 with the first settings).
        found = None if status != NOT_ON_BASE and base_fit >= 1 - self.cfg['direction_margin'] else self.search(points, previous)
        # A partial fit near the base station is not enough when the scan fits clearly better somewhere else
        # (0.5 m beside the base station still fits ~78 % there, map of room Test).
        if (status != NOT_ON_BASE and found is not None and found[0][0] > base_fit + self.cfg['direction_margin']
                and math.dist(found[0][1][:2], station[:2]) > self.cfg['base_tolerance_m']):
            status = NOT_ON_BASE
        if status != NOT_ON_BASE:
            pose, fit = base_pose, base_fit
        elif found is not None:
            fit, pose = found[0]
        else:
            pose, fit = None, 0.
        other = found[1] if found is not None else None
        ambiguous = other is not None and other[0] >= fit - self.cfg['ambiguity_margin']
        return {'status': status, 'pose': [round(v, 4) for v in pose] if pose is not None else None,
                'fit': round(fit, 3), 'base_fit': round(base_fit, 3), 'ambiguous': bool(ambiguous),
                'points': int(len(points))}
