"""Radiation map from scintillator readings and measured poses (no ROS).

A reading (counts per second over [stamp - interval, stamp]) is painted only where the sensor actually was:
at the sensor point of every trusted measured pose of that interval (the supervisor's /coverage/sample).
If the interval has no trusted pose, or a gap, the reading is kept in the log but not drawn.
"""
import math
import numpy as np


class RadiationMap:
    def __init__(self, grid, offset, cfg):
        self.grid, self.offset, self.cfg = grid, offset, cfg
        self.reset()

    def reset(self):
        """A new task: empty map, no background yet."""
        shape = self.grid.cells.shape
        self.value = np.full(shape, np.nan, np.float32)   # highest reading painted on each cell
        self.count = np.zeros(shape, np.int32)
        self.owner = np.zeros(shape, np.int32)            # hotspot id of each hot cell (stable ids)
        self.hotspots = {}
        self.next_id = 1
        self.background = None
        self.measuring = False
        self.samples = []

    # ---------------------------------------------------------------- background
    def start_background(self):
        self.reset()
        self.measuring = True

    def add_background(self, cps):
        """A reading while the robot stands on the base station. Returns True when the background is ready."""
        if not self.measuring:
            return False
        self.samples.append(float(cps))
        if len(self.samples) >= self.cfg['background_readings']:
            self.background = float(np.mean(self.samples))
            self.measuring = False
        return not self.measuring

    @property
    def threshold(self):
        return None if self.background is None else self.cfg['threshold_factor'] * self.background

    # ---------------------------------------------------------------- readings
    def sensor_points(self, poses, start, end, max_gap):
        """Sensor points of the trusted poses in [start, end], or None if that interval is not fully covered.

        poses: [(stamp, x, y, yaw, trusted)] sorted by stamp, around the interval."""
        inside = [p for p in poses if start <= p[0] <= end]
        if not inside or not all(p[4] for p in inside):
            return None
        stamps = [start] + [p[0] for p in inside] + [end]
        if max(b - a for a, b in zip(stamps, stamps[1:])) > max_gap:
            return None
        return [(x + self.offset * math.cos(yaw), y + self.offset * math.sin(yaw)) for _, x, y, yaw, _ in inside]

    def add_reading(self, cps, points):
        """Paint one reading on the cells of its sensor points. Returns the changed cells' count."""
        cells = {self.grid.cell(p) for p in points}
        cells = [rc for rc in cells if self.grid.valid(rc)]
        for rc in cells:
            old = self.value[rc]
            self.value[rc] = cps if np.isnan(old) else max(old, cps)
            self.count[rc] += 1
        return len(cells)

    # ---------------------------------------------------------------- hotspots and keepout
    def hot(self):
        if self.threshold is None:
            return np.zeros(self.value.shape, bool)
        with np.errstate(invalid='ignore'):
            return self.value >= self.threshold

    def update_hotspots(self, now):
        """Group hot cells closer than hotspot_join_m into hotspots. A hotspot keeps its id while it grows."""
        hot = np.argwhere(self.hot())
        join = self.cfg['hotspot_join_m'] / self.grid.resolution
        seen = np.zeros(len(hot), bool)
        alive = {}
        for i in range(len(hot)):
            if seen[i]:
                continue
            group, seen[i], k = [i], True, 0
            while k < len(group):
                near = np.flatnonzero(~seen & (np.sum((hot - hot[group[k]]) ** 2, axis=1) <= join * join))
                seen[near] = True
                group.extend(near.tolist())
                k += 1
            cells = hot[group]
            ids = [int(v) for v in self.owner[cells[:, 0], cells[:, 1]] if v and int(v) in self.hotspots]
            hid = min(ids) if ids else self.next_id
            if not ids:
                self.next_id += 1
                self.hotspots[hid] = {'id': hid, 'first_seen': now, 'checked': False}
            self.owner[cells[:, 0], cells[:, 1]] = hid
            values = self.value[cells[:, 0], cells[:, 1]]
            peak = cells[int(np.argmax(values))]
            x, y = self.grid.world(peak).tolist()
            self.hotspots[hid].update(peak=float(values.max()), x=x, y=y, cells=len(cells))
            alive[hid] = self.hotspots[hid]
        self.hotspots = alive
        return sorted(alive.values(), key=lambda h: h['id'])

    def mark_checked(self, hid):
        if hid in self.hotspots:
            self.hotspots[hid]['checked'] = True
            return True
        return False

    def keepout(self):
        """Hot cells plus keepout_margin_m around them: the robot avoids this floor."""
        hot = self.hot()
        r = int(math.ceil(self.cfg['keepout_margin_m'] / self.grid.resolution))
        out = hot.copy()
        if not hot.any() or r == 0:
            return out
        rows, cols = hot.shape
        for dr in range(-r, r + 1):
            for dc in range(-r, r + 1):
                if dr * dr + dc * dc > r * r:
                    continue
                shifted = np.zeros_like(hot)
                shifted[max(dr, 0):rows + min(dr, 0), max(dc, 0):cols + min(dc, 0)] = \
                    hot[max(-dr, 0):rows + min(-dr, 0), max(-dc, 0):cols + min(-dc, 0)]
                out |= shifted
        return out

    def state(self):
        return {'measuring': self.measuring, 'background_ready': self.background is not None,
                'background': self.background, 'threshold': self.threshold,
                'background_readings': len(self.samples), 'painted_cells': int(np.count_nonzero(self.count)),
                'hotspots': len(self.hotspots), 'to_check': sum(not h['checked'] for h in self.hotspots.values())}
