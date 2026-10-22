"""Radiation map from scintillator readings and measured poses (no ROS).

A reading (counts per second over [stamp - interval, stamp]) is painted only where the sensor actually was:
at the sensor point of every trusted measured pose of that interval (the supervisor's /coverage/sample).
If the interval has no trusted pose, or a gap, the reading is kept in the log but not drawn.

Contamination (user, 2026-10-22): the operator sets the threshold on the robot screen, as a count rate or as a factor
x the background. Enough readings at or above it (Alarm) end the task; the robot does not drive around contamination.
"""
from collections import deque
import math
import numpy as np

MODES = ('cps', 'background')   # threshold in counts/s, or a factor x the background


def threshold(setting, background):
    """Counts/s at which a reading counts as contaminated, or None (factor mode before the background is known).
    setting: {'mode': 'cps' | 'background', 'value': float}."""
    if setting['mode'] == 'cps':
        return float(setting['value'])
    return None if background is None else float(setting['value']) * float(background)


def default_setting(cfg):
    """The threshold from config/coverage.yaml (until the robot screen sends the operator's choice)."""
    mode = cfg['threshold_mode']
    return {'mode': mode, 'value': float(cfg['threshold_cps'] if mode == 'cps' else cfg['threshold_factor'])}


def valid_setting(setting):
    """A threshold the robot can use: a known mode; counts/s above 0, a factor above 1 (else the background itself
    would count as contamination)."""
    try:
        value = float(setting['value'])
        return setting['mode'] in MODES and np.isfinite(value) and value > (0. if setting['mode'] == 'cps' else 1.)
    except (KeyError, TypeError, ValueError):
        return False


class Alarm:
    """Contamination is found when `readings` of the last `window` readings are at or above the threshold (user,
    2026-10-22: any 2 in a short time, so one noisy reading does not stop the robot and one low one does not hide a
    spot). Every reading counts, also one without a trusted pose (user: the stop never depends on localization)."""

    def __init__(self, readings, window):
        self.readings, self.window = int(readings), int(window)
        self.reset()

    def reset(self):
        self.last = deque(maxlen=self.window)      # (counts/s, at or above the threshold)

    def add(self, cps, limit):
        """One reading against the threshold. True once contamination is found."""
        self.last.append((float(cps), float(cps) >= limit))
        return sum(hot for _, hot in self.last) >= self.readings


class RadiationMap:
    def __init__(self, grid, offset, cfg):
        self.grid, self.offset, self.cfg = grid, offset, cfg
        self.setting = default_setting(cfg)
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
        return threshold(self.setting, self.background)

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

    # ---------------------------------------------------------------- hotspots
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

    def state(self):
        return {'measuring': self.measuring, 'background_ready': self.background is not None,
                'background': self.background, 'threshold': self.threshold, 'setting': self.setting,
                'background_readings': len(self.samples), 'painted_cells': int(np.count_nonzero(self.count)),
                'hotspots': len(self.hotspots), 'to_check': sum(not h['checked'] for h in self.hotspots.values())}
