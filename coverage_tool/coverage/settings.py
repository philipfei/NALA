"""Planner settings. A dict with a `.collision` attribute (robot radius in m),
which is the shape planning.py expects.

The robot geometry comes from the repo config/ folder when it is there (see robot_config):
the outline from robot.yaml (robot repo) and the sensor from coverage.yaml (stage 3)."""
import copy
import json
import math
import os

import yaml

# The repo config/ folder (coverage_tool/ sits next to it in the robot repo).
CONFIG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'config')
# Kept from walls on top of the collision radius, so the robot still clears them with some localisation error.
LOCALISATION_MARGIN_M = 0.03

DEFAULTS = {
    'geometry': {
        # Keeps the robot centre (base_link) this far from walls/obstacles: the outline's farthest corner
        # + padding + localisation margin, so it can rotate in place anywhere on the path.
        # Set from config/ by robot_config() (NALA with the rod: 0.478 + 0.02 + 0.03 -> 0.53 m).
        'robot_radius_m': 0.53,
        # The sensor (scintillator). PLACEHOLDER values, as in config/coverage.yaml.
        'coverage_disk_radius_m': 0.6,   # radius of the area the sensor covers
        'sensor_offset_m': 0.26,         # sensor point this far straight ahead of base_link
        # NALA's outer box in base_link (x forward, y left), m; it must never touch anything.
        # Same as robot.yaml (outline: front, back, left, right); set from config/ by robot_config().
        'footprint_m': [[0.4425, 0.18], [0.4425, -0.18], [-0.2075, -0.18], [-0.2075, 0.18]],
        'footprint_padding_m': 0.02,     # the footprint check (coverage/footprint.py) keeps this margin
    },
    'spiral': {
        'overlap_pct': 10.0,             # 0-50 %; path spacing = 2 x coverage radius x (1 - overlap)
        'obstacle_mode': 'around',       # 'around' = loops also circle obstacles, 'outer' = follow outer walls only
        'wall_side': 'right',            # side the wall is on while following it ('right' = counter-clockwise)
    },
    'strategy': {
        'mode': 'auto',                  # 'auto' = cheapest of all, 'regions' = mixed lanes/spiral per region, 'spiral'
        # True: patterns start by following the outer walls. User 2026-10-22: no fixed order; the optimiser puts the
        # wall loops where the route is cheapest.
        'wall_loop_first': False,
    },
    'cost': {                            # all terms in seconds, see coverage/cost.py
        'smooth_turn_deg': 20.0,         # bends below this are driven without stopping
        'bend_s_per_rad': 1.5,           # ...but still cost this much per radian (straight = cheapest)
        # Extra time per stop-and-turn on top of the rotation: ~1 s to brake and accelerate + 3 s penalty (user
        # 2026-10-22: turns count more than their time, so the route prefers fewer stops; up to ~36 cm of extra
        # driving at 0.12 m/s per stop avoided).
        'stop_penalty_s': 4.0,
        'reverse_angle_deg': 150.0,      # direction change counted as a reversal (hairpin)
        'reverse_penalty_s': 10.0,       # extra cost per reversal
        # x time to sweep the double-covered area. User 2026-10-22: 2 (was 1) and connecting drives count too
        # (RouteOptimizer._connector, pipeline: the floor a connection sweeps is double coverage), so the route does
        # not trade long repeat drives over covered floor for a few stops.
        'overlap_weight': 2.0,
        'missed_weight': 10000.0,        # x time to sweep the missed reachable area (very heavy: ~167 s per 5 cm cell)
        'edge_band_m': 0.10,             # floor this close to a wall/obstacle counts as an 'edge' cell...
        'edge_missed_weight': 10000.0,   # ...and costs this weight when missed (lower it for straighter lines)
    },
    'planning': {
        'max_segment_length_m': 2.0,     # longest single Nav2 goal segment
    },
    'motion': {
        'linear_m_s': 0.3,
        'angular_rad_s': 0.8,
    },
    'drive': {                           # drivable path for the robot, see coverage/drivable.py
        'min_arc_radius_m': 0.2,         # narrowest corner arc; tighter corners become stop-and-turn points
        'spacing_m': 0.05,               # point spacing along corner arcs in the exported drivable path
        'straight_deg': 1.0,             # heading changes below this are left as they are
        'max_corner_group': 8,           # at most this many consecutive corners are replaced by one arc
    },
    'spur': {                            # drive-ins into pockets the robot cannot turn in, see coverage/spurs.py
        'max_depth_m': 2.0,              # longest drive-in (straight in, the same way back out)
        'step_m': 0.05,                  # depth steps of the outline check, and of making a drive-in shorter
        'angle_range_deg': 30.0,         # directions tried: towards the pocket's centre +- this ...
        'angle_step_deg': 5.0,           # ... in these steps
        'max_starts': 150,               # start points tried per pocket (evenly spread over the candidates)
        'per_pocket': 6,                 # drive-ins per pocket offered to the route optimiser
        # Back-out speed / normal driving speed: spur.speed_m_s / motion.linear_m_s of config/coverage.yaml (user
        # 2026-10-21: back out at the normal speed, 0.12 m/s; before: half). Set by robot_config().
        'speed_factor': 1.0,
        # The robot may start a drive-in this far beside its line and steers onto it while driving in, so the line is
        # checked with this much margin (if more than the localisation margin). spur.max_offset_m of
        # config/coverage.yaml; set by robot_config().
        'start_offset_m': 0.05,
    },
    'passage': {                         # narrow entrances into areas with turning room, see coverage/passages.py
        'max_length_m': 2.0,             # longest straight passage (from turning room to turning room)
        'min_island_m2': 0.05,           # areas with turning room smaller than this are not worth a passage
        'ends_per_island': 60,           # island points tried as the inner end (spread over the island)
        'starts_per_end': 120,           # reachable points within max_length_m tried per inner end (spread, closest first)
    },
    'optimizer': {
        'search_time_s': 4.0,            # route optimisation per candidate pattern
        'polish_time_s': 15.0,           # extra optimisation of the cheapest candidate
        'neighbours': 8,                 # nearby pieces tried per move
        # Wall-following stretches may be moved away from the wall (localsearch.shift; user 2026-10-22: the cost
        # decides per stretch how close to a wall the route drives): stretches up to this long ...
        'shift_window_m': 2.0,
        # ... tried at this many distances, evenly up to as far as the sensor still reaches the wall floor
        # (coverage_disk_radius_m - robot_radius_m beyond the safety limit).
        'shift_levels': 3,
        # A piece may grow at its ends, straight on or along the wall, up to this long, while pieces it makes
        # redundant are dropped or shortened (localsearch.extend; user 2026-10-22).
        'extend_max_m': 2.0,
        # Remove and repair (localsearch.rebuild; user 2026-10-22): squares this wide around at least this many
        # stops are emptied and covered again by the repair moves, kept when the whole route gets cheaper. Runs on
        # the cheapest candidate after the polish, until no square improves or this much time has passed (user:
        # planning may take as long as it needs; this is only a safety limit).
        'rebuild_window_m': 2.0,
        'rebuild_min_stops': 3,
        'rebuild_time_s': 1800.0,
        # ... and in that phase the local moves stop after a round that saved less than this (s).
        'rebuild_min_gain_s': 1.0,
    },
}


class Settings(dict):
    @property
    def collision(self):
        return float(self['geometry']['robot_radius_m'])


_current = Settings(copy.deepcopy(DEFAULTS))


def robot_config(settings, config_dir=CONFIG_DIR):
    """Set the geometry from the repo config: the outline from robot.yaml, the padding and the sensor from
    coverage.yaml. The robot radius is the outline's farthest corner + padding + LOCALISATION_MARGIN_M,
    rounded up to 1 cm. Returns False (settings unchanged) when config_dir has no robot.yaml."""
    robot_file = os.path.join(config_dir, 'robot.yaml')
    if not os.path.isfile(robot_file):
        return False
    with open(robot_file) as f:
        robot = yaml.safe_load(f)
    o = robot['outline']
    front, back, left, right = (float(o[k]) for k in ('front', 'back', 'left', 'right'))
    g = settings['geometry']
    g['footprint_m'] = [[front, left], [front, -right], [-back, -right], [-back, left]]
    corner = max(math.hypot(x, y) for x, y in g['footprint_m'])
    padding = g['footprint_padding_m']
    coverage_file = os.path.join(config_dir, 'coverage.yaml')
    if os.path.isfile(coverage_file):
        with open(coverage_file) as f:
            cov = next(iter(yaml.safe_load(f).values()))['ros__parameters']['geometry']
        params = next(iter(yaml.safe_load(open(coverage_file)).values()))['ros__parameters']
        if 'spur' in params:
            settings['spur']['speed_factor'] = float(params['spur']['speed_m_s']) / float(params['motion']['linear_m_s'])
            settings['spur']['start_offset_m'] = float(params['spur']['max_offset_m'])
        # The robot's own driving speeds (user 2026-10-22; before: SIMBA's 0.3 m/s / 0.8 rad/s), so times and the
        # trade-off between driving and turning are the robot's.
        for key in ('linear_m_s', 'angular_rad_s'):
            settings['motion'][key] = float(params['motion'][key])
        padding = g['footprint_padding_m'] = float(cov['planning_padding_m'])
        g['coverage_disk_radius_m'] = float(cov['coverage_disk_radius_m'])
        g['sensor_offset_m'] = float(cov['sensor_offset_m'])
    g['robot_radius_m'] = math.ceil((corner + padding + LOCALISATION_MARGIN_M) * 100 - 1e-9) / 100
    return True


def default_settings():
    s = Settings(copy.deepcopy(DEFAULTS))
    robot_config(s)
    return s


def set_settings(s):
    global _current
    _current = s if isinstance(s, Settings) else Settings(s)


def load_settings(path=None):
    if path is None:
        return _current
    with open(path) as f:
        data = json.load(f)
    s = default_settings()
    for k, v in data.items():
        if isinstance(v, dict) and k in s:
            s[k].update({kk: vv for kk, vv in v.items() if kk in s[k]})
    robot_config(s)   # the robot geometry always comes from config/, not from an older settings file
    return s


def save_settings(s, path):
    with open(path, 'w') as f:
        json.dump(s, f, indent=2)
