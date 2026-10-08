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
        # Keeps the robot centre (base_link) this far from walls/obstacles. NALA's 410 x 360 mm box
        # reaches 0.273 m from its rotation centre, so 0.273 + 0.02 padding + 0.03 localisation margin
        # lets it rotate in place anywhere on the path. Set from config/ by robot_config().
        'robot_radius_m': 0.33,
        # The sensor (scintillator). PLACEHOLDER values, as in config/coverage.yaml.
        'coverage_disk_radius_m': 0.6,   # radius of the area the sensor covers
        'sensor_offset_m': 0.26,         # sensor point this far straight ahead of base_link
        # NALA's outer box in base_link (x forward, y left), m; it must never touch anything.
        # Same as robot.yaml (body_length x body_width, centred); set from config/ by robot_config().
        'footprint_m': [[0.205, 0.18], [0.205, -0.18], [-0.205, -0.18], [-0.205, 0.18]],
        'footprint_padding_m': 0.02,     # the footprint check (coverage/footprint.py) keeps this margin
    },
    'spiral': {
        'overlap_pct': 10.0,             # 0-50 %; path spacing = 2 x coverage radius x (1 - overlap)
        'obstacle_mode': 'around',       # 'around' = loops also circle obstacles, 'outer' = follow outer walls only
        'wall_side': 'right',            # side the wall is on while following it ('right' = counter-clockwise)
    },
    'strategy': {
        'mode': 'auto',                  # 'auto' = cheapest of all, 'regions' = mixed lanes/spiral per region, 'spiral'
        'wall_loop_first': True,         # start by following the outer walls
    },
    'cost': {                            # all terms in seconds, see coverage/cost.py
        'smooth_turn_deg': 20.0,         # bends below this are driven without stopping
        'bend_s_per_rad': 1.5,           # ...but still cost this much per radian (straight = cheapest)
        'stop_penalty_s': 1.0,           # extra time per sharp corner (brake, rotate, accelerate)
        'reverse_angle_deg': 150.0,      # direction change counted as a reversal (hairpin)
        'reverse_penalty_s': 10.0,       # extra cost per reversal
        'overlap_weight': 1.0,           # x time to sweep the double-covered area
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
    'optimizer': {
        'search_time_s': 4.0,            # route optimisation per candidate pattern
        'polish_time_s': 15.0,           # extra optimisation of the cheapest candidate
        'neighbours': 8,                 # nearby pieces tried per move
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
    hx, hy = float(robot['body_length']) / 2, float(robot['body_width']) / 2
    g = settings['geometry']
    g['footprint_m'] = [[hx, hy], [hx, -hy], [-hx, -hy], [-hx, hy]]
    padding = g['footprint_padding_m']
    coverage_file = os.path.join(config_dir, 'coverage.yaml')
    if os.path.isfile(coverage_file):
        with open(coverage_file) as f:
            cov = next(iter(yaml.safe_load(f).values()))['ros__parameters']['geometry']
        padding = g['footprint_padding_m'] = float(cov['planning_padding_m'])
        g['coverage_disk_radius_m'] = float(cov['coverage_disk_radius_m'])
        g['sensor_offset_m'] = float(cov['sensor_offset_m'])
    g['robot_radius_m'] = math.ceil((math.hypot(hx, hy) + padding + LOCALISATION_MARGIN_M) * 100 - 1e-9) / 100
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
