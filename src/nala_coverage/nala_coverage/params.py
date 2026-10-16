"""Load, validate, canonicalize, and render NALA configuration."""
from copy import deepcopy
from decimal import Decimal
from pathlib import Path
import hashlib
import json
import math
import os
import tempfile
import yaml

# robot.yaml belongs to the robot repo (robot model). The other three are the stage-3 files.
CONFIG_FILES = ('robot.yaml', 'coverage.yaml', 'nav2_params.yaml', 'localization.yaml')


def default_config_dir():
    """The repo config/ folder: from the current directory, the source tree, or nala_bringup's install."""
    for candidate in (Path.cwd() / 'config', Path(__file__).resolve().parents[3] / 'config'):
        if (candidate / 'robot.yaml').is_file():
            return candidate
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('nala_bringup')) / 'config'
    except (ImportError, LookupError):
        raise ValueError('No config directory with robot.yaml found; pass --config-dir explicitly') from None



def resource(name):
    """Return a source-tree configuration path for tests and tooling."""
    return default_config_dir() / name


def _read(path):
    with Path(path).open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f'{path}: expected a mapping')
    return data


def _block(document, selector, path):
    unknown = set(document) - {selector}
    if unknown:
        raise ValueError(f'{path}: unknown node selectors: {sorted(unknown)}')
    try:
        block = document[selector]
        if set(block) != {'ros__parameters'} or not isinstance(block['ros__parameters'], dict):
            raise KeyError
        return deepcopy(block['ros__parameters'])
    except (KeyError, TypeError):
        raise ValueError(f'{path}: expected {selector}/ros__parameters') from None


def _float_token(value):
    if not math.isfinite(value):
        raise ValueError('Configuration contains NaN or infinity')
    return value.hex()


def canonical_value(value):
    if isinstance(value, dict):
        return {str(k): canonical_value(value[k]) for k in sorted(value, key=str)}
    if isinstance(value, (list, tuple)):
        return [canonical_value(v) for v in value]
    if isinstance(value, bool) or value is None or isinstance(value, str):
        return value
    if isinstance(value, (int, float)):
        return {'__number_hex__': _float_token(float(value))}
    raise TypeError(f'Unsupported canonical value type: {type(value).__name__}')


def canonical_bytes(value):
    return json.dumps(canonical_value(value), sort_keys=True, separators=(',', ':'),
                      ensure_ascii=True, allow_nan=False).encode('utf-8')


def canonical_hash(value):
    return hashlib.sha256(canonical_bytes(value)).hexdigest()


def file_hash(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


_REQUIRED = {
    'geometry': {'planning_padding_m', 'coverage_disk_radius_m', 'sensor_offset_m'},
    'motion': {'linear_m_s', 'angular_rad_s', 'linear_accel_m_s2', 'angular_accel_rad_s2',
               'manual_forward_m_s', 'manual_reverse_m_s', 'manual_angular_rad_s'},
    'runtime': {'map_frame', 'odom_frame', 'base_frame', 'map_yaml', 'base_station_yaml', 'output_dir'},
    'coverage': {'planning', 'localization', 'health', 'measurement', 'battery', 'deadlines',
                 'recovery', 'visualization'},
}
BEHAVIOR = ('geometry', 'motion', 'planning', 'localization', 'health', 'measurement', 'battery',
            'deadlines', 'recovery', 'visualization')


def robot_footprint(robot):
    """The outer box of the robot repo's robot.yaml (body_length x body_width), centred on base_link."""
    try:
        half_x, half_y = float(robot['body_length']) / 2, float(robot['body_width']) / 2
    except (KeyError, TypeError, ValueError):
        raise ValueError('robot.yaml: body_length and body_width are required') from None
    if not (half_x > 0 and half_y > 0):
        raise ValueError('robot.yaml: body_length and body_width must be positive')
    return [[half_x, half_y], [half_x, -half_y], [-half_x, -half_y], [-half_x, half_y]]


def footprint(points):
    """The robot outline in the base frame (x forward, y left) as [[x, y], ...]; it must enclose the base origin."""
    if not isinstance(points, list) or len(points) < 3:
        raise ValueError('geometry.footprint_m needs at least three [x, y] corners')
    out = []
    for p in points:
        if not isinstance(p, list) or len(p) != 2 or not all(
                isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in p):
            raise ValueError('geometry.footprint_m corners must be finite [x, y] pairs')
        out.append([float(p[0]), float(p[1])])
    area = sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(out, out[1:] + out[:1])) / 2
    inside = False
    for (x0, y0), (x1, y1) in zip(out, out[1:] + out[:1]):
        if (y0 > 0) != (y1 > 0) and 0 < (x1 - x0) * (0 - y0) / (y1 - y0) + x0:
            inside = not inside
    if abs(area) < 1e-6 or not inside:
        raise ValueError('geometry.footprint_m must be a polygon around the base frame origin')
    return out


def _exact_keys(data, expected, name):
    missing = expected - set(data)
    unknown = set(data) - expected
    if missing or unknown:
        raise ValueError(f'{name}: missing={sorted(missing)} unknown={sorted(unknown)}')


def _positive_tree(section, values):
    for key, value in values.items():
        if isinstance(value, bool) or value is None or isinstance(value, str):
            continue
        if isinstance(value, (int, float)) and (not math.isfinite(value) or value <= 0):
            raise ValueError(f'{section}.{key} must be positive')


class Settings:
    """Validated effective values with the collision radius derived exactly once."""
    def __init__(self, values, config_dir):
        self.values = values
        self.config_dir = Path(config_dir).resolve()
        geometry = values['geometry']
        corners = footprint(geometry['footprint_m'])
        # The robot turns on the spot anywhere on its path, so it needs the circle through its farthest corner.
        self.circumscribed = max(math.hypot(x, y) for x, y in corners)
        self.collision = float(Decimal(repr(self.circumscribed)) + Decimal(str(geometry['planning_padding_m'])))
        if geometry['planning_padding_m'] < 0:
            raise ValueError('Invalid planning padding')
        if geometry['coverage_disk_radius_m'] <= 0:
            raise ValueError('Invalid coverage disk radius')
        if not 0 <= geometry['sensor_offset_m'] < geometry['coverage_disk_radius_m']:
            raise ValueError('geometry.sensor_offset_m must be at least 0 and below the coverage disk radius')
        for section in ('motion', 'localization', 'health', 'measurement', 'deadlines',
                        'recovery', 'visualization'):
            _positive_tree(section, values[section])
        battery = values['battery']
        if not isinstance(battery.get('enabled'), bool):
            raise ValueError('battery.enabled must be true or false')
        if not 0 < battery['critical_ratio'] < battery['start_ratio'] <= 1:
            raise ValueError('Battery thresholds must be ordered')
        self.hash = canonical_hash(self.behavior_values())

    def __getitem__(self, key):
        return self.values[key]

    def behavior_values(self):
        return {key: self.values[key] for key in BEHAVIOR}

    def render_nav2(self, map_yaml='', directory=None, stamped_supported=True):
        nav2 = deepcopy(self.values['nav2'])
        runtime = self.values['runtime']
        motion = self.values['motion']
        # Global costmap (planner, detours): the circle the robot turns in, so a planned detour keeps the same
        # clearance as the plan (NavFn plans a point and keeps only the inscribed radius of a polygon).
        # Local costmap (the controller's collision check ahead): the real outline without padding. A plan keeps
        # the centre 0.33 m from walls, so a corner turning on the spot passes them at about 5 cm; padding would leave
        # less than one costmap cell and the controller would report false collisions next to walls.
        outline = '[' + ', '.join(f'[{x:g}, {y:g}]' for x, y in footprint(self.values['geometry']['footprint_m'])) + ']'
        for name in ('global_costmap', 'local_costmap'):
            params = nav2[name][name]['ros__parameters']
            params.pop('robot_radius', None)
            params.pop('footprint', None)
            if name == 'global_costmap':
                # NavFn keeps the centre out of robot_radius around obstacle cell centres; the supervisor checks
                # its paths exactly against the cell edges. Half a cell more makes NavFn paths pass that check.
                params['robot_radius'] = self.collision + params['resolution'] / 2
                params['footprint_padding'] = 0.0
            else:
                params['footprint'] = outline
                params['footprint_padding'] = 0.0
            params['robot_base_frame'] = runtime['base_frame']
        nav2['amcl']['ros__parameters'].update(
            base_frame_id=runtime['base_frame'], odom_frame_id=runtime['odom_frame'],
            global_frame_id=runtime['map_frame'])
        nav2['map_server']['ros__parameters'].update(
            frame_id=runtime['map_frame'], yaml_filename=str(Path(map_yaml).resolve()))
        nav2['global_costmap']['global_costmap']['ros__parameters']['global_frame'] = runtime['map_frame']
        nav2['local_costmap']['local_costmap']['ros__parameters']['global_frame'] = runtime['odom_frame']
        controller = nav2['controller_server']['ros__parameters']
        # Below speed / turn rate the controller slows down instead of asking for more turn rate than the
        # velocity gate passes; a clipped turn rate would widen the arc and leave the exact path.
        # Autodrive is forward only: no reversing, a goal behind the robot is turned towards on the spot.
        controller['FollowPath'].update(
            desired_linear_vel=motion['linear_m_s'],
            rotate_to_heading_angular_vel=motion['angular_rad_s'],
            max_angular_accel=motion['angular_accel_rad_s2'],
            use_regulated_linear_velocity_scaling=True,
            regulated_linear_scaling_min_radius=motion['linear_m_s'] / motion['angular_rad_s'],
            allow_reversing=False, use_rotate_to_heading=True)
        if stamped_supported:
            controller['enable_stamped_cmd_vel'] = False
        else:
            controller.pop('enable_stamped_cmd_vel', None)
        target = Path(directory) if directory else Path(tempfile.mkdtemp(prefix='nala-nav2-'))
        target.mkdir(parents=True, exist_ok=True)
        output = target / f'nav2-{self.hash}.yaml'
        output.write_text(yaml.safe_dump(nav2, sort_keys=False), encoding='utf-8')
        return output


def load_settings(config_dir=None, runtime_overrides=None):
    config_dir = Path(config_dir or default_config_dir()).resolve()
    missing = [name for name in CONFIG_FILES if not (config_dir / name).is_file()]
    if missing:
        raise ValueError(f'Missing configuration files in {config_dir}: {missing}')
    coverage = _block(_read(config_dir / 'coverage.yaml'), '/coverage_config', 'coverage.yaml')
    _exact_keys(coverage, {'geometry', 'motion', 'runtime'} | _REQUIRED['coverage'], 'coverage.yaml')
    for key in ('geometry', 'motion', 'runtime'):
        _exact_keys(coverage[key], _REQUIRED[key], f'coverage.yaml:{key}')
    values = deepcopy(coverage)
    # The outline is derived here and never configured twice.
    values['geometry']['footprint_m'] = robot_footprint(_read(config_dir / 'robot.yaml'))
    nav2 = _read(config_dir / 'nav2_params.yaml')
    localization = _read(config_dir / 'localization.yaml')
    overlap = set(nav2) & set(localization)
    if overlap:
        raise ValueError(f'nav2_params.yaml and localization.yaml both configure {sorted(overlap)}')
    values['nav2'] = {**localization, **nav2}
    if runtime_overrides:
        unknown = set(runtime_overrides) - _REQUIRED['runtime']
        if unknown:
            raise ValueError(f'Unknown runtime overrides: {sorted(unknown)}')
        values['runtime'].update(runtime_overrides)
    return Settings(values, config_dir)


def node_settings(node):
    from rclpy.parameter import Parameter
    # No default: a node started without config_dir fails at once (the launch files pass it).
    config_dir = node.declare_parameter('config_dir', Parameter.Type.STRING).value
    base = load_settings(config_dir)
    overrides = {}
    for key in ('map_yaml', 'base_station_yaml', 'output_dir'):
        overrides[key] = node.declare_parameter(key, base['runtime'][key]).value
    return load_settings(config_dir, overrides)
