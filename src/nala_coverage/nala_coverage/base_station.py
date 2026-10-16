"""Base station: the pose where coverage starts and ends (no ROS). Stored as maps/<name>/base_station.yaml."""
import math
from pathlib import Path
import yaml

FILE_NAME = 'base_station.yaml'


def default_path(map_yaml):
    """The base station file next to the map."""
    return Path(map_yaml).resolve().parent / FILE_NAME


def load(path, map_frame):
    """Return (x, y, yaw) in the map frame, or raise ValueError naming the problem."""
    path = Path(path)
    if not path.is_file():
        raise ValueError(f'BASE_STATION_MISSING: {path} (pick it with app3_base_station_pc.launch.py)')
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict):
        raise ValueError(f'BASE_STATION_INVALID: {path}')
    if data.get('frame_id') != map_frame:
        raise ValueError(f"BASE_STATION_INVALID: frame_id is {data.get('frame_id')}, the map uses {map_frame}")
    try:
        pose = tuple(float(data[key]) for key in ('x', 'y', 'yaw'))
    except (KeyError, TypeError, ValueError):
        raise ValueError(f'BASE_STATION_INVALID: {path} needs x, y and yaw') from None
    if not all(math.isfinite(v) for v in pose):
        raise ValueError('BASE_STATION_INVALID: non-finite value')
    return pose


def save(path, pose, map_frame):
    x, y, yaw = (float(v) for v in pose)
    text = yaml.safe_dump({'frame_id': map_frame, 'x': round(x, 4), 'y': round(y, 4),
                           'yaw': round(math.remainder(yaw, 2 * math.pi), 4)}, sort_keys=False)
    Path(path).write_text('# Base station: coverage starts and ends here (map frame, m and rad).\n' + text)


def check(grid, pose, collision):
    """The robot must be able to turn on the spot at the base station."""
    if not grid.segment_safe(pose[:2], pose[:2], collision):
        raise ValueError(f'BASE_STATION_TOO_CLOSE: x={pose[0]:.2f}, y={pose[1]:.2f} is closer than '
                         f'{collision:.2f} m to a wall or unknown cell')
