"""Map library on the robot (no ROS, no Qt): maps made by app 2, their base station and saved plans.

maps/<name>/map.yaml + map.pgm      from app 2 (scripts/save_map.sh)
maps/<name>/base_station.yaml       set on the panel
maps/<name>/plans/<time>.yaml/.png  coverage_tool plans
maps/<name>/runs/<time>/            task reports of app 3
"""
from dataclasses import dataclass
import datetime
from pathlib import Path
import yaml
from nala_coverage import base_station
from nala_coverage.plan_file import check_base_station, load_plan

MAP_FILE = 'map.yaml'
PLANS = 'plans'
RUNS = 'runs'


@dataclass
class MapEntry:
    name: str
    folder: Path

    @property
    def map_yaml(self):
        return self.folder / MAP_FILE

    @property
    def station_file(self):
        return base_station.default_path(self.map_yaml)

    @property
    def image(self):
        return self.folder / yaml.safe_load(self.map_yaml.read_text())['image']


@dataclass
class PlanEntry:
    name: str
    path: Path
    data: dict

    @property
    def png(self):
        return self.path.with_suffix('.png')

    @property
    def stats(self):
        return self.data.get('stats') or {}

    def summary(self):
        s = self.stats
        parts = [self.name]
        if 'coverage_pct' in s:
            parts.append(f"{s['coverage_pct']:.0f} % of the floor")
        if 'drive_length_m' in s:
            parts.append(f"{s['drive_length_m']:.0f} m")
        if 'estimated_time_s' in s:
            parts.append(f"~{s['estimated_time_s'] / 60:.0f} min")
        if self.data.get('area_polygon'):
            parts.append('area')
        return ', '.join(parts)


def list_maps(maps_dir):
    """Every folder in maps_dir with a map.yaml, sorted by name."""
    maps_dir = Path(maps_dir)
    if not maps_dir.is_dir():
        return []
    return [MapEntry(p.name, p) for p in sorted(maps_dir.iterdir()) if (p / MAP_FILE).is_file()]


def load_station(entry, map_frame):
    """The base station pose (x, y, yaw), or None when it is not set yet."""
    if not entry.station_file.is_file():
        return None
    return base_station.load(entry.station_file, map_frame)


def save_station(entry, pose, grid, collision, map_frame):
    """Save the base station; raises ValueError if the robot cannot turn on the spot there."""
    base_station.check(grid, pose, collision)
    base_station.save(entry.station_file, pose, map_frame)


def list_plans(entry):
    """Saved plans of a map, newest first."""
    folder = entry.folder / PLANS
    if not folder.is_dir():
        return []
    plans = []
    for path in sorted(folder.glob('*.yaml'), reverse=True):
        try:
            data = yaml.safe_load(path.read_text())
        except yaml.YAMLError:
            data = None
        plans.append(PlanEntry(path.stem, path, data if isinstance(data, dict) else {}))
    return plans


def plan_status(entry, plan, grid, settings, station):
    """'' if the robot can drive the plan, else the reason it would refuse it (the supervisor's own checks)."""
    if station is None:
        return 'Set the base station first'
    geometry = settings['geometry']
    try:
        loaded = load_plan(plan.path, grid, entry.map_yaml, settings['runtime']['map_frame'], settings.collision,
                           sensor=(geometry['sensor_offset_m'], geometry['coverage_disk_radius_m']))
        check_base_station(loaded, station, settings['planning']['plan_join_tolerance_m'])
    except ValueError as error:
        return str(error)
    return ''


def new_name(now=None):
    """A file name from the date and time, e.g. 2026-10-08_143005."""
    return (now or datetime.datetime.now()).strftime('%Y-%m-%d_%H%M%S')


def new_plan_paths(entry, now=None):
    """(yaml, png) for a new plan."""
    folder = entry.folder / PLANS
    folder.mkdir(parents=True, exist_ok=True)
    name = new_name(now)
    return folder / (name + '.yaml'), folder / (name + '.png')


def delete_plan(plan):
    for path in (plan.path, plan.png):
        path.unlink(missing_ok=True)


def new_run_dir(entry, now=None):
    """Output folder for one drive (task report, launch log)."""
    folder = entry.folder / RUNS / new_name(now)
    folder.mkdir(parents=True, exist_ok=True)
    return folder
