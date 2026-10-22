"""Map library on the robot (no ROS, no Qt): maps made by app 2, their base station and saved plans.

maps/<name>/map.yaml + map.pgm      from app 2 (scripts/save_map.sh)
maps/<name>/base_station.yaml       set on the panel
maps/<name>/plans/<time>.yaml/.png  coverage_tool plans
maps/<name>/runs/<time>/            task reports of app 3
maps/contamination_threshold.yaml   the operator's contamination threshold (Settings screen), for every map
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
THRESHOLD_FILE = 'contamination_threshold.yaml'


def load_threshold(maps_dir, default):
    """The operator's contamination threshold {'mode', 'value'}, or `default` (config/coverage.yaml) if none is saved."""
    from nala_coverage.radiation import valid_setting
    path = Path(maps_dir) / THRESHOLD_FILE
    try:
        setting = yaml.safe_load(path.read_text())
    except (OSError, yaml.YAMLError):
        return dict(default)
    return {'mode': setting['mode'], 'value': float(setting['value'])} if valid_setting(setting) else dict(default)


def save_threshold(maps_dir, setting):
    Path(maps_dir).mkdir(parents=True, exist_ok=True)
    (Path(maps_dir) / THRESHOLD_FILE).write_text(yaml.safe_dump({'mode': setting['mode'],
                                                                'value': float(setting['value'])}))


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

    def summary(self, settings=None, station=None):
        """One line for the plan list; with settings (config/) also the time the robot needs to drive it."""
        s = self.stats
        parts = [self.name]
        if 'coverage_pct' in s:
            parts.append(f"{s['coverage_pct']:.0f} % of the floor")
        if 'drive_length_m' in s:
            parts.append(f"{s['drive_length_m']:.0f} m")
        t = plan_drive_time(self.data, settings, station) if settings is not None else None
        if t is not None:
            parts.append(f'~{t / 60:.0f} min')
        if self.data.get('area_polygon'):
            parts.append('area')
        return ', '.join(parts)


def plan_drive_time(data, settings, station):
    """Seconds the robot needs to drive a plan and get back to the base station, with today's speeds and acceleration
    limits (user 2026-10-22: the screen shows the real driving time, never the planner's cost), or None."""
    from nala_coverage.plan_file import drive_time_s
    return drive_time_s(data, settings['motion'], float(settings['spur']['speed_m_s']), station)


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
                           sensor=(geometry['sensor_offset_m'], geometry['coverage_disk_radius_m']),
                           outline=geometry['footprint_m'], margin=geometry['planning_padding_m'])
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
    # .check.npz: the robot's saved plan check (supervisor.coverable_cached)
    for path in (plan.path, plan.png, plan.path.with_suffix('.check.npz')):
        path.unlink(missing_ok=True)


def new_run_dir(entry, now=None):
    """Output folder for one drive (task report, launch log)."""
    folder = entry.folder / RUNS / new_name(now)
    folder.mkdir(parents=True, exist_ok=True)
    return folder


def list_runs(entry):
    """Every scan of a map, newest first: [{'name', 'folder', 'when', 'fraction', 'state', 'reason'}]. A run folder
    counts when app 3 wrote a task report or coverage into it (user, 2026-10-20: browse all previous scans)."""
    import json
    runs_dir = entry.folder / RUNS
    out = []
    for folder in sorted((p for p in runs_dir.glob('*') if p.is_dir()), reverse=True) if runs_dir.is_dir() else []:
        tasks, covers = sorted(folder.glob('task_*.json')), sorted(folder.glob('coverage_*.json'))
        if not tasks and not covers:
            continue
        info = {'name': folder.name, 'folder': folder, 'fraction': None, 'state': '', 'reason': ''}
        try:
            info['when'] = datetime.datetime.strptime(folder.name, '%Y-%m-%d_%H%M%S')
        except ValueError:
            info['when'] = None
        try:
            if covers:
                info['fraction'] = float(json.loads(covers[-1].read_text()).get('fraction'))
            if tasks:
                task = json.loads(tasks[-1].read_text())
                info['state'], info['reason'] = task.get('state', ''), task.get('reason', '')
        except (OSError, ValueError, TypeError):
            pass
        out.append(info)
    return out


def run_summary(info):
    """One line for the list of scans."""
    when = info['when'].strftime('%d-%m-%Y %H:%M') if info['when'] else info['name']
    cov = f"{100 * info['fraction']:.0f} % covered" if info['fraction'] is not None else 'no coverage data'
    ended = {'FINISHED': 'finished', 'CANCELED': 'cancelled', 'FAILED': 'failed', 'PAUSED': 'stopped while paused'}
    if info.get('reason') == 'CONTAMINATION_FOUND':
        return f"{when}\n{cov}, CONTAMINATION FOUND"
    return f"{when}\n{cov}, {ended.get(info['state'], info['state'].lower() or 'unknown')}"


def last_run(entry):
    """The newest run of a map with radiation data: heatmap, hotspots and the actual trail, or None."""
    runs = [r for r in list_runs(entry) if (r['folder'] / 'heatmap.npz').is_file()]
    return load_run(runs[0]['folder']) if runs else None


def load_run(folder):
    """One run's results: heatmap, hotspots, the actual trail, the covered floor, map changes and where contamination
    ended the scan (or None)."""
    import json
    import numpy as np
    folder = Path(folder)
    value = None
    threshold, background = float('nan'), float('nan')
    if (folder / 'heatmap.npz').is_file():
        data = np.load(folder / 'heatmap.npz')
        value = data['value']
        threshold, background = float(data['threshold']), float(data['background'])
    hotspots = json.loads((folder / 'hotspots.json').read_text()) if (folder / 'hotspots.json').is_file() else {}
    trail = []                     # segments of trusted measured poses; no line across a gap in time
    covered = None                 # flat indices of the floor the sensor covered (the coverage meter)
    fraction = None
    for path in folder.glob('coverage_*.json'):
        report = json.loads(path.read_text())
        gap = report.get('effective_config', {}).get('measurement', {}).get('max_sample_gap_s', 0.3)
        last = None
        for stamp, x, y, *_ in report.get('actual_trajectory') or []:
            if last is None or stamp - last > gap:
                trail.append([])
            trail[-1].append([x, y])
            last = stamp
        covered = report.get('covered_indices')
        fraction = report.get('fraction')
    contamination = None                         # the supervisor's task report: contamination found
    tasks = sorted(folder.glob('task_*.json'))
    if tasks:
        try:
            contamination = json.loads(tasks[-1].read_text()).get('contamination')
        except (OSError, ValueError):
            pass
    appeared = vanished = None                   # map changes seen during the run (mapchange_node)
    if (folder / 'changes.npz').is_file():
        changes = np.load(folder / 'changes.npz')
        appeared, vanished = changes['appeared'], changes['vanished']
    return {'name': folder.name, 'folder': folder, 'value': value, 'contamination': contamination,
            'threshold': threshold if np.isfinite(threshold) else None,
            'background': background, 'hotspots': hotspots.get('hotspots', []), 'trail': trail,
            'covered': covered, 'fraction': fraction, 'appeared': appeared, 'vanished': vanished}
