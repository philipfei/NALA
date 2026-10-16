"""Headless coverage planner (cost-optimised).

python plan_cli.py maps/house1/map.yaml -o maps/house1/plan.yaml
The base station (maps/house1/base_station.yaml, next to the map) is recorded in the plan: the robot starts and
ends there. Coverage starts at the base station, or at --start X Y (the robot drives there first).
The robot outline comes from ../config/robot.yaml, the sensor from ../config/coverage.yaml (--config-dir)."""
import argparse

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from coverage import export, pipeline, render
from coverage.grid import load_ros_map
from coverage.settings import CONFIG_DIR, default_settings, load_settings, robot_config


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('map', help='ROS map .yaml')
    ap.add_argument('--start', nargs=2, type=float, metavar=('X', 'Y'),
                    help='where coverage starts (default: the base station)')
    ap.add_argument('--base-station', help='base_station.yaml (default: next to the map)')
    ap.add_argument('--config-dir', default=CONFIG_DIR, help='repo config/ folder with robot.yaml and coverage.yaml')
    ap.add_argument('--area', nargs='+', type=float, help='polygon x1 y1 x2 y2 ... (default: whole map)')
    ap.add_argument('--settings', help='settings .json (saved from the app)')
    ap.add_argument('--mode', choices=['auto', 'regions', 'spiral'], help='auto = cheapest of all patterns')
    ap.add_argument('--overlap', type=float, help='loop overlap in %% (0-50)')
    ap.add_argument('--obstacles', choices=['around', 'outer'], help='loop around obstacles or follow outer walls only')
    ap.add_argument('--robot-radius', type=float)
    ap.add_argument('--coverage-radius', type=float)
    ap.add_argument('--sensor-offset', type=float, help='sensor ahead of the robot centre (m)')
    ap.add_argument('-o', '--output', default='coverage_path.yaml', help='.yaml or .json')
    ap.add_argument('--png', help='also save a preview image')
    ap.add_argument('--cells-png', help='also save the boustrophedon cells to this PNG')
    ap.add_argument('--path-png', help='also save the robot path and coverage to this PNG')
    a = ap.parse_args()

    grid = load_ros_map(a.map)
    settings = load_settings(a.settings) if a.settings else default_settings()
    if not robot_config(settings, a.config_dir):
        print(f'Note: no robot.yaml in {a.config_dir}; using the built-in NALA outline')
    station_file = a.base_station or export.base_station_path(a.map)
    station = export.load_base_station(station_file)
    if station is None and (a.base_station or a.start is None):
        ap.error(f'no base station file {station_file}: set the base station first or give --start X Y')
    if station is not None:
        print(f'Base station: x={station[0]:.2f} y={station[1]:.2f} yaw={station[2]:.2f}')
    start = list(a.start) if a.start is not None else list(station[:2])
    print(f'Coverage starts at x={start[0]:.2f} y={start[1]:.2f}')
    if a.overlap is not None:
        settings['spiral']['overlap_pct'] = max(0.0, min(50.0, a.overlap))
    if a.mode:
        settings['strategy']['mode'] = a.mode
    if a.obstacles:
        settings['spiral']['obstacle_mode'] = a.obstacles
    if a.robot_radius:
        settings['geometry']['robot_radius_m'] = a.robot_radius
    if a.coverage_radius:
        settings['geometry']['coverage_disk_radius_m'] = a.coverage_radius
    if a.sensor_offset is not None:
        settings['geometry']['sensor_offset_m'] = a.sensor_offset
    polygon = None
    if a.area:
        if len(a.area) % 2 or len(a.area) < 6:
            ap.error('--area needs at least 3 x/y pairs')
        polygon = [list(a.area[i:i + 2]) for i in range(0, len(a.area), 2)]
    result = pipeline.plan(grid, start, polygon, settings, progress=print)
    n = export.save(result, a.output, a.map, settings, station)
    for k, v in result['stats'].items():
        print(f'  {k}: {v:.2f}' if isinstance(v, float) else f'  {k}: {v}')
    print('Candidates (cost in s):')
    for c in result['candidates']:
        print(f"  {c['total_s']:8.0f}  {c['name']}")
    print(f'Wrote {n} poses to {a.output}')
    for path, draw in ((a.cells_png, lambda ax: render.draw_cells(ax, grid, result['cells'])),
                       (a.path_png, lambda ax: render.draw_robot_path(ax, grid, result))):
        if path:
            fig, ax = plt.subplots(figsize=(10, 8))
            render.draw_map(ax, grid)
            draw(ax)
            fig.savefig(path, dpi=130, bbox_inches='tight')
            print(f'Wrote {path}')
    if a.png:
        fig, ax = plt.subplots(figsize=(10, 8))
        render.draw_map(ax, grid)
        render.draw_polygon(ax, polygon)
        render.draw_result(ax, grid, result)
        fig.savefig(a.png, dpi=130, bbox_inches='tight')
        print(f'Wrote {a.png}')


if __name__ == '__main__':
    main()
