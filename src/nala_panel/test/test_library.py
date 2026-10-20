"""Map library: maps, base station, plans and whether the robot would drive a plan (no ROS, no Qt)."""
import datetime
import pytest
import yaml
from nala_coverage.geometry import Grid
from nala_panel import library


def room(maps_dir):
    [entry] = library.list_maps(maps_dir)
    return entry


def test_only_folders_with_a_map_are_listed(maps_dir):
    assert [m.name for m in library.list_maps(maps_dir)] == ['room']
    assert library.list_maps(maps_dir / 'missing') == []
    assert room(maps_dir).image.name == 'map.pgm'


def test_plans_are_listed_newest_first_with_a_summary(maps_dir):
    entry = room(maps_dir)
    older = entry.folder / 'plans' / '2026-10-01_090000.yaml'
    older.write_text((entry.folder / 'plans' / '2026-10-08_120000.yaml').read_text())
    plans = library.list_plans(entry)
    assert [p.name for p in plans] == ['2026-10-08_120000', '2026-10-01_090000']
    assert '% of the floor' in plans[0].summary() and ' m' in plans[0].summary()


def test_example_plan_is_drivable(maps_dir, settings):
    entry = room(maps_dir)
    station = library.load_station(entry, 'map')
    [plan] = library.list_plans(entry)
    assert library.plan_status(entry, plan, Grid.load(entry.map_yaml), settings, station) == ''


def test_plan_is_refused_without_base_station_or_after_it_moved(maps_dir, settings):
    entry = room(maps_dir)
    grid = Grid.load(entry.map_yaml)
    [plan] = library.list_plans(entry)
    assert 'base station' in library.plan_status(entry, plan, grid, settings, None)
    station = library.load_station(entry, 'map')
    moved = (station[0] + 0.5, station[1], station[2])
    assert 'PLAN_BASE_MISMATCH' in library.plan_status(entry, plan, grid, settings, moved)


def test_plan_is_refused_for_another_map_image_or_sensor(maps_dir, settings):
    entry = room(maps_dir)
    grid = Grid.load(entry.map_yaml)
    station = library.load_station(entry, 'map')
    [plan] = library.list_plans(entry)
    data = yaml.safe_load(plan.path.read_text())
    data['settings']['geometry']['sensor_offset_m'] = 0.1
    plan.path.write_text(yaml.safe_dump(data))
    assert 'PLAN_ROBOT_MISMATCH' in library.plan_status(entry, plan, grid, settings, station)
    data['settings']['geometry']['sensor_offset_m'] = settings['geometry']['sensor_offset_m']
    data['map_image_sha256'] = '0' * 64
    plan.path.write_text(yaml.safe_dump(data))
    assert 'PLAN_MAP_MISMATCH' in library.plan_status(entry, plan, grid, settings, station)


def test_base_station_is_saved_only_where_the_robot_can_turn(maps_dir, settings):
    entry = room(maps_dir)
    grid = Grid.load(entry.map_yaml)
    entry.station_file.unlink()
    assert library.load_station(entry, 'map') is None
    library.save_station(entry, (1.157, -1.875, 0.5), grid, settings.collision, 'map')
    assert library.load_station(entry, 'map') == pytest.approx((1.157, -1.875, 0.5))
    with pytest.raises(ValueError, match='BASE_STATION_TOO_CLOSE'):
        library.save_station(entry, (-1.1, -4.7, 0.), grid, settings.collision, 'map')


def test_new_plan_and_run_paths_and_delete(maps_dir):
    entry = room(maps_dir)
    now = datetime.datetime(2026, 10, 8, 14, 30, 5)
    yaml_path, png_path = library.new_plan_paths(entry, now)
    assert yaml_path.name == '2026-10-08_143005.yaml' and png_path.name == '2026-10-08_143005.png'
    assert library.new_run_dir(entry, now) == entry.folder / 'runs' / '2026-10-08_143005'
    [plan] = library.list_plans(entry)
    plan.png.write_bytes(b'png')
    library.delete_plan(plan)
    assert library.list_plans(entry) == [] and not plan.png.exists()


def make_run(entry, name, fraction, state, heat=True):
    """A run folder as app 3 writes it: task report, coverage report, optionally the heatmap."""
    import json
    import numpy as np
    folder = entry.folder / 'runs' / name
    folder.mkdir(parents=True)
    (folder / 'task_x.json').write_text(json.dumps({'state': state, 'reason': 'RETURNED_TO_BASE'}))
    (folder / 'coverage_x.json').write_text(json.dumps({'fraction': fraction, 'covered_indices': [5, 6, 7],
        'actual_trajectory': [[0., 1., 2., 0.], [.1, 1.1, 2., 0.]]}))
    if heat:
        shape = (10, 10)
        np.savez_compressed(folder / 'heatmap.npz', value=np.full(shape, np.nan), keepout=np.zeros(shape, bool),
                            threshold=np.nan, background=30.)
    return folder


def test_all_previous_scans_are_listed_newest_first_and_load(maps_dir):
    [entry] = library.list_maps(maps_dir)
    make_run(entry, '2026-10-19_101500', .95, 'CANCELED', heat=False)
    make_run(entry, '2026-10-20_103019', .998, 'FINISHED')
    (entry.folder / 'runs' / '2026-10-20_120000').mkdir()           # started, but app 3 wrote nothing: not a scan
    runs = library.list_runs(entry)
    assert [r['name'] for r in runs] == ['2026-10-20_103019', '2026-10-19_101500']
    assert library.run_summary(runs[0]) == '20-10-2026 10:30\n100 % covered, finished'
    assert library.run_summary(runs[1]) == '19-10-2026 10:15\n95 % covered, cancelled'
    old = library.load_run(runs[1]['folder'])
    assert old['value'] is None and old['covered'] == [5, 6, 7] and old['trail'] == [[[1., 2.], [1.1, 2.]]]
    assert library.load_run(runs[0]['folder'])['background'] == 30.



def test_plan_with_a_drive_in_closer_to_a_wall_than_the_turning_circle_is_drivable(maps_dir, settings):
    # Robot 2026-10-20: the screen refused the test_basement2 plan with drive-ins (spurs) as "too close to
    # obstacles": its check used the turning circle for them too, not the outline like the robot.
    import math
    entry = room(maps_dir)
    grid = Grid.load(entry.map_yaml)
    station = library.load_station(entry, 'map')
    [plan] = library.list_plans(entry)
    data = yaml.safe_load(plan.path.read_text())
    drive = data['drive_path']
    g = settings['geometry']
    p = drive['points'][-1]
    found = None
    for k in range(16):                                   # straight on from the end towards a wall
        yaw = k * math.pi / 8
        d = 0.05
        while True:
            q = [p[0] + (d + 0.05) * math.cos(yaw), p[1] + (d + 0.05) * math.sin(yaw)]
            swept = grid.outline_swept(p, q, yaw, g['footprint_m'], g['planning_padding_m'])
            if swept is None or (swept & ~grid.free).any():
                break
            d += 0.05
        q = [p[0] + d * math.cos(yaw), p[1] + d * math.sin(yaw)]
        if d > 0.05 and not grid.segment_safe(p, q, settings.collision):
            found = q
            break
    assert found, 'no wall within reach of the plan end'
    drive['points'] += [found, list(p)]
    drive['kinds'] = (drive.get('kinds') or ['wall'] * (len(drive['points']) - 2)) + ['spur', 'reverse']
    n = len(drive['points'])
    drive['stops'] = sorted(set(drive.get('stops', [])) | {n - 3, n - 2})
    plan.path.write_text(yaml.safe_dump(data))
    assert library.plan_status(entry, plan, grid, settings, station) == ''
