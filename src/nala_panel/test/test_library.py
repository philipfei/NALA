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
