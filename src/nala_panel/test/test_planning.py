"""Planning from the panel: input checks, the plan_cli command, a real run on a small map, cancel (no Qt)."""
import time
import numpy as np
from PIL import Image
import pytest
import yaml
from nala_coverage.geometry import Grid
from nala_panel import library, planning
from conftest import REPO


@pytest.fixture
def small_map(tmp_path):
    """A 3 x 2.5 m empty room with its base station: plans in seconds."""
    folder = tmp_path / 'maps' / 'small'
    folder.mkdir(parents=True)
    a = np.full((50, 60), 254, np.uint8)
    a[[0, -1], :] = 0
    a[:, [0, -1]] = 0
    Image.fromarray(a).save(folder / 'map.pgm')
    (folder / 'map.yaml').write_text('image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\nnegate: 0\n'
                                     'occupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n')
    (folder / 'base_station.yaml').write_text('frame_id: map\nx: 0.6\ny: 0.6\nyaw: 0.0\n')
    return library.MapEntry('small', folder)


def wait(run, timeout):
    end = time.monotonic() + timeout
    while not run.done() and time.monotonic() < end:
        time.sleep(0.2)
    return run.done()


def test_inputs_are_checked_before_planning(small_map, settings):
    grid = Grid.load(small_map.map_yaml)
    c = settings.collision
    assert planning.check_inputs(grid, c, None, None) == ''
    assert planning.check_inputs(grid, c, (1.5, 1.2), [[1, 1], [2, 1], [2, 2]]) == ''
    assert 'wall' in planning.check_inputs(grid, c, (0.1, 1.2), None)
    assert 'Area' in planning.check_inputs(grid, c, None, [[1, 1], [2, 2], [2, 1], [1, 2]])   # crosses itself


def test_command_starts_at_the_base_station_or_the_start_point(small_map):
    out = small_map.folder / 'plans' / 'p.yaml'
    cmd = planning.command(REPO, small_map.map_yaml, small_map.station_file, out, out.with_suffix('.png'))
    assert cmd[2].endswith('coverage_tool/plan_cli.py') and '--start' not in cmd and '--area' not in cmd
    assert cmd[cmd.index('--base-station') + 1] == str(small_map.station_file)
    assert cmd[cmd.index('--config-dir') + 1] == str(REPO / 'config')
    cmd = planning.command(REPO, small_map.map_yaml, small_map.station_file, out, out.with_suffix('.png'),
                           start=[1.5, 1.2], area=[[1, 1], [2, 1], [2, 2]])
    i = cmd.index('--start')
    assert cmd[i + 1:i + 3] == ['1.5000', '1.2000']
    i = cmd.index('--area')
    assert [float(v) for v in cmd[i + 1:i + 7]] == [1, 1, 2, 1, 2, 2]


def test_a_plan_from_another_start_point_records_the_base_station(small_map):
    out_yaml, out_png = library.new_plan_paths(small_map)
    cmd = planning.command(REPO, small_map.map_yaml, small_map.station_file, out_yaml, out_png, start=[2.2, 1.6])
    run = planning.PlanRun(cmd, out_yaml, out_png)
    assert wait(run, 240), 'planning did not finish'
    output = '\n'.join(run.lines())
    assert run.ok(), output
    data = yaml.safe_load(out_yaml.read_text())
    assert data['base_station'] == {'x': 0.6, 'y': 0.6, 'yaw': 0.0}
    assert data['start'] == pytest.approx([2.2, 1.6], abs=0.06)
    assert out_png.is_file() and all(ok for ok, _ in planning.check_results(run.stats()))


def test_cancel_stops_planning_and_removes_its_files(small_map):
    out_yaml, out_png = library.new_plan_paths(small_map)
    cmd = planning.command(REPO, small_map.map_yaml, small_map.station_file, out_yaml, out_png)
    run = planning.PlanRun(cmd, out_yaml, out_png)
    time.sleep(1.0)
    run.cancel()
    assert run.done() and not run.ok() and not out_yaml.exists() and not out_png.exists()


def test_missing_statistics_count_as_failed_checks():
    assert planning.check_results({}) == [(False, text) for _, _, text in planning.CHECKS]
    good = {'coverage_of_reachable_pct': 100.0, 'footprint_hits': 0, 'drive_unsafe_segments': 0, 'drive_lost_cells': 0}
    assert all(ok for ok, _ in planning.check_results(good))
