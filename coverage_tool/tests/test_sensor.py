"""Unit tests for the sensor offset and the outline check. Run from the project root:

    python3 -m pytest tests/test_sensor.py
"""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from coverage import footprint, planning  # noqa: E402
from coverage.cost import piece_cells  # noqa: E402
from coverage.grid import Grid  # noqa: E402
from coverage.sensor import point_radius, sensor_path  # noqa: E402

BOX = [[0.30, 0.18], [0.30, -0.18], [-0.18, -0.18], [-0.18, 0.18]]


def room(size=80, res=0.05):
    a = np.zeros((size, size), np.int8)
    a[[0, -1], :] = 100
    a[:, [0, -1]] = 100
    return Grid(a, res)


def test_straight_segment_is_shifted_forward():
    assert np.allclose(sensor_path([[0, 0], [1, 0]], 0.26), [[0.26, 0], [1.26, 0]])
    assert np.allclose(sensor_path([[1, 0], [0, 0]], 0.26), [[0.74, 0], [-0.26, 0]])
    assert sensor_path([[0, 0], [1, 0]], 0.0) == [[0, 0], [1, 0]]


def test_corner_swings_the_sensor_on_an_arc_around_the_corner():
    pts = np.asarray(sensor_path([[0, 0], [1, 0], [1, 1]], 0.26))
    assert np.allclose(pts[0], [0.26, 0]) and np.allclose(pts[-1], [1, 1.26])
    arc = pts[1:-1]
    assert np.allclose(np.linalg.norm(arc - [1, 0], axis=1), 0.26)
    assert np.allclose(arc[0], [1.26, 0]) and np.allclose(arc[-1], [1, 0.26])


def test_gentle_bends_stay_close_to_the_true_sensor_path():
    r, n = 1.0, 60
    base = [[r * math.cos(t), r * math.sin(t)] for t in np.linspace(0, math.pi / 2, n)]
    pts = np.asarray(sensor_path(base, 0.26))
    # On a circle of radius r driven forwards, the sensor runs on a circle of radius sqrt(r^2 + d^2);
    # driving the chords puts it within a few millimetres of that.
    assert np.allclose(np.linalg.norm(pts, axis=1), math.hypot(r, 0.26), atol=1e-2)


def test_coverage_depends_on_driving_direction():
    g = room()
    region = g.free.copy()
    fwd = set(piece_cells(g, region, [[1.0, 2.0], [2.0, 2.0]], 0.3, 0.26))
    rev = set(piece_cells(g, region, [[2.0, 2.0], [1.0, 2.0]], 0.3, 0.26))
    ahead, behind = g.cell([2.5, 2.0]), g.cell([0.5, 2.0])
    flat = lambda rc: rc[0] * g.shape[1] + rc[1]
    assert flat(ahead) in fwd and flat(ahead) not in rev
    assert flat(behind) in rev and flat(behind) not in fwd


def test_single_pose_counts_only_what_any_heading_covers():
    g = room()
    t = planning.poly_target([[2.0, 2.0], [2.0, 2.0]], 'gap')
    t.points = [[2.0, 2.0], [2.0, 2.0]]
    mask = planning.stroke_union(g, g.free, [t], 0.6, 0.26)
    r = point_radius(0.6, 0.26)
    xy = g.world(np.argwhere(mask))
    assert np.linalg.norm(xy - [2.0, 2.0], axis=1).max() <= r + 1e-9


def test_outline_check_finds_a_wall_and_passes_open_floor():
    g = room()
    assert footprint.hits(g, [[2.0, 2.0], [3.0, 2.0], [3.0, 3.0]], BOX, 0.02)[0] == 0
    # Driving towards the wall until the centre is 0.25 m from it: the front (0.30 m) hits.
    assert footprint.hits(g, [[2.0, 2.0], [3.7, 2.0]], BOX, 0.0)[0] > 0
    # Turning on the spot 0.30 m from a wall: the front corners (0.35 m) hit, the sides do not.
    assert footprint.hits(g, [[2.0, 0.35 + 0.05], [2.5, 0.35 + 0.05]], BOX, 0.0)[0] == 0
    assert footprint.hits(g, [[2.0, 0.30], [2.5, 0.30], [2.5, 1.0]], BOX, 0.0)[0] > 0


def test_route_optimiser_counts_each_piece_in_its_driving_direction():
    from coverage import localsearch, pipeline
    from coverage.grid import load_ros_map
    from coverage.settings import default_settings, set_settings
    g = load_ros_map(str(pathlib.Path(__file__).resolve().parents[1] / 'examples' / 'office.yaml'))
    s = default_settings()
    set_settings(s)
    reach, start = g.reachable_from([2, 2], s.collision)
    area = [[-100, -100], [100, -100], [100, 100], [-100, 100]]
    den = pipeline.coverage_region(g, reach, area)
    cov = pipeline.brush_reachable(g, den, pipeline.sensor_reachable(g, reach, s), 0.6)
    targets, _ = pipeline._spiral_candidate(g, reach, area, start, s, 'around')
    opt = localsearch.RouteOptimizer(g, localsearch.pieces_from_targets(targets), start, reach, den, cov, s, 8)
    opt.run(3.0)
    rng = np.random.default_rng(0)
    for _ in range(5):                               # random reversals: flips pieces for certain
        opt.perturb(rng)
    assert any(rv for _, rv in opt.seq)
    ref = np.zeros_like(opt.count)
    for i, rv in opt.seq:
        ref[opt.pieces[i].cells(rv)] += 1
    assert np.array_equal(ref, opt.count)


def test_default_outline_and_radius_come_from_the_robot_repo_config():
    import yaml
    from coverage.settings import CONFIG_DIR, LOCALISATION_MARGIN_M, default_settings
    robot_file = pathlib.Path(CONFIG_DIR) / 'robot.yaml'
    if not robot_file.is_file():
        import pytest
        pytest.skip('no robot repo config/robot.yaml next to coverage_tool (run in the integrated workspace)')
    robot = yaml.safe_load(robot_file.read_text())
    hx, hy = robot['body_length'] / 2, robot['body_width'] / 2
    g = default_settings()['geometry']
    assert g['footprint_m'] == [[hx, hy], [hx, -hy], [-hx, -hy], [-hx, hy]]
    corner = math.hypot(hx, hy) + g['footprint_padding_m'] + LOCALISATION_MARGIN_M
    assert corner <= g['robot_radius_m'] < corner + 0.01
