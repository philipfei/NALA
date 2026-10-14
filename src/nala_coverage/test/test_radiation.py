"""Radiation map (no ROS): background, painting only at trusted measured poses, hotspots, keepout."""
import math
import numpy as np
import pytest
from nala_coverage.geometry import Grid
from nala_coverage.measurement import Meter
from nala_coverage.radiation import RadiationMap

CFG = {'background_readings': 3, 'threshold_factor': 2.0, 'hotspot_join_m': 0.6, 'keepout_margin_m': 0.2}


def room():
    cells = np.zeros((80, 80), np.int8)
    cells[[0, -1], :] = 100
    cells[:, [0, -1]] = 100
    return Grid.from_cells(cells, .05, (0., 0., 0.))


def ready_map(background=10.):
    m = RadiationMap(room(), 0.26, CFG)
    m.start_background()
    for _ in range(CFG['background_readings']):
        m.add_background(background)
    return m


def test_background_is_the_mean_of_the_first_readings_and_sets_the_threshold():
    m = RadiationMap(room(), 0.26, CFG)
    assert m.threshold is None and not m.add_background(5.)   # not measuring: ignored
    m.start_background()
    assert not m.add_background(9.) and not m.add_background(10.) and m.add_background(11.)
    assert m.background == pytest.approx(10.) and m.threshold == pytest.approx(20.) and not m.measuring
    assert m.state()['background_ready']


def test_a_reading_is_drawn_only_where_trusted_measured_poses_put_the_sensor():
    m = ready_map()
    poses = [(t / 10, 1.0 + t * 0.01, 2.0, 0.0, True) for t in range(11)]    # 1 s at 0.1 m/s, heading +x
    points = m.sensor_points(poses, 0.0, 1.0, max_gap=0.3)
    assert points[0] == pytest.approx((1.26, 2.0)) and points[-1] == pytest.approx((1.36, 2.0))
    m.add_reading(15., points)
    painted = np.argwhere(~np.isnan(m.value))
    xs = m.grid.world(painted)[:, 0]
    assert len(painted) >= 2 and xs.min() >= 1.25 and xs.max() <= 1.4
    # One untrusted pose in the interval, a gap, or no pose at all: not drawn.
    untrusted = poses[:5] + [(0.5, 1.05, 2.0, 0.0, False)] + poses[6:]
    assert m.sensor_points(untrusted, 0.0, 1.0, 0.3) is None
    assert m.sensor_points(poses[:3] + poses[8:], 0.0, 1.0, 0.3) is None
    assert m.sensor_points([], 0.0, 1.0, 0.3) is None


def test_each_cell_keeps_its_highest_reading():
    m = ready_map()
    m.add_reading(12., [(2.0, 2.0)])
    m.add_reading(30., [(2.0, 2.0)])
    m.add_reading(8., [(2.0, 2.0)])
    rc = m.grid.cell((2.0, 2.0))
    assert m.value[rc] == 30. and m.count[rc] == 3


def test_hotspots_join_close_cells_keep_their_id_and_can_be_checked():
    m = ready_map()
    m.add_reading(50., [(1.0, 1.0)])
    m.add_reading(15., [(3.0, 3.0)])                     # below the threshold of 20
    [h] = m.update_hotspots(now=100.)
    assert h['id'] == 1 and h['peak'] == 50. and math.dist((h['x'], h['y']), (1.0, 1.0)) < 0.05
    m.add_reading(60., [(1.3, 1.0)])                     # 0.3 m away: the same hotspot grows
    m.add_reading(40., [(3.0, 1.0)])                     # 2 m away: a new one
    hs = m.update_hotspots(now=101.)
    assert [h['id'] for h in hs] == [1, 2] and hs[0]['peak'] == 60. and hs[0]['first_seen'] == 100.
    assert m.mark_checked(1) and not m.mark_checked(99)
    assert m.update_hotspots(now=102.)[0]['checked'] and m.state()['to_check'] == 1


def test_keepout_is_the_hot_floor_plus_the_margin():
    m = ready_map()
    m.add_reading(50., [(2.0, 2.0)])
    keepout = m.keepout()
    assert keepout[m.grid.cell((2.0, 2.0))] and keepout[m.grid.cell((2.15, 2.0))]
    assert not keepout[m.grid.cell((2.35, 2.0))] and not keepout[m.grid.cell((2.0, 2.4))]
    assert m.state()['hotspots'] == 0                    # update_hotspots not called yet
    assert not RadiationMap(room(), 0.26, CFG).keepout().any()   # no background yet: nothing is hot


def test_avoided_floor_is_not_counted_as_missing():
    grid = room()
    meter = Meter(grid, grid.free.copy(), radius=.3)
    keepout = np.zeros(grid.cells.shape, bool)
    keepout[20:40, 20:40] = True
    whole, avoided = meter.report(), meter.report(keepout)
    assert avoided['avoided_m2'] == pytest.approx(400 * .05 ** 2)
    assert avoided['coverable_m2'] == pytest.approx(whole['coverable_m2'] - avoided['avoided_m2'])
