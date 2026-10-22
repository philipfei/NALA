"""Radiation map (no ROS): background, painting only at trusted measured poses, hotspots; the contamination
threshold (counts/s or x background) and the alarm (2 of the last 3 readings)."""
import math
import numpy as np
import pytest
from nala_coverage.geometry import Grid
from nala_coverage.radiation import Alarm, RadiationMap, default_setting, threshold, valid_setting

CFG = {'background_readings': 3, 'threshold_mode': 'background', 'threshold_cps': 100.0, 'threshold_factor': 2.0,
       'hotspot_join_m': 0.6}


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


def test_threshold_as_counts_per_second_or_times_the_background():
    assert threshold({'mode': 'cps', 'value': 80.}, None) == 80.               # no background needed
    assert threshold({'mode': 'background', 'value': 2.5}, None) is None       # not measured yet
    assert threshold({'mode': 'background', 'value': 2.5}, 12.) == 30.
    assert default_setting(CFG) == {'mode': 'background', 'value': 2.0}
    assert default_setting({**CFG, 'threshold_mode': 'cps'}) == {'mode': 'cps', 'value': 100.0}
    m = ready_map(background=10.)
    m.setting = {'mode': 'cps', 'value': 35.}
    assert m.threshold == 35. and m.state()['setting'] == {'mode': 'cps', 'value': 35.}


def test_only_usable_thresholds_are_accepted():
    assert valid_setting({'mode': 'cps', 'value': 0.5}) and valid_setting({'mode': 'background', 'value': 1.2})
    for bad in ({'mode': 'cps', 'value': 0}, {'mode': 'background', 'value': 1.0}, {'mode': 'x', 'value': 5},
                {'mode': 'cps', 'value': float('nan')}, {'mode': 'cps', 'value': 'a'}, {'mode': 'cps'}, None):
        assert not valid_setting(bad)


def test_alarm_after_2_of_the_last_3_readings():
    a = Alarm(2, 3)
    assert not a.add(120., 100.)                    # one high reading: not yet
    assert not a.add(40., 100.)
    assert a.add(100., 100.)                        # 2 of the last 3 (at the threshold counts)
    a.reset()
    assert [a.add(c, 100.) for c in (150., 20., 30., 150., 20., 30.)] == [False] * 6   # high ones 3 apart
    a.reset()
    assert not a.add(101., 100.) and a.add(101., 100.)                          # 2 in a row
    assert [c for c, _ in a.last] == [101., 101.]
