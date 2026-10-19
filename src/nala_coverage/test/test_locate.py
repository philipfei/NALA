"""Locator (no ROS): find the robot on the map from a scan, and the base station check."""
import math
import numpy as np
from nala_coverage.geometry import Grid
from nala_coverage.locate import Locator, endpoints, ON_BASE, BACKWARDS, TURNED, NOT_ON_BASE

CFG = {'position_step_m': .1, 'yaw_step_deg': 4., 'match_distance_m': .1, 'min_fit': .7, 'base_tolerance_m': .15,
       'base_tolerance_deg': 15., 'direction_margin': .05, 'ambiguity_margin': .05}
LASER = (0., 0., math.pi)    # NALA's LiDAR faces backward


def room(asymmetric=True):
    """A 5 x 2.5 m room (0.05 m cells); asymmetric: a box against one wall and a short wall stub."""
    a = np.zeros((50, 100), np.int8)
    a[[0, -1], :] = 100
    a[:, [0, -1]] = 100
    if asymmetric:
        a[1:12, 70:80] = 100
        a[30:49, 20:22] = 100
    return Grid.from_cells(a, .05, (0., 0., 0.))


def scan_from(grid, pose, beams=360):
    """Ray-cast scan of the map from pose, in the LiDAR's angles (facing backward)."""
    ranges = []
    for a in np.linspace(-math.pi, math.pi, beams, endpoint=False):
        ang = pose[2] + LASER[2] + a
        for r in np.arange(.15, 12, .02):
            rc = grid.cell((pose[0] + r * math.cos(ang), pose[1] + r * math.sin(ang)))
            if not grid.valid(rc):
                ranges.append(math.inf)
                break
            if grid.cells[rc] == 100:
                ranges.append(r)
                break
        else:
            ranges.append(math.inf)
    return endpoints(ranges, -math.pi, 2 * math.pi / beams, .15, 12., LASER, 90)


def test_robot_is_found_anywhere_on_the_map():
    g = room()
    loc = Locator(g, CFG)
    truth = (3.9, 0.6, 1.2)
    r = loc.locate(scan_from(g, truth), (1.5, 1.25, 0.))
    assert r['status'] == NOT_ON_BASE and r['fit'] > .9
    assert math.dist(r['pose'][:2], truth[:2]) < .1 and abs(math.remainder(r['pose'][2] - truth[2], 2 * math.pi)) < math.radians(5)


def test_base_station_check_on_base_backwards_and_beside_it():
    g = room()
    loc = Locator(g, CFG)
    station = (1.5, 1.25, 0.)
    assert loc.locate(scan_from(g, station), station)['status'] == ON_BASE
    assert loc.locate(scan_from(g, (1.58, 1.22, math.radians(8))), station)['status'] == ON_BASE   # put on roughly
    assert loc.locate(scan_from(g, (1.5, 1.25, math.pi)), station)['status'] == BACKWARDS
    assert loc.locate(scan_from(g, (2.0, 1.25, 0.)), station)['status'] == NOT_ON_BASE         # 0.5 m beside it
    r = loc.locate(scan_from(g, (1.5, 1.25, math.radians(-130))), station)                   # on the spot, turned
    assert r['status'] == TURNED and abs(r['turn_deg'] - 130) <= 3


def test_direction_cannot_be_told_in_a_symmetric_room():
    # Turned 180 deg around the centre the room looks the same: the robot counts as on the base station.
    g = room(asymmetric=False)
    loc = Locator(g, CFG)
    station = (2.5, 1.25, 0.)
    assert loc.locate(scan_from(g, (2.5, 1.25, math.pi)), station)['status'] == ON_BASE


def test_no_scan_points_is_not_on_base():
    g = room()
    r = Locator(g, CFG).locate(np.zeros((0, 2)), (1.5, 1.25, 0.))
    assert r['status'] == NOT_ON_BASE and r['pose'] is None and r['fit'] == 0.


def test_mirror_images_are_told_apart_by_where_the_robot_was_last():
    # In a symmetric room a corner and the opposite corner turned 180 deg give the same scan.
    g = room(asymmetric=False)
    loc = Locator(g, CFG)
    truth = (4.2, 2.0, .6)
    mirror = (5. - truth[0], 2.5 - truth[1], truth[2] + math.pi)
    points = scan_from(g, truth)
    for previous in (truth, mirror):
        r = loc.locate(points, (2.5, 1.25, 0.), previous)
        assert r['ambiguous'] and math.dist(r['pose'][:2], previous[:2]) < .15
