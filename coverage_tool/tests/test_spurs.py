"""Drive-in stretches into narrow pockets (coverage/spurs.py): drive straight in, reverse straight out."""
import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from coverage import footprint, spurs                       # noqa: E402
from coverage.cost import piece_cells                        # noqa: E402
from coverage.grid import Grid                               # noqa: E402
from coverage.planning import poly_target, stroke_union      # noqa: E402
from coverage.settings import LOCALISATION_MARGIN_M, default_settings, set_settings  # noqa: E402


def room_with_pocket():
    """A 4 x 3 m room (0.05 m cells) with a 0.8 m wide, 1 m deep pocket in the east wall (too narrow to turn in)."""
    a = np.full((60 + 2, 80 + 22), 100, np.int8)
    a[1:61, 1:81] = 0                     # the room
    a[22:38, 81:101] = 0                  # the pocket: rows 22..37 = 0.8 m wide, 1 m deep
    return Grid(a, 0.05, (0.0, 0.0, 0.0))


def test_a_pocket_too_narrow_to_turn_in_gets_a_spur_and_reversing_stays_on_measured_floor():
    s = default_settings()
    set_settings(s)
    g = room_with_pocket()
    radius = s['geometry']['coverage_disk_radius_m']
    # Only the strip the route covers and the pocket count, so the pocket is the only missed floor.
    ys = (np.arange(g.cells.shape[0]) + 0.5) * g.resolution
    region = g.free & ((np.abs(ys - 1.55) < 0.55)[:, None])
    # A route along the room's middle, ending at the pocket's mouth side (all of it with turning room).
    points = [[0.6, 1.55], [3.4, 1.55]]
    result = {'drive': {'points': [list(p) for p in points], 'kinds': ['wall', 'wall'], 'stops': [],
                        'stats': {}},
              'region': region, 'covered': stroke_union(g, region, [poly_target(points, 'x')], radius)}
    before = int(result['covered'].sum())
    assert spurs.add_spurs(g, result, s) >= 1
    d = result['drive']
    i = max((j for j, k in enumerate(d['kinds']) if k == 'spur'), key=lambda j: d['points'][j][0])
    p, q, back = d['points'][i - 1], d['points'][i], d['points'][i + 1]
    ends = {0, len(d['points']) - 1}                         # the route's own ends are no "stops"
    assert d['kinds'][i + 1] == 'reverse' and back == p and {i - 1, i, i + 1} - ends <= set(d['stops'])
    assert 4.1 < q[0] < 4.4    # into the pocket (x 4.05..5.05 m), only as deep as the sensor needs to reach its end
    assert int(result['covered'].sum()) > before
    # The real outline (+ margin) touches nothing, forwards in and backwards out, and the robot does not turn at Q.
    margin = s['geometry']['footprint_padding_m'] + LOCALISATION_MARGIN_M
    n, _ = footprint.hits(g, d['points'], s['geometry']['footprint_m'], margin, kinds=d['kinds'])
    assert n == 0
    # Backing out only passes floor the sensor measured on the way in.
    body = footprint._body_points(s['geometry']['footprint_m'], 0.0, g.resolution / 2)
    measured = np.zeros(g.cells.size, bool)
    measured[piece_cells(g, region, [p, q], radius)] = True
    yaw = math.atan2(q[1] - p[1], q[0] - p[0])
    for t in np.linspace(0, 1, 30):
        x, y = p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t
        c, si = math.cos(yaw), math.sin(yaw)
        X, Y = x + c * body[:, 0] - si * body[:, 1], y + si * body[:, 0] + c * body[:, 1]
        rc = np.floor(g.local(np.stack([X, Y], 1)) / g.resolution).astype(int)
        under = rc[:, 1] * g.cells.shape[1] + rc[:, 0]
        outside_start = np.linalg.norm(np.stack([X, Y], 1) - np.asarray(p), axis=1) > 0.6   # P is on the route
        assert measured[under[outside_start]].all()


def test_a_path_end_the_drive_ins_make_redundant_is_cut():
    # Robot 2026-10-20: after the drive-ins the route still went back across the room to the pocket mouth at its end.
    s = default_settings()
    set_settings(s)
    g = room_with_pocket()
    radius = s['geometry']['coverage_disk_radius_m']
    ys = (np.arange(g.cells.shape[0]) + 0.5) * g.resolution
    region = g.free & ((np.abs(ys - 1.55) < 0.55)[:, None])
    points = [[0.6, 1.55], [3.4, 1.55], [2.0, 1.55]]             # ... and back west over floor already covered
    result = {'drive': {'points': [list(p) for p in points], 'kinds': ['wall'] * 3, 'stops': [1], 'stats': {}},
              'region': region, 'covered': stroke_union(g, region, [poly_target(points[:2], 'x')], radius)}
    assert spurs.add_spurs(g, result, s) >= 1
    d = result['drive']
    assert [2.0, 1.55] not in d['points'] and d['stats']['drive_trimmed_m'] == pytest.approx(1.4)
