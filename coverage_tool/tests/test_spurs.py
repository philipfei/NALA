"""Drive-ins into narrow pockets (coverage/spurs.py): planned by the route optimiser like any other piece."""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from coverage import footprint, pipeline, spurs              # noqa: E402
from coverage.cost import piece_cells                        # noqa: E402
from coverage.grid import Grid                               # noqa: E402
from coverage.settings import LOCALISATION_MARGIN_M, default_settings, set_settings  # noqa: E402


def room_with_pocket():
    """A 4 x 3 m room (0.05 m cells) with a 0.8 m wide, 1 m deep pocket in the east wall (too narrow to turn in)."""
    a = np.full((60 + 2, 80 + 22), 100, np.int8)
    a[1:61, 1:81] = 0                     # the room
    a[22:38, 81:101] = 0                  # the pocket: rows 22..37 = y 1.1..1.9 m, x 4.05..5.05 m
    return Grid(a, 0.05, (0.0, 0.0, 0.0))


def quick_settings():
    s = default_settings()
    s['optimizer'].update(search_time_s=1.0, polish_time_s=2.0)
    set_settings(s)
    return s


def pocket_mask(g):
    m = np.zeros(g.cells.shape, bool)
    m[22:38, 81:101] = True
    return m


def test_candidates_reach_a_pocket_the_turning_circle_cannot():
    s = quick_settings()
    g = room_with_pocket()
    reach, _ = g.reachable_from([1.0, 1.5], s.collision)
    region = pipeline.coverage_region(g, reach, [[-1, -1], [9, -1], [9, 9], [-1, 9]])
    coverable = pipeline.brush_reachable(g, region, pipeline.sensor_reachable(g, reach, s), 0.6)
    assert (pocket_mask(g) & ~coverable).any()                # the route alone cannot cover the pocket's end
    found, _ = spurs.candidates(g, reach, region, coverable, s)
    assert found
    room_ = spurs.turn_room(s)
    for p, q, cells, _ in found:
        r, c = g.cell(p)
        assert g.clearance[r, c] >= room_ - 1e-9               # turning room where it starts (and turns back)
        n, _ = footprint.hits(g, [p, q, p], s['geometry']['footprint_m'],
                              s['geometry']['footprint_padding_m'] + max(LOCALISATION_MARGIN_M, s['spur']['start_offset_m']),
                              kinds=['transit', 'spur', 'reverse'])
        assert n == 0                                           # the outline fits, in forwards and out backwards
        assert np.array_equal(np.sort(cells), np.sort(piece_cells(g, region, [p, q], 0.6)))


def test_the_route_is_planned_with_one_drive_in_and_backing_out_stays_on_measured_floor():
    s = quick_settings()
    g = room_with_pocket()
    res = pipeline.plan(g, [1.0, 1.5], [[-1, -1], [9, -1], [9, 9], [-1, 9]], s)
    st = res['stats']
    assert st['coverage_pct'] > 99.9 and st['drive_ins'] == 1 and st['footprint_hits'] == 0
    assert st['drive_lost_cells'] == 0 and st['drive_unsafe_segments'] == 0
    assert (res['covered'] & pocket_mask(g)).sum() == (pocket_mask(g) & g.free).sum()   # the whole pocket
    d = res['drive']
    i = d['kinds'].index('spur')
    p, q, back = d['points'][i - 1], d['points'][i], d['points'][i + 1]
    assert d['kinds'][i + 1] == 'reverse' and back == p and q[0] > 4.0
    assert {i - 1, i, i + 1} - {len(d['points']) - 1} <= set(d['stops'])
    # Backing out only passes floor the sensor measured on the way in (P itself is where the robot stood).
    body = footprint._body_points(s['geometry']['footprint_m'], 0.0, g.resolution / 2)
    measured = np.zeros(g.cells.size, bool)
    measured[piece_cells(g, res['region'], [p, q], 0.6)] = True
    yaw = math.atan2(q[1] - p[1], q[0] - p[0])
    for t in np.linspace(0, 1, 30):
        x, y = p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t
        c, si = math.cos(yaw), math.sin(yaw)
        X, Y = x + c * body[:, 0] - si * body[:, 1], y + si * body[:, 0] + c * body[:, 1]
        rc = np.floor(g.local(np.stack([X, Y], 1)) / g.resolution).astype(int)
        under = rc[:, 1] * g.cells.shape[1] + rc[:, 0]
        beyond_p = np.linalg.norm(np.stack([X, Y], 1) - np.asarray(p), axis=1) > 0.6
        assert measured[under[beyond_p]].all()
