"""Passages (coverage/passages.py): straight through an entrance narrower than the turning circle into an area with
turning room, planned like any connection; the route never ends inside such an area."""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from coverage import footprint, passages, pipeline                         # noqa: E402
from coverage.grid import Grid                                             # noqa: E402
from coverage.settings import default_settings, set_settings              # noqa: E402


def two_rooms():
    """Room A (x 0..3 m) and room B (x 3.2..6.2 m), 3 m deep, joined by a 0.70 m door in the 0.2 m wall between them
    (y 1.15..1.85 m): the robot fits through, its 1.06 m turning circle does not; both rooms have turning room."""
    a = np.full((60 + 2, 124 + 2), 100, np.int8)
    a[1:61, 1:61] = 0                     # room A: cells 1..60 = x 0.05..3.05
    a[1:61, 65:125] = 0                   # room B: cells 65..124 = x 3.25..6.25
    a[23:37, 61:65] = 0                   # the door: 14 cells = 0.70 m wide, through the 4-cell wall
    return Grid(a, 0.05, (0.0, 0.0, 0.0))


def quick_settings():
    s = default_settings()
    s['optimizer'].update(search_time_s=1.0, polish_time_s=2.0)
    set_settings(s)
    return s


def test_a_room_behind_a_narrow_door_is_joined_by_a_passage_both_ways():
    s = quick_settings()
    g = two_rooms()
    reach, _ = g.reachable_from([1.0, 1.5], s.collision)
    assert not reach[g.cell([4.7, 1.5])]                               # room B: no way in with the turning circle
    found, joined = passages.find(g, reach, s)
    assert len(found) == 1 and joined[g.cell([4.7, 1.5])]
    a, b = found[0]
    assert g.clearance[g.cell(a)] >= s.collision and g.clearance[g.cell(b)] >= s.collision   # turning room both ends
    g.passages = found
    there, back = g.astar([1.0, 1.5], [4.7, 1.5], joined, s.collision), g.astar([4.7, 1.5], [1.0, 1.5], joined, s.collision)
    for path in (there, back):
        # The only stretch without turning-circle clearance is the passage, and the outline drives it forwards.
        unsafe = [(p, q) for p, q in zip(path, path[1:]) if not g.segment_safe(p, q, s.collision)]
        assert len(unsafe) == 1 and {tuple(unsafe[0][0]), tuple(unsafe[0][1])} == {tuple(a), tuple(b)}
        n, _ = footprint.hits(g, path, s['geometry']['footprint_m'], s['geometry']['footprint_padding_m'])
        assert n == 0


def test_the_route_covers_the_room_behind_the_door_and_never_ends_inside_it():
    s = quick_settings()
    g = two_rooms()
    res = pipeline.plan(g, [1.0, 1.5], [[-1, -1], [9, -1], [9, 9], [-1, 9]], s)
    st = res['stats']
    room_b = np.zeros(g.cells.shape, bool)
    room_b[1:61, 65:125] = True
    assert st['passages'] == 1 and st['coverage_of_reachable_pct'] > 99.9
    assert (res['covered'] & room_b).sum() == (room_b & res['coverable']).sum() > 0.9 * room_b.sum()
    assert st['footprint_hits'] == 0 and st['drive_unsafe_segments'] == 0 and st['drive_lost_cells'] == 0
    d = res['drive']
    assert d['kinds'].count('passage') >= 2                            # in and out
    end = d['points'][-1]
    assert not room_b[g.cell(end)] and g.clearance[g.cell(end)] >= s.collision - 1e-9
