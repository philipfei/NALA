"""Map changes (no ROS): a cabinet moved between mapping and scanning; walls with a small pose error stay unchanged."""
import math
import numpy as np
from nala_coverage.geometry import Grid
from nala_coverage.mapchange import ChangeMap

CFG = {'max_range_m': 6.0, 'wall_margin_m': 0.10, 'min_hits': 4, 'hit_ratio': 0.7, 'min_passes': 6}
RES = 0.05


def room(cabinet):
    """A 4 x 4 m room with a 0.4 x 0.4 m cabinet outline at `cabinet` (x0, y0); None = no cabinet."""
    cells = np.zeros((80, 80), np.int8)
    cells[[0, -1], :] = 100
    cells[:, [0, -1]] = 100
    if cabinet is not None:
        c0, r0 = int(cabinet[0] / RES), int(cabinet[1] / RES)
        cells[r0:r0 + 8, c0] = cells[r0:r0 + 8, c0 + 7] = 100
        cells[r0, c0:c0 + 8] = cells[r0 + 7, c0:c0 + 8] = 100
    return Grid.from_cells(cells, RES, (0., 0., 0.))


def cast(world, origin, angles, range_max=12.0):
    """Ranges of beams from `origin` in the true world (step 1 cm)."""
    out = []
    for a in angles:
        d = 0.0
        while d < range_max:
            rc = world.cell((origin[0] + d * math.cos(a), origin[1] + d * math.sin(a)))
            if not world.valid(rc) or world.cells[rc] > 0:
                break
            d += 0.01
        out.append(d)
    return np.array(out)


def scan_from(changes, world, poses, error=(0., 0.)):
    angles = np.linspace(-math.pi, math.pi, 360, endpoint=False)
    for x, y in poses:
        ranges = cast(world, (x, y), angles)
        # The robot thinks it is `error` away from where it really is.
        changes.add_scan((x + error[0], y + error[1]), angles, ranges, 12.0)


def test_a_moved_cabinet_shows_as_new_and_gone():
    saved = room(cabinet=(1.0, 1.0))           # the map: cabinet near (1.2, 1.2)
    world = room(cabinet=(2.6, 2.6))           # now: moved to near (2.8, 2.8)
    changes = ChangeMap(saved, CFG)
    scan_from(changes, world, [(2.0, 2.0), (1.8, 1.0), (1.0, 2.2), (3.0, 1.5), (1.5, 3.2), (2.2, 1.8),
                               (1.9, 2.3), (2.3, 2.1)])
    new, gone = changes.appeared(), changes.vanished()
    xy_new = saved.world(np.argwhere(new))
    xy_gone = saved.world(np.argwhere(gone))
    assert len(xy_new) >= 5 and np.all(np.hypot(xy_new[:, 0] - 2.8, xy_new[:, 1] - 2.8) < 0.35)
    assert len(xy_gone) >= 5 and np.all(np.hypot(xy_gone[:, 0] - 1.2, xy_gone[:, 1] - 1.2) < 0.35)
    a, g = changes.area_m2()
    assert a > 0 and g > 0


def test_unchanged_room_with_a_small_pose_error_shows_no_changes():
    saved = room(cabinet=(1.0, 1.0))
    changes = ChangeMap(saved, CFG)
    scan_from(changes, saved, [(2.0, 2.0), (3.0, 1.5), (1.5, 3.2)], error=(0.03, -0.02))
    assert not changes.appeared().any() and not changes.vanished().any()


def test_a_person_walking_by_is_not_a_new_object():
    saved = room(cabinet=None)
    changes = ChangeMap(saved, CFG)
    walker = room(cabinet=(2.5, 1.8))          # something stands there for 2 scans...
    scan_from(changes, walker, [(1.0, 2.0), (1.0, 2.1)])
    scan_from(changes, saved, [(1.0, 2.0), (1.0, 2.1), (1.2, 2.0), (1.2, 2.2), (1.4, 2.0)])   # ...then it is gone
    assert not changes.appeared().any()
