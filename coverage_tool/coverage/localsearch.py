"""Cost-driven local search over the complete route.

The route is a sequence of pieces (a lane, a loop, a spiral ring, a centre
line, a single goal). The optimiser repeatedly applies whichever move lowers
the *total* cost from cost.py:

  drop      remove a piece (saves its driving; costs missed cells it alone covers)
  flip      drive a piece in the other direction
  move      take a piece out and insert it elsewhere (either direction)
  2-opt     reverse a whole stretch of the route
  rotate    enter a closed loop at the point that is cheapest to reach and leave
  straighten  replace a wiggly piece (a loop along a bumpy wall) by fewer,
            longer straight lines that are still safe to drive
  detour    add a goal that covers still-missed cells, where the missed-cell
            penalty it removes outweighs the extra driving and turning: a single
            pose, an aimed goal, a line along a missed strip, or a drive-in into a
            pocket (spurs.py: straight in from a start with turning room, straight
            back out; only offered for floor nothing else can reach)

Every move is scored with exact cost differences: transition time (straight
connector or A* around obstacles), stop-and-turn costs at every junction,
hairpin penalties, double coverage and missed reachable cells. Nothing is
forced: with a very high missed-cell weight covering everything is simply the
cheapest option; with a low weight tiny specks may be left if a detour costs
more than they are worth.
"""
import math
import time

import numpy as np

from . import cost as costmod
from . import sensor, spurs
from .planning import least_bend, poly_target, split_path, stroke_union, visible_many

AIM_RUN_M = 0.05   # an aimed goal is reached with a short straight forward move, so the robot faces its aim
# An aimed goal at a piece end where the route already is: the robot turns on the spot there and moves this far
# (the same as AIM_RUN_M; on the robot it is a turn on the spot: Nav2's goal tolerance is 0.05 m). Room Test:
# 0.05 m from the north-west lane end covers the whole corner patch, 0.01 m not (sensor sight past wall bumps).
AIM_AT_ROUTE_RUN_M = AIM_RUN_M
EPS = 1e-6


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _first_heading(pts):
    for p, q in zip(pts, pts[1:]):
        if math.dist(p, q) > EPS:
            return math.atan2(q[1] - p[1], q[0] - p[0])
    return None


def _last_heading(pts):
    for p, q in zip(pts[-2::-1], pts[::-1]):
        if math.dist(p, q) > EPS:
            return math.atan2(q[1] - p[1], q[0] - p[0])
    return None


class Piece:
    def __init__(self, pts, kind):
        self.pts = [list(map(float, p)) for p in pts]
        self.kind = kind
        self.point = len(self.pts) < 2 or all(math.dist(self.pts[0], q) < EPS for q in self.pts)
        self.h0 = None if self.point else _first_heading(self.pts)
        self.h1 = None if self.point else _last_heading(self.pts)
        self.h = None     # single goal: the heading the robot has while passing it (set by orient())
        self.aim = None   # aimed goal: a fixed heading that points the sensor (ahead of the robot) at missed floor
        self.run = AIM_RUN_M   # aimed goal: length of its short forward move
        self.covs = None  # (cells covered driven forwards, cells covered driven backwards), set by _measure
        self.spur = None  # drive-in: its end Q (pts = [P]); in from P facing `aim`, back out to P facing the same way

    def cells(self, rev):
        """Floor cells this piece covers when driven in that direction (the sensor leads the robot)."""
        return self.covs[1 if rev else 0]


    def state(self, rev):
        """(start xy, start heading, end xy, end heading) when driven forwards/backwards."""
        a, b = self.pts[0], self.pts[-1]
        if self.point:    # the turn at a single goal = turn onto h + turn from h to the way out
            return a, self.h, a, self.h
        if not rev:
            return a, self.h0, b, self.h1
        f = lambda h: None if h is None else _wrap(h + math.pi)
        return b, f(self.h1), a, f(self.h0)

    def targets(self, rev, max_seg):
        pts = self.pts[::-1] if rev else self.pts
        if self.spur is not None:
            t = poly_target([list(self.pts[0]), list(self.spur), list(self.pts[0])], 'spur')
            t.kind = 'spur'
            return [t]
        if self.aim is not None:
            t = poly_target(aim_line(self), 'aim')
            t.kind = 'aim'
            return [t]
        if self.point:
            t = poly_target([pts[0], pts[0]], self.kind)
            t.points = [list(pts[0]), list(pts[0])]
            t.kind = self.kind
            return [t]
        return split_path(pts, max_seg, self.kind)



def aimed(xy, heading, run=AIM_RUN_M):
    """A single goal the robot passes facing `heading`: driven as a short forward move (run) ending at xy."""
    p = Piece([xy], 'aim')
    p.aim = p.h = float(heading)
    p.run = float(run)
    return p


def drivein(p, q, pocket=None):
    """A drive-in (spurs.py): straight from P to Q facing Q, and back out to P (still facing Q). Like an aimed goal it
    has a fixed heading at P and is never driven the other way, trimmed or straightened."""
    piece = Piece([p], 'spur')
    piece.aim = piece.h = math.atan2(q[1] - p[1], q[0] - p[0])
    piece.spur = [float(q[0]), float(q[1])]
    piece.run = 0.0
    piece.pocket = pocket    # index of the pocket it was made for (RouteOptimizer.rechoose)
    return piece


def aim_line(p):
    """The short forward move of an aimed goal (AIM_RUN_M long, ending at the goal)."""
    x, y = p.pts[0]
    return [[x - p.run * math.cos(p.aim), y - p.run * math.sin(p.aim)], [x, y]]


def pieces_from_targets(targets):
    """Join consecutive, continuous targets of the same kind into pieces."""
    pieces, cur, kind = [], None, None
    for t in targets:
        pts = [list(map(float, p)) for p in (getattr(t, 'points', None) or [t.start, t.end])]
        k = getattr(t, 'kind', 'spiral')
        if k == 'spur' and len(pts) == 3:     # a drive-in: P, Q, P
            if cur:
                pieces.append(Piece(cur, kind))
                cur = None
            pieces.append(drivein(pts[0], pts[1]))
            continue
        if k == 'aim' and len(pts) == 2:      # an aimed goal keeps its heading (it is never driven backwards)
            if cur:
                pieces.append(Piece(cur, kind))
                cur = None
            pieces.append(aimed(pts[1], math.atan2(pts[1][1] - pts[0][1], pts[1][0] - pts[0][0]),
                                math.dist(pts[0], pts[1])))
            continue
        is_point = len(pts) < 2 or math.dist(pts[0], pts[-1]) < EPS and len(pts) <= 2
        if cur and not is_point and k == kind and math.dist(cur[-1], pts[0]) < EPS:
            cur.extend(pts[1:])
            continue
        if cur:
            pieces.append(Piece(cur, kind))
        cur, kind = pts, k
        if is_point:
            pieces.append(Piece(pts[:1], k))
            cur = None
    if cur:
        pieces.append(Piece(cur, kind))
    return pieces


class RouteOptimizer:
    def __init__(self, grid, pieces, start, reach, den, coverable, settings, neighbours=8, drive_ins=([], []),
                 home_area=None, home=None):
        self.g, self.reach, self.den = grid, reach, den
        self.start = list(map(float, start))
        # Where the robot drives after the route (the base station; user 2026-10-22: that drive counts, so the
        # route does not end far away for free). None: the route's end is free (as before).
        self.home = None if home is None else list(map(float, home))
        self.s = settings
        c = settings['cost']
        self.v = float(settings['motion']['linear_m_s'])
        self.w = float(settings['motion']['angular_rad_s'])
        self.smooth = math.radians(float(c['smooth_turn_deg']))
        self.hairpin = math.radians(float(c['reverse_angle_deg']))
        self.stop = float(c['stop_penalty_s'])
        self.revpen = float(c['reverse_penalty_s'])
        self.bend = float(c.get('bend_s_per_rad', 0.0))
        at = costmod.area_time(settings)
        self.cell = grid.resolution ** 2
        self.k_over = float(c['overlap_weight']) * at           # s per m2 double covered
        self.k_miss = float(c['missed_weight']) * at * self.cell  # s per missed cell
        # cells right against a wall/obstacle (the bottom of a recess between lidar bumps)
        # may have their own, lower weight: that is what lets a line stay straight
        edge = (grid.clearance < float(c.get('edge_band_m', 0.0))).ravel()
        k_edge = float(c.get('edge_missed_weight', c['missed_weight'])) * at * self.cell
        self.wmiss = np.where(edge, k_edge, self.k_miss)
        self.r = float(settings['geometry']['coverage_disk_radius_m'])
        self.collision = settings.collision
        self.K = neighbours
        # The area with turning room the start lies in: a route ending outside it (in an island behind a passage)
        # also drives the way out (passages.way_out), counted in total().
        self.home_area = home_area
        self._way_out = {}
        # Drive-in candidates (spurs.candidates): (P, Q, cells, pocket), the pockets, and per cell the candidates.
        self.drive_ins, self.pockets = list(drive_ins[0]), list(drive_ins[1])
        self._drive_in_of = {}
        for n, (_, _, cells, _) in enumerate(self.drive_ins):
            for c in cells:
                self._drive_in_of.setdefault(int(c), []).append(n)
        self._outline = spurs.body(settings, grid.resolution) if self.drive_ins else None
        self.cov_ok = coverable.ravel().copy()
        self.count = np.zeros(grid.cells.size, np.int32)
        self._conn = {}
        self.pieces = []
        self.seq = []   # list of [piece index, reversed]
        for p in pieces:
            self._register(p)
            self.seq.append([len(self.pieces) - 1, False])
        for p in pieces:                         # drive-ins of an earlier plan (pieces_from_targets): their pocket
            if p.spur is not None and getattr(p, 'pocket', None) is None and self.pockets:
                p.pocket = int(np.argmax([pk[p.cells(False)].sum() for pk in self.pockets]))

    def _miss(self, idx):
        """Missed-area cost (s) of the given cells if none of them were covered."""
        idx = np.asarray(idx, dtype=np.int64)
        return float(self.wmiss[idx][self.cov_ok[idx]].sum()) if len(idx) else 0.0

    # ------------------------------------------------------------ pieces
    def _register(self, p, rev=False):
        self._measure(p)
        self.pieces.append(p)
        self.count[p.cells(rev)] += 1
        return len(self.pieces) - 1

    def _measure(self, p):
        """Coverage cells (per driving direction) and internal time of a piece.

        The sensor sits ahead of the robot, so a piece covers a little more at the end it drives towards:
        forwards and backwards cover different cells. Every move that flips a piece updates the counts."""
        if p.spur is not None:                   # a drive-in covers its way in (backing out passes the same floor)
            fwd = costmod.piece_cells(self.g, self.den, [p.pts[0], p.spur], self.r)
            p.covs = (fwd, fwd)
            p.internal = spurs.drive_in_time(self.s, math.dist(p.pts[0], p.spur))
            return p
        if p.aim is not None:
            fwd = costmod.piece_cells(self.g, self.den, aim_line(p), self.r)
            p.covs = (fwd, fwd)                  # an aimed goal is never driven the other way
        else:
            fwd = costmod.piece_cells(self.g, self.den, p.pts, self.r)
            rev = costmod.piece_cells(self.g, self.den, p.pts[::-1], self.r) \
                if not p.point and sensor.offset(self.s) > 0 else fwd
            p.covs = (fwd, rev)
        terms = costmod.route_terms(p.pts, self.s) if not p.point else None
        p.internal = (terms['drive_s'] + terms['turn_s'] + terms['reverse_s']) if terms else \
            0.0                                  # turning at a single goal is charged at its junctions
        return p

    def _cov_delta(self, swaps):
        """Cost change (s) of double-covered and missed floor when each (old cells, new cells) pair in
        `swaps` replaces the old cells by the new ones (a piece flipped, moved reversed or replaced)."""
        if not swaps:
            return 0.0
        u = np.unique(np.concatenate([c for pair in swaps for c in pair]))
        d = np.zeros(len(u), np.int32)
        for old, new in swaps:
            np.subtract.at(d, np.searchsorted(u, old), 1)
            np.add.at(d, np.searchsorted(u, new), 1)
        before = self.count[u]
        after = before + d
        lost, gained = u[(before > 0) & (after <= 0)], u[(before <= 0) & (after > 0)]
        dswept = sum(len(new) - len(old) for old, new in swaps)
        return (self.k_over * self.cell * (dswept - (len(gained) - len(lost)))
                + self._miss(lost) - self._miss(gained))

    def _swap_counts(self, swaps):
        for old, new in swaps:
            self.count[old] -= 1
            self.count[new] += 1

    def _flips(self, items):
        """(old cells, new cells) of reversing every piece in items ([piece index, reversed] pairs)."""
        return [(self.pieces[i].cells(r), self.pieces[i].cells(not r)) for i, r in items
                if not self.pieces[i].point]

    # ------------------------------------------------------- transitions
    def _turn(self, h1, h2):
        """Heading change at a junction between pieces: treated as a real corner."""
        if h1 is None or h2 is None:
            return 0.0
        d = abs(_wrap(h2 - h1))
        if d <= self.smooth:
            return d * self.bend
        cost = d / self.w + self.stop
        if d >= self.hairpin:
            cost += self.revpen
        return cost

    def _connector(self, a, b):
        key = (round(a[0], 4), round(a[1], 4), round(b[0], 4), round(b[1], 4))
        if key in self._conn:
            return self._conn[key]
        g = self.g
        if g.segment_safe(a, b, self.collision):
            path = [a, b]
        else:
            path = g.astar(a, b, self.reach, self.collision)
            if path:
                compact, i = [path[0]], 0
                while i < len(path) - 1:
                    j = len(path) - 1
                    while j > i + 1 and not g.segment_safe(path[i], path[j], self.collision):
                        j -= 1
                    compact.append(path[j])
                    i = j
                path = least_bend(g, compact, self.collision)
        if not path:
            res = None
        else:
            terms = costmod.route_terms(path, self.s)
            # A connection drives over floor the route covers anyway: everything it sweeps is double coverage
            # (user 2026-10-22).
            swept = self._swept_estimate(path) * self.cell if len(path) > 1 else 0.0
            res = (terms['drive_s'] + terms['turn_s'] + terms['reverse_s'] + self.k_over * swept,
                   _first_heading(path), _last_heading(path), path)
        self._conn[key] = res
        return res

    def _swept_estimate(self, path):
        """Floor cells within the brush radius of a polyline (no line of sight, no sensor offset): a fast estimate
        of what a connection sweeps (the exact stroke_union per connection made planning ~2x slower)."""
        g = self.g
        if getattr(self, '_disk', None) is None:
            n = int(math.ceil(self.r / g.resolution))
            yy, xx = np.mgrid[-n:n + 1, -n:n + 1]
            keep = np.hypot(xx, yy) * g.resolution <= self.r
            self._disk = np.stack([yy[keep], xx[keep]], axis=1)
            self._den_flat = self.den.ravel()
        centres = []
        for a, b in zip(path, path[1:]):
            m = max(1, int(math.ceil(math.dist(a, b) / g.resolution)))
            for t in np.linspace(0.0, 1.0, m + 1):
                centres.append(g.cell([a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])]))
        rc = (np.unique(np.asarray(centres), axis=0)[:, None, :] + self._disk[None]).reshape(-1, 2)
        H, W = g.shape
        rc = rc[(rc[:, 0] >= 0) & (rc[:, 0] < H) & (rc[:, 1] >= 0) & (rc[:, 1] < W)]
        idx = np.unique(rc[:, 0] * W + rc[:, 1])
        return int(self._den_flat[idx].sum())

    def T(self, frm, to):
        """Seconds from the end of state `frm` (None = robot start) to the start of state `to` (None = after the
        route: the drive home, when there is a home)."""
        if to is None:
            if frm is None or self.home is None:
                return 0.0
            _, _, xy, h = self.pieces[frm[0]].state(frm[1])
            if math.dist(xy, self.home) < EPS:
                return 0.0
            c = self._connector(xy, self.home)
            return 1e7 if c is None else c[0] + self._turn(h, c[1])
        if frm is None:
            xy, h = self.start, None
        else:
            _, _, xy, h = self.pieces[frm[0]].state(frm[1])
        sxy, sh, _, _ = self.pieces[to[0]].state(to[1])
        if math.dist(xy, sxy) < EPS:
            return self._turn(h, sh)
        c = self._connector(xy, sxy)
        if c is None:
            return 1e7
        t, ho, hi, _ = c
        return t + self._turn(h, ho) + self._turn(hi, sh)

    def _slot_cost(self, a, st, b):
        """T(a, st) + T(st, b). A single goal gets its best heading for this slot
        (the way in or the way out), so its turn is charged correctly."""
        p = self.pieces[st[0]]
        if not p.point or p.aim is not None:
            self.slot_heading = p.h
            return self.T(a, st) + self.T(st, b)
        xy = p.pts[0]
        cands = []
        fa = self.start if a is None else self.pieces[a[0]].state(a[1])[2]
        hb = self._heading_between(fa, xy)
        if hb:
            cands.append(hb[1])
        elif a is not None:
            cands.append(self.pieces[a[0]].state(a[1])[3])
        if b is not None:
            hb = self._heading_between(xy, self.pieces[b[0]].state(b[1])[0])
            if hb:
                cands.append(hb[0])
        old, best = p.h, None
        for h in cands or [None]:
            p.h = h
            c = self.T(a, st) + self.T(st, b)
            if best is None or c < best[0]:
                best = (c, h)
        p.h = old
        self.slot_heading = best[1]          # the heading this cost assumes: a move into the slot must keep it
        return best[0]

    def _at(self, k):
        return tuple(self.seq[k]) if 0 <= k < len(self.seq) else None

    # --------------------------------------------------------------- cost
    def total(self):
        time_s = sum(self.pieces[i].internal for i, _ in self.seq)
        prev = None
        for st in self.seq:
            time_s += self.T(prev, tuple(st))
            prev = tuple(st)
        if prev is not None and self.home is not None:
            time_s += self.T(prev, None)         # the drive home (also out of an island, through its passage)
        elif prev is not None and self.home_area is not None:
            time_s += self.way_out_time(self.pieces[prev[0]].state(prev[1])[2])
        covered = self.count > 0
        swept = sum(len(self.pieces[i].cells(rv)) for i, rv in self.seq) * self.cell
        overlap = max(0.0, swept - covered.sum() * self.cell)
        open_ = self.cov_ok & ~covered
        return time_s + self.k_over * overlap + float(self.wmiss[open_].sum()), int(open_.sum())

    def way_out_time(self, xy):
        """Seconds from the route's end to the home area when it ends inside an island (0 otherwise)."""
        key = (round(xy[0], 4), round(xy[1], 4))
        if key not in self._way_out:
            from .passages import way_out
            path = way_out(self.g, xy, self.start, self.reach, self.home_area, self.collision)
            terms = costmod.route_terms(path, self.s) if path and len(path) > 1 else None
            self._way_out[key] = (terms['drive_s'] + terms['turn_s'] + terms['reverse_s']) if terms else 0.0
        return self._way_out[key]

    # -------------------------------------------------------------- moves
    def _drop_delta(self, k):
        i, rv = self.seq[k]
        p = self.pieces[i]
        prev, cur, nxt = self._at(k - 1), (i, rv), self._at(k + 1)
        dt = -p.internal - self.T(prev, cur) - self.T(cur, nxt) + self.T(prev, nxt)
        cells = p.cells(rv)
        uniq = cells[self.count[cells] == 1]
        return dt + self.k_over * self.cell * (len(uniq) - len(cells)) + self._miss(uniq)

    def _drop(self, k):
        i, rv = self.seq.pop(k)
        self.count[self.pieces[i].cells(rv)] -= 1

    def _neighbour_positions(self, k):
        """Positions whose end lies near the start or end of the piece at k."""
        if len(self.seq) < 3:
            return []
        ends = np.array([self.pieces[i].state(rv)[2] for i, rv in self.seq])
        i, rv = self.seq[k]
        a, _, b, _ = self.pieces[i].state(rv)
        d = np.minimum(np.linalg.norm(ends - a, axis=1), np.linalg.norm(ends - b, axis=1))
        d[k] = np.inf
        return list(np.argsort(d)[:self.K])

    def improve_once(self, deadline):
        """One pass over the route; returns number of improving moves applied."""
        applied = 0
        k = 0
        while k < len(self.seq):
            if time.monotonic() > deadline:
                break
            i, rv = self.seq[k]
            prev, cur, nxt = self._at(k - 1), (i, rv), self._at(k + 1)
            # drop
            if self._drop_delta(k) < -1e-6:
                self._drop(k)
                applied += 1
                continue
            # flip
            if not self.pieces[i].point:
                flip = (i, not rv)
                swaps = self._flips([(i, rv)])
                d = self.T(prev, flip) + self.T(flip, nxt) - self.T(prev, cur) - self.T(cur, nxt) \
                    + self._cov_delta(swaps)
                if d < -1e-6:
                    self._swap_counts(swaps)
                    self.seq[k][1] = not rv
                    applied += 1
                    continue
            best = None
            if applied > 10 * len(self.seq):     # safety: never loop for ever on one pass
                break
            # move elsewhere
            removal = self.T(prev, nxt) - self.T(prev, cur) - self.T(cur, nxt)
            for j in self._neighbour_positions(k):
                if j in (k - 1, k):
                    continue
                a, b = self._at(j), self._at(j + 1)
                for o in ((False, True) if not self.pieces[i].point else (False,)):
                    st = (i, o)
                    d = removal + self._slot_cost(a, st, b) - self.T(a, b)
                    h = self.slot_heading
                    if o != rv:
                        d += self._cov_delta(self._flips([(i, rv)]))
                    if d < -1e-6 and (best is None or d < best[0]):
                        best = (d, 'move', j, o, h)
            # 2-opt: reverse the stretch between k and j
            for j in self._neighbour_positions(k):
                lo, hi = min(k, j), max(k, j)
                if hi - lo < 1:
                    continue
                A, B = self._at(lo), self._at(hi)
                p0, n0 = self._at(lo - 1), self._at(hi + 1)
                Br, Ar = (B[0], not B[1]), (A[0], not A[1])
                d = self.T(p0, Br) + self.T(Ar, n0) - self.T(p0, A) - self.T(B, n0)
                if d < -1e-6:      # (coverage first only where the route gets shorter: it is the costly part)
                    d += self._cov_delta(self._flips(self.seq[lo:hi + 1]))
                if d < -1e-6 and (best is None or d < best[0]):
                    best = (d, '2opt', lo, hi)
            if best:
                if best[1] == 'move':
                    _, _, j, o, h = best
                    if self.pieces[i].point and self.pieces[i].aim is None:
                        self.pieces[i].h = h         # the heading the move's cost was computed with
                    item = self.seq.pop(k)
                    if o != item[1]:
                        self._swap_counts(self._flips([item]))
                    item[1] = o
                    self.seq.insert(j + 1 if j < k else j, item)
                else:
                    _, _, lo, hi = best
                    self._swap_counts(self._flips(self.seq[lo:hi + 1]))
                    block = [[i2, not r2] for i2, r2 in reversed(self.seq[lo:hi + 1])]
                    self.seq[lo:hi + 1] = block
                applied += 1
                continue
            k += 1
        return applied

    # --------------------------------------------------------------- trim
    @staticmethod
    def _cut(pts, d, from_end):
        """Polyline with length d removed from one end (None if nothing would remain)."""
        pts = [list(p) for p in (pts[::-1] if from_end else pts)]
        while len(pts) >= 2:
            seg = math.dist(pts[0], pts[1])
            if seg > d + EPS:
                f = d / seg
                pts[0] = [pts[0][0] + f * (pts[1][0] - pts[0][0]), pts[0][1] + f * (pts[1][1] - pts[0][1])]
                break
            d -= seg
            pts.pop(0)
        if len(pts) < 2 or all(math.dist(pts[0], q) < EPS for q in pts):
            return None
        return pts[::-1] if from_end else pts

    def _replace_delta(self, k, q):
        """Exact cost change of driving piece q instead of the piece at position k, in the same direction
        (q is registered in self.pieces; returns (delta, index of q))."""
        i, rv = self.seq[k]
        p = self.pieces[i]
        prev, nxt = self._at(k - 1), self._at(k + 1)
        self.pieces.append(q)
        qi = len(self.pieces) - 1
        dt = (q.internal - p.internal + self.T(prev, (qi, rv)) + self.T((qi, rv), nxt)
              - self.T(prev, (i, rv)) - self.T((i, rv), nxt))
        return dt + self._cov_delta([(p.cells(rv), q.cells(rv))]), qi

    def _commit(self, k, qi):
        i, rv = self.seq[k]
        self._swap_counts([(self.pieces[i].cells(rv), self.pieces[qi].cells(rv))])
        self.seq[k][0] = qi

    def trim(self, deadline, step):
        """Shorten open pieces from either end while that lowers the cost."""
        applied = 0
        for k in range(len(self.seq)):
            if time.monotonic() > deadline:
                break
            for from_end in (False, True):
                for _ in range(20):
                    i, rv = self.seq[k]
                    p = self.pieces[i]
                    if p.point:                  # closed loops may be opened: their seam double-covers
                        break
                    cut = self._cut(p.pts, step, from_end)
                    if cut is None:
                        break
                    d, qi = self._replace_delta(k, self._measure(Piece(cut, p.kind)))
                    if d < -1e-6:
                        self._commit(k, qi)
                        applied += 1
                    else:
                        self.pieces.pop()
                        break
        return applied

    # -------------------------------------------------------------- orient
    def _heading_between(self, a, b):
        """Direction the robot drives in when leaving a towards b (via the connector)."""
        if math.dist(a, b) < EPS:
            return None
        c = self._connector(a, b)
        return None if c is None else (c[1], c[2])

    def orient(self, deadline):
        """Give every single goal the heading that makes passing it cheapest: the
        way the robot arrives, or the way it leaves. Then the turn at the goal
        (none when it drives straight through, a full U-turn when it has to go
        back the way it came) is part of every cost comparison."""
        applied = 0
        for k in range(len(self.seq)):
            if time.monotonic() > deadline:
                break
            i, _ = self.seq[k]
            p = self.pieces[i]
            if not p.point or p.aim is not None:
                continue
            prev, nxt = self._at(k - 1), self._at(k + 1)
            xy = p.pts[0]
            cands = set()
            if prev is not None or self.start is not None:
                a = self.start if prev is None else self.pieces[prev[0]].state(prev[1])[2]
                hb = self._heading_between(a, xy)
                if hb:
                    cands.add(round(hb[1], 6))       # arrival heading
                elif prev is not None and self.pieces[prev[0]].state(prev[1])[3] is not None:
                    cands.add(round(self.pieces[prev[0]].state(prev[1])[3], 6))
            if nxt is not None:
                hb = self._heading_between(xy, self.pieces[nxt[0]].state(nxt[1])[0])
                if hb:
                    cands.add(round(hb[0], 6))       # departure heading
            base = self.T(prev, (i, False)) + self.T((i, False), nxt)
            best = None
            for h in cands:
                q = Piece([xy], p.kind)
                q.covs, q.internal, q.h = p.covs, p.internal, h
                self.pieces.append(q)
                qi = len(self.pieces) - 1
                c = self.T(prev, (qi, False)) + self.T((qi, False), nxt)
                self.pieces.pop()
                if c < base - 1e-6 and (best is None or c < best[0]):
                    best = (c, q)
            if best:
                self.pieces.append(best[1])
                self.seq[k] = [len(self.pieces) - 1, False]   # same cells: coverage counts unchanged
                applied += 1
        return applied

    # -------------------------------------------------------------- rotate
    def rotate(self, deadline, candidates=8):
        """Closed loops (wall loop, loops around obstacles, rings) can be entered
        anywhere. Start each loop at the point that makes getting there and
        leaving it cheapest, instead of where it happened to be generated."""
        applied = 0
        for k in range(len(self.seq)):
            if time.monotonic() > deadline:
                break
            i, rv = self.seq[k]
            p = self.pieces[i]
            if p.point or len(p.pts) < 4 or math.dist(p.pts[0], p.pts[-1]) > EPS:
                continue
            prev, nxt = self._at(k - 1), self._at(k + 1)
            a = self.start if prev is None else self.pieces[prev[0]].state(prev[1])[2]
            b = None if nxt is None else self.pieces[nxt[0]].state(nxt[1])[0]
            ring = np.asarray(p.pts[:-1], float)
            d = np.linalg.norm(ring - a, axis=1) + (np.linalg.norm(ring - b, axis=1) if b is not None else 0)
            base = p.internal + self.T(prev, (i, rv)) + self.T((i, rv), nxt)
            best = None
            for j in np.argsort(d)[:candidates]:
                if j == 0:
                    continue
                pts = np.vstack([ring[j:], ring[:j], ring[j:j + 1]]).tolist()
                q = Piece(pts, p.kind)
                q.covs = p.covs                          # nearly the same cells: measured exactly below
                terms = costmod.route_terms(q.pts, self.s)
                q.internal = terms['drive_s'] + terms['turn_s'] + terms['reverse_s']
                self.pieces.append(q)
                qi = len(self.pieces) - 1
                for o in (False, True):
                    c = q.internal + self.T(prev, (qi, o)) + self.T((qi, o), nxt)
                    if c < base - 1e-6 and (best is None or c < best[0]):
                        best = (c, q, o)
                self.pieces.pop()
            if best:
                # Entered elsewhere (or driven the other way) the loop covers slightly different cells at its seam.
                c, q, o = best
                self._measure(q)
                swaps = [(p.cells(rv), q.cells(o))]
                if c - base + self._cov_delta(swaps) < -1e-6:
                    self.pieces.append(q)
                    self._swap_counts(swaps)
                    self.seq[k] = [len(self.pieces) - 1, o]
                    applied += 1
        return applied

    # ---------------------------------------------------------- straighten
    def _safe_dp(self, pts, tol):
        """Douglas-Peucker that only accepts a straight chord if the robot can drive it
        safely: fewer, longer straight lines, never closer to walls than allowed."""
        pts = np.asarray(pts, float)
        n = len(pts)
        keep = np.zeros(n, bool)
        keep[0] = keep[-1] = True
        stack = [(0, n - 1)]
        while stack:
            i, j = stack.pop()
            if j <= i + 1:
                continue
            a, b = pts[i], pts[j]
            ab = b - a
            L = math.hypot(*ab)
            seg = pts[i + 1:j] - a
            dev = np.abs(ab[0] * seg[:, 1] - ab[1] * seg[:, 0]) / L if L > 1e-12 else np.linalg.norm(seg, axis=1)
            m = int(np.argmax(dev))
            if dev[m] <= tol and L > 1e-12 and self.g.segment_safe(a, b, self.collision):
                continue
            keep[i + 1 + m] = True
            stack += [(i, i + 1 + m), (i + 1 + m, j)]
        return pts[keep].tolist()

    def straighten(self, deadline):
        """Replace a piece by a straighter version of itself (fewer, longer straight
        lines) whenever the cost function says that is cheaper overall. The straight
        line may leave the exact contour: brush over walls is free, while any floor it
        no longer reaches is charged as missed area."""
        applied = 0
        res = self.g.resolution
        tols = sorted({res, 2 * res, 0.5 * self.r, self.r, 2 * self.r, 4 * self.r})
        for k in range(len(self.seq)):
            if time.monotonic() > deadline:
                break
            i, rv = self.seq[k]
            p = self.pieces[i]
            if p.point or len(p.pts) < 3 or getattr(p, 'straight', False):
                continue
            best, tried = None, set()
            for tol in tols[::-1]:              # straightest first
                q = self._safe_dp(p.pts, tol)
                key = tuple(map(tuple, q))
                if len(q) >= len(p.pts) or key in tried:
                    continue
                tried.add(key)
                d, qi = self._replace_delta(k, self._measure(Piece(q, p.kind)))
                self.pieces.pop()
                if d < -1e-6 and (best is None or d < best[0]):
                    best = (d, q)
            if best:
                q = self._measure(Piece(best[1], p.kind))
                q.straight = True
                self.pieces.append(q)
                self._commit(k, len(self.pieces) - 1)
                applied += 1
            else:
                p.straight = True               # nothing better: do not try this piece again
        return applied

    # ---------------------------------------------------- block moves
    def block_moves(self, deadline):
        """Move runs of 2-3 consecutive pieces elsewhere (optionally reversed)."""
        applied = 0
        for L in (2, 3):
            k = 0
            while k + L <= len(self.seq):
                if time.monotonic() > deadline or applied > 10 * len(self.seq):   # (never loop for ever)
                    return applied
                first, last = tuple(self.seq[k]), tuple(self.seq[k + L - 1])
                prev, nxt = self._at(k - 1), self._at(k + L)
                removal = self.T(prev, nxt) - self.T(prev, first) - self.T(last, nxt)
                best = None
                for j in self._neighbour_positions(k):
                    if k - 1 <= j <= k + L - 1:
                        continue
                    a, b = self._at(j), self._at(j + 1)
                    for rev in (False, True):
                        f, l = ((last[0], not last[1]), (first[0], not first[1])) if rev else (first, last)
                        d = removal + self.T(a, f) + self.T(l, b) - self.T(a, b)
                        if rev and d < -1e-6:
                            d += self._cov_delta(self._flips(self.seq[k:k + L]))
                        if d < -1e-6 and (best is None or d < best[0]):
                            best = (d, j, rev)
                if best:
                    _, j, rev = best
                    block = self.seq[k:k + L]
                    if rev:
                        self._swap_counts(self._flips(block))
                        block = [[i2, not r2] for i2, r2 in reversed(block)]
                    del self.seq[k:k + L]
                    pos = j + 1 if j < k else j + 1 - L
                    self.seq[pos:pos] = block
                    applied += 1
                    continue
                k += 1
        return applied

    def local_optimum_done(self, deadline, step):
        """Make sure the route is at a local optimum (e.g. after the last detours)."""
        self.local_optimum(deadline + 2.0, step)        # small grace period: never stop mid-repair
        return time.monotonic() < deadline

    def rechoose(self, deadline):
        """Drive-ins chosen again in the route as it is now: every other candidate for the same pocket, and the best
        drive-in starting where the route already is (the end of the piece before it, the start of the piece after
        it), in its place. Kept when the total cost drops (robot 2026-10-20: after a drive-in the route drove 18 cm on
        to a lane start and turned 155 deg there; a drive-in from that lane start covers the same pocket)."""
        applied = 0
        for k in range(len(self.seq)):
            if time.monotonic() > deadline:
                break
            i, rv = self.seq[k]
            p = self.pieces[i]
            if p.spur is None or p.pocket is None:
                continue
            options = [(c[0], c[1]) for c in self.drive_ins if c[3] == p.pocket]
            for nb, end in ((self._at(k - 1), 2), (self._at(k + 1), 0)):
                if nb is not None:
                    xy = self.pieces[nb[0]].state(nb[1])[end]
                    line = spurs.lines_from(self.g, self.den, self.pockets[p.pocket], xy, self.s, self._outline)
                    if line:
                        options.append(line)
            best = None
            for a, b in options:
                if math.dist(a, p.pts[0]) < EPS and math.dist(b, p.spur) < EPS:
                    continue
                d, qi = self._replace_delta(k, self._measure(drivein(a, b, p.pocket)))
                self.pieces.pop()
                if d < -1e-6 and (best is None or d < best[0]):
                    best = (d, a, b)
            if best:
                q = self._measure(drivein(best[1], best[2], p.pocket))
                self.pieces.append(q)
                self._commit(k, len(self.pieces) - 1)
                applied += 1
        return applied

    def _away_from_walls(self):
        """Unit vectors (x, y per cell) pointing away from the nearest wall: the gradient of the smoothed clearance."""
        if getattr(self, '_away', None) is None:
            from scipy.ndimage import gaussian_filter
            self._clear_s = gaussian_filter(self.g.clearance.astype(float), 1.0)
            gy, gx = np.gradient(self._clear_s)
            c, s = math.cos(self.g.origin[2]), math.sin(self.g.origin[2])
            wx, wy = c * gx - s * gy, s * gx + c * gy          # grid axes -> world axes
            n = np.hypot(wx, wy)
            n[n < 1e-9] = np.inf
            self._away = (wx / n, wy / n)
        return self._away

    def shift(self, deadline):
        """Stretches of a piece that run at the safety limit along a wall, moved further from the wall (as far as
        the sensor still reaches the wall floor), each stretch on its own, when that lowers the total cost (user
        2026-10-22: the cost decides per stretch how close to a wall the route drives). A line at the limit follows the
        map's staircase along diagonal walls; a few cm further in, the straighten move can make it straight."""
        R = self.collision
        room = max(0.0, self.r - R)
        levels = int(self.s['optimizer'].get('shift_levels', 0))
        if room <= 0 or levels <= 0:
            return 0
        steps = [room * (j + 1) / levels for j in range(levels)]
        window = float(self.s['optimizer'].get('shift_window_m', 2.0))
        ax, ay = self._away_from_walls()
        applied = 0
        for k in range(len(self.seq)):
            if time.monotonic() > deadline:
                break
            i, rv = self.seq[k]
            p = self.pieces[i]
            if p.point or p.spur is not None or p.aim is not None or len(p.pts) < 3:
                continue
            pts = np.asarray(p.pts, float)
            rc = [self.g.cell(q) for q in pts]
            near = np.array([self.g.valid(c) and self.g.clearance[c] < R + 0.5 * room for c in rc])
            # Stretches of consecutive points at the limit, cut into windows of up to `window` metres.
            best, start, spans = None, None, []
            for j in range(len(pts) + 1):
                if j < len(pts) and near[j]:
                    start = j if start is None else start
                    continue
                if start is not None and j - start >= 2:
                    a = start
                    while a < j - 1:
                        b, length = a, 0.0
                        while b + 1 < j and length + np.linalg.norm(pts[b + 1] - pts[b]) <= window:
                            length += np.linalg.norm(pts[b + 1] - pts[b])
                            b += 1
                        if b > a:
                            spans.append((a, b))
                        a = max(b, a + 1)
                start = None
            safe = [self.g.segment_safe(pts[m], pts[m + 1], R) for m in range(len(pts) - 1)]
            for a, b in spans:
                for d in steps:
                    q = pts.copy()
                    for j in range(a, b + 1):
                        r, c = rc[j]
                        q[j] = pts[j] + d * np.array([ax[r, c], ay[r, c]])
                    ql = q.tolist()
                    lo, hi = max(0, a - 1), min(len(ql) - 1, b + 1)
                    # Never less safe than before (the optimiser's pieces are made safe later, in sanitize).
                    if not all(self.g.segment_safe(ql[m], ql[m + 1], R) or not safe[m] for m in range(lo, hi)):
                        continue
                    # The shifted stretch as it is, and straightened (where the gain usually is).
                    for v in [ql] + [self._safe_dp(ql, tol) for tol in (self.g.resolution, 2 * self.g.resolution)]:
                        cand = self._measure(Piece(v, p.kind))
                        delta, qi = self._replace_delta(k, cand)
                        self.pieces.pop()
                        if delta < -1e-6 and (best is None or delta < best[0]):
                            best = (delta, cand)
            if best:
                self.pieces.append(best[1])
                self._commit(k, len(self.pieces) - 1)
                applied += 1
        return applied

    # ------------------------------------------------------------- extend
    def _wall_trace(self, e, u, length):
        """From end point e, heading on along u: a line that follows the wall at e's distance from it (the
        smoothed clearance), up to `length` m; stops where it would come closer than the robot radius."""
        ax, ay = self._away_from_walls()
        R, step = self.collision, self.g.resolution
        level = max(R, float(self._clear_s[self.g.cell(e)]))
        p, d, out, run = np.asarray(e, float), np.asarray(u, float), [list(map(float, e))], 0.0
        while run < length - 1e-9:
            rc = self.g.cell(p)
            if not self.g.valid(rc):
                break
            n = np.array([ax[rc], ay[rc]])
            if not np.isfinite(n).all():
                break
            t = np.array([-n[1], n[0]])
            t = t if t @ d >= 0 else -t
            if t @ d < 0.2:                          # the wall turns away more than ~80 deg: not "on along it"
                break
            corr = max(-step, min(step, level - float(self._clear_s[rc])))
            q = p + step * t + corr * n
            if not self.g.segment_safe(p.tolist(), q.tolist(), R):
                break
            run += float(np.linalg.norm(q - p))
            d, p = (q - p) / max(np.linalg.norm(q - p), 1e-9), q
            out.append(p.tolist())
        return out

    def _extensions(self, pts):
        """Longer versions of a polyline: each end continued straight on and along the wall, at several lengths
        up to extend_max_m (user 2026-10-22: a line may grow where that saves pieces and stops)."""
        opt = self.s['optimizer']
        longest = float(opt.get('extend_max_m', 0.0))
        if longest <= 0:
            return []
        lengths = [x for x in (0.1, 0.2, 0.35, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0) if x <= longest + 1e-9]
        out = []
        for at_end in (True, False):
            line = pts if at_end else pts[::-1]
            e, b = np.asarray(line[-1], float), np.asarray(line[-2], float)
            if np.linalg.norm(e - b) < EPS:
                continue
            u = (e - b) / np.linalg.norm(e - b)
            for L in lengths:                        # straight on
                q = (e + L * u).tolist()
                if not self.g.segment_safe(e.tolist(), q, self.collision):
                    break
                new = list(line) + [q]
                out.append(new if at_end else new[::-1])
            trace = self._wall_trace(e, u, lengths[-1])   # along the wall
            if len(trace) < 3:
                continue
            run = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(np.asarray(trace), axis=0), axis=1))]
            for L in lengths:
                n = int(np.searchsorted(run, L - 1e-9)) + 1
                if n > len(trace):
                    break
                part = self._safe_dp(trace[:n], self.g.resolution)
                new = list(line) + part[1:]
                out.append(new if at_end else new[::-1])
        return out

    def _cleanup(self, grown, deadline, trims=True):
        """After piece `grown` got longer: drop (and with `trims` shorten) other pieces that cover the same floor,
        while that lowers the cost. Returns the exact cost change of what it did."""
        delta, step = 0.0, 0.5 * self.r
        cells = np.asarray(grown.cells(False))
        reach = self.r + sensor.offset(self.s) + self.g.resolution
        lo, hi = np.min(grown.pts, axis=0) - reach, np.max(grown.pts, axis=0) + reach
        own = next(n for n in range(len(self.pieces) - 1, -1, -1) if self.pieces[n] is grown)
        changed = True
        while changed and time.monotonic() < deadline:
            changed = False
            for k in range(len(self.seq)):
                i, rv = self.seq[k]
                if i == own:
                    continue
                pts = np.asarray(self.pieces[i].pts)
                if (pts.max(axis=0) + reach < lo).any() or (pts.min(axis=0) - reach > hi).any() \
                        or not len(np.intersect1d(self.pieces[i].cells(rv), cells)):
                    continue
                d = self._drop_delta(k)
                if d < -1e-6:
                    self._drop(k)
                    delta += d
                    changed = True
                    break
                p = self.pieces[i]
                if not trims or p.point or p.spur is not None or p.aim is not None:
                    continue
                for from_end in (False, True):
                    for _ in range(20):
                        cut = self._cut(self.pieces[self.seq[k][0]].pts, step, from_end)
                        if cut is None:
                            break
                        d, qi = self._replace_delta(k, self._measure(Piece(cut, p.kind)))
                        if d < -1e-6:
                            self._commit(k, qi)
                            delta += d
                            changed = True
                        else:
                            self.pieces.pop()
                            break
        return delta

    def _save(self):
        return [list(x) for x in self.seq], self.count.copy(), len(self.pieces), self.cov_ok.copy()

    def _restore(self, saved):
        self.seq, self.count, self.cov_ok = [list(x) for x in saved[0]], saved[1].copy(), saved[3].copy()
        del self.pieces[saved[2]:]

    def extend(self, deadline, positions=None, min_gain=1e-6):
        """Lengthen pieces at their ends (straight on or along the wall) and in the same move drop or shorten the
        pieces that are then not needed; kept only when that lowers the total cost (user 2026-10-22: e.g. one line
        up a corridor end instead of three short crossing pieces)."""
        applied = 0
        for k in (range(len(self.seq)) if positions is None else positions):
            if time.monotonic() > deadline or k >= len(self.seq):
                break
            i, rv = self.seq[k]
            p = self.pieces[i]
            if p.point or p.spur is not None or p.aim is not None or len(p.pts) < 2 \
                    or math.dist(p.pts[0], p.pts[-1]) < EPS:      # closed loops are entered by rotate()
                continue
            def apply(pts, trims):
                q = self._measure(Piece(pts, p.kind))
                d, qi = self._replace_delta(k, q)
                self._commit(k, qi)
                return d + self._cleanup(q, deadline, trims)
            # Every candidate with drops only (fast), then the most promising with shortening too.
            scored = []
            for pts in self._extensions(p.pts):
                if time.monotonic() > deadline:
                    break
                saved = self._save()
                scored.append((apply(pts, False), len(scored), pts))
                self._restore(saved)
            best = None
            for _, _, pts in sorted(scored)[:3]:
                saved = self._save()
                d = apply(pts, True)
                if d < -min_gain and (best is None or d < best[0]):
                    best = (d, pts)
                self._restore(saved)
            if best:
                apply(best[1], True)                     # the same steps again (they are deterministic)
                applied += 1
        return applied

    # ------------------------------------------------------------- rebuild
    def _stop_points(self):
        """Where the route stops to turn: at junctions (and their connectors) and at sharp corners inside pieces."""
        out, prev = [], None
        for st in self.seq:
            st = tuple(st)
            p = self.pieces[st[0]]
            sxy, sh, exy, eh = p.state(st[1])
            if prev is not None:
                _, _, pxy, ph = self.pieces[prev[0]].state(prev[1])
                if math.dist(pxy, sxy) < EPS:
                    if self._turn(ph, sh) > self.smooth * self.bend + 1e-9:
                        out.append(sxy)
                else:
                    c = self._connector(pxy, sxy)
                    if c is not None:
                        if self._turn(ph, c[1]) > self.smooth * self.bend + 1e-9:
                            out.append(pxy)
                        if self._turn(c[2], sh) > self.smooth * self.bend + 1e-9:
                            out.append(sxy)
            if p.point or p.aim is not None or p.spur is not None:
                out.append(p.pts[0])
            elif len(p.pts) > 2:
                a = np.asarray(p.pts, float)
                dv = np.diff(a, axis=0)
                seg = np.hypot(dv[:, 0], dv[:, 1])
                keep = seg > 1e-6
                a2 = np.vstack([a[:1], a[1:][keep]])
                dv, seg = dv[keep], seg[keep]
                if len(dv) > 1:
                    h = np.arctan2(dv[:, 1], dv[:, 0])
                    dh = np.abs(np.arctan2(np.sin(np.diff(h)), np.cos(np.diff(h))))
                    _, stop = costmod.vertex_cost(dh, costmod.corner_radius(seg[:-1], seg[1:], dh), self.s)
                    out.extend(a2[1:-1][stop].tolist())
            prev = st
        return np.asarray(out, float).reshape(-1, 2)

    def _clusters(self):
        """Square areas (rebuild_window_m) around the densest groups of stops, worst first, not overlapping."""
        opt = self.s['optimizer']
        half = 0.5 * float(opt.get('rebuild_window_m', 2.0))
        need = int(opt.get('rebuild_min_stops', 3))
        pts = self._stop_points()
        left = np.ones(len(pts), bool)
        out = []
        while left.any():
            inside = [(np.abs(pts - c) <= half).all(axis=1) & left for c in pts]
            score = np.array([m.sum() if left[n] else 0 for n, m in enumerate(inside)])
            n = int(np.argmax(score))
            if score[n] < need:
                break
            members = pts[inside[n]]
            out.append(((members.min(axis=0) + members.max(axis=0)) / 2, int(score[n])))
            left &= ~(np.abs(pts - out[-1][0]) <= half).all(axis=1)
        return out

    def rebuild(self, deadline, step):
        """Remove and repair (user 2026-10-22): in a square around a cluster of stops, take out every piece inside,
        then cover the floor again with the planner's own repair moves (extend the pieces around it, detours: lines,
        turns on the spot, drive-ins) and re-optimise. Kept only when the whole route gets cheaper. Repeats until no
        area improves."""
        half = 0.5 * float(self.s['optimizer'].get('rebuild_window_m', 2.0))
        improved = 0
        tried = self.__dict__.setdefault('_rebuild_tried', set())   # (area, route) already tried without gain
        while time.monotonic() < deadline:
            found = False
            route = hash(tuple(map(tuple, self.seq)))
            for centre, _ in self._clusters():
                key = (round(centre[0], 1), round(centre[1], 1), route)
                if key in tried or time.monotonic() > deadline:
                    continue
                tried.add(key)
                before = self.total()[0]
                saved = self._save()
                inside = [k for k, (i, _) in enumerate(self.seq)
                          if (np.abs(np.asarray(self.pieces[i].pts) - centre) <= half).all()]
                if not inside:
                    continue
                for k in reversed(inside):
                    self._drop(k)
                near = [k for k, (i, rv) in enumerate(self.seq)
                        if any((np.abs(np.asarray(xy) - centre) <= half + self.r).all()
                               for xy in (self.pieces[i].pts[0], self.pieces[i].pts[-1]))]
                self.extend(deadline, near)
                self.detours(deadline)
                gain = float(self.s['optimizer'].get('rebuild_min_gain_s', 0.0))
                self.local_optimum(deadline, step, gain)
                if self.total()[0] < before - max(gain, 1e-6):   # tiny gains: never ending rounds (small room)
                    improved += 1
                    found = True
                    break                                # the stops moved: look for clusters again
                self._restore(saved)
            if not found:
                break
        return improved

    def local_optimum(self, deadline, step, min_gain=0.0):
        """Local moves until none helps. min_gain > 0: also stop after a round that saved less than that (s); on a
        large map the rounds can go on for a long time with tiny gains (RDG 2026-10-22: 25 min)."""
        while time.monotonic() < deadline:
            before = self.total()[0] if min_gain > 0 else 0.0
            n = (self.orient(deadline) + self.shift(deadline) + self.straighten(deadline) + self.improve_once(deadline)
                 + self.block_moves(deadline) + self.rotate(deadline) + self.orient(deadline)
                 + self.trim(deadline, step) + self.trim(deadline, self.g.resolution)   # also by single cells
                 + self.rechoose(deadline))
            if n == 0 or (min_gain > 0 and before - self.total()[0] < min_gain):
                break

    def reaim(self, deadline, step):
        """Take out each aimed goal and cover its floor again from the route as it is now ordered: an aim can
        then become a turn on the spot at a lane end (_aims_at_route) instead of a drive back to a corner.
        Kept only when the total cost drops."""
        improved = 0
        k = 0
        while k < len(self.seq) and time.monotonic() < deadline:
            if self.pieces[self.seq[k][0]].aim is None:
                k += 1
                continue
            saved = ([list(x) for x in self.seq], self.count.copy(), len(self.pieces), self.cov_ok.copy())
            before = self.total()[0]
            self._drop(k)
            self.detours(deadline)
            self.local_optimum(deadline, step)
            if self.total()[0] < before - 1e-6:
                improved += 1
                k = 0                            # the route changed: look at every aim again
            else:
                self.seq, self.count, self.cov_ok = saved[0], saved[1], saved[3]
                del self.pieces[saved[2]:]
                k += 1
        return improved

    def perturb(self, rng):
        """Random 2-opt reversal plus a random relocation (the caller keeps it only if the total is cheaper)."""
        n = len(self.seq)
        if n < 4:
            return
        i, j = sorted(rng.choice(n, 2, replace=False))
        self._swap_counts(self._flips(self.seq[i:j + 1]))
        self.seq[i:j + 1] = [[a, not r] for a, r in reversed(self.seq[i:j + 1])]
        k = int(rng.integers(n))
        item = self.seq.pop(k)
        self.seq.insert(int(rng.integers(n)), item)

    # ------------------------------------------------------------- detours
    def detours(self, deadline, max_new=500):
        """Add a goal (a single pose, or a line along a missed strip) for missed
        cells when that lowers the total cost."""
        g = self.g
        off = sensor.offset(self.s)
        r = sensor.point_radius(self.r, off)   # a single pose: sensor heading unknown
        reach_xy = g.world(np.argwhere(self.reach))
        added, declined = 0, set()
        while added < max_new and time.monotonic() < deadline:
            missed = np.flatnonzero(self.cov_ok & (self.count == 0))
            missed = [m for m in missed if m not in declined]
            if not missed:
                break
            m = missed[0]
            rc = divmod(int(m), g.shape[1])
            p = g.world(rc)
            cand = reach_xy[np.linalg.norm(reach_xy - p, axis=1) <= r + 1e-9]
            open_idx = np.array(missed)
            open_xy = g.world(np.stack(np.divmod(open_idx, g.shape[1]), axis=1))
            best = None
            for q in cand:
                near = np.flatnonzero(np.linalg.norm(open_xy - q, axis=1) <= r + 1e-9)
                if not len(near):
                    continue
                vis = near[visible_many(g, np.tile(q, (len(near), 1)), open_xy[near])]
                if m in set(open_idx[vis]) and (best is None or len(vis) > best[1]):
                    best = (q, len(vis))
            aim = self._aim_at(p, open_xy, reach_xy) if off > 0 else None
            ends = np.array([self.start] + [self.pieces[i].state(rv)[2] for i, rv in self.seq])
            at_route = self._aims_at_route(p, open_xy, ends) if off > 0 else []
            drive_ins = [drivein(self.drive_ins[n][0], self.drive_ins[n][1], self.drive_ins[n][3])
                         for n in self._drive_in_of.get(int(m), [])]
            if best is None and aim is None and not at_route and not drive_ins:
                self.cov_ok[m] = False           # no safe pose can see this cell and no drive-in reaches it
                continue
            options = drive_ins + ([Piece([best[0].tolist()], 'gap')] if best is not None else []) + \
                ([aimed(*aim)] if aim is not None else []) + \
                [aimed(xy, h, AIM_AT_ROUTE_RUN_M) for xy, h in at_route] + self._strip_lines(m, open_idx)
            choice = None
            for piece in options:
                self._measure(piece)                 # evaluated before it is counted
                self.pieces.append(piece)
                idx = len(self.pieces) - 1
                anchor = np.asarray(piece.pts[0])
                order = np.argsort(np.linalg.norm(ends - anchor, axis=1))[:self.K]
                for o in ((False,) if piece.point else (False, True)):
                    cells = piece.cells(o)
                    new = cells[self.count[cells] == 0]
                    if not self.cov_ok[new].any():
                        continue
                    gain = self._miss(new)
                    over = self.k_over * self.cell * (len(cells) - len(new))
                    for pos in order:       # insert after position pos-1 (pos=0 -> right after start)
                        a, b = self._at(pos - 1) if pos > 0 else None, self._at(pos)
                        d = self._slot_cost(a, (idx, o), b) - self.T(a, b)
                        delta = piece.internal + d + over - gain
                        if choice is None or delta < choice[0]:
                            choice = (delta, idx, pos, o, new, self.slot_heading)
            if choice is not None and choice[0] < 0:
                _, idx, pos, o, _, h = choice
                if self.pieces[idx].point and self.pieces[idx].aim is None:
                    self.pieces[idx].h = h           # the heading its cost was computed with
                self.count[self.pieces[idx].cells(o)] += 1
                self.seq.insert(int(pos), [idx, o])
                added += 1
            else:
                declined.update(int(x) for x in (choice[4] if choice else [m]))
        return added

    def _aim_at(self, p, open_xy, reach_xy, max_candidates=120):
        """An aimed goal for missed cell p: a reachable robot position q and heading u (pointing at p) with
        the sensor q + offset * u within the brush radius of p, a safe short forward move into q, and the
        most missed cells in sight of the sensor. (xy, heading) or None."""
        g, off = self.g, sensor.offset(self.s)
        d = np.linalg.norm(reach_xy - p, axis=1)
        cand = reach_xy[(d <= self.r + off + 1e-9) & (d > 1e-9)]
        if len(cand) > max_candidates:
            cand = cand[np.linspace(0, len(cand) - 1, max_candidates).astype(int)]
        best = None
        for q in cand:
            u = (p - q) / np.linalg.norm(p - q)
            s = q + off * u
            if np.linalg.norm(p - s) > self.r + 1e-9 or not visible_many(g, s[None], p[None])[0]:
                continue
            if not g.segment_safe((q - AIM_RUN_M * u).tolist(), q.tolist(), self.collision):
                continue
            near = np.flatnonzero(np.linalg.norm(open_xy - s, axis=1) <= self.r + 1e-9)
            n = int(visible_many(g, np.tile(s, (len(near), 1)), open_xy[near]).sum()) if len(near) else 0
            if best is None or n > best[0]:
                best = (n, q.tolist(), math.atan2(u[1], u[0]))
        return None if best is None else (best[1], best[2])

    def _aims_at_route(self, p, open_xy, ends, max_aims=2):
        """Aimed goals where the route already is: at a piece end the robot turns on the spot towards missed
        cell p (and moves AIM_AT_ROUTE_RUN_M). Without them an aimed goal chosen only by "most missed cells in
        sight" lay just behind a lane end, and the cheapest order was to come back to that corner later (room
        Test, 2026-10-19). [(xy, heading), ...] for the ends that see the most missed cells."""
        g, off = self.g, sensor.offset(self.s)
        found = []
        for e in ends:
            e = np.asarray(e, float)
            if not 1e-9 < np.linalg.norm(p - e) <= self.r + off + AIM_AT_ROUTE_RUN_M + 1e-9:
                continue
            # Every heading in 5 deg steps, not only straight at p: in a corner the cell can be hidden behind the
            # wall corner from that one sensor position, but seen from a slightly different heading.
            best = None
            for h in np.radians(np.arange(0., 360., 5.)):
                u = np.array([math.cos(h), math.sin(h)])
                q = e + AIM_AT_ROUTE_RUN_M * u
                sensor_xy = q + off * u
                if np.linalg.norm(p - sensor_xy) > self.r + 1e-9 or not visible_many(g, sensor_xy[None], p[None])[0]:
                    continue
                if not g.segment_safe(e.tolist(), q.tolist(), self.collision):
                    continue
                near = np.flatnonzero(np.linalg.norm(open_xy - sensor_xy, axis=1) <= self.r + 1e-9)
                n = int(visible_many(g, np.tile(sensor_xy, (len(near), 1)), open_xy[near]).sum()) if len(near) else 0
                if best is None or n > best[0]:
                    best = (n, q.tolist(), float(h))
            if best is not None:
                found.append(best)
        found.sort(key=lambda item: -item[0])
        return [(xy, h) for _, xy, h in found[:max_aims]]

    def _strip_lines(self, m, open_idx, max_lines=3):
        """Line detours for the strip of missed cells around cell m: centre lines of
        the robot positions that can reach it (a row of point detours along a strip
        means a stop at every point; one line along it is usually far cheaper)."""
        from scipy.ndimage import binary_dilation, label
        from .spiral import SIMPLIFY, _safe_parts, _simplify, _trace_skeleton, skeletonize
        g, r = self.g, self.r
        H, W = g.shape
        rc = np.stack(np.divmod(open_idx, W), axis=1)
        r0, c0 = divmod(int(m), W)
        win = int(math.ceil(1.5 / g.resolution))
        near = (np.abs(rc[:, 0] - r0) <= win) & (np.abs(rc[:, 1] - c0) <= win)
        if near.sum() < 3:
            return []
        pad = int(math.ceil(r / g.resolution)) + 2
        R0, C0 = max(0, r0 - win - pad), max(0, c0 - win - pad)
        R1, C1 = min(H, r0 + win + pad + 1), min(W, c0 + win + pad + 1)
        miss = np.zeros((R1 - R0, C1 - C0), bool)
        sel = rc[near]
        miss[sel[:, 0] - R0, sel[:, 1] - C0] = True
        lab, _ = label(miss, structure=np.ones((3, 3)))
        comp = lab == lab[r0 - R0, c0 - C0]
        if comp.sum() < 3:
            return []
        n = int(math.ceil(r / g.resolution))
        yy, xx = np.mgrid[-n:n + 1, -n:n + 1]
        disk = np.hypot(xx, yy) * g.resolution <= r
        pos = self.reach[R0:R1, C0:C1] & binary_dilation(comp, disk)
        out = []
        for line in _trace_skeleton(skeletonize(pos)):
            if len(line) < 3:
                continue
            pts = g.world(line + [R0, C0])
            for part in _safe_parts(g, [q.tolist() for q in _simplify(pts, SIMPLIFY * g.resolution)],
                                    self.collision):
                if len(part) > 1 and sum(math.dist(a, b) for a, b in zip(part, part[1:])) > g.resolution:
                    out.append(Piece(part, 'fill'))
        out.sort(key=lambda p: -sum(math.dist(a, b) for a, b in zip(p.pts, p.pts[1:])))
        return out[:max_lines]

    # ---------------------------------------------------------------- run
    def run(self, budget_s, seed=0, rebuild=False):
        """rebuild: afterwards also remove and repair stop clusters (rebuild), on its own time
        (optimizer.rebuild_time_s): used for the cheapest candidate only (pipeline polish)."""
        deadline = time.monotonic() + budget_s
        step = 0.5 * float(self.s['geometry']['coverage_disk_radius_m'])
        for _ in range(20):                      # detours + local moves until stable
            n = self.detours(deadline)
            before = self.total()[0]
            self.local_optimum(deadline, step)
            if n == 0 and self.total()[0] >= before - 1e-6 or time.monotonic() > deadline:
                break
        self.reaim(deadline, step)
        # use the remaining time: perturb, re-optimise, keep only improvements
        if not self.local_optimum_done(deadline, step):
            return self._rebuild_phase(step) if rebuild else self
        rng = np.random.default_rng(seed)
        best_cost = self.total()[0]
        while time.monotonic() < deadline:
            saved = ([list(x) for x in self.seq], self.count.copy(), len(self.pieces), self.cov_ok.copy())
            self.perturb(rng)
            self.local_optimum(deadline, step)
            if self.detours(deadline):
                self.local_optimum(deadline, step)     # new goals get ordered too
            c = self.total()[0]
            if c < best_cost - 1e-6:
                best_cost = c
            else:
                self.seq, self.count, self.cov_ok = saved[0], saved[1], saved[3]
                del self.pieces[saved[2]:]
        return self._rebuild_phase(step) if rebuild else self

    def _rebuild_phase(self, step):
        """Extend and remove-and-repair on their own time (optimizer.rebuild_time_s), until neither improves."""
        end = time.monotonic() + float(self.s['optimizer'].get('rebuild_time_s', 0.0))
        gain = max(float(self.s['optimizer'].get('rebuild_min_gain_s', 0.0)), 1e-6)
        while time.monotonic() < end:
            before = self.total()[0]
            if self.extend(end, min_gain=gain):
                self.local_optimum(end, step, gain)
            self.rebuild(end, step)
            if before - self.total()[0] < gain:      # a whole round saved less than rebuild_min_gain_s: done
                break
        return self

    def targets(self, max_seg):
        out = []
        for i, rv in self.seq:
            out.extend(self.pieces[i].targets(rv, max_seg))
        return out
