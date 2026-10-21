"""Map widget (QPainter): the map with coverage, heatmap, keepout, the actual trail, hotspots and the robot.

Touch: one finger drags the map, two fingers pinch to zoom, a short touch is a tap (tapped(x, y) in map metres,
or hotspot_tapped(id) on a hotspot). The mouse does the same (wheel = zoom).
"""
import math
import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
from . import style

TAP_PX = 12          # a touch that moves less than this is a tap


def _equal(a, b):
    """Same drawing data (positions, lists, numbers; numpy arrays by content)."""
    try:
        same = a == b
        if isinstance(same, np.ndarray):
            return np.shape(a) == np.shape(b) and bool(same.all())
        return bool(same)
    except (ValueError, TypeError):
        return False


def rgba_image(rgba):
    """QImage from an (rows, cols, 4) uint8 array; row 0 of the grid (lowest y) at the bottom of the image."""
    a = np.ascontiguousarray(rgba[::-1])
    image = QtGui.QImage(a.data, a.shape[1], a.shape[0], a.shape[1] * 4, QtGui.QImage.Format_RGBA8888)
    return image.copy()


def colour(hex_colour, alpha=255):
    c = QtGui.QColor(hex_colour)
    return [c.red(), c.green(), c.blue(), alpha]


def widen(value, cells):
    """Each reading also on the cells within `cells` of it (the highest one wins): easier to see on a small screen."""
    if cells <= 0:
        return value
    out = value.copy()
    filled = np.where(np.isnan(value), -np.inf, value)
    rows, cols = value.shape
    for dr in range(-cells, cells + 1):
        for dc in range(-cells, cells + 1):
            if dr * dr + dc * dc > cells * cells:
                continue
            shifted = np.full_like(filled, -np.inf)
            shifted[max(dr, 0):rows + min(dr, 0), max(dc, 0):cols + min(dc, 0)] = \
                filled[max(-dr, 0):rows + min(-dr, 0), max(-dc, 0):cols + min(-dc, 0)]
            out = np.where(shifted > np.nan_to_num(out, nan=-np.inf), shifted, out)
    return out


def heat_rgba(value, threshold):
    """Readings below the threshold from pale yellow to amber, contaminated (>= threshold) magenta."""
    rgba = np.zeros(value.shape + (4,), np.uint8)
    seen = ~np.isnan(value)
    if not seen.any():
        return rgba
    if threshold is None or not np.isfinite(threshold):
        rgba[seen] = colour(style.AMBER, 170)
        return rgba
    v = np.where(seen, value, 0.)
    f = np.clip(v / threshold, 0., 1.)[..., None]
    low, high = np.array(colour('#FFF1A8', 150), float), np.array(colour(style.AMBER, 210), float)
    rgba[seen] = (low + (high - low) * f)[seen].astype(np.uint8)
    rgba[seen & (v >= threshold)] = colour(style.MAG, 235)
    return rgba


class MapWidget(QtWidgets.QWidget):
    tapped = QtCore.pyqtSignal(float, float)
    hotspot_tapped = QtCore.pyqtSignal(int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(QtCore.Qt.WA_AcceptTouchEvents)
        self.setMinimumSize(200, 200)
        self._trail_mark = None
        self.grid = None
        self.base = None
        self.images = {}                       # layer name -> QImage on the grid
        self.layers = {'coverage': True, 'contamination': True, 'path': True, 'changes': True}
        self.heat_width_m = 0.             # display width of the readings around the sensor track
        self.trail = []                        # list of segments [[x, y], ...]
        self.plan = []
        self.route = []
        self.station = None
        self.footprint = []                    # robot outline [[x, y], ...] in the robot frame (robot.yaml)
        self.robot = None
        self.sensor_offset = 0.
        self.sensor_radius = 0.
        self.hotspots = []
        self.selected = None
        self.marks = {'start': None, 'area': [], 'area_closed': False, 'pending': None}
        self.scale, self.tx, self.ty = 1., 0., 0.
        self.press = None
        self.pinch = None
        # No zoom buttons: they did not work on the robot screen, the touch gestures do (user, 2026-10-19).

    # ---------------------------------------------------------------- data
    def set_grid(self, grid, fit=True):
        """fit: zoom to the whole map (False keeps the view, e.g. for app 2's growing map of the same size)."""
        self.grid = grid
        rgba = np.zeros(grid.cells.shape + (4,), np.uint8)
        rgba[:] = colour(style.BG)
        rgba[grid.cells == 0] = colour('#FFFFFF')
        rgba[grid.cells > 0] = colour(style.WALL)
        self.base = rgba_image(rgba)
        self.images = {}
        self.trail, self.plan, self.route, self.hotspots = [], [], [], []
        self._trail_mark = None
        self.robot = self.station = self.selected = None
        self.marks = {'start': None, 'area': [], 'area_closed': False, 'pending': None}
        if fit:
            self.fit()
        else:
            self.update()

    def set_mask(self, name, mask, hex_colour, alpha):
        if mask is None or self.grid is None or mask.shape != self.grid.cells.shape:
            self.images.pop(name, None)
        else:
            rgba = np.zeros(mask.shape + (4,), np.uint8)
            rgba[mask] = colour(hex_colour, alpha)
            self.images[name] = rgba_image(rgba)
        self.update()

    def set_changes(self, appeared, vanished):
        """Cells that changed against the saved map: new objects and removed objects."""
        if appeared is None or self.grid is None or appeared.shape != self.grid.cells.shape:
            self.images.pop('changes', None)
        else:
            rgba = np.zeros(appeared.shape + (4,), np.uint8)
            rgba[vanished] = colour(style.GONE, 230)
            rgba[appeared] = colour(style.NEW, 245)
            self.images['changes'] = rgba_image(rgba)
        self.update()

    def set_heatmap(self, value, threshold):
        if value is None or self.grid is None or value.shape != self.grid.cells.shape:
            self.images.pop('heat', None)
        else:
            cells = int(round(self.heat_width_m / 2 / self.grid.resolution))
            self.images['heat'] = rgba_image(heat_rgba(widen(value, cells), threshold))
        self.update()

    def refresh(self, **data):
        """New data to draw; repaints only when some of it changed (the screen asks every update_period; a repaint
        draws the whole map with every trail point, and cost 30-65 % of a Pi 4 core while nothing changed)."""
        changed = False
        for key, value in data.items():
            if key == 'trail':
                # The trail is one list that grows in place: compare its size and its newest point.
                mark = (len(value), len(value[-1]) if value else 0,
                        tuple(value[-1][-1]) if value and value[-1] else None)
                changed |= mark != self._trail_mark
                self._trail_mark = mark
            elif not _equal(getattr(self, key, None), value):
                changed = True
            setattr(self, key, value)
        if changed:
            self.update()

    # ---------------------------------------------------------------- view
    def fit(self):
        if self.grid is None:
            return
        corners = self._corners()
        w, h = max(self.width() - 80, 50), max(self.height() - 70, 50)
        span = corners.max(axis=0) - corners.min(axis=0)
        self.scale = min(w / span[0], h / span[1])
        cx, cy = corners.mean(axis=0)
        self.tx = self.width() / 2 - 30 - self.scale * cx
        self.ty = (self.height() + 50) / 2 + self.scale * cy
        self.update()

    def _corners(self):
        rows, cols = self.grid.cells.shape
        return self.grid.world(np.array([[-.5, -.5], [-.5, cols - .5], [rows - .5, -.5], [rows - .5, cols - .5]]))

    def zoom(self, factor, center=None):
        cx, cy = center if center is not None else (self.width() / 2, self.height() / 2)
        self.tx = cx - (cx - self.tx) * factor
        self.ty = cy - (cy - self.ty) * factor
        self.scale *= factor
        self.update()

    def to_screen(self, x, y):
        return QtCore.QPointF(self.tx + self.scale * x, self.ty - self.scale * y)

    def to_world(self, px, py):
        return (px - self.tx) / self.scale, (self.ty - py) / self.scale

    def resizeEvent(self, event):
        if self.grid is not None and event.oldSize().width() <= 0:
            self.fit()

    # ---------------------------------------------------------------- drawing
    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        p.fillRect(self.rect(), QtGui.QColor(style.BG))
        if self.grid is None:
            return
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        rows, _ = self.grid.cells.shape
        res = self.grid.resolution
        ox, oy, yaw = self.grid.origin
        c, s = math.cos(yaw), math.sin(yaw)
        image_to_local = QtGui.QTransform(res, 0, 0, -res, 0, res * rows)
        local_to_world = QtGui.QTransform(c, s, -s, c, ox, oy)
        world_to_screen = QtGui.QTransform(self.scale, 0, 0, -self.scale, self.tx, self.ty)
        p.save()
        p.setTransform(image_to_local * local_to_world * world_to_screen)
        p.drawImage(0, 0, self.base)
        for name, on in (('coverage', self.layers['coverage']), ('heat', self.layers['contamination']),
                         ('keepout', self.layers['contamination']), ('changes', self.layers['changes'])):
            if on and name in self.images:
                p.drawImage(0, 0, self.images[name])
        p.restore()
        if self.layers['path']:
            self._line(p, self.plan, QtGui.QColor(106, 63, 181, 110), 1.5, dashed=True)
            for segment in self.trail:
                self._line(p, segment, QtGui.QColor(11, 94, 138, 150), 2.5)
        self._marks(p)
        if self.station is not None:
            # Where and which way to put the robot: its outline (body + rod) dashed at the base station, and a
            # clear heading arrow (user, 2026-10-19: the old dark arrow on a dark square could not be seen).
            x, y, yaw = self.station
            c, s = math.cos(yaw), math.sin(yaw)
            if self.footprint:
                pen = QtGui.QPen(QtGui.QColor(style.INK), 2, QtCore.Qt.DashLine)
                p.setPen(pen)
                p.setBrush(QtGui.QColor(16, 37, 52, 35))
                p.drawPolygon(QtGui.QPolygonF([self.to_screen(x + c * fx - s * fy, y + s * fx + c * fy)
                                               for fx, fy in self.footprint]))
            self._arrow(p, self.station, QtGui.QColor(style.INK), min(34, max(20, self.scale * 0.3)))
        if self.layers['contamination']:
            for h in self.hotspots:
                self._diamond(p, h)
        if self.robot is not None:
            x, y, yaw = self.robot
            sensor = self.to_screen(x + self.sensor_offset * math.cos(yaw), y + self.sensor_offset * math.sin(yaw))
            pen = QtGui.QPen(QtGui.QColor(11, 94, 138, 130), 1.5, QtCore.Qt.DashLine)
            p.setPen(pen)
            p.setBrush(QtCore.Qt.NoBrush)
            r = self.sensor_radius * self.scale
            p.drawEllipse(sensor, r, r)
            self._arrow(p, self.robot, QtGui.QColor(style.BLUE), min(30, max(16, self.scale * 0.15)))

    def _line(self, p, points, qcolor, width, dashed=False):
        if len(points) < 2:
            return
        pen = QtGui.QPen(qcolor, width, QtCore.Qt.DashLine if dashed else QtCore.Qt.SolidLine)
        pen.setJoinStyle(QtCore.Qt.RoundJoin)
        p.setPen(pen)
        p.setBrush(QtCore.Qt.NoBrush)
        p.drawPolyline(QtGui.QPolygonF([self.to_screen(*pt[:2]) for pt in points]))

    def _arrow(self, p, pose, qcolor, z, outline=True):
        pt = self.to_screen(*pose[:2])
        p.save()
        p.translate(pt)
        p.rotate(-math.degrees(pose[2]))
        p.setBrush(qcolor)
        p.setPen(QtGui.QPen(QtGui.QColor('white'), 2.5) if outline else QtCore.Qt.NoPen)
        p.drawPolygon(QtGui.QPolygonF([QtCore.QPointF(z * .7, 0), QtCore.QPointF(-z * .5, z * .5),
                                       QtCore.QPointF(-z * .25, 0), QtCore.QPointF(-z * .5, -z * .5)]))
        p.restore()

    def _diamond(self, p, h):
        pt = self.to_screen(h['x'], h['y'])
        def shape(r):
            return QtGui.QPolygonF([QtCore.QPointF(pt.x(), pt.y() - r), QtCore.QPointF(pt.x() + r, pt.y()),
                                    QtCore.QPointF(pt.x(), pt.y() + r), QtCore.QPointF(pt.x() - r, pt.y())])
        if h['id'] == self.selected:
            p.setPen(QtGui.QPen(QtGui.QColor(style.INK), 3))
            p.setBrush(QtCore.Qt.NoBrush)
            p.drawPolygon(shape(26))
        p.setPen(QtGui.QPen(QtGui.QColor(style.MAG), 3))
        p.setBrush(QtGui.QColor('white' if h.get('checked') else style.MAG))
        p.drawPolygon(shape(19))
        p.setPen(QtGui.QColor(style.MAG if h.get('checked') else 'white'))
        font = p.font()
        font.setBold(True)
        p.setFont(font)
        p.drawText(QtCore.QRectF(pt.x() - 19, pt.y() - 19, 38, 38), QtCore.Qt.AlignCenter, str(h['id']))

    def _marks(self, p):
        """Planner marks: start point, area, the first tap of the base station."""
        m = self.marks
        if m['area']:
            pts = m['area'] + (m['area'][:1] if m['area_closed'] else [])
            self._line(p, pts, QtGui.QColor(style.GREEN), 2.5)
            p.setBrush(QtGui.QColor(style.GREEN))
            p.setPen(QtCore.Qt.NoPen)
            for pt in m['area']:
                p.drawEllipse(self.to_screen(*pt), 5, 5)
        for key, hex_colour in (('start', style.MAG), ('pending', style.INK)):
            if m[key] is not None:
                p.setBrush(QtGui.QColor(hex_colour))
                p.setPen(QtGui.QPen(QtGui.QColor('white'), 2))
                p.drawEllipse(self.to_screen(*m[key]), 9, 9)

    # ---------------------------------------------------------------- touch and mouse
    def _tap(self, px, py):
        if self.layers['contamination']:
            best, hit = 34., None
            for h in self.hotspots:
                pt = self.to_screen(h['x'], h['y'])
                d = math.hypot(pt.x() - px, pt.y() - py)
                if d < best:
                    best, hit = d, h['id']
            if hit is not None:
                self.hotspot_tapped.emit(hit)
                return
        self.tapped.emit(*self.to_world(px, py))

    def event(self, e):
        if e.type() in (QtCore.QEvent.TouchBegin, QtCore.QEvent.TouchUpdate, QtCore.QEvent.TouchEnd):
            points = e.touchPoints()
            if len(points) >= 2:
                a, b = points[0].pos(), points[1].pos()
                d = math.hypot(a.x() - b.x(), a.y() - b.y())
                mid = ((a.x() + b.x()) / 2, (a.y() + b.y()) / 2)
                if self.pinch and d > 0:
                    self.zoom(d / self.pinch[0], mid)
                    self.tx += mid[0] - self.pinch[1][0]
                    self.ty += mid[1] - self.pinch[1][1]
                self.pinch = (d, mid)
                self.press = None
            elif points:
                pos = points[0].pos()
                if e.type() == QtCore.QEvent.TouchBegin:
                    self._down(pos.x(), pos.y())
                elif e.type() == QtCore.QEvent.TouchUpdate:
                    self._drag(pos.x(), pos.y())
                else:
                    self._up(pos.x(), pos.y())
            if e.type() == QtCore.QEvent.TouchEnd:
                self.pinch = None
            e.accept()
            return True
        return super().event(e)

    def _down(self, x, y):
        self.press = [x, y, x, y, False]

    def _drag(self, x, y):
        if self.press is None:
            return
        if math.hypot(x - self.press[0], y - self.press[1]) >= TAP_PX:
            self.press[4] = True
        if self.press[4]:
            self.tx += x - self.press[2]
            self.ty += y - self.press[3]
            self.press[2], self.press[3] = x, y
            self.update()

    def _up(self, x, y):
        press, self.press = self.press, None
        if press is not None and not press[4]:
            self._tap(x, y)

    def mousePressEvent(self, e):
        self._down(e.x(), e.y())

    def mouseMoveEvent(self, e):
        self._drag(e.x(), e.y())

    def mouseReleaseEvent(self, e):
        self._up(e.x(), e.y())

    def wheelEvent(self, e):
        self.zoom(1.15 if e.angleDelta().y() > 0 else 1 / 1.15, (e.x(), e.y()))
