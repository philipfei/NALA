"""Touchscreen panel for app 3 (PyQt5): choose a map, set the base station, plan a path, choose a plan, drive.

Screens: Maps -> Map (plans, new plan) -> Planning -> Drive. Sizes come from config/panel.yaml.
A tap is a short touch without movement; dragging moves the map; +/- zoom.
"""
import math
import sys
import threading
from pathlib import Path
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from PyQt5 import QtCore, QtGui, QtWidgets
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg  # noqa: E402
from matplotlib.colors import ListedColormap  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402
from matplotlib.transforms import Affine2D  # noqa: E402
from nala_coverage.geometry import Grid  # noqa: E402
from nala_coverage.params import load_settings  # noqa: E402
from . import library, planning  # noqa: E402
from .drive import DriveSession, heading_arrow, plain_state  # noqa: E402

PARAMS = {'repo_dir': Parameter.Type.STRING, 'maps_dir': Parameter.Type.STRING, 'fullscreen': Parameter.Type.BOOL,
          'window_width': Parameter.Type.INTEGER, 'window_height': Parameter.Type.INTEGER,
          'font_size': Parameter.Type.INTEGER, 'button_height': Parameter.Type.INTEGER,
          'update_period': Parameter.Type.DOUBLE, 'manual_period': Parameter.Type.DOUBLE,
          'launch_stop_timeout': Parameter.Type.DOUBLE, 'hardware': Parameter.Type.BOOL}
STATION_COLOR, START_COLOR, AREA_COLOR, PLAN_COLOR = '#1f77b4', '#d62728', '#1b9e4b', '#6a3fb5'
ROBOT_COLOR, COVERED_COLOR = '#ff7f0e', '#2ca02c'
TAP_PIXELS = 15      # a touch that moves less than this is a tap, more is a drag


def read_params(node):
    """All panel settings from config/panel.yaml (and repo_dir from scripts/start_panel.sh); no defaults."""
    for name, kind in PARAMS.items():
        node.declare_parameter(name, kind)
    return {name: node.get_parameter(name).value for name in PARAMS}


def plan_points(data):
    """The path a plan drives: its drivable path, or its poses for older plans."""
    drive = data.get('drive_path') or {}
    if drive.get('points'):
        return drive['points']
    return [[p['x'], p['y']] for p in data.get('poses') or []]


class MapView(FigureCanvasQTAgg):
    """The map with overlays. Emits tapped(x, y) in map coordinates."""
    tapped = QtCore.pyqtSignal(float, float)

    def __init__(self):
        self.figure = Figure(tight_layout=True)
        super().__init__(self.figure)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_axis_off()
        self.grid = None
        self.layers = {}
        self.press = None
        self.mpl_connect('button_press_event', self._press)
        self.mpl_connect('motion_notify_event', self._move)
        self.mpl_connect('button_release_event', self._release)

    def set_grid(self, grid):
        self.ax.clear()
        self.ax.set_axis_off()
        self.layers = {}
        self.grid = grid
        img = np.full(grid.cells.shape, 1, np.uint8)
        img[grid.cells == 0] = 2
        img[grid.cells > 0] = 0
        self.ax.imshow(img, cmap=ListedColormap(['#2b2b2b', '#c8c8c8', '#ffffff']), vmin=0, vmax=2, origin='lower',
                       extent=self._extent(), transform=self._transform(), interpolation='nearest', zorder=0)
        self.ax.set_aspect('equal')
        self.fit()

    def _extent(self):
        h, w = self.grid.cells.shape
        ox, oy, _ = self.grid.origin
        return [ox, ox + w * self.grid.resolution, oy, oy + h * self.grid.resolution]

    def _transform(self):
        ox, oy, yaw = self.grid.origin
        return Affine2D().rotate_around(ox, oy, yaw) + self.ax.transData

    def fit(self):
        x0, x1, y0, y1 = self._extent()
        self.ax.set_xlim(x0, x1)
        self.ax.set_ylim(y0, y1)
        self.draw_idle()

    def zoom(self, factor):
        (x0, x1), (y0, y1) = self.ax.get_xlim(), self.ax.get_ylim()
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        self.ax.set_xlim(cx - (cx - x0) / factor, cx + (x1 - cx) / factor)
        self.ax.set_ylim(cy - (cy - y0) / factor, cy + (y1 - cy) / factor)
        self.draw_idle()

    def layer(self, name, draw):
        """Replace overlay `name` with what draw(ax) adds (a list of artists); draw=None removes it."""
        for artist in self.layers.pop(name, []):
            artist.remove()
        if draw is not None and self.grid is not None:
            self.layers[name] = [a for a in draw(self.ax) if a is not None]
        self.draw_idle()

    def mask_image(self, mask, color, alpha):
        rgba = np.zeros(mask.shape + (4,))
        rgba[mask] = list(matplotlib.colors.to_rgb(color)) + [alpha]
        return self.ax.imshow(rgba, origin='lower', extent=self._extent(), transform=self._transform(),
                              interpolation='nearest', zorder=1)

    def _press(self, event):
        if event.inaxes == self.ax:
            self.press = (event.x, event.y, self.ax.get_xlim(), self.ax.get_ylim(), False)

    def _move(self, event):
        if self.press is None:
            return
        x, y, xlim, ylim, _ = self.press
        if math.hypot(event.x - x, event.y - y) < TAP_PIXELS and not self.press[4]:
            return
        self.press = (x, y, xlim, ylim, True)
        bbox = self.ax.bbox
        dx = (event.x - x) * (xlim[1] - xlim[0]) / bbox.width
        dy = (event.y - y) * (ylim[1] - ylim[0]) / bbox.height
        self.ax.set_xlim(xlim[0] - dx, xlim[1] - dx)
        self.ax.set_ylim(ylim[0] - dy, ylim[1] - dy)
        self.draw_idle()

    def _release(self, event):
        press, self.press = self.press, None
        if press and not press[4] and event.inaxes == self.ax and event.xdata is not None:
            self.tapped.emit(float(event.xdata), float(event.ydata))


def draw_pose(ax, pose, color, label):
    x, y, dx, dy = heading_arrow(pose, 0.4)
    return [ax.arrow(x, y, dx, dy, width=0.06, color=color, zorder=8, length_includes_head=True),
            ax.text(x, y + 0.25, label, color=color, fontsize=9, ha='center', zorder=9)]


def draw_point(ax, xy, color, label):
    return [ax.plot(xy[0], xy[1], 'o', color=color, ms=10, mec='white', zorder=8)[0],
            ax.text(xy[0], xy[1] + 0.25, label, color=color, fontsize=9, ha='center', zorder=9)]


def draw_area(ax, corners, closed):
    if not corners:
        return []
    a = np.asarray(corners + (corners[:1] if closed else []), float)
    return ax.plot(a[:, 0], a[:, 1], '-o', color=AREA_COLOR, lw=2, ms=6, zorder=7)


def draw_path(ax, points, color=PLAN_COLOR, lw=1.4):
    if len(points) < 2:
        return []
    a = np.asarray(points, float)
    return ax.plot(a[:, 0], a[:, 1], '-', color=color, lw=lw, zorder=3)


def button(text, slot, style=''):
    b = QtWidgets.QPushButton(text)
    b.clicked.connect(slot)
    if style:
        # A coloured button must still look grey when it cannot be pressed.
        b.setStyleSheet(f'QPushButton {{{style}}} QPushButton:disabled {{background:#bdbdbd;color:#eeeeee}}')
    return b


class MapsScreen(QtWidgets.QWidget):
    """Choose a map made by app 2."""

    def __init__(self, panel):
        super().__init__()
        self.panel = panel
        layout = QtWidgets.QVBoxLayout(self)
        title = QtWidgets.QHBoxLayout()
        title.addWidget(QtWidgets.QLabel('<b>Choose a map</b>'))
        title.addStretch()
        title.addWidget(button('Refresh', self.refresh))
        layout.addLayout(title)
        self.list = QtWidgets.QListWidget()
        self.list.setViewMode(QtWidgets.QListView.IconMode)
        self.list.setIconSize(QtCore.QSize(200, 140))
        self.list.setResizeMode(QtWidgets.QListView.Adjust)
        self.list.setMovement(QtWidgets.QListView.Static)
        self.list.setSpacing(12)
        self.list.itemClicked.connect(lambda item: panel.open_map(item.data(QtCore.Qt.UserRole)))
        layout.addWidget(self.list)
        self.empty = QtWidgets.QLabel()
        self.empty.setWordWrap(True)
        layout.addWidget(self.empty)

    def refresh(self):
        self.list.clear()
        maps = library.list_maps(self.panel.maps_dir)
        for entry in maps:
            plans = len(library.list_plans(entry))
            icon = QtGui.QIcon(QtGui.QPixmap(str(entry.image)))
            item = QtWidgets.QListWidgetItem(icon, f'{entry.name}\n{plans} plan{"s" if plans != 1 else ""}')
            item.setData(QtCore.Qt.UserRole, entry)
            self.list.addItem(item)
        self.empty.setText('' if maps else f'No maps in {self.panel.maps_dir}. Make one with app 2 over ssh '
                           '(scripts/save_map.sh <name>); it appears here.')


class MapScreen(QtWidgets.QWidget):
    """One map: its plans, and a new plan (base station, start point, area)."""

    def __init__(self, panel):
        super().__init__()
        self.panel = panel
        self.entry = None
        self.grid = None
        self.station = None
        self.plans = []
        self.mode = None
        self.station_xy = None      # first tap of "Set base station"
        self.start = None
        self.area, self.area_closed = [], False
        outer = QtWidgets.QHBoxLayout(self)
        left = QtWidgets.QVBoxLayout()
        top = QtWidgets.QHBoxLayout()
        top.addWidget(button('< Maps', panel.show_maps))
        self.title = QtWidgets.QLabel()
        top.addWidget(self.title, 1)
        top.addWidget(button('+', lambda: self.view.zoom(1.5)))
        top.addWidget(button('-', lambda: self.view.zoom(1 / 1.5)))
        top.addWidget(button('Fit', lambda: self.view.fit()))
        left.addLayout(top)
        self.view = MapView()
        self.view.tapped.connect(self.tap)
        left.addWidget(self.view, 1)
        self.status = QtWidgets.QLabel()
        self.status.setWordWrap(True)
        left.addWidget(self.status)
        outer.addLayout(left, 3)

        self.tabs = QtWidgets.QTabWidget()
        self.tabs.setMinimumWidth(340)
        plans = QtWidgets.QWidget()
        pl = QtWidgets.QVBoxLayout(plans)
        self.plan_list = QtWidgets.QListWidget()
        self.plan_list.setWordWrap(True)
        self.plan_list.currentRowChanged.connect(self.show_plan)
        pl.addWidget(self.plan_list, 1)
        row = QtWidgets.QHBoxLayout()
        self.drive_button = button('Drive', self.drive, 'background:#2e7d32;color:white;font-weight:bold')
        row.addWidget(self.drive_button)
        row.addWidget(button('Delete', self.delete))
        pl.addLayout(row)
        self.tabs.addTab(plans, 'Plans')

        new = QtWidgets.QWidget()
        nl = QtWidgets.QGridLayout(new)
        self.station_label = QtWidgets.QLabel()
        self.station_label.setWordWrap(True)
        nl.addWidget(self.station_label, 0, 0, 1, 2)
        nl.addWidget(button('Set base station', lambda: self.set_mode('station')), 1, 0, 1, 2)
        nl.addWidget(QtWidgets.QLabel('Coverage starts at:'), 2, 0, 1, 2)
        self.at_station = QtWidgets.QRadioButton('Base station')
        self.at_point = QtWidgets.QRadioButton('Start point')
        self.at_station.setChecked(True)
        self.at_station.toggled.connect(self.redraw)
        nl.addWidget(self.at_station, 3, 0)
        nl.addWidget(self.at_point, 3, 1)
        nl.addWidget(button('Set start point', lambda: self.set_mode('start')), 4, 0, 1, 2)
        nl.addWidget(QtWidgets.QLabel('Area (optional, else the whole map):'), 5, 0, 1, 2)
        nl.addWidget(button('Draw area', lambda: self.set_mode('area')), 6, 0)
        nl.addWidget(button('Undo corner', self.undo_corner), 6, 1)
        nl.addWidget(button('Close area', self.close_area), 7, 0)
        nl.addWidget(button('Clear area', self.clear_area), 7, 1)
        nl.addWidget(button('Plan', self.plan, 'background:#1565c0;color:white;font-weight:bold'), 8, 0, 1, 2)
        nl.setRowStretch(9, 1)
        self.tabs.addTab(new, 'New plan')
        self.tabs.currentChanged.connect(lambda _: self.redraw())
        outer.addWidget(self.tabs, 2)

    # ---------------------------------------------------------------- loading
    def open(self, entry):
        self.entry = entry
        self.title.setText(f'<b>{entry.name}</b>')
        self.grid = Grid.load(entry.map_yaml)
        self.view.set_grid(self.grid)
        self.start, self.area, self.area_closed, self.mode = None, [], False, None
        self.at_station.setChecked(True)
        self.load_station()
        self.load_plans()
        self.tabs.setCurrentIndex(0 if self.plans else 1)
        self.redraw()

    def load_station(self):
        try:
            self.station = library.load_station(self.entry, self.panel.settings['runtime']['map_frame'])
        except ValueError as error:
            self.station = None
            self.say(str(error))
        if self.station is None:
            self.station_label.setText('Base station: <b>not set</b>. Tap "Set base station".')
        else:
            x, y, yaw = self.station
            self.station_label.setText(f'Base station: x {x:.2f}, y {y:.2f}, {math.degrees(yaw):.0f} deg')

    def load_plans(self):
        self.plans = library.list_plans(self.entry)
        self.plan_list.clear()
        for plan in self.plans:
            reason = library.plan_status(self.entry, plan, self.grid, self.panel.settings, self.station)
            item = QtWidgets.QListWidgetItem(plan.summary() + (f'\nCannot drive: {reason}' if reason else ''))
            item.setData(QtCore.Qt.UserRole, reason)
            if reason:
                item.setForeground(QtGui.QColor('#888888'))
            self.plan_list.addItem(item)
        if self.plans:
            self.plan_list.setCurrentRow(0)

    # ---------------------------------------------------------------- map taps
    def say(self, text):
        self.status.setText(text)

    def set_mode(self, mode):
        self.mode = mode
        self.station_xy = None
        if mode == 'area' and self.area_closed:
            self.area, self.area_closed = [], False
        hints = {'station': 'Tap where the base station is.', 'start': 'Tap where coverage starts.',
                 'area': 'Tap the corners of the area, then "Close area".'}
        self.say(hints[mode])
        self.redraw()

    def tap(self, x, y):
        collision = self.panel.settings.collision
        if self.mode == 'station' and self.station_xy is None:
            self.station_xy = (x, y)
            self.say('Now tap the direction the robot faces on the base station.')
        elif self.mode == 'station':
            sx, sy = self.station_xy
            pose = (sx, sy, math.atan2(y - sy, x - sx))
            try:
                library.save_station(self.entry, pose, self.grid, collision, self.panel.settings['runtime']['map_frame'])
            except ValueError as error:
                self.say(f'{error}. Tap "Set base station" and try another place.')
            else:
                self.say('Base station saved. Plans made for the old base station can no longer be driven.')
                self.load_station()
                self.load_plans()
            self.mode = self.station_xy = None
        elif self.mode == 'start':
            reason = planning.check_inputs(self.grid, collision, (x, y), None)
            if reason:
                self.say(reason)
                return
            self.start = [x, y]
            self.at_point.setChecked(True)
            self.mode = None
            self.say(f'Coverage starts at x {x:.2f}, y {y:.2f}. The robot drives there from the base station.')
        elif self.mode == 'area':
            self.area.append([x, y])
            self.say(f'{len(self.area)} corner(s). Tap more corners, then "Close area".')
        self.redraw()

    def undo_corner(self):
        if self.area and not self.area_closed:
            self.area.pop()
            self.redraw()

    def close_area(self):
        if len(self.area) < 3:
            self.say('An area needs at least 3 corners.')
            return
        reason = planning.check_inputs(self.grid, self.panel.settings.collision, None, self.area)
        if reason:
            self.say(reason)
            return
        self.area_closed, self.mode = True, None
        self.say(f'Area closed ({len(self.area)} corners). Only the floor inside it is covered.')
        self.redraw()

    def clear_area(self):
        self.area, self.area_closed = [], False
        if self.mode == 'area':
            self.mode = None
        self.say('Area cleared: the whole reachable map is covered.')
        self.redraw()

    # ---------------------------------------------------------------- drawing
    def redraw(self):
        v = self.view
        v.layer('station', (lambda ax: draw_pose(ax, self.station, STATION_COLOR, 'base')) if self.station else None)
        new_plan = self.tabs.currentIndex() == 1
        show_start = new_plan and self.at_point.isChecked() and self.start
        v.layer('start', (lambda ax: draw_point(ax, self.start, START_COLOR, 'start')) if show_start else None)
        v.layer('area', (lambda ax: draw_area(ax, self.area, self.area_closed)) if new_plan else None)
        if new_plan:
            v.layer('plan', None)
        else:
            self.show_plan(self.plan_list.currentRow())

    def show_plan(self, row):
        if not 0 <= row < len(self.plans):
            self.view.layer('plan', None)
            self.drive_button.setEnabled(False)
            return
        plan = self.plans[row]
        reason = self.plan_list.item(row).data(QtCore.Qt.UserRole)
        self.drive_button.setEnabled(not reason)
        area = plan.data.get('area_polygon') or []
        self.view.layer('plan', lambda ax: draw_path(ax, plan_points(plan.data)) + draw_area(ax, area, True))
        self.say(reason or 'Tap "Drive" to start app 3 with this plan. The robot moves only after "Start".')

    # ---------------------------------------------------------------- actions
    def plan(self):
        if self.station is None:
            self.say('Set the base station first: the robot starts and ends there.')
            return
        start = None
        if self.at_point.isChecked():
            if self.start is None:
                self.say('Tap "Set start point" first, or choose "Base station".')
                return
            start = self.start
        if self.area and not self.area_closed:
            self.say('Close the area first, or clear it.')
            return
        area = self.area if self.area_closed else None
        reason = planning.check_inputs(self.grid, self.panel.settings.collision, start, area)
        if reason:
            self.say(reason)
            return
        self.panel.start_planning(self.entry, start, area)

    def delete(self):
        row = self.plan_list.currentRow()
        if not 0 <= row < len(self.plans):
            return
        plan = self.plans[row]
        answer = QtWidgets.QMessageBox.question(self, 'Delete plan', f'Delete plan {plan.name}?')
        if answer == QtWidgets.QMessageBox.Yes:
            library.delete_plan(plan)
            self.load_plans()
            self.redraw()

    def drive(self):
        row = self.plan_list.currentRow()
        if 0 <= row < len(self.plans) and not self.plan_list.item(row).data(QtCore.Qt.UserRole):
            self.panel.start_drive(self.entry, self.plans[row])


class PlanningScreen(QtWidgets.QWidget):
    """A planning run: progress, then the result with its checks; keep or discard it."""

    def __init__(self, panel):
        super().__init__()
        self.panel = panel
        self.run = None
        self.entry = None
        outer = QtWidgets.QHBoxLayout(self)
        self.pages = QtWidgets.QStackedWidget()
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.image = QtWidgets.QLabel()
        self.image.setAlignment(QtCore.Qt.AlignCenter)
        self.pages.addWidget(self.log)
        self.pages.addWidget(self.image)
        outer.addWidget(self.pages, 3)
        right = QtWidgets.QVBoxLayout()
        self.title = QtWidgets.QLabel()
        self.title.setWordWrap(True)
        right.addWidget(self.title)
        self.checks = QtWidgets.QLabel()
        self.checks.setWordWrap(True)
        right.addWidget(self.checks, 1)
        self.cancel_button = button('Cancel', self.cancel)
        self.keep_button = button('Keep plan', self.keep, 'background:#2e7d32;color:white;font-weight:bold')
        self.discard_button = button('Discard', self.discard)
        for b in (self.cancel_button, self.keep_button, self.discard_button):
            right.addWidget(b)
        outer.addLayout(right, 2)

    def begin(self, entry, run):
        self.entry, self.run = entry, run
        self.log.clear()
        self.pages.setCurrentIndex(0)
        self.title.setText('<b>Planning...</b><br>This can take several minutes on the robot.')
        self.checks.setText('')
        self.cancel_button.show()
        self.keep_button.hide()
        self.discard_button.hide()

    def poll(self):
        if self.run is None or self.cancel_button.isHidden():
            return
        for line in self.run.lines():
            self.log.appendPlainText(line)
        if not self.run.done():
            return
        self.cancel_button.hide()
        self.discard_button.show()
        if not self.run.ok():
            self.title.setText('<b>Planning failed.</b> See the messages. Discard and change the start or area.')
            return
        stats = self.run.stats()
        results = planning.check_results(stats)
        good = all(ok for ok, _ in results)
        self.title.setText(f"<b>{'Plan ready' if good else 'Plan has problems'}</b><br>"
                           f"{stats.get('coverage_pct', 0):.0f} % of the floor, "
                           f"{stats.get('drive_length_m', 0):.0f} m, ~{stats.get('estimated_time_s', 0) / 60:.0f} min")
        self.checks.setText('<br>'.join(f"{'&#10004;' if ok else '&#10008;'} {text}" for ok, text in results))
        pixmap = QtGui.QPixmap(str(self.run.outputs[1]))
        self.image.setPixmap(pixmap.scaled(self.image.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))
        self.pages.setCurrentIndex(1)
        self.keep_button.setVisible(good)

    def cancel(self):
        self.run.cancel()
        self.panel.open_map(self.entry)

    def keep(self):
        self.panel.open_map(self.entry)

    def discard(self):
        self.run.discard()
        self.panel.open_map(self.entry)


class DriveScreen(QtWidgets.QWidget):
    """App 3 running: live map, state, Start / Pause / Resume / Cancel / STOP, driving by hand while paused."""

    def __init__(self, panel):
        super().__init__()
        self.panel = panel
        self.session = panel.session
        self.entry = None
        self.held = None
        self.manual_on = False
        self.drawn = (None, None)    # covered mask and pose last drawn: redraw the map only when they change
        outer = QtWidgets.QHBoxLayout(self)
        left = QtWidgets.QVBoxLayout()
        top = QtWidgets.QHBoxLayout()
        self.title = QtWidgets.QLabel()
        top.addWidget(self.title, 1)
        top.addWidget(button('+', lambda: self.view.zoom(1.5)))
        top.addWidget(button('-', lambda: self.view.zoom(1 / 1.5)))
        top.addWidget(button('Fit', lambda: self.view.fit()))
        left.addLayout(top)
        self.view = MapView()
        left.addWidget(self.view, 1)
        outer.addLayout(left, 3)

        right = QtWidgets.QVBoxLayout()
        right.addWidget(button('STOP', self.session.emergency_stop,
                               'background:#c62828;color:white;font-weight:bold;font-size:20pt'))
        self.state_label = QtWidgets.QLabel()
        self.state_label.setWordWrap(True)
        right.addWidget(self.state_label)
        self.message = QtWidgets.QLabel()
        self.message.setWordWrap(True)
        right.addWidget(self.message)
        grid = QtWidgets.QGridLayout()
        self.start_button = button('Start', self.session.start, 'background:#2e7d32;color:white;font-weight:bold')
        self.pause_button = button('Pause', self.session.pause)
        self.resume_button = button('Resume', self.resume)
        self.cancel_button = button('Cancel task', self.session.cancel)
        grid.addWidget(self.start_button, 0, 0)
        grid.addWidget(self.pause_button, 0, 1)
        grid.addWidget(self.resume_button, 1, 0)
        grid.addWidget(self.cancel_button, 1, 1)
        right.addLayout(grid)
        self.manual_button = button('Drive by hand', self.toggle_manual)
        right.addWidget(self.manual_button)
        arrows = QtWidgets.QGridLayout()
        self.arrows = {}
        for key, text, row, col in (('w', '^', 0, 1), ('a', '<', 1, 0), ('d', '>', 1, 2), ('s', 'v', 2, 1)):
            b = QtWidgets.QPushButton(text)
            b.pressed.connect(lambda k=key: self.hold(k))
            b.released.connect(lambda: self.hold(None))
            arrows.addWidget(b, row, col)
            self.arrows[key] = b
        right.addLayout(arrows)
        right.addStretch()
        right.addWidget(button('Finish', self.finish))
        outer.addLayout(right, 2)
        self.manual_timer = QtCore.QTimer(self)
        self.manual_timer.timeout.connect(self.manual_tick)

    def begin(self, entry, plan, grid, station):
        self.entry = entry
        self.title.setText(f'<b>{entry.name}</b> / {plan.name}')
        self.view.set_grid(grid)
        self.view.layer('plan', lambda ax: draw_path(ax, plan_points(plan.data), lw=1.0))
        self.view.layer('station', lambda ax: draw_pose(ax, station, STATION_COLOR, 'base'))
        self.manual_on = False
        self.drawn = (None, None)
        self.manual_timer.start(int(self.panel.params['manual_period'] * 1000))
        teleop = self.session.start_launch(entry.map_yaml, plan.path, library.new_run_dir(entry))
        self.panel.executor.add_node(teleop)

    def refresh(self):
        s = self.session
        s.update()
        state = s.state
        paused = state.get('state') == 'PAUSED'
        reason = state.get('reason') or state.get('trust_reason') or ''
        running = s.launch_running()
        text = plain_state(state) if running else 'App 3 stopped. Tap "Finish".'
        self.state_label.setText(f"<b style='font-size:16pt'>{text}</b><br>"
                                 f"{100 * state.get('fraction', 0.):.0f} % covered<br>{reason}")
        self.message.setText(s.message)
        self.start_button.setEnabled(running and s.can_start())
        self.pause_button.setEnabled(running and s.active())
        self.resume_button.setEnabled(running and paused and not self.manual_on)
        self.cancel_button.setEnabled(running and (s.active() or paused))
        self.manual_button.setEnabled(running and paused)
        self.manual_button.setText('Hand control off' if self.manual_on else 'Drive by hand')
        for b in self.arrows.values():
            b.setEnabled(self.manual_on)
        covered, pose = s.covered, s.pose
        if covered is not None and covered is not self.drawn[0] and covered.shape == self.view.grid.cells.shape:
            self.view.layer('covered', lambda ax: [self.view.mask_image(covered, COVERED_COLOR, 0.35)])
        if pose is not None and pose is not self.drawn[1]:
            self.view.layer('robot', lambda ax: draw_pose(ax, pose, ROBOT_COLOR, 'NALA'))
        self.drawn = (covered, pose)

    def resume(self):
        self.session.resume()

    def toggle_manual(self):
        self.manual_on = not self.manual_on
        self.session.manual(self.manual_on)

    def hold(self, key):
        self.held = key
        if key is None:
            self.session.manual_key('x')

    def manual_tick(self):
        if self.manual_on and self.held:
            self.session.manual_key(self.held)
        self.session.manual_tick()

    def finish(self):
        s = self.session
        if s.active() or s.state.get('state') == 'PAUSED':
            answer = QtWidgets.QMessageBox.question(self, 'Finish', 'A task is still running. Cancel it and stop app 3?')
            if answer != QtWidgets.QMessageBox.Yes:
                return
            s.cancel()
        if self.manual_on:
            self.toggle_manual()
        self.manual_timer.stop()
        self.state_label.setText('<b>Stopping app 3...</b>')
        QtWidgets.QApplication.processEvents()
        s.stop_launch()
        report = s.report()
        if report:
            skipped = sum(b.get('reason') == 'PLAN_STRETCH_SKIPPED' for b in report.get('temporary_blockages', []))
            QtWidgets.QMessageBox.information(
                self, 'Task report', f"{report.get('state')}: {report.get('reason')}\n"
                f"Skipped stretches: {skipped}\nReport: {s.output_dir}")
        self.panel.open_map(self.entry)


class Panel(QtWidgets.QMainWindow):
    def __init__(self, node, executor, params):
        super().__init__()
        self.node, self.executor, self.params = node, executor, params
        repo = Path(params['repo_dir'])
        self.repo_dir = repo
        self.maps_dir = repo / params['maps_dir']
        self.settings = load_settings(repo / 'config')
        self.session = DriveSession(repo / 'config', params['hardware'], params['launch_stop_timeout'])
        executor.add_node(self.session)
        self.setWindowTitle('NALA')
        self.stack = QtWidgets.QStackedWidget()
        self.setCentralWidget(self.stack)
        self.maps = MapsScreen(self)
        self.map = MapScreen(self)
        self.planning = PlanningScreen(self)
        self.drive = DriveScreen(self)
        for screen in (self.maps, self.map, self.planning, self.drive):
            self.stack.addWidget(screen)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(int(params['update_period'] * 1000))
        self.show_maps()

    def show_maps(self):
        self.maps.refresh()
        self.stack.setCurrentWidget(self.maps)

    def open_map(self, entry):
        self.map.open(entry)
        self.stack.setCurrentWidget(self.map)

    def start_planning(self, entry, start, area):
        out_yaml, out_png = library.new_plan_paths(entry)
        cmd = planning.command(self.repo_dir, entry.map_yaml, entry.station_file, out_yaml, out_png, start, area)
        self.planning.begin(entry, planning.PlanRun(cmd, out_yaml, out_png))
        self.stack.setCurrentWidget(self.planning)

    def start_drive(self, entry, plan):
        self.drive.begin(entry, plan, self.map.grid, self.map.station)
        self.stack.setCurrentWidget(self.drive)

    def tick(self):
        current = self.stack.currentWidget()
        if current is self.planning:
            self.planning.poll()
        elif current is self.drive:
            self.drive.refresh()

    def closeEvent(self, event):
        if self.planning.run is not None and not self.planning.run.done():
            self.planning.run.cancel()
        self.session.stop_launch()
        super().closeEvent(event)


def make_app(params):
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv[:1])
    font = app.font()
    font.setPointSize(params['font_size'])
    app.setFont(font)
    app.setStyleSheet(f"QPushButton {{ min-height: {params['button_height']}px; }}"
                      f"QRadioButton::indicator {{ width: 28px; height: 28px; }}")
    return app


def main(args=None):
    rclpy.init(args=args)
    node = Node('nala_panel')
    params = read_params(node)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    app = make_app(params)
    panel = Panel(node, executor, params)
    threading.Thread(target=_spin, args=(executor,), daemon=True).start()
    if params['fullscreen']:
        panel.showFullScreen()
    else:
        panel.resize(params['window_width'], params['window_height'])
        panel.show()
    code = app.exec_()
    executor.shutdown()
    rclpy.try_shutdown()
    sys.exit(code)


def _spin(executor):
    try:
        executor.spin()
    except ExternalShutdownException:
        pass
