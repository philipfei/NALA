"""Touchscreen panel for the robot (PyQt5), styled after the scan screen design (Scan_screen_visualiser.html).

Screens: Maps -> Map (plans, new plan, last results) -> Planning -> Scan (app 3); Maps -> App (app 1: drive with
the Xbox controller, app 2: make a map and save it, with an on-screen keyboard for its name).
The Scan screen shows where the robot actually was (trusted measured poses), what the sensor covered and what
the scintillator measured (heatmap, numbered hotspots), with Start / Pause / Stop and go home / STOP.
Sizes come from config/panel.yaml.
"""
import datetime
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
from nala_coverage.geometry import Grid
from nala_coverage.params import load_settings
from . import apps, library, planning, style
from .drive import DriveSession, plain_state
from .mapwidget import MapWidget

PARAMS = {'repo_dir': Parameter.Type.STRING, 'maps_dir': Parameter.Type.STRING, 'fullscreen': Parameter.Type.BOOL,
          'window_width': Parameter.Type.INTEGER, 'window_height': Parameter.Type.INTEGER,
          'font_size': Parameter.Type.INTEGER, 'button_height': Parameter.Type.INTEGER,
          'update_period': Parameter.Type.DOUBLE, 'manual_period': Parameter.Type.DOUBLE,
          'launch_stop_timeout': Parameter.Type.DOUBLE, 'preview_retry': Parameter.Type.DOUBLE, 'hardware': Parameter.Type.BOOL,
          'heatmap_width_m': Parameter.Type.DOUBLE}


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


def button(text, slot, kind=''):
    b = QtWidgets.QPushButton(text)
    b.clicked.connect(slot)
    if kind:
        style.set_kind(b, kind)
    return b


def label(text='', name='', wrap=True):
    lab = QtWidgets.QLabel(text)
    lab.setWordWrap(wrap)
    if name:
        lab.setObjectName(name)
    return lab


def hotspot_message(hotspots):
    n = len(hotspots)
    to_check = sum(not h.get('checked') for h in hotspots)
    spots = 'spot' if n == 1 else 'spots'
    if n == 0:
        return 'No contamination found yet', False
    if to_check:
        return f'{n} contaminated {spots}, {to_check} to check', True
    return f'All {n} contaminated {spots} checked', False


class AlertBar(QtWidgets.QFrame):
    """'N contaminated spots, M to check' over the map (magenta while spots are not checked)."""

    def __init__(self, parent):
        super().__init__(parent)
        self.setObjectName('alert')
        lay = QtWidgets.QHBoxLayout(self)
        lay.setContentsMargins(14, 8, 14, 8)
        self.text = label('', wrap=False)
        font = self.text.font()
        font.setBold(True)
        self.text.setFont(font)
        lay.addWidget(self.text)

    def show_hotspots(self, hotspots):
        text, hot = hotspot_message(hotspots)
        self.text.setText('◆  ' + text)
        if self.property('hot') != hot:
            self.setProperty('hot', hot)
            for w in (self, self.text):          # the label's colour depends on the frame's property
                w.style().unpolish(w)
                w.style().polish(w)


class MapsScreen(QtWidgets.QWidget):
    """Choose a map made by app 2."""

    def __init__(self, panel):
        super().__init__()
        self.panel = panel
        layout = QtWidgets.QVBoxLayout(self)
        title = QtWidgets.QHBoxLayout()
        title.addWidget(label('<b>NALA</b> - choose a map', wrap=False))
        title.addStretch()
        title.addWidget(button('Drive (app 1)', lambda: panel.start_app('app1')))
        title.addWidget(button('Make a map (app 2)', lambda: panel.start_app('app2')))
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
        self.empty = label(name='mute')
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
        self.empty.setText('' if maps else f'No maps in {self.panel.maps_dir}. Make one with "Make a map (app 2)"; '
                           'it appears here.')


class MapScreen(QtWidgets.QWidget):
    """One map: its plans, a new plan (base station, start point, area), and the results of its last run."""

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
        self.run = None
        outer = QtWidgets.QHBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        left = QtWidgets.QVBoxLayout()
        left.setContentsMargins(8, 8, 0, 8)
        top = QtWidgets.QHBoxLayout()
        top.addWidget(button('< Maps', panel.show_maps))
        self.title = label(wrap=False)
        top.addWidget(self.title, 1)
        left.addLayout(top)
        self.view = MapWidget()
        self.view.heat_width_m = panel.params['heatmap_width_m']
        self.view.footprint = panel.settings['geometry']['footprint_m']
        self.view.tapped.connect(self.tap)
        left.addWidget(self.view, 1)
        self.status = label(name='mute')
        left.addWidget(self.status)
        outer.addLayout(left, 3)

        side = QtWidgets.QFrame()
        side.setObjectName('side')
        side_lay = QtWidgets.QVBoxLayout(side)
        self.tabs = QtWidgets.QTabWidget()
        side_lay.addWidget(self.tabs)
        side.setMinimumWidth(360)
        side.setMaximumWidth(380)

        plans = QtWidgets.QWidget()
        pl = QtWidgets.QVBoxLayout(plans)
        self.plan_list = QtWidgets.QListWidget()
        self.plan_list.setWordWrap(True)
        self.plan_list.currentRowChanged.connect(self.show_plan)
        pl.addWidget(self.plan_list, 1)
        row = QtWidgets.QHBoxLayout()
        self.drive_button = button('Scan', self.drive, 'primary')
        row.addWidget(self.drive_button)
        row.addWidget(button('Delete', self.delete))
        pl.addLayout(row)
        self.tabs.addTab(plans, 'Plans')

        new = QtWidgets.QWidget()
        nl = QtWidgets.QGridLayout(new)
        self.station_label = label()
        nl.addWidget(self.station_label, 0, 0, 1, 2)
        nl.addWidget(button('Set base station', lambda: self.set_mode('station')), 1, 0, 1, 2)
        nl.addWidget(label('Coverage starts at:'), 2, 0, 1, 2)
        self.at_station = QtWidgets.QRadioButton('Base station')
        self.at_point = QtWidgets.QRadioButton('Start point')
        self.at_station.setChecked(True)
        self.at_station.toggled.connect(self.redraw)
        nl.addWidget(self.at_station, 3, 0)
        nl.addWidget(self.at_point, 3, 1)
        nl.addWidget(button('Set start point', lambda: self.set_mode('start')), 4, 0, 1, 2)
        nl.addWidget(label('Area (optional, else the whole map):'), 5, 0, 1, 2)
        nl.addWidget(button('Draw area', lambda: self.set_mode('area')), 6, 0)
        nl.addWidget(button('Undo corner', self.undo_corner), 6, 1)
        nl.addWidget(button('Close area', self.close_area), 7, 0)
        nl.addWidget(button('Clear area', self.clear_area), 7, 1)
        nl.addWidget(button('Plan', self.plan, 'primary'), 8, 0, 1, 2)
        nl.setRowStretch(9, 1)
        self.tabs.addTab(new, 'New plan')

        results = QtWidgets.QWidget()
        rl = QtWidgets.QVBoxLayout(results)
        self.result_text = label()
        rl.addWidget(self.result_text)
        rl.addStretch()
        self.tabs.addTab(results, 'Results')
        self.tabs.currentChanged.connect(lambda _: self.redraw())
        outer.addWidget(side)

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
        self.load_results()
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
            item = QtWidgets.QListWidgetItem(plan.summary() + (f'\nCannot scan: {reason}' if reason else ''))
            item.setData(QtCore.Qt.UserRole, reason)
            if reason:
                item.setForeground(QtGui.QColor(style.MUTE))
            self.plan_list.addItem(item)
        if self.plans:
            self.plan_list.setCurrentRow(0)

    def load_results(self):
        self.run = library.last_run(self.entry)
        if self.run is None:
            self.result_text.setText('No scan of this map yet.')
            return
        r = self.run
        text, _ = hotspot_message(r['hotspots'])
        text = text.replace(' yet', '')          # a finished scan
        bg = f"{r['background']:.1f} counts/s" if np.isfinite(r['background']) else 'not measured'
        if r['appeared'] is not None:
            cell = self.grid.resolution ** 2
            text += (f"<br>Map changed: {r['appeared'].sum() * cell:.1f} m² new, "
                     f"{r['vanished'].sum() * cell:.1f} m² removed")
        self.result_text.setText(f"<b>Last scan: {r['name']}</b><br>Background: {bg}<br>{text}<br>"
                                 f"<span style='color:{style.MUTE}'>Saved in {r['folder']}</span>")

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
                self.say('Base station saved. Plans made for the old base station can no longer be scanned.')
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
        tab = self.tabs.currentIndex()
        v.station = self.station
        v.marks = {'start': self.start if tab == 1 and self.at_point.isChecked() else None,
                   'area': self.area if tab == 1 else [], 'area_closed': self.area_closed,
                   'pending': self.station_xy}
        run = self.run if tab == 2 else None
        v.set_heatmap(run['value'] if run else None, run['threshold'] if run else None)
        v.set_mask('keepout', run['keepout'] if run else None, style.MAG, 90)
        v.set_changes(run['appeared'] if run else None, run['vanished'] if run else None)
        v.trail = run['trail'] if run else []
        v.hotspots = run['hotspots'] if run else []
        if tab == 0:
            self.show_plan(self.plan_list.currentRow())
        else:
            v.plan = []
            v.update()

    def show_plan(self, row):
        if not 0 <= row < len(self.plans):
            self.view.refresh(plan=[])
            self.drive_button.setEnabled(False)
            return
        plan = self.plans[row]
        reason = self.plan_list.item(row).data(QtCore.Qt.UserRole)
        self.drive_button.setEnabled(not reason)
        area = plan.data.get('area_polygon') or []
        self.view.marks = {'start': None, 'area': area, 'area_closed': True, 'pending': None}
        self.view.refresh(plan=plan_points(plan.data))
        self.say(reason or 'Tap "Scan" to start app 3 with this plan. The robot moves only after "Start".')

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
        self.title = label()
        right.addWidget(self.title)
        self.checks = label()
        right.addWidget(self.checks, 1)
        self.cancel_button = button('Cancel', self.cancel)
        self.keep_button = button('Keep plan', self.keep, 'primary')
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


class ScanScreen(QtWidgets.QWidget):
    """App 3 running, as the scan screen design: map + side panel; a large progress view while driving."""

    def __init__(self, panel):
        super().__init__()
        self.panel = panel
        self.session = panel.session
        self.entry = None
        self.held = None
        self.manual_on = False
        self.show_map_while_driving = False
        self.drawn = {}
        body = QtWidgets.QHBoxLayout(self)
        body.setContentsMargins(0, 0, 0, 0)
        body.setSpacing(0)

        # Left: the map (or the large progress view while driving).
        self.left = QtWidgets.QStackedWidget()
        mapbox = QtWidgets.QWidget()
        self.view = MapWidget(mapbox)
        self.view.footprint = panel.settings['geometry']['footprint_m']
        # The STOP banner of the design, over the top of the map (it takes no height from the 600 px screen).
        self.banner = QtWidgets.QFrame(mapbox)
        self.banner.setObjectName('banner')
        bl = QtWidgets.QHBoxLayout(self.banner)
        bl.addWidget(label('■  STOP active. Motors are off. Tap Resume or Go home.', wrap=False))
        self.banner.hide()
        self.view.hotspot_tapped.connect(self.select)
        self.view.tapped.connect(lambda x, y: self.select(None))
        self.alert = AlertBar(mapbox)
        self.sheet = self._sheet(mapbox)
        mapbox.resizeEvent = self._layout_map
        self.mapbox = mapbox
        self.left.addWidget(mapbox)
        self.left.addWidget(self._drive_view())
        body.addWidget(self.left, 1)

        # Right: the side panel.
        side = QtWidgets.QFrame()
        side.setObjectName('side')
        side.setFixedWidth(330)
        sl = QtWidgets.QVBoxLayout(side)
        # Tight gaps: with the Pi's fonts the default ones made the panel 615 px, more than the 600 px screen
        # (robot, 2026-10-19). It must fit without scrolling.
        sl.setSpacing(3)
        sl.setContentsMargins(8, 6, 8, 6)
        head = QtWidgets.QHBoxLayout()
        self.title = label('<b>NALA</b>', wrap=False)
        head.addWidget(self.title, 1)
        self.big_button = button('Big view', lambda: self._toggle_map(False))   # back to the progress view
        head.addWidget(self.big_button)
        self.back_button = button('Back', self.finish)
        head.addWidget(self.back_button)
        sl.addLayout(head)
        chips = QtWidgets.QHBoxLayout()
        self.chip_mode = label(wrap=False, name='chip')
        self.chip_loc = label(wrap=False, name='chip')
        chips.addWidget(self.chip_mode)
        chips.addWidget(self.chip_loc)
        chips.addStretch()
        sl.addLayout(chips)
        self.pct = label('0<small>%</small>', name='pct', wrap=False)
        sl.addWidget(self.pct)
        self.status = label(name='status')
        sl.addWidget(self.status)
        self.bar = QtWidgets.QProgressBar()
        self.bar.setTextVisible(False)
        sl.addWidget(self.bar)
        self.area = label(name='mute')
        sl.addWidget(self.area)
        self.legend = label(f"<span style='color:{style.TEAL}'>&#9632;</span> Scanned&nbsp; "
                            f"<span style='color:{style.MAG}'>&#9670;</span> Contaminated&nbsp; "
                            f"<span style='color:{style.NEW}'>&#9632;</span> New object&nbsp; "
                            f"<span style='color:{style.GONE}'>&#9632;</span> Removed object", name='mute')
        sl.addWidget(self.legend)
        self.toggles = QtWidgets.QWidget()
        toggles = QtWidgets.QHBoxLayout(self.toggles)
        toggles.setContentsMargins(0, 0, 0, 0)
        toggles.setSpacing(4)
        for key, text in (('coverage', 'Coverage'), ('contamination', 'Radiation'), ('path', 'Path'),
                          ('changes', 'Changes')):
            b = style.set_kind(QtWidgets.QPushButton(text), 'toggle')
            b.setCheckable(True)
            b.setChecked(True)
            b.toggled.connect(lambda on, k=key: self._layer(k, on))
            toggles.addWidget(b)
        sl.addWidget(self.toggles)
        sl.addStretch()
        self.message = label(name='mute')
        sl.addWidget(self.message)
        self.manual_box = self._manual()
        sl.addWidget(self.manual_box)
        self.side_controls = QtWidgets.QGridLayout()
        sl.addLayout(self.side_controls)
        body.addWidget(side)
        self.side = side

        self.start_button = button('Start', self.start, 'primary')
        self.pause_button = button('Pause', self.session.pause)
        self.home_button = button('Stop and go home', self.confirm_home, 'dark')
        self.stop_button = button('STOP', self.session.emergency_stop, 'stop')
        self._place_controls(driving=False)

        self.manual_timer = QtCore.QTimer(self)
        self.manual_timer.timeout.connect(self.manual_tick)

    # ---------------------------------------------------------------- building blocks
    def _sheet(self, parent):
        sheet = QtWidgets.QFrame(parent)
        sheet.setObjectName('sheet')
        lay = QtWidgets.QVBoxLayout(sheet)
        self.sheet_title = label()
        self.sheet_text = label()
        lay.addWidget(self.sheet_title)
        lay.addWidget(self.sheet_text)
        row = QtWidgets.QHBoxLayout()
        self.check_button = button('Mark as checked', self.mark_checked, 'dark')
        row.addWidget(self.check_button, 1)
        row.addWidget(button('Close', lambda: self.select(None)))
        lay.addLayout(row)
        sheet.hide()
        return sheet

    def _drive_view(self):
        w = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(w)
        lay.setContentsMargins(32, 16, 32, 16)
        lay.setSpacing(10)
        self.d_alert = AlertBar(w)
        lay.addWidget(self.d_alert)
        lay.addStretch()
        self.d_pct = label('0', name='bigpct', wrap=False)
        lay.addWidget(self.d_pct)
        self.d_status = label(name='bigstatus')
        lay.addWidget(self.d_status)
        self.d_bar = QtWidgets.QProgressBar()
        self.d_bar.setTextVisible(False)
        self.d_bar.setStyleSheet('QProgressBar { max-height: 22px; border-radius: 11px; } '
                                 'QProgressBar::chunk { border-radius: 11px; }')
        lay.addWidget(self.d_bar)
        lay.addStretch()
        self.drive_controls = QtWidgets.QGridLayout()
        self.drive_controls.setSpacing(12)
        lay.addLayout(self.drive_controls)
        lay.addWidget(button('Show map', lambda: self._toggle_map(True)))
        return w

    def _manual(self):
        box = QtWidgets.QWidget()
        lay = QtWidgets.QGridLayout(box)
        lay.setContentsMargins(0, 0, 0, 0)
        self.manual_button = button('Drive by hand', self.toggle_manual)
        lay.addWidget(self.manual_button, 0, 0, 1, 4)
        self.arrows = {}
        for key, text, row, col in (('a', '◀', 1, 0), ('w', '▲', 1, 1), ('s', '▼', 1, 2),
                                    ('d', '▶', 1, 3)):
            b = QtWidgets.QPushButton(text)
            b.pressed.connect(lambda k=key: self.hold(k))
            b.released.connect(lambda: self.hold(None))
            lay.addWidget(b, row, col)
            self.arrows[key] = b
        return box

    def _place_controls(self, driving):
        grid = self.drive_controls if driving else self.side_controls
        for b in (self.start_button, self.pause_button, self.home_button, self.stop_button):
            b.setParent(None)
        if driving:                              # as the design: no Start while driving (it cannot be pressed)
            grid.addWidget(self.stop_button, 0, 0, 1, 2)
            grid.addWidget(self.pause_button, 1, 0)
            grid.addWidget(self.home_button, 1, 1)
        else:                                    # side panel: two rows, so everything fits 600 px
            grid.addWidget(self.stop_button, 0, 0)
            grid.addWidget(self.start_button, 0, 1)
            grid.addWidget(self.pause_button, 1, 0)
            grid.addWidget(self.home_button, 1, 1)
        self.home_button.setText('Stop and go home' if driving else 'Go home')
        big = int(self.panel.params['button_height'] * (1.5 if driving else 1))
        for b in (self.start_button, self.pause_button, self.home_button, self.stop_button):
            b.setMinimumHeight(big)
        self.driving_layout = driving

    def _layout_map(self, event):
        w, h = self.mapbox.width(), self.mapbox.height()
        self.view.setGeometry(0, 0, w, h)
        self.banner.setGeometry(0, 0, w, 44)
        top = 56 if self.banner.isVisible() else 12
        self.alert.setGeometry(12, top, w - 24, 46)
        self.sheet.setGeometry(12, h - 212, w - 90, 200)

    def _layer(self, key, on):
        self.view.layers[key] = on
        self.view.update()

    def _toggle_map(self, show):
        self.show_map_while_driving = show

    # ---------------------------------------------------------------- start / stop
    def begin(self, entry, plan, grid, station):
        self.entry = entry
        self.title.setText(f'<b>NALA</b> <span style="color:{style.MUTE}">{entry.name}</span>')
        self.view.set_grid(grid)
        self.view.heat_width_m = self.panel.params['heatmap_width_m']
        cfg = self.panel.settings['geometry']
        self.view.refresh(plan=plan_points(plan.data), station=station, sensor_offset=cfg['sensor_offset_m'],
                          sensor_radius=cfg['coverage_disk_radius_m'])
        self.manual_on = False
        self.show_map_while_driving = False
        self.drawn = {}
        self.select(None)
        self.chip_mode.setText('Robot' if self.panel.params['hardware'] else 'Simulation')
        self.manual_timer.start(int(self.panel.params['manual_period'] * 1000))
        teleop = self.session.start_launch(entry.map_yaml, plan.path, library.new_run_dir(entry))
        self.panel.executor.add_node(teleop)
        self.refresh()          # buttons follow the (still empty) state at once: no Start before the plan is checked

    def start(self):
        if self.session.state.get('state') == 'PAUSED':
            self.session.resume()
        else:
            self.session.start()

    def confirm_home(self):
        if self.ask_home():
            self.session.go_home()

    def ask_home(self):
        """The confirm dialog of the design: True = Stop and go home."""
        box = QtWidgets.QMessageBox(self)
        box.setWindowTitle('Stop scanning?')
        box.setText('<b>Stop scanning?</b><br>The robot stops and drives back to the base station.')
        yes = box.addButton('Stop and go home', QtWidgets.QMessageBox.AcceptRole)
        style.set_kind(yes, 'dark')
        box.addButton('Keep scanning', QtWidgets.QMessageBox.RejectRole)
        box.exec_()
        return box.clickedButton() is yes

    # ---------------------------------------------------------------- hotspots
    def select(self, hotspot_id):
        self.view.selected = hotspot_id
        h = next((h for h in self.session.hotspots if h['id'] == hotspot_id), None)
        if h is None:
            self.view.selected = None
            self.sheet.hide()
        else:
            bg = self.session.radiation.get('background')
            first = datetime.datetime.fromtimestamp(h['first_seen']).strftime('%H:%M:%S')
            self.sheet_title.setText(f"<span style='font-size:{self.panel.params['font_size'] + 6}pt'>"
                                     f"<b>Hotspot {h['id']}</b></span>")
            self.sheet_text.setText(f"Highest reading: {h['peak']:.0f} counts/s"
                                    + (f' (background about {bg:.0f})' if bg is not None else '')
                                    + f"<br>Location: x {h['x']:.2f} m, y {h['y']:.2f} m<br>First seen: {first}")
            self.check_button.setText('Checked' if h.get('checked') else 'Mark as checked')
            self.check_button.setEnabled(not h.get('checked'))
            self.sheet.show()
            self.sheet.raise_()
        self.view.update()

    def mark_checked(self):
        if self.view.selected is not None:
            self.session.mark_checked(self.view.selected)
            self.select(None)

    # ---------------------------------------------------------------- refresh (timer)
    def refresh(self):
        s = self.session
        s.update()
        state = s.state.get('state')
        running = s.launch_running()
        # A pause for a localization reason that ends by itself is not a real stop (user, 2026-10-19): the
        # driving view stays, with "Recalculating...".
        recalculating = running and state == 'PAUSED' and bool(s.state.get('auto_resume'))
        paused = state == 'PAUSED' and not recalculating
        driving = running and (recalculating or state in ('BACKGROUND', 'PREPARING', 'CONNECT_PLAN', 'CONNECT',
                                                          'SWEEP', 'DWELL', 'TURN', 'PHASE_WAIT', 'RETURN_PLAN',
                                                          'RETURN'))
        # The large progress view of the design while driving; after "Show map" the map stays until "Big view" or
        # a new scan (user, 2026-10-20: it used to switch back by itself after every pause or state change).
        big = driving and not self.show_map_while_driving
        if big != self.driving_layout:
            self._place_controls(big)
        self.left.setCurrentIndex(1 if big else 0)
        self.side.setVisible(not big)
        self.big_button.setVisible(driving and not big)
        self.back_button.setVisible(not driving)
        if self.banner.isVisible() != s.stop_active():
            self.banner.setVisible(s.stop_active())
            self._layout_map(None)

        m = s.measurement
        pct = 100 * s.state.get('fraction', 0.)
        reason = s.state.get('reason') or ''
        text = plain_state(s.state) if running else 'App 3 stopped. Tap Back.'
        if state == 'FINISHED' and reason == 'RETURNED_TO_BASE':
            text = 'Scan stopped, back home' if s.state.get('root_cause') == 'OPERATOR_GO_HOME' else 'Scan complete'
        self.pct.setText(f'{pct:.0f}<span style="font-size:20pt;color:{style.MUTE}"> %</span>')
        self.d_pct.setText(f'{pct:.0f}<span style="font-size:40pt;color:{style.MUTE}"> %</span>')
        self.status.setText(text)
        self.d_status.setText(text)
        self.bar.setValue(int(pct))
        self.d_bar.setValue(int(pct))
        lines = []
        if m.get('coverable_m2'):
            lines.append(f"{m.get('covered_m2', 0):.1f} of {m['coverable_m2']:.1f} m² scanned")
        if s.radiation.get('avoided_m2'):
            lines.append(f"{s.radiation['avoided_m2']:.1f} m² avoided (contaminated)")
        if s.radiation.get('background') is not None:
            lines.append(f"Background {s.radiation['background']:.0f} counts/s")
        if s.changes is not None and (s.changes > 0).any():
            cell = self.view.grid.resolution ** 2
            lines.append(f"Map changed: {(s.changes == 100).sum() * cell:.1f} m\u00b2 new, "
                         f"{(s.changes == 50).sum() * cell:.1f} m\u00b2 removed")
        if paused and reason:
            lines.append(f'Paused: {reason}')
        self.area.setText('<br>'.join(lines))
        self.message.setText(s.message)
        self.chip_loc.setText('Localized' if s.state.get('trusted') else 'Not localized')
        self.chip_loc.setProperty('on', bool(s.state.get('trusted')))
        self.chip_loc.style().unpolish(self.chip_loc)
        self.chip_loc.style().polish(self.chip_loc)
        self.alert.show_hotspots(s.hotspots)
        self.d_alert.show_hotspots(s.hotspots)

        self.start_button.setText('Resume' if paused else 'New scan' if state in ('FINISHED', 'CANCELED', 'FAILED')
                                  else 'Start')
        self.start_button.setEnabled(running and ((paused and not self.manual_on) or s.can_start()))
        self.pause_button.setEnabled(running and (s.active() or recalculating))
        self.home_button.setEnabled(running and (s.active() or paused or recalculating))
        # Hide before show: if both were visible for a moment, Qt would grow the window past the screen.
        self.legend.setVisible(not (running and paused))     # room for the hand control arrows
        self.toggles.setVisible(not (running and paused))
        self.manual_box.setVisible(running and paused)
        self.manual_button.setText('Hand control off' if self.manual_on else 'Drive by hand')
        for b in self.arrows.values():
            b.setEnabled(self.manual_on)
        self._draw()

    def _draw(self):
        """Redraw only the map layers whose data changed (cheap on the Pi)."""
        s, v, d = self.session, self.view, self.drawn
        if s.covered is not None and s.covered is not d.get('covered'):
            v.set_mask('coverage', s.covered, style.TEAL, 120)
            d['covered'] = s.covered
        if s.keepout is not None and s.keepout is not d.get('keepout'):
            v.set_mask('keepout', s.keepout, style.MAG, 90)
            d['keepout'] = s.keepout
        heat = (s.heatmap, s.radiation.get('threshold'))
        if s.heatmap is not None and (heat[0] is not d.get('heat', (None, None))[0] or heat[1] != d['heat'][1]):
            v.set_heatmap(*heat)
            d['heat'] = heat
        if s.changes is not None and s.changes is not d.get('changes'):
            v.set_changes(s.changes == 100, s.changes == 50)
            d['changes'] = s.changes
        v.refresh(trail=s.trail, hotspots=s.hotspots, robot=s.display_pose())
        if v.selected is not None:
            self.select(v.selected)

    # ---------------------------------------------------------------- driving by hand
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
            answer = QtWidgets.QMessageBox.question(
                self, 'Leave', 'A scan is still running. Cancel it and stop app 3? '
                '(Use "Stop and go home" to bring the robot back first.)')
            if answer != QtWidgets.QMessageBox.Yes:
                return
            s.cancel()
        if self.manual_on:
            self.toggle_manual()
        self.manual_timer.stop()
        self.status.setText('Stopping app 3...')
        QtWidgets.QApplication.processEvents()
        s.stop_launch()
        report = s.report()
        if report:
            skipped = sum(b.get('reason') == 'PLAN_STRETCH_SKIPPED' for b in report.get('temporary_blockages', []))
            avoided = sum(b.get('reason') == 'CONTAMINATION_AVOIDED' for b in report.get('temporary_blockages', []))
            QtWidgets.QMessageBox.information(
                self, 'Scan report', f"{report.get('state')}: {report.get('reason')}\n"
                f"Skipped stretches: {skipped}, avoided (contaminated): {avoided}\nSaved in: {s.output_dir}")
        self.panel.open_map(self.entry)


class Keyboard(QtWidgets.QWidget):
    """On-screen keyboard for a map name (letters, digits, - and _: the name becomes a folder)."""
    ROWS = ('1234567890', 'qwertyuiop', 'asdfghjkl', 'zxcvbnm-_')

    def __init__(self, on_change):
        super().__init__()
        self.text, self.on_change, self.upper = '', on_change, False
        grid = QtWidgets.QGridLayout(self)
        grid.setSpacing(6)
        self.keys = []
        for r, row in enumerate(self.ROWS):
            for c, ch in enumerate(row):
                b = QtWidgets.QPushButton(ch)
                b.setFocusPolicy(QtCore.Qt.NoFocus)
                b.setStyleSheet('padding: 0px;')        # 10 keys a row must fit next to the side panel
                b.setMinimumWidth(40)
                b.clicked.connect(lambda _, k=ch: self.type(k))
                grid.addWidget(b, r, c)
                self.keys.append(b)
        self.shift = button('Shift', self.toggle_shift)
        delete = button('Del', self.delete)
        for b in (self.shift, delete):
            b.setStyleSheet('padding: 0px;')
        grid.addWidget(self.shift, 3, 9)
        grid.addWidget(delete, 2, 9)

    def type(self, ch):
        self.set_text(self.text + (ch.upper() if self.upper else ch))

    def delete(self):
        self.set_text(self.text[:-1])

    def toggle_shift(self):
        self.upper = not self.upper
        for b in self.keys:
            b.setText(b.text().upper() if self.upper else b.text().lower())

    def set_text(self, text):
        self.text = text
        self.on_change(text)


class AppScreen(QtWidgets.QWidget):
    """App 1 (drive with the Xbox controller) or app 2 (make a map, then save it under a typed name)."""
    TITLES = {'app1': 'Drive (app 1)', 'app2': 'Make a map (app 2)'}
    XBOX = ('Xbox controller: hold <b>LB</b> to drive, then let both sticks go to the centre once. Left stick: '
            'forward / back and sideways, right stick: turn. Release LB = stop.')

    def __init__(self, panel):
        super().__init__()
        self.panel = panel
        self.session = panel.apps
        self.app = None
        lay = QtWidgets.QHBoxLayout(self)
        lay.setContentsMargins(8, 8, 8, 8)
        self.pages = QtWidgets.QStackedWidget()                 # the live map (app 2) or the name keyboard
        self.view = MapWidget()
        self.pages.addWidget(self.view)
        name_page = QtWidgets.QWidget()
        nl = QtWidgets.QVBoxLayout(name_page)
        self.name = label('', name='status')                   # wraps: a 40-character name must fit the screen
        nl.addWidget(label('Name of the new map:', wrap=False))
        nl.addWidget(self.name)
        self.name_problem = label(name='mute')
        nl.addWidget(self.name_problem)
        self.keyboard = Keyboard(self._name_changed)
        nl.addWidget(self.keyboard)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(button('Cancel', lambda: self.pages.setCurrentIndex(0)))
        self.save_button = button('Save', self.save, 'primary')
        row.addWidget(self.save_button)
        nl.addLayout(row)
        self.pages.addWidget(name_page)
        self.info = label()                                      # app 1: no map, the instructions instead
        self.info.setAlignment(QtCore.Qt.AlignTop)
        self.pages.addWidget(self.info)
        lay.addWidget(self.pages, 1)
        side = QtWidgets.QFrame()
        side.setObjectName('side')
        side.setFixedWidth(330)
        sl = QtWidgets.QVBoxLayout(side)
        head = QtWidgets.QHBoxLayout()
        self.title = label(wrap=False)
        head.addWidget(self.title, 1)
        head.addWidget(button('Back', self.back))
        sl.addLayout(head)
        self.status = label(name='status')
        sl.addWidget(self.status)
        self.detail = label(name='mute')
        sl.addWidget(self.detail)
        self.xbox = label(self.XBOX, name='mute')
        sl.addWidget(self.xbox)
        sl.addStretch()
        self.save_map_button = button('Save map', self.ask_name, 'dark')
        sl.addWidget(self.save_map_button)
        self.start_button = button('Start', self.start, 'primary')
        sl.addWidget(self.start_button)
        self.stop_button = button('STOP', self.stop, 'stop')
        sl.addWidget(self.stop_button)
        lay.addWidget(side)
        self.message = ''

    def begin(self, app):
        self.app, self.message = app, ''
        self.title.setText(f'<b>{self.TITLES[app]}</b>')
        self.pages.setCurrentIndex(0 if app == 'app2' else 2)
        if app == 'app1':
            self.info.setText('<b>Drive the robot with the Xbox controller.</b><br><br>' + self.XBOX +
                              '<br><br>STOP stops app 1 at once (the motors stop).')
        self.xbox.setVisible(app == 'app2')
        self.view.set_grid(Grid.from_cells(np.full((2, 2), -1, np.int8), .05, (0., 0., 0.)))
        self.start()

    def start(self):
        busy = self.session.busy()
        if busy:
            self.message = 'Another app is running (' + ', '.join(busy) + '). Stop it first.'
        else:
            self.session.start(self.app)
            self.message = ''
        self.refresh()

    def stop(self):
        self.session.stop()
        self.message = 'Stopped.'
        self.refresh()

    def back(self):
        self.session.stop()
        self.panel.show_maps()

    def ask_name(self):
        self.keyboard.set_text(datetime.datetime.now().strftime('map_%Y-%m-%d_%H%M'))
        self.pages.setCurrentIndex(1)
        self.refresh()

    def _name_changed(self, text):
        self.name.setText(text or ' ')
        problem = apps.name_problem(text, self.panel.maps_dir)
        self.name_problem.setText(problem)
        self.save_button.setEnabled(not problem)

    def save(self):
        self.session.save_map(self.keyboard.text)
        self.pages.setCurrentIndex(0)
        self.refresh()

    def refresh(self):
        s = self.session
        running = s.running()
        result = s.poll_save()
        if result is not None:
            ok, line = result
            self.message = (f'Saved as {self.keyboard.text}: it is now in the map list (for app 3).' if ok
                            else f'Not saved: {line}')
        if not running:
            text = 'Not running.'
        elif not s.driver_ok():
            text = 'Starting... (waiting for the motor driver)'
        elif s.saving is not None:
            text = 'Saving the map...'
        else:
            text = 'Running: drive with the Xbox controller.'
        self.status.setText(text)
        detail = []
        if self.app == 'app2' and s.grid is not None:
            if s.grid is not self.view.grid:
                self.view.set_grid(s.grid, fit=s.grid.cells.shape != self.view.grid.cells.shape)
            seen = int((s.grid.cells >= 0).sum()) * s.grid.resolution ** 2
            detail.append(f'Map so far: {seen:.0f} m\u00b2 seen. Drive slowly, turn slowly.')
        if self.app == 'app2':
            self.view.refresh(robot=s.robot_pose() if running else None)
        if self.message:
            detail.append(self.message)
        self.detail.setText('<br>'.join(detail))
        self.start_button.setVisible(not running)
        self.stop_button.setEnabled(running)
        self.save_map_button.setVisible(self.app == 'app2')
        self.save_map_button.setEnabled(running and s.grid is not None and s.saving is None
                                        and self.pages.currentIndex() == 0)


class Panel(QtWidgets.QMainWindow):
    def __init__(self, node, executor, params):
        super().__init__()
        self.node, self.executor, self.params = node, executor, params
        repo = Path(params['repo_dir'])
        self.repo_dir = repo
        self.maps_dir = repo / params['maps_dir']
        self.settings = load_settings(repo / 'config')
        self.session = DriveSession(repo / 'config', params['hardware'], params['launch_stop_timeout'],
                                    params['preview_retry'])
        executor.add_node(self.session)
        self.apps = apps.AppSession(repo, params['launch_stop_timeout'])
        executor.add_node(self.apps)
        self.setWindowTitle('NALA')
        self.stack = QtWidgets.QStackedWidget()
        self.setCentralWidget(self.stack)
        self.maps = MapsScreen(self)
        self.map = MapScreen(self)
        self.planning = PlanningScreen(self)
        self.scan = ScanScreen(self)
        self.app = AppScreen(self)
        for screen in (self.maps, self.map, self.planning, self.scan, self.app):
            self.stack.addWidget(screen)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(int(params['update_period'] * 1000))
        self.show_maps()

    def show_maps(self):
        self.maps.refresh()
        self.stack.setCurrentWidget(self.maps)

    def open_map(self, entry):
        self.stack.setCurrentWidget(self.map)
        self.map.open(entry)

    def start_planning(self, entry, start, area):
        out_yaml, out_png = library.new_plan_paths(entry)
        cmd = planning.command(self.repo_dir, entry.map_yaml, entry.station_file, out_yaml, out_png, start, area)
        self.planning.begin(entry, planning.PlanRun(cmd, out_yaml, out_png))
        self.stack.setCurrentWidget(self.planning)

    def start_app(self, app):
        self.stack.setCurrentWidget(self.app)
        self.app.begin(app)

    def start_drive(self, entry, plan):
        self.stack.setCurrentWidget(self.scan)
        self.scan.begin(entry, plan, self.map.grid, self.map.station)

    def tick(self):
        current = self.stack.currentWidget()
        if current is self.planning:
            self.planning.poll()
        elif current is self.scan:
            self.scan.refresh()
        elif current is self.app:
            self.app.refresh()

    def closeEvent(self, event):
        if self.planning.run is not None and not self.planning.run.done():
            self.planning.run.cancel()
        self.session.stop_launch()
        self.apps.close()
        super().closeEvent(event)


def make_app(params):
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv[:1])
    app.setStyleSheet(style.stylesheet(params['font_size'], params['button_height']))
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
