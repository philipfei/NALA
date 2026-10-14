"""Panel smoke test without a screen (Qt offscreen) at 1024 x 600: every screen builds and the taps work.

NALA_PANEL_SHOTS=<folder> saves a screenshot of every screen there, to check the layout by eye.
"""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import math
import numpy as np
from pathlib import Path
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from PyQt5.QtTest import QTest
from nala_panel import ui
from conftest import REPO

PARAMS = {'maps_dir': 'maps', 'fullscreen': False, 'window_width': 1024, 'window_height': 600, 'font_size': 13,
          'button_height': 52, 'update_period': 0.3, 'manual_period': 0.05, 'heatmap_width_m': 0.15, 'launch_stop_timeout': 5.0,
          'hardware': False}


def shot(widget, name):
    # Everything must fit the 7 inch screen (1024 x 600): no screen page needs more room.
    assert widget.height() <= 600, (name, widget.height())          # the window itself never grew
    for page in (widget.stack.widget(i) for i in range(widget.stack.count())):
        hint = page.minimumSizeHint()
        assert hint.width() <= 1024 and hint.height() <= 600, (name, type(page).__name__, hint.width(), hint.height())
    folder = os.environ.get('NALA_PANEL_SHOTS')
    if folder:
        Path(folder).mkdir(parents=True, exist_ok=True)
        widget.grab().save(str(Path(folder) / f'{name}.png'))


class FakeRun:
    """A finished planning run (the real one is tested in test_planning.py)."""

    def __init__(self, png):
        self.outputs = (png.with_suffix('.yaml'), png)

    def lines(self):
        return ['Finding reachable space...', 'Wrote 123 poses']

    def done(self):
        return True

    def ok(self):
        return True

    def stats(self):
        return {'coverage_pct': 54.2, 'drive_length_m': 14.5, 'estimated_time_s': 330, 'coverage_of_reachable_pct': 100.0,
                'footprint_hits': 0, 'drive_unsafe_segments': 0, 'drive_lost_cells': 0}

    def discard(self):
        pass


@pytest.fixture
def panel(maps_dir):
    repo = maps_dir.parent
    (repo / 'config').symlink_to(REPO / 'config')
    rclpy.init()
    node = Node('nala_panel_test')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    app = ui.make_app(PARAMS)
    window = ui.Panel(node, executor, {**PARAMS, 'repo_dir': str(repo)})
    window.resize(1024, 600)
    window.show()
    app.processEvents()
    yield window, app
    window.timer.stop()
    window.close()
    executor.shutdown()
    rclpy.shutdown()


def test_maps_map_new_plan_and_planning_screens(panel, tmp_path, monkeypatch):
    window, app = panel
    assert window.maps.list.count() == 1
    shot(window, '1_maps')

    window.open_map(window.maps.list.item(0).data(ui.QtCore.Qt.UserRole))
    app.processEvents()
    screen = window.map
    assert screen.station is not None and len(screen.plans) == 1 and screen.drive_button.isEnabled()
    shot(window, '2_map_plans')

    # New plan: base station (position, then direction), a start point, an area of 4 corners.
    screen.tabs.setCurrentIndex(1)
    screen.set_mode('station')
    screen.tap(1.0, -3.4)
    screen.tap(1.0, -2.4)
    assert screen.station == pytest.approx((1.0, -3.4, math.pi / 2), abs=1e-3), screen.status.text()
    # The old plan was made for the old base station: it can no longer be driven.
    assert 'PLAN_BASE_MISMATCH' in screen.plan_list.item(0).data(ui.QtCore.Qt.UserRole)
    screen.set_mode('start')
    screen.tap(1.157, -1.875)
    assert screen.start == [1.157, -1.875] and screen.at_point.isChecked()
    screen.set_mode('area')
    for corner in ([0.0, -4.0], [2.0, -4.0], [2.0, -2.0], [0.0, -2.0]):
        screen.tap(*corner)
    screen.close_area()
    assert screen.area_closed and len(screen.area) == 4
    app.processEvents()
    shot(window, '3_map_new_plan')

    started = []
    monkeypatch.setattr(window, 'start_planning', lambda entry, start, area: started.append((start, area)))
    screen.plan()
    assert started == [([1.157, -1.875], screen.area)]

    # Planning result.
    png = tmp_path / 'result.png'
    window.map.view.grab().save(str(png))
    window.planning.begin(screen.entry, FakeRun(png))
    window.stack.setCurrentWidget(window.planning)
    window.planning.poll()
    app.processEvents()
    assert window.planning.keep_button.isVisible()
    shot(window, '4_planning_result')


def scan_screen(window, app, monkeypatch):
    """The Scan screen of the room plan, with app 3 replaced by data set in the session."""
    window.open_map(window.maps.list.item(0).data(ui.QtCore.Qt.UserRole))
    screen = window.map
    session = window.session
    monkeypatch.setattr(session, 'start_launch', lambda *a: session)
    monkeypatch.setattr(session, 'launch_running', lambda: True)
    window.start_drive(screen.entry, screen.plans[0])
    # Nothing known from app 3 yet: Start must not be pressable.
    assert not window.scan.start_button.isEnabled()
    grid = screen.grid
    heat = np.full(grid.cells.shape, np.nan, np.float32)
    covered = np.zeros(grid.cells.shape, bool)
    keepout = np.zeros(grid.cells.shape, bool)
    for x in np.arange(0.2, 2.2, 0.05):                       # a scanned stripe at y = -3.0
        rc = grid.cell((x, -3.0))
        heat[rc] = 30.0
        covered[max(rc[0] - 10, 0):rc[0] + 10, rc[1]] = True
    rc = grid.cell((1.9, -3.0))
    heat[rc] = 400.0
    keepout[rc[0] - 4:rc[0] + 5, rc[1] - 4:rc[1] + 5] = True
    session.heatmap, session.covered, session.keepout = heat, covered, keepout
    session.radiation = {'background': 30.0, 'threshold': 60.0}
    session.hotspots = [{'id': 1, 'peak': 400.0, 'x': 1.9, 'y': -3.0, 'first_seen': 1.8e9, 'checked': False}]
    session.trail = [[[0.2, -3.0], [1.0, -3.0], [1.7, -3.0]]]
    session.measurement = {'covered_m2': 4.2, 'coverable_m2': 9.7, 'avoided_m2': 0.2}
    session.pose = (1.7, -3.0, 0.0)
    return window.scan, session


def test_scan_screen_shows_progress_heatmap_hotspots_and_controls(panel, monkeypatch):
    window, app = panel
    scan, session = scan_screen(window, app, monkeypatch)
    calls = []
    for name in ('start', 'pause', 'resume', 'go_home', 'emergency_stop', 'mark_checked'):
        monkeypatch.setattr(session, name, lambda *a, n=name: calls.append((n, *a)))
    scan.stop_button.clicked.disconnect()
    scan.stop_button.clicked.connect(session.emergency_stop)

    # Ready: the map view with the side panel; Start only once the plan is checked.
    session.state = {'state': 'IDLE', 'trusted': True, 'preview_ready': True, 'nav_active': True, 'fraction': 0.}
    scan.refresh()
    app.processEvents()
    assert scan.left.currentIndex() == 0 and scan.start_button.isEnabled() and not scan.pause_button.isEnabled()
    assert '1 contaminated spot, 1 to check' in scan.alert.text.text()
    shot(window, '5_scan_ready')

    # Driving: the large progress view, then the map on request.
    session.state = {'state': 'SWEEP', 'trusted': True, 'fraction': 0.42}
    scan.refresh()
    app.processEvents()
    assert scan.left.currentIndex() == 1 and '42' in scan.d_pct.text() and scan.pause_button.isEnabled()
    shot(window, '6_scan_driving')
    scan._toggle_map(True)
    scan.refresh()
    app.processEvents()
    assert scan.left.currentIndex() == 0
    # The map while driving still has STOP, Pause and Stop and go home, and a way back to the big view.
    assert all(b.isVisible() for b in (scan.stop_button, scan.pause_button, scan.home_button, scan.big_button))
    shot(window, '7_scan_driving_map')
    scan.big_button.click()
    scan.refresh()
    assert scan.left.currentIndex() == 1 and scan.stop_button.isVisible()
    scan._toggle_map(True)
    scan.refresh()

    # Tap the hotspot: the sheet, then Mark as checked.
    pt = scan.view.to_screen(1.9, -3.0)
    QTest.mouseClick(scan.view, ui.QtCore.Qt.LeftButton, pos=ui.QtCore.QPoint(int(pt.x()), int(pt.y())))
    app.processEvents()
    assert scan.sheet.isVisible() and 'Hotspot 1' in scan.sheet_title.text() and '400 counts/s' in scan.sheet_text.text()
    shot(window, '8_scan_hotspot')
    scan.check_button.click()
    assert ('mark_checked', 1) in calls and not scan.sheet.isVisible()

    # Paused after STOP: banner, Resume, drive by hand; Stop and go home asks first.
    session.state = {'state': 'PAUSED', 'reason': 'OPERATOR_PAUSE', 'trusted': True, 'fraction': 0.6}
    session.gate = {'fault': 'OPERATOR_OR_ACTION_STOP'}
    scan.refresh()
    app.processEvents()
    assert scan.banner.isVisible() and scan.start_button.text() == 'Resume' and scan.manual_box.isVisible()
    assert scan.minimumSizeHint().height() <= 600, scan.minimumSizeHint().height()   # with the hand control arrows
    shot(window, '9_scan_stopped')
    scan.start_button.click()
    assert calls[-1] == ('resume',)
    monkeypatch.setattr(scan, 'ask_home', lambda: False)        # Keep scanning
    scan.home_button.click()
    assert calls[-1] == ('resume',)
    monkeypatch.setattr(scan, 'ask_home', lambda: True)         # Stop and go home
    scan.home_button.click()
    assert calls[-1] == ('go_home',)


def test_map_widget_tap_drag_and_pinch(panel):
    window, app = panel
    window.open_map(window.maps.list.item(0).data(ui.QtCore.Qt.UserRole))
    view = window.map.view
    app.processEvents()
    taps = []
    view.tapped.connect(lambda x, y: taps.append((x, y)))
    pt = view.to_screen(1.0, -3.0)
    QTest.mouseClick(view, ui.QtCore.Qt.LeftButton, pos=ui.QtCore.QPoint(int(pt.x()), int(pt.y())))
    assert taps and math.dist(taps[0], (1.0, -3.0)) < 2 / view.scale
    tx = view.tx
    QTest.mousePress(view, ui.QtCore.Qt.LeftButton, pos=ui.QtCore.QPoint(100, 100))
    # QTest.mouseMove does not send a move with the button held (Qt 5): send the event itself.
    move = ui.QtGui.QMouseEvent(ui.QtCore.QEvent.MouseMove, ui.QtCore.QPointF(160, 100), ui.QtCore.Qt.NoButton,
                                ui.QtCore.Qt.LeftButton, ui.QtCore.Qt.NoModifier)
    ui.QtWidgets.QApplication.sendEvent(view, move)
    QTest.mouseRelease(view, ui.QtCore.Qt.LeftButton, pos=ui.QtCore.QPoint(160, 100))
    assert view.tx == pytest.approx(tx + 60) and len(taps) == 1          # a drag is not a tap
    scale = view.scale
    view.zoom(2.0)
    assert view.scale == pytest.approx(2 * scale)
