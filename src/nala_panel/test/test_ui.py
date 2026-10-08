"""Panel smoke test without a screen (Qt offscreen) at 1024 x 600: every screen builds and the taps work.

NALA_PANEL_SHOTS=<folder> saves a screenshot of every screen there, to check the layout by eye.
"""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import math
from pathlib import Path
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from nala_panel import ui
from conftest import REPO

PARAMS = {'maps_dir': 'maps', 'fullscreen': False, 'window_width': 1024, 'window_height': 600, 'font_size': 13,
          'button_height': 52, 'update_period': 0.3, 'manual_period': 0.05, 'launch_stop_timeout': 5.0,
          'hardware': False}


def shot(widget, name):
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


def test_maps_map_new_plan_planning_and_drive_screens(panel, tmp_path, monkeypatch):
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
    window.map.view.figure.savefig(png)
    window.planning.begin(screen.entry, FakeRun(png))
    window.stack.setCurrentWidget(window.planning)
    window.planning.poll()
    app.processEvents()
    assert window.planning.keep_button.isVisible()
    shot(window, '4_planning_result')

    # Drive screen with a running task (app 3 itself is not started here).
    session = window.session
    monkeypatch.setattr(session, 'start_launch', lambda *a: session)
    monkeypatch.setattr(session, 'launch_running', lambda: True)
    window.drive.begin(screen.entry, screen.plans[0], screen.grid, screen.station)
    window.stack.setCurrentWidget(window.drive)
    session.state = {'state': 'SWEEP', 'reason': '', 'trusted': True, 'fraction': 0.42, 'preview_ready': True}
    session.pose = (1.0, -3.0, 0.3)
    window.drive.refresh()
    app.processEvents()
    assert window.drive.pause_button.isEnabled() and not window.drive.start_button.isEnabled()
    assert not window.drive.manual_button.isEnabled()
    shot(window, '5_drive_covering')
    session.state = {'state': 'PAUSED', 'reason': 'BLOCKED_IN_PLACE: FOLLOW_PATH_ERROR_106', 'trusted': True, 'fraction': 0.6}
    window.drive.refresh()
    app.processEvents()
    assert window.drive.resume_button.isEnabled() and window.drive.manual_button.isEnabled()
    shot(window, '6_drive_paused')
