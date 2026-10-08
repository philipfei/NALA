"""Drive a plan from the panel (rclpy, no Qt): start app 3, follow its state, send the operator's commands.

The robot only moves after start() (the Start button). Commands go to the coverage supervisor's services,
exactly as an operator would call them in a terminal (README, app 3).
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path as PathMsg
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from nala_coverage.keyboard_teleop import KeyboardTeleop
from nala_coverage.ros_common import LATCHED, LATEST, pose3

TRIGGERS = ('/coverage/preview', '/coverage/start', '/coverage/pause', '/coverage/resume', '/coverage/cancel',
            '/safety/estop', '/safety/reset')
# Supervisor states in which a task is running (the robot may move).
ACTIVE = ('PREPARING', 'CONNECT_PLAN', 'CONNECT', 'SWEEP', 'DWELL', 'PHASE_WAIT', 'RETURN_PLAN', 'RETURN')


def launch_command(map_yaml, plan_yaml, output_dir, hardware):
    return ['ros2', 'launch', 'nala_bringup', 'app3_coverage_pi.launch.py', f'map:={map_yaml}',
            f'plan:={plan_yaml}', f'output_dir:={output_dir}', f'hardware:={str(bool(hardware)).lower()}']


def latest_report(output_dir):
    """The last task report written by the supervisor, or None."""
    reports = sorted(Path(output_dir).glob('task_*.json'), key=os.path.getmtime)
    return json.loads(reports[-1].read_text()) if reports else None


class DriveSession(Node):
    """App 3 for one map and plan. Data from callbacks is read by the screen with plain attribute reads."""

    def __init__(self, config_dir, hardware, stop_timeout):
        super().__init__('nala_panel_drive')
        self.config_dir, self.hardware, self.stop_timeout = str(config_dir), hardware, stop_timeout
        self.process = None
        self.output_dir = None
        self.state = {}
        self.pose = None
        self.covered = None          # bool mask (rows, cols) in the map grid
        self.route = []
        self.message = ''            # last service reply, for the screen
        self.preview_requested = False
        self.create_subscription(String, '/coverage/state', self._state, LATEST)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._pose, 10)
        self.create_subscription(OccupancyGrid, '/coverage/covered', self._covered, LATCHED)
        self.create_subscription(PathMsg, '/coverage/route', self._route, LATCHED)
        self.clients_ = {name: self.create_client(Trigger, name) for name in TRIGGERS}
        self.teleop = None           # created when the launch starts (it reads the config)

    # ---------------------------------------------------------------- callbacks
    def _state(self, msg):
        self.state = json.loads(msg.data)

    def _pose(self, msg):
        self.pose = pose3(msg.pose.pose)

    def _covered(self, msg):
        self.covered = np.asarray(msg.data, np.int8).reshape(msg.info.height, msg.info.width) > 0

    def _route(self, msg):
        self.route = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]

    # ---------------------------------------------------------------- app 3 process
    def start_launch(self, map_yaml, plan_yaml, output_dir):
        self.output_dir = Path(output_dir)
        self.state, self.pose, self.covered, self.route, self.message = {}, None, None, [], ''
        self.preview_requested = False
        log = open(self.output_dir / 'launch.log', 'w')
        self.process = subprocess.Popen(launch_command(map_yaml, plan_yaml, output_dir, self.hardware),
                                        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        if self.teleop is None:
            self.teleop = KeyboardTeleop(self.config_dir)
        return self.teleop

    def launch_running(self):
        return self.process is not None and self.process.poll() is None

    def stop_launch(self):
        """Ctrl-C for app 3: nodes cancel their actions and send zero speed; killed after stop_timeout."""
        if self.process is None:
            return
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(self.stop_timeout)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait()
        self.process = None

    # ---------------------------------------------------------------- operator commands
    def call(self, name, then=None):
        """Call a Trigger service without blocking; the reply goes to self.message."""
        client = self.clients_[name]
        if not client.service_is_ready():
            self.message = f'{name} is not available yet'
            return

        def done(future):
            result = future.result()
            self.message = result.message if result else f'{name}: no reply'
            if then and result and result.success:
                then()
        client.call_async(Trigger.Request()).add_done_callback(done)

    def update(self):
        """Called by the screen: ask for the preview once localization is trusted (no motion)."""
        if (self.launch_running() and not self.preview_requested and self.state.get('trusted')
                and not self.state.get('preview_ready') and self.state.get('state') == 'IDLE'):
            self.preview_requested = True
            self.call('/coverage/preview')

    def can_start(self):
        s = self.state
        return bool(s.get('preview_ready') and s.get('trusted') and s.get('nav_active')
                    and s.get('state') in ('IDLE', 'CANCELED', 'FINISHED', 'FAILED'))

    def start(self):
        self.call('/coverage/start')

    def pause(self):
        self.call('/coverage/pause')

    def resume(self):
        # After STOP the gate stays stopped until it is reset.
        self.call('/safety/reset', then=lambda: self.call('/coverage/resume'))

    def cancel(self):
        self.call('/coverage/cancel')

    def emergency_stop(self):
        """STOP: the velocity gate sends zero at once and stays stopped; the task pauses."""
        self.call('/safety/estop')
        self.call('/coverage/pause')

    def active(self):
        return self.state.get('state') in ACTIVE

    # ---------------------------------------------------------------- driving by hand while paused
    def manual(self, enabled):
        """Ask the supervisor for manual control through the gate (only while paused and stopped)."""
        if self.teleop is None:
            return
        if not enabled:
            self.teleop.stop()
        request = SetBool.Request()
        request.data = enabled

        def done(future):
            result = future.result()
            self.message = result.message if result else 'manual: no reply'
        self.teleop.manual.call_async(request).add_done_callback(done)

    def manual_key(self, key):
        """Hold-to-drive: called every tick while an arrow is pressed ('w', 's', 'a', 'd'), 'x' to stop."""
        if self.teleop is not None:
            self.teleop.set_key(key)

    def manual_tick(self):
        if self.teleop is not None:
            self.teleop.tick()

    def report(self):
        return latest_report(self.output_dir) if self.output_dir else None


def plain_state(state):
    """The supervisor state in plain words for the screen."""
    words = {
        'IDLE': 'Ready', 'PREPARING': 'Checking before driving', 'CONNECT_PLAN': 'Planning the way to the path',
        'CONNECT': 'Driving to the path', 'SWEEP': 'Covering', 'DWELL': 'Covering',
        'PHASE_WAIT': 'Plan done, checking coverage', 'RETURN_PLAN': 'Planning the way back',
        'RETURN': 'Driving back to the base station', 'PAUSED': 'Paused', 'FINISHED': 'Finished',
        'CANCELED': 'Cancelled', 'FAILED': 'Failed'}
    s = state.get('state')
    if not s:
        return 'Starting app 3...'
    text = words.get(s, s)
    if s == 'IDLE' and not state.get('trusted'):
        text = 'Finding the robot on the map'
    elif s == 'IDLE' and not state.get('preview_ready'):
        text = 'Checking the plan'
    return text


def heading_arrow(pose, length):
    """(x, y, dx, dy) of an arrow for a pose."""
    x, y, yaw = pose
    return x, y, length * math.cos(yaw), length * math.sin(yaw)
