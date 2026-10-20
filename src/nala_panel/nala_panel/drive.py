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
import time
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path as PathMsg
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from nala_coverage.keyboard_teleop import KeyboardTeleop
from nala_coverage.ros_common import LATCHED, LATEST, json_msg, pose3

TRIGGERS = ('/coverage/preview', '/coverage/start', '/coverage/pause', '/coverage/resume', '/coverage/cancel',
            '/coverage/go_home', '/safety/estop', '/safety/reset')
# Supervisor states in which a task is running (the robot may move).
# No task holds the robot (it may be moved by hand); the same as the supervisor's IDLE_STATES.
IDLE_STATES = ('IDLE', 'FINISHED', 'CANCELED', 'FAILED')
ACTIVE = ('BACKGROUND', 'PREPARING', 'CONNECT_PLAN', 'CONNECT', 'SWEEP', 'DWELL', 'TURN', 'PHASE_WAIT', 'RETURN_PLAN',
          'RETURN')
STOP_FAULT = 'OPERATOR_OR_ACTION_STOP'   # the velocity gate's latched fault after STOP


def launch_command(map_yaml, plan_yaml, output_dir, hardware):
    return ['ros2', 'launch', 'nala_bringup', 'app3_coverage_pi.launch.py', f'map:={map_yaml}',
            f'plan:={plan_yaml}', f'output_dir:={output_dir}', f'hardware:={str(bool(hardware)).lower()}']


def latest_report(output_dir):
    """The last task report written by the supervisor, or None."""
    reports = sorted(Path(output_dir).glob('task_*.json'), key=os.path.getmtime)
    return json.loads(reports[-1].read_text()) if reports else None


class DriveSession(Node):
    """App 3 for one map and plan. Data from callbacks is read by the screen with plain attribute reads."""

    def __init__(self, config_dir, hardware, stop_timeout, preview_retry):
        super().__init__('nala_panel_drive')
        self.config_dir, self.hardware, self.stop_timeout = str(config_dir), hardware, stop_timeout
        self.preview_retry = preview_retry
        self.process = None
        self.output_dir = None
        self.state = {}
        self.pose = None
        self.covered = None          # bool mask (rows, cols) in the map grid: floor the sensor actually covered
        self.route = []
        self.trail = []              # actual trail: trusted measured poses only (the coverage meter)
        self.measurement = {}        # coverage meter: covered / coverable / avoided m2
        self.gate = {}               # velocity gate state (fault = STOP latched)
        self.heatmap = None          # float32 (rows, cols): highest counts/s per cell, NaN = no reading
        self.keepout = None          # bool (rows, cols): contaminated floor plus margin, avoided
        self.hotspots = []
        self.radiation = {}          # background, threshold, ...
        self.changes = None          # int8 (rows, cols): 100 new object, 50 removed object, 0 unchanged (mapchange_node)
        self.message = ''            # last service reply, for the screen
        self.located = {}            # locator_node: where the robot is on the map, and whether on the base station
        self.preview_asked = -math.inf   # time of the last preview request (asked again if it did not come)
        self.create_subscription(String, '/coverage/state', self._state, LATEST)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._pose, 10)
        self.create_subscription(OccupancyGrid, '/coverage/covered', self._covered, LATCHED)
        self.create_subscription(PathMsg, '/coverage/route', self._route, LATCHED)
        self.create_subscription(PathMsg, '/coverage/trajectory', self._trail, LATCHED)
        self.create_subscription(String, '/coverage/measurement', self._measurement, LATEST)
        self.create_subscription(String, '/safety/state', self._gate, LATEST)
        self.create_subscription(Image, '/radiation/heatmap', self._heatmap, LATCHED)
        self.create_subscription(OccupancyGrid, '/radiation/keepout', self._keepout, LATCHED)
        self.create_subscription(String, '/radiation/hotspots', self._hotspots, LATCHED)
        self.create_subscription(String, '/radiation/state', self._radiation, LATEST)
        self.create_subscription(OccupancyGrid, '/map_changes', self._changes, LATCHED)
        self.create_subscription(String, '/coverage/located', self._located, LATEST)
        self.checked_pub = self.create_publisher(String, '/radiation/mark_checked', 10)
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

    def _trail(self, msg):
        # The meter publishes the trail since its last gap; keep the earlier parts (a gap = no line).
        points = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
        if self.trail and points and self.trail[-1] and math.dist(points[0], self.trail[-1][0]) < 1e-9:
            self.trail[-1] = points
        elif points:
            self.trail.append(points)

    def _measurement(self, msg):
        self.measurement = json.loads(msg.data)

    def _gate(self, msg):
        self.gate = json.loads(msg.data)

    def _heatmap(self, msg):
        self.heatmap = np.frombuffer(bytes(msg.data), '<f4').reshape(msg.height, msg.width).copy()

    def _keepout(self, msg):
        self.keepout = np.asarray(msg.data, np.int8).reshape(msg.info.height, msg.info.width) > 0

    def _hotspots(self, msg):
        self.hotspots = json.loads(msg.data)

    def _radiation(self, msg):
        self.radiation = json.loads(msg.data)

    def _changes(self, msg):
        self.changes = np.asarray(msg.data, np.int8).reshape(msg.info.height, msg.info.width)

    # ---------------------------------------------------------------- app 3 process
    def _located(self, msg):
        self.located = json.loads(msg.data)

    def display_pose(self):
        """Where to draw the robot: before a task the locator's pose from the scan (the robot may stand anywhere),
        once it is on the base station and localized, and while a task runs, AMCL's pose."""
        idle = self.state.get('state') in IDLE_STATES or not self.state
        if idle and self.located.get('pose') and (self.located.get('status') != 'ON_BASE' or not self.state.get('trusted')):
            return tuple(self.located['pose'])
        return self.pose

    def start_launch(self, map_yaml, plan_yaml, output_dir):
        self.output_dir = Path(output_dir)
        self.state, self.pose, self.covered, self.route, self.message = {}, None, None, [], ''
        self.trail, self.measurement, self.gate, self.heatmap, self.keepout = [], {}, {}, None, None
        self.hotspots, self.radiation, self.changes, self.located = [], {}, None, {}
        self.preview_asked = -math.inf
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
        """Called by the screen: ask for the preview once the robot is on the base station and localization is
        trusted (no motion). Asked again later if none came (the supervisor drops it when AMCL is reset)."""
        if (self.launch_running() and self.state.get('trusted') and self.state.get('located') == 'ON_BASE'
                and not self.state.get('preview_ready') and self.state.get('state') == 'IDLE'
                and time.monotonic() - self.preview_asked > self.preview_retry):
            self.preview_asked = time.monotonic()
            self.call('/coverage/preview')

    def can_start(self):
        s = self.state
        return bool(s.get('preview_ready') and s.get('trusted') and s.get('nav_active') and s.get('located') == 'ON_BASE'
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

    def go_home(self):
        """Stop the plan and drive back to the base station (after STOP the gate is reset first)."""
        if self.stop_active():
            self.call('/safety/reset', then=lambda: self.call('/coverage/go_home'))
        else:
            self.call('/coverage/go_home')

    def stop_active(self):
        return self.gate.get('fault') == STOP_FAULT

    def mark_checked(self, hotspot_id):
        self.checked_pub.publish(json_msg({'id': int(hotspot_id)}))

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
        'IDLE': 'Ready', 'BACKGROUND': 'Measuring the background', 'PREPARING': 'Checking before driving', 'CONNECT_PLAN': 'Planning the way to the path',
        'CONNECT': 'Driving to the path', 'SWEEP': 'Covering', 'DWELL': 'Covering',
        'PHASE_WAIT': 'Plan done, checking coverage', 'RETURN_PLAN': 'Planning the way back',
        'RETURN': 'Driving back to the base station', 'PAUSED': 'Paused', 'FINISHED': 'Finished',
        'CANCELED': 'Cancelled', 'FAILED': 'Failed'}
    s = state.get('state')
    if not s:
        return 'Starting app 3...'
    text = words.get(s, s)
    if s in IDLE_STATES and state.get('located') == 'NOT_ON_BASE':
        text = 'Robot is not on the base station. Put it on the base station.'
    elif s in IDLE_STATES and state.get('located') == 'BACKWARDS':
        text = 'Robot is on the base station the wrong way round. Turn it around.'
    elif s in IDLE_STATES and state.get('located') == 'TURNED' and state.get('turn_deg') is not None:
        turn = state['turn_deg']
        side = 'left' if turn > 0 else 'right'    # seen from above: positive = counter-clockwise
        text = f'Robot is on the base station but turned. Turn it about {abs(turn)} degrees to the {side}.'
    elif s == 'PAUSED' and state.get('auto_resume'):
        text = 'Recalculating...'
    elif s == 'IDLE' and not state.get('trusted'):
        text = 'Finding the robot on the map'
    elif s == 'IDLE' and not state.get('preview_ready'):
        text = 'Checking the plan'
    return text


def heading_arrow(pose, length):
    """(x, y, dx, dy) of an arrow for a pose."""
    x, y, yaw = pose
    return x, y, length * math.cos(yaw), length * math.sin(yaw)
