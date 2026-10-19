"""Apps 1 (drive with the Xbox controller) and 2 (make a map) from the robot screen (user, 2026-10-19).

The screen starts `ros2 launch nala_bringup app1_teleop_pi.launch.py` or `app2_slam_pi.launch.py` like app 3
(drive.py), stops it with Ctrl-C, and for app 2 shows the map as it grows and saves it with scripts/save_map.sh.
Only one app runs at a time: they share the motor driver's UART and the LiDAR.
"""
import os
from pathlib import Path
import re
import signal
import subprocess
import time
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from nala_coverage.geometry import Grid
from nala_coverage.ros_common import LATCHED, transform3

LAUNCH_FILES = {'app1': 'app1_teleop_pi.launch.py', 'app2': 'app2_slam_pi.launch.py'}
# Nodes of an app that is already running (started over ssh, or app 3): starting another one is refused.
APP_NODES = ('base_node', 'slam_toolbox', 'coverage_supervisor', 'velocity_safety_gate')
NAME = re.compile(r'^[A-Za-z0-9_-]{1,40}$')


def launch_command(app):
    return ['ros2', 'launch', 'nala_bringup', LAUNCH_FILES[app]]


def name_problem(name, maps_dir):
    """Why a new map name cannot be used, or '' when it can (it becomes a folder in maps/)."""
    if not NAME.match(name or ''):
        return 'Use 1-40 letters, digits, - or _'
    if (Path(maps_dir) / name).exists():
        return f'A map called {name} exists already'
    return ''


class AppSession(Node):
    """App 1 or 2 for the robot screen. Data from callbacks is read by the screen with plain attribute reads."""

    def __init__(self, repo_dir, stop_timeout):
        super().__init__('nala_panel_apps')
        self.repo_dir, self.stop_timeout = Path(repo_dir), stop_timeout
        self.app = None
        self.process = None
        self.saving = None           # save_map.sh while it runs
        self.save_result = ''        # its last line
        self.grid = None             # app 2: the map so far (nala_coverage Grid)
        self.odom_time = -1e9
        self.tf = Buffer(cache_time=Duration(seconds=5.))
        # Own small TF node and thread, like the coverage nodes: /tf does not wake the panel's nodes.
        self.listener = TransformListener(self.tf, None, spin_thread=True)
        self.create_subscription(Odometry, '/odom', self._odom, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/map', self._map, LATCHED)

    def _odom(self, _msg):
        self.odom_time = time.monotonic()

    def _map(self, msg):
        if not self.running():
            return
        cells = np.asarray(msg.data, np.int8).reshape(msg.info.height, msg.info.width)
        o = msg.info.origin
        yaw = 2 * np.arctan2(o.orientation.z, o.orientation.w)
        self.grid = Grid.from_cells(cells, msg.info.resolution, (o.position.x, o.position.y, float(yaw)))

    # ---------------------------------------------------------------- start / stop
    def busy(self):
        """Nodes of another app that already runs, or [] (the screen refuses to start then)."""
        names = {n for n, _ in self.get_node_names_and_namespaces()}
        return [n for n in APP_NODES if n in names]

    def start(self, app):
        self.app, self.grid, self.save_result, self.odom_time = app, None, '', -1e9
        logs = self.repo_dir / 'log'
        logs.mkdir(exist_ok=True)
        log = open(logs / f'{app}_{time.strftime("%Y-%m-%d_%H%M%S")}.log', 'w')
        self.process = subprocess.Popen(launch_command(app), stdout=log, stderr=subprocess.STDOUT,
                                        start_new_session=True)

    def running(self):
        return self.process is not None and self.process.poll() is None

    def stop(self):
        """Ctrl-C for the app: base_node sends zero speed when it exits (the MCU also stops after 0.2 s without
        frames); killed after stop_timeout."""
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

    def close(self):
        self.stop()
        self.listener.executor.shutdown()
        self.listener.dedicated_listener_thread.join()

    # ---------------------------------------------------------------- status
    def driver_ok(self):
        """The motor driver answers: base_node publishes /odom from the MCU's state frames."""
        return time.monotonic() - self.odom_time < 1.0

    def robot_pose(self):
        """App 2: the robot on the map (SLAM's map -> base_footprint), or None."""
        try:
            return transform3(self.tf.lookup_transform('map', 'base_footprint', Time()))
        except TransformException:
            return None

    # ---------------------------------------------------------------- app 2: save the map
    def save_map(self, name):
        self.save_result = ''
        self.saving = subprocess.Popen(['bash', str(self.repo_dir / 'scripts' / 'save_map.sh'), name],
                                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                       start_new_session=True)

    def poll_save(self):
        """None while saving; then (ok, last line of save_map.sh)."""
        if self.saving is None or self.saving.poll() is None:
            return None
        out = (self.saving.stdout.read() or '').strip().splitlines()
        ok = self.saving.returncode == 0
        self.saving = None
        self.save_result = next((line for line in reversed(out) if line.startswith(('Saved', 'Saving failed'))),
                                out[-1] if out else '')
        return ok, self.save_result
