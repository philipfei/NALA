"""Run app3_coverage_pi.launch.py hardware:=false (real Nav2 + NALA nodes) against fake_nala.py and record the run.

The fake robot boots at the base station, where AMCL starts (the supervisor sends it; no initial pose by hand). Acts like an
operator: waits until localization is trusted and converged, calls /coverage/preview and /coverage/start,
resumes after pauses listed in --resume-on, and stops when the task finishes (back at the base station),
fails, or stays paused. Everything is written to tools/sim/runs/<name>/.
Source tools/sim/env.sh first.
"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
import rclpy
import yaml
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from nala_coverage.ros_common import LATEST

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
EXAMPLES = REPO / 'coverage_tool/examples'
# Results go here; set SIM_RUNS_DIR to a folder outside an editor's workspace if its file watcher slows the PC.
RUNS = Path(os.environ.get('SIM_RUNS_DIR', HERE / 'runs'))
T0 = time.monotonic()


def since():
    return round(time.monotonic() - T0, 2)


class Monitor(Node):
    def __init__(self):
        super().__init__('sim_monitor')
        self.state = {}
        self.timeline = []
        self.amcl = []
        self.map_odom = []  # [arrival s, stamp - arrival s] for every map->odom transform from AMCL
        self.trust_reasons = []
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, 10)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100)
        self.create_subscription(String, '/coverage/state', self.on_state, LATEST)
        self.gate = []   # [time, fault, reason] whenever the velocity gate's fault or reason changes
        self.create_subscription(String, '/safety/state', self.on_gate, LATEST)
        # Not self.services/self.clients: those are read-only Node properties.
        self.triggers = {n: self.create_client(Trigger, '/coverage/' + n) for n in ('preview', 'start', 'resume')}

    def on_amcl(self, msg):
        c = msg.pose.covariance
        p = msg.pose.pose
        self.amcl.append([since(), p.position.x, p.position.y,
                          math.atan2(2 * p.orientation.w * p.orientation.z, 1 - 2 * p.orientation.z ** 2),
                          math.sqrt(max(c[0], c[7], 0.)), math.degrees(math.sqrt(max(c[35], 0.)))])

    def on_tf(self, msg):
        for t in msg.transforms:
            if t.header.frame_id == 'map' and t.child_frame_id == 'odom':
                now = self.get_clock().now().nanoseconds * 1e-9
                self.map_odom.append([since(), round(t.header.stamp.sec + t.header.stamp.nanosec * 1e-9 - now, 3)])

    def on_state(self, msg):
        d = json.loads(msg.data)
        if d.get('trust_reason') and (not self.trust_reasons or self.trust_reasons[-1][1] != d['trust_reason']):
            self.trust_reasons.append([since(), d['trust_reason']])
        if not self.timeline or self.timeline[-1][1:3] != [d['state'], d['reason']]:
            self.timeline.append([round(since(), 1), d['state'], d['reason'], round(d['fraction'], 4)])
            print(f"[{self.timeline[-1][0]:7.1f}s] {d['state']:12s} {d['reason']}  fraction={d['fraction']:.3f}", flush=True)
        self.state = d

    def on_gate(self, msg):
        d = json.loads(msg.data)
        key = [d.get('fault', ''), d.get('reason', '')]
        if not self.gate or self.gate[-1][1:] != key:
            self.gate.append([since()] + key)
            if key[0]:
                print(f'[{since():7.1f}s] gate fault: {key[0]} ({key[1]})', flush=True)

    def call(self, name):
        c = self.triggers[name]
        if not c.wait_for_service(timeout_sec=2.):
            return False, 'service unavailable'
        f = c.call_async(Trigger.Request())
        end = time.monotonic() + 10
        while not f.done() and time.monotonic() < end:
            time.sleep(.05)
        return (f.result().success, f.result().message) if f.done() else (False, 'timeout')


def wait(cond, timeout, step=.2):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(step)
    return False


def other_runs_in_domain():
    """PIDs of coverage launches or fake robots already running in this ROS_DOMAIN_ID."""
    domain = os.environ.get('ROS_DOMAIN_ID', '0')
    # This process and the shells that started it are not another simulation.
    own, pid = set(), os.getpid()
    while pid > 1:
        own.add(pid)
        try:
            pid = int(Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()[1])
        except (OSError, ValueError, IndexError):
            break
    found = []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit() or int(proc.name) in own:
            continue
        try:
            cmd = (proc / 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
            env = (proc / 'environ').read_bytes().split(b'\0')
        except OSError:
            continue
        if ('app3_coverage_pi.launch.py' in cmd or 'fake_nala.py' in cmd or 'run_sim.py' in cmd) and \
                f'ROS_DOMAIN_ID={domain}'.encode() in env:
            found.append(int(proc.name))
    return found


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('name', help='run name; results go to tools/sim/runs/<name>/ (or $SIM_RUNS_DIR/<name>/)')
    ap.add_argument('--base-station', help='base station file (default: <map>_base_station.yaml in examples, '
                                           'else base_station.yaml next to the map); the fake robot boots there')
    ap.add_argument('--start-offset', type=float, nargs=3, default=[0., 0., 0.], metavar=('DX', 'DY', 'DYAW'),
                    help='true start pose minus the base station (map frame), to test a robot not placed exactly')
    ap.add_argument('--obstacle', action='append', default=[], metavar='X,Y,R',
                    help='round obstacle that is in the world but not in the map; repeatable')
    ap.add_argument('--plan', default=str(EXAMPLES / 'map_ME_room1v4_nala.yaml'))
    ap.add_argument('--map', default=str(EXAMPLES / 'map_ME_room1v4.yaml'))
    ap.add_argument('--config', default=str(REPO / 'config'),
                    help='config directory passed to app3_coverage_pi.launch.py (copy and edit it to experiment)')
    ap.add_argument('--timeout', type=float, default=1500., help='max mission time in seconds')
    ap.add_argument('--settle-sigma', type=float, default=.03,
                    help='wait until the AMCL position sigma is below this before starting (m)')
    ap.add_argument('--resume-on', nargs='*', default=['TF_UNAVAILABLE', 'AMCL_STALE', 'SCAN_STALE', 'SAFETY_GATE_STALE'],
                    help='pause reasons after which to call /coverage/resume')
    ap.add_argument('--pause-timeout', type=float, default=15.,
                    help='stop when the task stays paused this long (s), e.g. because resume keeps being refused')
    ap.add_argument('--max-resumes', type=int, default=5,
                    help='how often to resume at most (raise it on a busy PC, where timing pauses are more frequent)')
    a = ap.parse_args()
    if 'install' not in os.environ.get('AMENT_PREFIX_PATH', '') or 'ROS_DOMAIN_ID' not in os.environ:
        sys.exit('Source tools/sim/env.sh first (after colcon build).')
    busy = other_runs_in_domain()
    if busy:
        sys.exit(f'Another simulation is running in ROS_DOMAIN_ID={os.environ["ROS_DOMAIN_ID"]} (PIDs {busy}). '
                 'Stop it or use another domain.')
    if os.environ.get('RMW_IMPLEMENTATION') == 'rmw_fastrtps_cpp':
        # Segments left by killed processes break discovery of the next run.
        subprocess.run(['fastdds', 'shm', 'clean'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    station_file = Path(a.base_station) if a.base_station else Path(a.map).with_name(Path(a.map).stem + '_base_station.yaml')
    if not station_file.is_file():
        station_file = Path(a.map).resolve().parent / 'base_station.yaml'
    station = yaml.safe_load(station_file.read_text())
    start = [station['x'] + a.start_offset[0], station['y'] + a.start_offset[1], station['yaw'] + a.start_offset[2]]
    out = RUNS / a.name
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    mon = Monitor()
    threading.Thread(target=rclpy.spin, args=(mon,), daemon=True).start()
    result = {'name': a.name, 'start': start, 'base_station': str(station_file), 'obstacles': a.obstacle, 'plan': a.plan, 'map': a.map,
              'config': a.config, 'resumes': []}
    launch = robot = None
    try:
        launch = subprocess.Popen(
            ['ros2', 'launch', 'nala_bringup', 'app3_coverage_pi.launch.py', f'config_dir:={a.config}',
             f'map:={a.map}', f'plan:={a.plan}', f'base_station:={station_file}', f'output_dir:={out}/output',
             'hardware:=false'],
            stdout=open(out / 'launch.log', 'w'), stderr=subprocess.STDOUT, start_new_session=True)
        robot = subprocess.Popen(
            [sys.executable, str(HERE / 'fake_nala.py'), '--map', a.map, '--start', *map(str, start),
             '--config', str(Path(a.config) / 'robot.yaml'),
             '--log', str(out / 'truth.csv')] + [x for o in a.obstacle for x in ('--obstacle', o)],
            stdout=open(out / 'robot.log', 'w'), stderr=subprocess.STDOUT, start_new_session=True)
        result['robot_started_s'] = since()
        # AMCL starts at the base station: the supervisor sends base_station.yaml as the initial pose.
        if not wait(lambda: mon.amcl, 90):
            raise RuntimeError('AMCL never published a pose')
        if not wait(lambda: mon.state.get('trusted'), 60):
            raise RuntimeError('never trusted: ' + str(mon.state.get('trust_reason')))
        # A careful operator waits until AMCL has converged before starting.
        wait(lambda: mon.amcl and mon.amcl[-1][4] < a.settle_sigma, 120, .5)
        result['sigma_at_start'] = round(mon.amcl[-1][4], 4)
        print('AMCL sigma at start', result['sigma_at_start'], flush=True)
        print('preview:', *mon.call('preview'), flush=True)
        if not wait(lambda: mon.state.get('preview_ready') or 'PREVIEW_FAILED' in mon.state.get('reason', ''), 300):
            raise RuntimeError('preview timeout')
        if not mon.state.get('preview_ready'):
            raise RuntimeError(mon.state.get('reason'))
        started, message = False, ''
        end = time.monotonic() + 180   # Nav2 may need a while to activate on a large map (NAV2_NOT_ACTIVE)
        while time.monotonic() < end:
            started, message = mon.call('start')
            print('start:', started, message, flush=True)
            if started:
                break
            time.sleep(1.)
        if not started:
            raise RuntimeError('start refused: ' + message)
        t_start = time.monotonic()
        paused_since = [None]

        def done():
            s = mon.state.get('state')
            paused_since[0] = (paused_since[0] or time.monotonic()) if s == 'PAUSED' else None
            if paused_since[0] and time.monotonic() - paused_since[0] > 3 and \
                    mon.state.get('reason') in a.resume_on and len(result['resumes']) < a.max_resumes:
                reason = mon.state.get('reason')
                ok, msg = mon.call('resume')
                if not ok and msg != result.get('last_refusal'):
                    print(f'[{since():7.1f}s] resume refused: {msg}', flush=True)
                    result['last_refusal'] = msg
                if ok:
                    print('resume:', msg, flush=True)
                    result['resumes'].append([round(since(), 1), reason])
                    paused_since[0] = None
            return s in ('FINISHED', 'FAILED', 'CANCELED') or (paused_since[0] and time.monotonic() - paused_since[0] > a.pause_timeout)
        wait(done, a.timeout, .5)
        result['mission_s'] = round(time.monotonic() - t_start, 1)
    except Exception as e:
        result['error'] = str(e)
        print('ERROR', e, flush=True)
    finally:
        result.update(final=mon.state, timeline=mon.timeline)
        json.dump(mon.amcl, open(out / 'amcl.json', 'w'))
        json.dump({'map_odom': mon.map_odom, 'trust': mon.trust_reasons, 'gate': mon.gate}, open(out / 'tf.json', 'w'))
        procs = [p for p in (launch, robot) if p]
        for p in procs:
            try:
                os.killpg(p.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        for p in procs:
            try:
                p.wait(20)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
        reports = sorted((out / 'output').glob('task_*.json'), key=os.path.getmtime)
        if reports:
            result['report'] = reports[-1].name
        json.dump(result, open(out / 'result.json', 'w'), indent=2)
        print(json.dumps({k: result.get(k) for k in ('error', 'mission_s')} |
                         {'state': mon.state.get('state'), 'reason': mon.state.get('reason')}), flush=True)
        # A hung spin thread must never leave a stale monitor calling services in the DDS domain.
        os._exit(0)


if __name__ == '__main__':
    main()
