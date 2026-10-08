"""Drive session: app 3 process, automatic preview, operator commands, manual control. Loopback ROS, domain 92."""
import os
os.environ['ROS_DOMAIN_ID'] = '92'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
os.environ['CYCLONEDDS_URI'] = ('<CycloneDDS><Domain Id="any"><General><Interfaces><NetworkInterface name="lo" '
                                'multicast="false"/></Interfaces><AllowMulticast>false</AllowMulticast></General>'
                                '<Discovery><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>')
import json
import subprocess
import time
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from nala_coverage.ros_common import LATEST
from nala_panel import drive
from conftest import REPO


@pytest.fixture
def ros():
    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=4)
    nodes = []

    def add(node):
        nodes.append(node)
        executor.add_node(node)
        return node

    def pump(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.01)
    yield add, pump
    for node in nodes:
        executor.remove_node(node)
        node.destroy_node()
    executor.shutdown()
    rclpy.shutdown()


class FakeSupervisor(Node):
    """The services and state topic of app 3; records which services were called."""

    def __init__(self):
        super().__init__('fake_app3')
        self.calls = []
        for name in drive.TRIGGERS:
            self.create_service(Trigger, name, lambda req, res, n=name: self.reply(n, res))
        self.create_service(SetBool, '/control/manual', self.manual)
        self.state_pub = self.create_publisher(String, '/coverage/state', LATEST)

    def reply(self, name, res):
        self.calls.append(name)
        res.success = True
        res.message = 'ok ' + name
        return res

    def manual(self, req, res):
        self.calls.append(('manual', req.data))
        res.success = True
        res.message = 'manual ' + str(req.data)
        return res

    def publish(self, **state):
        self.state_pub.publish(String(data=json.dumps(state)))


def session_with_fake_launch(add, tmp_path):
    session = add(drive.DriveSession(REPO / 'config', hardware=False, stop_timeout=5.0))
    session.output_dir = tmp_path
    session.process = subprocess.Popen(['sleep', '60'], start_new_session=True)   # stands in for app 3
    return session


def test_launch_command_passes_map_plan_output_and_hardware():
    cmd = drive.launch_command('/m/map.yaml', '/m/plans/p.yaml', '/m/runs/r', hardware=False)
    assert cmd[:4] == ['ros2', 'launch', 'nala_bringup', 'app3_coverage_pi.launch.py']
    assert cmd[4:] == ['map:=/m/map.yaml', 'plan:=/m/plans/p.yaml', 'output_dir:=/m/runs/r', 'hardware:=false']


def test_preview_is_asked_once_when_trusted_and_start_only_after_it(ros, tmp_path):
    add, pump = ros
    fake = add(FakeSupervisor())
    session = session_with_fake_launch(add, tmp_path)
    pump(1.0)
    fake.publish(state='IDLE', trusted=False, preview_ready=False, nav_active=True)
    pump(0.5)
    session.update()
    pump(0.3)
    assert fake.calls == [] and not session.can_start()
    fake.publish(state='IDLE', trusted=True, preview_ready=False, nav_active=True)
    pump(0.5)
    session.update()
    session.update()
    pump(0.5)
    assert fake.calls == ['/coverage/preview'] and not session.can_start()
    fake.publish(state='IDLE', trusted=True, preview_ready=True, nav_active=True)
    pump(0.5)
    assert session.can_start()
    session.start()
    pump(0.5)
    assert fake.calls[-1] == '/coverage/start' and session.message == 'ok /coverage/start'
    session.stop_launch()
    assert not session.launch_running()


def test_stop_resume_and_cancel_call_the_safety_and_coverage_services(ros, tmp_path):
    add, pump = ros
    fake = add(FakeSupervisor())
    session = session_with_fake_launch(add, tmp_path)
    pump(1.0)
    session.emergency_stop()
    pump(0.5)
    assert sorted(fake.calls) == ['/coverage/pause', '/safety/estop']
    fake.calls.clear()
    session.resume()
    pump(0.8)
    assert fake.calls == ['/safety/reset', '/coverage/resume']   # the gate is reset before the task resumes
    session.cancel()
    pump(0.5)
    assert fake.calls[-1] == '/coverage/cancel'
    session.stop_launch()


def test_hand_control_asks_the_supervisor_and_sends_the_held_key(ros, tmp_path):
    from nala_coverage.keyboard_teleop import KeyboardTeleop
    add, pump = ros
    fake = add(FakeSupervisor())
    session = add(drive.DriveSession(REPO / 'config', hardware=False, stop_timeout=5.0))
    session.teleop = add(KeyboardTeleop(str(REPO / 'config')))
    pump(1.0)
    session.manual(True)
    pump(0.5)
    assert fake.calls == [('manual', True)]
    session.manual_key('w')
    session.manual_tick()
    assert session.teleop.command.linear.x == session.teleop.forward
    session.manual(False)
    pump(0.5)
    assert fake.calls[-1] == ('manual', False)


def test_report_and_plain_words(tmp_path):
    assert drive.latest_report(tmp_path) is None
    (tmp_path / 'task_a.json').write_text(json.dumps({'state': 'FINISHED', 'reason': 'RETURNED_TO_BASE'}))
    assert drive.latest_report(tmp_path)['reason'] == 'RETURNED_TO_BASE'
    assert drive.plain_state({}) == 'Starting app 3...'
    assert drive.plain_state({'state': 'IDLE', 'trusted': False}) == 'Finding the robot on the map'
    assert drive.plain_state({'state': 'RETURN'}) == 'Driving back to the base station'
