"""xbox_pause node: B button -> /coverage/pause, with a fake supervisor service. Loopback ROS, domain 93."""
import os
os.environ['ROS_DOMAIN_ID'] = '93'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
os.environ['CYCLONEDDS_URI'] = ('<CycloneDDS><Domain Id="any"><General><Interfaces><NetworkInterface name="lo" '
                                'multicast="false"/></Interfaces><AllowMulticast>false</AllowMulticast></General>'
                                '<Discovery><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>')
import threading
import time
import pytest
pytest.importorskip('rclpy', reason='ROS 2 Python runtime is not installed')
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger
from nala_coverage.xbox_pause_node import XboxPause

B = 1


def wait_for(condition, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if condition():
            return True
        time.sleep(0.02)
    return condition()


class FakeSupervisor:
    """A /coverage/pause service that counts calls; success and reply delay can be set."""
    def __init__(self, success=True, delay=0.0):
        self.node = rclpy.create_node('fake_supervisor')
        self.calls = 0
        self.success = success
        self.delay = delay
        self.node.create_service(Trigger, '/coverage/pause', self.pause, callback_group=ReentrantCallbackGroup())

    def pause(self, _request, response):
        self.calls += 1
        time.sleep(self.delay)
        response.success = self.success
        response.message = 'Paused' if self.success else 'Task is not running'
        return response


@pytest.fixture
def ros():
    rclpy.init(args=['--ros-args', '-p', f'pause_button:={B}'])
    executor = MultiThreadedExecutor(num_threads=4)
    nodes = []

    def add(node):
        nodes.append(node)
        executor.add_node(node)
        return node

    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    yield add, thread
    executor.shutdown()
    for node in nodes:
        node.destroy_node()
    rclpy.shutdown()
    thread.join(timeout=2.0)


def setup_nodes(add, supervisor=None):
    pause = add(XboxPause())
    joy = add(rclpy.create_node('fake_joy'))
    pub = joy.create_publisher(Joy, 'joy', 10)
    if supervisor is not None:
        add(supervisor.node)
        assert wait_for(pause.client.service_is_ready)
    assert wait_for(lambda: pub.get_subscription_count() == 1)

    def send(*buttons):
        msg = Joy()
        msg.buttons = list(buttons)
        msg.axes = [0.0] * 8
        pub.publish(msg)
        time.sleep(0.1)
    return pause, send


def test_one_pause_call_per_press_none_while_held(ros):
    add, _ = ros
    supervisor = FakeSupervisor()
    _, send = setup_nodes(add, supervisor)
    for buttons in ([0, 0, 0], [0, 1, 0], [0, 1, 0], [0, 1, 0], [0, 0, 0], [0, 1, 0]):
        send(*buttons)
    assert wait_for(lambda: supervisor.calls == 2)
    time.sleep(0.3)
    assert supervisor.calls == 2


def test_other_buttons_and_short_messages_do_not_pause(ros):
    add, _ = ros
    supervisor = FakeSupervisor()
    _, send = setup_nodes(add, supervisor)
    send(1, 0, 1, 1, 1)     # A, X, Y, LB held, B not
    send(1)                 # too short: B missing
    send()
    time.sleep(0.3)
    assert supervisor.calls == 0


def test_no_supervisor_keeps_running(ros):
    add, thread = ros
    pause, send = setup_nodes(add)
    send(0, 1)
    send(0, 0)
    send(0, 1)
    assert pause.pending is None          # nothing sent: the service is not there
    assert thread.is_alive()
    # The supervisor comes up later: the next press is sent.
    supervisor = FakeSupervisor()
    add(supervisor.node)
    assert wait_for(pause.client.service_is_ready)
    send(0, 0)
    send(0, 1)
    assert wait_for(lambda: supervisor.calls == 1)


def test_refused_pause_keeps_running(ros):
    add, thread = ros
    supervisor = FakeSupervisor(success=False)
    pause, send = setup_nodes(add, supervisor)
    send(0, 1)
    assert wait_for(lambda: pause.pending is not None and pause.pending.done())
    assert pause.pending.result().success is False
    send(0, 0)
    send(0, 1)
    assert wait_for(lambda: supervisor.calls == 2)
    assert thread.is_alive()


def test_press_while_a_request_is_open_is_not_sent_again(ros):
    add, _ = ros
    supervisor = FakeSupervisor(delay=1.0)
    pause, send = setup_nodes(add, supervisor)
    send(0, 1)
    send(0, 0)
    send(0, 1)              # the first reply is still open
    assert wait_for(lambda: pause.pending.done(), timeout=3.0)
    assert supervisor.calls == 1
