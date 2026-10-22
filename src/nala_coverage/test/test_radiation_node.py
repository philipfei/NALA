"""radiation_node on loopback ROS (domain 93): background, painting at trusted measured poses, hotspots, the
operator's threshold from the robot screen."""
import os
os.environ['ROS_DOMAIN_ID'] = '93'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
os.environ['CYCLONEDDS_URI'] = ('<CycloneDDS><Domain Id="any"><General><Interfaces><NetworkInterface name="lo" '
                                'multicast="false"/></Interfaces><AllowMulticast>false</AllowMulticast></General>'
                                '<Discovery><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>')
import json
import shutil
import time
import numpy as np
import pytest
import yaml
from PIL import Image as PILImage
rclpy = pytest.importorskip('rclpy')
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from nala_interfaces.msg import CountRate
from nala_coverage.params import default_config_dir
from nala_coverage.radiation_node import RadiationNode
from nala_coverage.ros_common import LATCHED, LATEST, json_msg


@pytest.fixture
def ros(tmp_path):
    a = np.full((80, 80), 254, np.uint8)
    a[[0, -1], :] = 0
    a[:, [0, -1]] = 0
    PILImage.fromarray(a).save(tmp_path / 'map.pgm')
    (tmp_path / 'map.yaml').write_text('image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\nnegate: 0\n'
                                       'occupied_thresh: 0.65\nfree_thresh: 0.196\n')
    config = tmp_path / 'config'
    shutil.copytree(default_config_dir(), config)
    raw = yaml.safe_load((config / 'coverage.yaml').read_text())
    raw['/coverage_config']['ros__parameters']['radiation'].update(background_readings=3, publish_period_s=0.2)
    (config / 'coverage.yaml').write_text(yaml.safe_dump(raw))
    rclpy.init(args=['--ros-args', '-p', f'config_dir:={config}', '-p', f'map_yaml:={tmp_path / "map.yaml"}',
                     '-p', f'output_dir:={tmp_path / "run"}'])
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
    yield add, pump, tmp_path
    for node in nodes:
        executor.remove_node(node)
        node.destroy_node()
    executor.shutdown()
    rclpy.shutdown()


class Feeder(Node):
    """Plays the supervisor (measured poses) and the scintillator, and listens to the outputs."""

    def __init__(self):
        super().__init__('radiation_test_feeder')
        self.samples = self.create_publisher(String, '/coverage/sample', LATEST)
        self.readings = self.create_publisher(CountRate, '/scintillator/count_rate', 10)
        self.checked = self.create_publisher(String, '/radiation/mark_checked', 10)
        self.threshold = self.create_publisher(String, '/radiation/threshold', LATCHED)
        self.background = self.create_client(Trigger, '/radiation/measure_background')
        self.out = {}
        self.create_subscription(Image, '/radiation/heatmap', lambda m: self.out.__setitem__('heatmap', m), LATCHED)
        self.create_subscription(String, '/radiation/hotspots', lambda m: self.out.__setitem__('hotspots', json.loads(m.data)), LATCHED)
        self.create_subscription(String, '/radiation/state', lambda m: self.out.__setitem__('state', json.loads(m.data)), LATEST)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def second(self, pump, x, y, cps, trusted=True):
        """One second at (x, y) heading +x: poses at 10 Hz, then the reading of that second."""
        start = self.now()
        for i in range(11):
            self.samples.publish(json_msg({'generation': 'g', 'stamp': start + i * 0.1, 'pose': [x, y, 0.0],
                                           'trusted': trusted, 'active': True}))
            pump(0.1)
        msg = CountRate()
        msg.header.stamp = rclpy.time.Time(seconds=start + 1.0).to_msg()
        msg.counts_per_second, msg.interval_s = cps, 1.0
        self.readings.publish(msg)
        self.samples.publish(json_msg({'generation': 'g', 'stamp': start + 1.1, 'pose': [x, y, 0.0],
                                       'trusted': trusted, 'active': True}))
        pump(0.3)


def test_radiation_node_background_heatmap_hotspot_and_threshold(ros):
    add, pump, tmp_path = ros
    node = add(RadiationNode())
    feed = add(Feeder())
    pump(1.0)
    future = feed.background.call_async(Trigger.Request())
    pump(0.5)
    assert future.result().success
    for _ in range(3):
        feed.second(pump, 1.0, 2.0, 10.0)                                  # background on the base station
    pump(0.5)
    assert feed.out['state']['background_ready'] and feed.out['state']['threshold'] == pytest.approx(20.0)
    feed.second(pump, 2.5, 2.0, 50.0, trusted=False)                       # not localized: logged, not drawn
    feed.second(pump, 2.0, 2.0, 50.0)                                      # contaminated, sensor at x 2.26
    pump(0.6)
    value = np.frombuffer(bytes(feed.out['heatmap'].data), '<f4').reshape(80, 80)
    assert value[40, 45] == pytest.approx(50.0)                            # cell of (2.26, 2.0)
    assert np.isnan(value[40, 55])                                         # cell of (2.76, 2.0): untrusted
    [h] = feed.out['hotspots']
    assert h['id'] == 1 and h['peak'] == 50.0 and not h['checked']
    feed.checked.publish(json_msg({'id': 1}))
    pump(0.6)
    assert feed.out['hotspots'][0]['checked'] and feed.out['state']['to_check'] == 0
    log = (tmp_path / 'run' / 'readings.csv').read_text().splitlines()
    assert len(log) == 1 + 5 and log[4].endswith(',,0') and log[5].endswith(',1')
    assert json.loads((tmp_path / 'run' / 'hotspots.json').read_text())['hotspots'][0]['checked']
    # The operator's threshold from the robot screen: 60 counts/s, so the 50 counts/s spot is no hotspot any more.
    feed.threshold.publish(json_msg({'mode': 'cps', 'value': 60.0}))
    pump(0.6)
    assert feed.out['state']['threshold'] == 60.0 and feed.out['hotspots'] == []
    feed.threshold.publish(json_msg({'mode': 'background', 'value': 0.5}))   # not usable: ignored
    pump(0.6)
    assert feed.out['state']['setting'] == {'mode': 'cps', 'value': 60.0}
