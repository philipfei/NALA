"""Radiation map node: scintillator readings + measured poses -> heatmap, hotspots.

In:  /scintillator/count_rate (nala_interfaces/CountRate), /coverage/sample (the supervisor's measured poses),
     /radiation/mark_checked (JSON {"id": n}), /radiation/threshold (JSON {"mode", "value"} from the robot screen),
     service /radiation/measure_background (start of a task).
Out: /radiation/heatmap (sensor_msgs/Image 32FC1 on the map grid, NaN = no reading), /radiation/hotspots (JSON),
     /radiation/state (JSON). The supervisor stops the task when contamination is found (it reads the readings itself).
Run folder (output_dir): readings.csv (every reading), hotspots.json, heatmap.npz.
"""
import csv
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from nala_interfaces.msg import CountRate
from .geometry import Grid
from .params import node_settings
from .radiation import RadiationMap, valid_setting
from .ros_common import run_until_stopped, LATCHED, LATEST, decode, json_msg, stamp_seconds


class RadiationNode(Node):
    def __init__(self):
        super().__init__('radiation_node')
        self.settings = node_settings(self)
        self.cfg = self.settings['radiation']
        self.grid = Grid.load(self.settings['runtime']['map_yaml'])
        self.map = RadiationMap(self.grid, self.settings['geometry']['sensor_offset_m'], self.cfg)
        self.output = Path(self.settings['runtime']['output_dir'])
        self.poses = []           # (stamp, x, y, yaw, trusted) from /coverage/sample
        self.pending = []         # readings waiting for the poses of their interval
        self.last_reading = None  # monotonic time of the last reading
        self.heatmap_pub = self.create_publisher(Image, '/radiation/heatmap', LATCHED)
        self.hotspots_pub = self.create_publisher(String, '/radiation/hotspots', LATCHED)
        self.state_pub = self.create_publisher(String, '/radiation/state', LATEST)
        self.create_subscription(CountRate, '/scintillator/count_rate', self.reading, 10)
        self.create_subscription(String, '/coverage/sample', self.sample, LATEST)
        self.create_subscription(String, '/radiation/mark_checked', self.mark_checked, 10)
        self.create_subscription(String, '/radiation/threshold', self.threshold_received, LATCHED)
        self.create_service(Trigger, '/radiation/measure_background', self.measure_background)
        self.create_timer(self.cfg['publish_period_s'], self.publish)

    # ---------------------------------------------------------------- inputs
    def sample(self, msg):
        try:
            d = decode(msg)
            x, y, yaw = (float(v) for v in d['pose'])
            self.poses.append((float(d['stamp']), x, y, yaw, bool(d['trusted'])))
        except (ValueError, KeyError, TypeError):
            return
        keep = self.poses[-1][0] - self.cfg['pose_history_s']
        while self.poses and self.poses[0][0] < keep:
            self.poses.pop(0)
        self.match()

    def reading(self, msg):
        self.last_reading = time.monotonic()
        end = stamp_seconds(msg.header.stamp)
        self.pending.append((end - msg.interval_s, end, float(msg.counts_per_second)))
        if self.map.measuring and self.map.add_background(msg.counts_per_second):
            self.get_logger().info(f'Background {self.map.background:.1f} counts/s, '
                                   f'threshold {self.map.threshold:.1f} counts/s')
        self.match()

    def match(self):
        """Paint the readings whose interval is over (a pose after it has arrived, or it is too old)."""
        latest = self.poses[-1][0] if self.poses else None
        waiting = []
        for start, end, cps in self.pending:
            if latest is not None and latest < end and end - latest < self.cfg['pose_history_s']:
                waiting.append((start, end, cps))
                continue
            points = self.map.sensor_points(self.poses, start, end, self.cfg['max_gap_s'])
            if points:
                self.map.add_reading(cps, points)
            self.log(end, cps, points)
        self.pending = waiting

    def log(self, end, cps, points):
        """Every reading goes to readings.csv; x, y is the mean sensor point, empty if it was not drawn."""
        self.output.mkdir(parents=True, exist_ok=True)
        path = self.output / 'readings.csv'
        new = not path.exists()
        with path.open('a', newline='') as f:
            w = csv.writer(f)
            if new:
                w.writerow(['stamp', 'counts_per_second', 'x', 'y', 'drawn'])
            xy = np.mean(points, axis=0).round(3).tolist() if points else ['', '']
            w.writerow([f'{end:.3f}', f'{cps:.2f}', *xy, int(bool(points))])

    def mark_checked(self, msg):
        try:
            hid = int(decode(msg)['id'])
        except (ValueError, KeyError, TypeError):
            return
        if self.map.mark_checked(hid):
            self.publish()

    def threshold_received(self, msg):
        """The operator's threshold (robot screen, Settings): which cells the map shows as contaminated."""
        try:
            setting = decode(msg)
        except ValueError:
            return
        if valid_setting(setting):
            self.map.setting = {'mode': setting['mode'], 'value': float(setting['value'])}
            self.publish()

    def measure_background(self, req, res):
        """Start of a task: a new map, then the background from the next readings (robot on the base station)."""
        self.map.start_background()
        self.pending = []
        res.success = True
        res.message = f"Measuring the background ({self.cfg['background_readings']} readings)"
        return res

    # ---------------------------------------------------------------- outputs
    def publish(self):
        hotspots = self.map.update_hotspots(time.time())
        img = Image()
        img.header.frame_id = self.settings['runtime']['map_frame']
        img.header.stamp = self.get_clock().now().to_msg()
        img.height, img.width = self.map.value.shape
        img.encoding = '32FC1'
        img.step = img.width * 4
        img.data = self.map.value.astype('<f4').tobytes()
        self.heatmap_pub.publish(img)
        self.hotspots_pub.publish(json_msg(hotspots))
        age = None if self.last_reading is None else time.monotonic() - self.last_reading
        state = {**self.map.state(), 'last_reading_age_s': age}
        self.state_pub.publish(json_msg(state))
        self.output.mkdir(parents=True, exist_ok=True)
        (self.output / 'hotspots.json').write_text(json.dumps({**state, 'hotspots': hotspots}, indent=2) + '\n')
        np.savez_compressed(self.output / 'heatmap.npz', value=self.map.value,
                            background=np.nan if self.map.background is None else self.map.background,
                            threshold=np.nan if self.map.threshold is None else self.map.threshold)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = RadiationNode()
    try:
        run_until_stopped(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
