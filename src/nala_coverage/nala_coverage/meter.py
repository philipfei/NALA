"""Independent node for actual trusted map-frame pose coverage, painted at the sensor ahead of the robot."""
import json
import math
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid, Path as PathMsg
from .geometry import Grid
from .params import node_settings
from .measurement import Meter
from .ros_common import run_until_stopped,LATCHED,LATEST,parameter,decode,json_msg,grid_msg,path_msg,pose_msg,ros_now


class CoverageMeter(Node):
    def __init__(self):
        super().__init__('coverage_meter')
        self.settings=node_settings(self)
        self.base=Grid.load(self.settings['runtime']['map_yaml'])
        self.radius=self.settings['geometry']['coverage_disk_radius_m']
        self.linear=self.settings['motion']['linear_m_s'];self.angular=self.settings['motion']['angular_rad_s']
        self.output=Path(self.settings['runtime']['output_dir']);self.sample_seq=0;self.last_active_sample_seq=0
        self.config_pub=self.create_publisher(String,'/coverage/meter_config',LATCHED)
        self.config_pub.publish(json_msg({'config_hash':self.settings.hash}))
        self.meter=None;self.generation='';self.fault='';self.last_sample=0.
        self.cache={}          # trace message parts (trace_points, trace_path)
        self.status=self.create_publisher(String,'/coverage/measurement',LATEST)
        self.covered=self.create_publisher(OccupancyGrid,'/coverage/covered',LATCHED)
        self.remaining=self.create_publisher(OccupancyGrid,'/coverage/remaining',LATCHED)
        self.trajectory=self.create_publisher(PathMsg,'/coverage/trajectory',LATCHED)
        self.trace_segments=self.create_publisher(Marker,'/coverage/trajectory_segments',LATCHED)
        self.create_subscription(String,'/coverage/definition',self.definition,LATCHED)
        self.create_subscription(String,'/coverage/sample',self.sample,LATEST)
        self.create_subscription(String,'/coverage/save_report',self.save,LATEST)
        # Contaminated floor the robot avoids (radiation_node): not counted as missing.
        self.keepout=None
        self.create_subscription(OccupancyGrid,'/radiation/keepout',self.keepout_received,LATCHED)
        self.create_timer(self.settings['measurement']['publish_period_s'],self.publish)

    def definition(self,msg):
        try:
            d=decode(msg)
            if d['generation']==self.generation:return
            if d.get('config_hash')!=self.settings.hash:raise ValueError('Config identity mismatch')
            if d['map_id']!=self.base.identity:raise ValueError('Map identity mismatch')
            mask=np.zeros(self.base.cells.size,bool);mask[d['denominator']]=True
            grid=self.base;cells=grid.cells
            if not mask.any() or np.any(mask.reshape(cells.shape)&~grid.free):raise ValueError('Invalid denominator')
            self.meter=Meter(grid,mask.reshape(cells.shape),self.radius,max_gap=self.settings['measurement']['max_sample_gap_s'],linear=self.linear,angular=self.angular,translation_slack=self.settings['measurement']['translation_slack_m'],rotation_slack=self.settings['measurement']['rotation_slack_deg'],offset=self.settings['geometry']['sensor_offset_m'])
            self.generation=d['generation'];self.fault='';self.sample_seq=0;self.last_active_sample_seq=0;self.last_sample=0.
        except (ValueError,KeyError,IndexError,TypeError) as e:self.fault=str(e)

    def keepout_received(self,msg):
        if (msg.info.height,msg.info.width)==self.base.cells.shape:
            self.keepout=np.asarray(msg.data,np.int8).reshape(msg.info.height,msg.info.width)>0

    def sample(self,msg):
        if self.meter is None:return
        try:
            d=decode(msg)
            if d['generation']!=self.generation:return
            age=ros_now(self)-d['stamp']
            trusted=d['trusted'] and 0<=age<=self.settings['measurement']['max_sample_gap_s']
            self.meter.sample(d['pose'],d['stamp'],trusted,d['active'])
            self.last_sample=time.monotonic();self.sample_seq=d.get('sample_seq',0)
            if trusted and d['active'] and self.meter.previous is not None:
                self.last_active_sample_seq=self.sample_seq
        except (ValueError,KeyError,TypeError) as e:self.fault=str(e);self.meter.previous=None

    def publish(self):
        if self.meter is None:return
        if time.monotonic()-self.last_sample>self.settings['measurement']['max_sample_gap_s']:self.meter.previous=None
        data={**self.meter.report(self.keepout),'config_hash':self.settings.hash,'sample_seq':self.sample_seq,'last_active_sample_seq':self.last_active_sample_seq,'generation':self.generation,'map_id':self.base.identity,
              'known_free_m2':float(self.base.free.sum()*self.base.resolution**2),
              'fault':self.fault}
        # (The covered cells are /coverage/covered; this message used to carry them as a list too, which every
        # subscriber decoded twice per second. The saved report keeps the list.)
        self.status.publish(json_msg(data))
        self.covered.publish(grid_msg(self,self.meter.grid,self.meter.covered))
        self.remaining.publish(grid_msg(self,self.meter.grid,self.meter.denominator&~self.meter.covered))
        # Never draw a line through a sampling gap in the trajectory display.
        # A LINE_LIST retains all verified edges without joining separate trace segments.
        marker=Marker();marker.header.frame_id=self.settings['runtime']['map_frame'];marker.header.stamp=self.get_clock().now().to_msg()
        marker.ns='actual_trace';marker.id=0;marker.type=Marker.LINE_LIST;marker.action=Marker.ADD
        marker.pose.orientation.w=1.;marker.scale.x=.025
        marker.color.r=1.;marker.color.g=.35;marker.color.a=1.
        marker.points.extend(self.trace_points())
        self.trace_segments.publish(marker)
        self.trajectory.publish(self.trace_path())

    # The trace grows all run long and is published twice per second: its message parts are built once per point
    # (robot 2026-10-20: rebuilding them was a fifth of the meter's CPU, more the longer the run).
    def trace_points(self):
        """Marker points of every verified edge of the trace (a LINE_LIST: two points per edge)."""
        c=self.cache
        if c.get('edges_of') is not self.meter:c.update(edges_of=self.meter,edges=0,points=[])
        for edge in self.meter.trajectory_edges[c['edges']:]:
            for x,y in edge:c['points'].append(Point(x=float(x),y=float(y),z=.04))
        c['edges']=len(self.meter.trajectory_edges)
        return c['points']

    def trace_path(self):
        """The trace since its last gap as a Path, the same as path_msg(): each pose faces the next point, the last
        one the way it came. Poses whose next point is known are built once."""
        trace=self.meter.trajectory;begin=max(0,self.meter.trajectory_segment_start);c=self.cache
        if c.get('path_of') is not self.meter or c.get('begin')!=begin:c.update(path_of=self.meter,begin=begin,poses=[])
        n=len(trace)-begin;poses=c['poses']
        for i in range(begin+len(poses),begin+n-1):
            p,q=trace[i][1:3],trace[i+1][1:3]
            poses.append(pose_msg(self,p,math.atan2(q[1]-p[1],q[0]-p[0])))
        msg=PathMsg();msg.header.frame_id=self.settings['runtime']['map_frame'];msg.header.stamp=self.get_clock().now().to_msg()
        msg.poses.extend(poses[:max(0,n-1)])
        if n==1:msg.poses.append(pose_msg(self,trace[begin][1:3],0.))
        elif n>1:
            p,o=trace[-1][1:3],trace[-2][1:3]
            msg.poses.append(pose_msg(self,p,math.atan2(p[1]-o[1],p[0]-o[0])))
        return msg

    def save(self,msg):
        if self.meter is None:return
        try:
            d=decode(msg)
            if d['generation']!=self.generation:return
            self.output.mkdir(parents=True,exist_ok=True)
            dest=self.output/('coverage_'+self.generation+'.json')
            data={**d,**self.meter.report(self.keepout),'map_id':self.base.identity,'coverage_disk_radius_m':self.radius,
                  'sensor_offset_m':self.meter.offset,
                  'denominator_indices':np.flatnonzero(self.meter.denominator).tolist(),
                  'covered_indices':np.flatnonzero(self.meter.covered).tolist(),'actual_trajectory':self.meter.trajectory,'effective_config':self.settings.values,'config_hash':self.settings.hash,
                  'known_free_m2':float(self.base.free.sum()*self.base.resolution**2)}
            tmp=dest.with_suffix('.json.tmp');tmp.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n');tmp.replace(dest)
        except (ValueError,KeyError,OSError) as e:self.get_logger().error('Report write failed: '+str(e))


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO);node=CoverageMeter()
    try:run_until_stopped(node)
    finally:node.destroy_node();rclpy.try_shutdown()
