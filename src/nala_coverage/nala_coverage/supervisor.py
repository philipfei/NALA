"""Mission supervisor: drives an exported coverage_tool plan with Nav2 from the base station and back.
Explicit commands, bounded actions, no automatic launch motion. Autodrive is forward only: every piece is
followed forwards and the robot turns on the spot between pieces. After the plan the Nav2 planner drives the
robot back to the base station (RETURN_PLAN, RETURN).

Radiation (radiation_node): a task starts with the background measurement on the base station (BACKGROUND).
Contaminated floor (/radiation/keepout) is avoided: plan stretches in it are skipped (CONTAMINATION_AVOIDED).
/coverage/go_home stops the plan and drives back to the base station.

Drive-ins into pockets: driving in (plan kind 'spur') is a normal piece Nav2 follows; backing out ('reverse',
state SPUR) is driven by the supervisor itself, straight back, only over floor the meter has measured."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import gc
import os
import signal
import sys
import hashlib
import json
import math
import time
import numpy as np
from scipy.spatial import cKDTree
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from tf2_ros import ExtrapolationException
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path as PathMsg
from nav2_msgs.action import ComputePathToPose, FollowPath
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool, Empty
from diagnostic_msgs.msg import DiagnosticArray
from visualization_msgs.msg import Marker, MarkerArray
from . import base_station
from .geometry import Grid, wrap
from .state import Mission
from .params import node_settings
from .planning import densify, points_of, poly_target, length
from .health import Trust
from .plan_file import check_base_station, load_plan, spur_yaw

# Targets the supervisor drives itself in state SPUR: backing out of a pocket (driving in is a normal piece).
SELF_DRIVEN=('reverse',)
from .ros_common import (run_until_stopped, Inputs, ActionSlot, LATCHED, LATEST, decode, json_msg,
                         diagnostic, pose_msg, path_msg, pose3, transform3, yaw, ros_now, stamp_seconds)

# Pauses that end by themselves once localization is trusted again (recovery.auto_resume_s). Not: operator
# STOP/pause, blocked paths, LOCALIZATION_JUMP (in an almost symmetric room a jump can be to the mirror image).
AUTO_RESUME=('POSITION_UNCERTAIN','HEADING_UNCERTAIN','SCAN_MAP_MISMATCH','LOCALIZATION_SETTLING','TOO_FEW_SCAN_ENDPOINTS',
             'AMCL_STALE','AMCL_INVALID','TF_UNAVAILABLE','SENSOR_STALE','SCAN_STALE','ODOM_STALE','SAFETY_GATE_STALE',
             'COVERAGE_METER_STALE')
# No task holds the robot: it may be moved by hand, and the locator works.
IDLE_STATES=('IDLE','FINISHED','CANCELED','FAILED')
ACTIVE={'BACKGROUND','CONNECT_PLAN','CONNECT','SWEEP','DWELL','TURN','SPUR','PHASE_WAIT','RETURN_PLAN','RETURN','TEST_PLAN','TEST'}
DRIVING=('CONNECT','SWEEP','TURN','SPUR','RETURN','TEST')


class Supervisor(Node):
    def __init__(self):
        super().__init__('coverage_supervisor')
        self.settings=node_settings(self);cfg=self.settings
        # A reviewed coverage_tool export is what the robot drives.
        self.plan_file=self.declare_parameter('plan_file','').value
        self.map_frame=cfg['runtime']['map_frame'];self.odom_frame=cfg['runtime']['odom_frame'];self.base_frame=cfg['runtime']['base_frame']
        self.base=Grid.load(cfg['runtime']['map_yaml']);self.grid=self.base
        self.trust_stamp=0.  # time of the last trust check's pose (s)
        # Wall cell centres for the scan check, as a KD-tree: the old loop over all walls per scan point took
        # 55 ms per scan on the Pi 4 (half a core at 10 Hz) and starved the node (robot, 2026-10-16).
        walls=self.base.world(np.argwhere(self.base.cells==100))
        self.wall_tree=cKDTree(walls) if len(walls) else None
        # Coverage starts and ends at the base station.
        self.station_file=cfg['runtime']['base_station_yaml'] or str(base_station.default_path(cfg['runtime']['map_yaml']))
        self.station=base_station.load(self.station_file,self.map_frame)
        base_station.check(self.base,self.station,cfg.collision)
        self.returning=False
        # Radiation: background measured once per task; contaminated floor is avoided.
        self.radiation={};self.radiation_time=0.;self.background_requested=None;self.background_done=False
        self.keepout=None;self.keepout_changed=False
        h,w=self.base.cells.shape
        self.region=self.base.world([[-.5,-.5],[-.5,w-.5],[h-.5,w-.5],[h-.5,-.5]]).tolist()
        self.collision=cfg.collision;self.radius=cfg['geometry']['coverage_disk_radius_m']
        self.offset=cfg['geometry']['sensor_offset_m'];self.timeout=cfg['deadlines']['mission_s']
        self.output=Path(cfg['runtime']['output_dir'])
        self.sample_seq=0;self.phase_barrier=0;self.meter_config_hash=''
        self.inputs=Inputs(self);self.mission=Mission();self.slot=ActionSlot(self,self.mission)
        for typ,name in [(ComputePathToPose,'/compute_path_to_pose'),(FollowPath,'/follow_path')]:
            self.slot.client(typ,name)
        self.trust=Trust(self.settings);self.trusted=False;self.pose=None;self.odom_pose=None;self.amcl=None
        self.live_map=False;self.amcl_received=0.;self.scan_stamp=None;self.scan_score=(0,0)
        self.gate={};self.gate_time=0.;self.measurement={};self.meter_time=0.
        self.preview=None;self.targets=[];self.current=None
        self.definition=None;self.manual=False;self.phase_since=time.monotonic()
        self.target_since=0.;self.last_motion=time.monotonic();self.motion_pose=None
        self.pending_phase=None;self.path=None;self.path_end_yaw=None;self.test_goal=None
        self.sweep_points=None;self.sweep_index=0;self.stuck=None
        self.worker_pool=ThreadPoolExecutor(max_workers=1);self.work=None
        self.lease_pub=self.create_publisher(String,'/coverage/lease',LATEST)
        # Corner turns (TURN): the supervisor turns the robot on the spot itself, through the velocity gate like Nav2.
        self.turn_pub=self.create_publisher(Twist,'/cmd_vel_nav',LATEST);self.turn_left=[]
        # Backing out of a pocket (SPUR): the only reverse commands the gate passes in autodrive.
        self.reverse_pub=self.create_publisher(Twist,'/cmd_vel_reverse',LATEST);self.spur=None;self.scan_xy=None
        self.outline=np.asarray(cfg['geometry']['footprint_m'],float)
        self.trust_pub=self.create_publisher(String,'/coverage/trust',LATEST)
        self.sample_pub=self.create_publisher(String,'/coverage/sample',LATEST)
        self.definition_pub=self.create_publisher(String,'/coverage/definition',LATCHED)
        self.report_pub=self.create_publisher(String,'/coverage/save_report',LATEST)
        self.state_pub=self.create_publisher(String,'/coverage/state',LATEST)
        self.diag=self.create_publisher(DiagnosticArray,'/coverage/status',10)
        self.route_pub=self.create_publisher(PathMsg,'/coverage/route',LATCHED)
        self.markers=self.create_publisher(MarkerArray,'/coverage/markers',LATCHED)
        self.stop_client=self.create_client(Trigger,'/safety/estop')
        self.nomotion=self.create_client(Empty,'/request_nomotion_update');self.nomotion_pending=False
        # Nav2's lifecycle manager reports whether the planner and controller are active; their action
        # servers exist (and reject goals) before that.
        self.nav_state=self.create_client(Trigger,'/lifecycle_manager_navigation/is_active');self.nav_active=False;self.nav_pending=False
        self.create_subscription(OccupancyGrid,'/map',self.map_received,LATCHED)
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.amcl_pose,10)
        self.create_subscription(String,'/safety/state',self.gate_received,LATEST)
        self.create_subscription(String,'/coverage/measurement',self.meter_received,LATEST)
        # The floor the meter has measured (backing out of a pocket only passes measured floor).
        self.covered_grid=None
        self.create_subscription(OccupancyGrid,'/coverage/covered',self.covered_received,LATCHED)
        self.create_subscription(String,'/coverage/meter_config',self.meter_config_received,LATCHED)
        self.create_subscription(String,'/radiation/state',self.radiation_received,LATEST)
        self.create_subscription(OccupancyGrid,'/radiation/keepout',self.keepout_received,LATCHED)
        self.background_client=self.create_client(Trigger,'/radiation/measure_background')
        self.create_subscription(PoseStamped,'/coverage/test_goal',self.goal_received,10)
        # Where the locator finds the robot from the scan (locator_node): Start needs it on the base station.
        self.located={};self.located_time=0.;self.paused_at=0.;self.escape=None
        self.timing=dict(since=time.monotonic(),ticks=0,tick=0.,trust=0.,gap=0.,last=None,amcl_n=0,amcl_age=0.)
        self.gc_pause={'max':0.,'gen':-1,'start':0.}
        gc.callbacks.append(self.gc_timing)
        self.create_subscription(String,'/coverage/located',self.located_received,LATEST)
        self.create_service(SetBool,'/control/manual',self.manual_service)
        for name,fn in [('preview',self.preview_service),('start',self.start),
                        ('pause',self.pause_service),('resume',self.resume),('cancel',self.cancel_service),
                        ('go_home',self.go_home),('test_navigation',self.test_navigation)]:
            self.create_service(Trigger,'/coverage/'+name,self.service(fn))
        self.create_timer(1/self.settings['localization']['pose_rate_hz'],self.tick);self.create_timer(self.settings['localization']['nomotion_update_period_s'],self.stationary_update)
        self.create_timer(1.,self.check_nav_active)
        # Localization starts at the base station (no global relocalization).
        self.initial_pub=self.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
        self.initial_timer=self.create_timer(cfg['localization']['initial_pose_period_s'],self.publish_initial_pose)
        self.create_timer(self.settings['visualization']['period_s'],self.publish_visuals)

    def service(self,fn):
        def call(req,res):
            try:res.message=fn();res.success=True
            except (ValueError,RuntimeError,OSError,KeyError) as e:res.success=False;res.message=str(e)
            return res
        return call

    def idle_for_edit(self):
        if self.mission.state in ACTIVE or self.mission.state in ('PREPARING','PAUSED') or self.manual or not self.slot.idle:
            raise ValueError('Cancel the task/relinquish control before editing configuration')
        if self.work is not None:raise ValueError('Planning is still running')

    def map_received(self,msg):
        try:
            grid=Grid.from_cells(np.asarray(msg.data,np.int8).reshape(msg.info.height,msg.info.width),
                                 msg.info.resolution,(msg.info.origin.position.x,msg.info.origin.position.y,yaw(msg.info.origin.orientation)))
            self.live_map=msg.header.frame_id==self.map_frame and grid.identity==self.base.identity
        except (ValueError,TypeError):self.live_map=False

    def amcl_pose(self,msg):
        self.amcl=msg;self.amcl_received=time.monotonic()
        tm=self.timing;tm['amcl_n']+=1;tm['amcl_age']=max(tm['amcl_age'],ros_now(self)-stamp_seconds(msg.header.stamp))

    def gc_timing(self,phase,info):
        """Diagnostics: the longest garbage-collection pause (Python stops every thread while it runs)."""
        g=self.gc_pause
        if phase=='start':g['start']=time.perf_counter()
        elif time.perf_counter()-g['start']>g['max']:g['max']=time.perf_counter()-g['start'];g['gen']=info['generation']

    def log_timing(self,now):
        """Diagnostics (robot 2026-10-20: SCAN_STALE / AMCL_STALE at the pocket ends while the LiDAR and AMCL were fine
        and the Pi had a load of 15): one log line per health.timing_log_period_s on how far this node falls behind."""
        tm=self.timing
        if now-tm['since']<self.settings['health']['timing_log_period_s']:return
        lag,cnt=self.inputs.lag,self.inputs.count
        self.get_logger().info(
            f"timing: state {self.mission.state} ticks {tm['ticks']} tick max {1000*tm['tick']:.0f} ms "
            f"(trust {1000*tm['trust']:.0f} ms) gap max {1000*tm['gap']:.0f} ms | amcl {tm['amcl_n']} msgs oldest "
            f"{tm['amcl_age']:.2f} s | scan {cnt.get('scan',0)} msgs oldest {lag.get('scan',0.):.2f} s | odom "
            f"{cnt.get('odom',0)} msgs oldest {lag.get('odom',0.):.2f} s | gc max {1000*self.gc_pause['max']:.0f} ms "
            f"(gen {self.gc_pause['gen']}) | load {os.getloadavg()[0]:.1f}")
        self.gc_pause['max']=0.;self.gc_pause['gen']=-1
        self.timing=dict(since=now,ticks=0,tick=0.,trust=0.,gap=0.,last=tm['last'],amcl_n=0,amcl_age=0.)
        self.inputs.lag={};self.inputs.count={}

    def publish_initial_pose(self):
        """Tell AMCL the robot is on the base station, until AMCL reports a pose. An operator can still correct
        it with RViz's 2D Pose Estimate."""
        if self.amcl is not None:self.initial_timer.cancel();return
        self.send_station_pose()

    def located_received(self,msg):
        """The robot may be put on the base station only after app 3 started (user, 2026-10-19): when the locator
        finds it there and no task holds it, AMCL starts again where the locator sees it, unless AMCL already
        agrees. AMCL is compared with the locator's pose, not the base station: a robot put 10-15 cm off (still
        on the base station) made AMCL restart every 2 s and drop the plan check each time (robot 2026-10-19)."""
        try:self.located=decode(msg);self.located_time=time.monotonic()
        except ValueError:return
        if self.located.get('status')!='ON_BASE' or self.mission.state not in IDLE_STATES or self.manual:return
        cfg=self.settings['locator'];amcl=pose3(self.amcl.pose.pose) if self.amcl else None
        seen=self.located.get('pose') or list(self.station)
        if amcl is None or math.dist(amcl[:2],seen[:2])>cfg['base_tolerance_m'] or \
                abs(math.remainder(amcl[2]-seen[2],2*math.pi))>math.radians(cfg['base_tolerance_deg']):
            self.send_station_pose(seen)
            # A plan check made at the old, wrong pose is not valid.
            if self.work is None:self.preview=None

    def send_station_pose(self,pose=None):
        """AMCL's initial pose: the base station, or `pose` (where the locator sees the robot on it)."""
        pose=pose or self.station
        loc=self.settings['localization'];msg=PoseWithCovarianceStamped()
        msg.header.frame_id=self.map_frame;msg.header.stamp=self.get_clock().now().to_msg()
        msg.pose.pose=pose_msg(self,pose[:2],pose[2]).pose
        msg.pose.covariance[0]=msg.pose.covariance[7]=loc['base_station_sigma_m']**2
        msg.pose.covariance[35]=math.radians(loc['base_station_sigma_deg'])**2
        self.initial_pub.publish(msg)
    def gate_received(self,msg):
        try:self.gate=decode(msg);self.gate_time=time.monotonic()
        except ValueError:pass
    def meter_received(self,msg):
        try:
            d=decode(msg)
            if d['generation']==self.mission.generation:self.measurement=d;self.meter_time=time.monotonic()
        except (ValueError,KeyError):pass

    def covered_received(self,msg):
        if (msg.info.height,msg.info.width)==self.base.cells.shape:
            self.covered_grid=np.asarray(msg.data,np.int8).reshape(msg.info.height,msg.info.width)>0

    def radiation_received(self,msg):
        try:self.radiation=decode(msg);self.radiation_time=time.monotonic()
        except ValueError:pass

    def keepout_received(self,msg):
        if (msg.info.height,msg.info.width)!=self.base.cells.shape:return
        keepout=np.asarray(msg.data,np.int8).reshape(msg.info.height,msg.info.width)>0
        if self.keepout is None or not np.array_equal(keepout,self.keepout):
            self.keepout=keepout;self.keepout_changed=True

    def meter_config_received(self,msg):
        try:self.meter_config_hash=decode(msg)['config_hash']
        except (ValueError,KeyError):self.meter_config_hash=''

    def config_ready(self):
        return self.gate.get('config_hash')==self.settings.hash and self.meter_config_hash==self.settings.hash

    def update_trust(self,now):
        tf_ok=True;pose=None;odom=None
        try:
            # A short interpolation delay prevents testing against a TF not yet received.
            stamp=Time(nanoseconds=self.get_clock().now().nanoseconds-int(self.settings['localization']['tf_delay_s']*1e9)).to_msg()
            try:pose=transform3(self.inputs.lookup(self.map_frame,self.base_frame,stamp))
            except ExtrapolationException:
                # On the Pi 4 under load the EKF's odom TF is sometimes >100 ms old (robot, 2026-10-16): use the
                # newest TF if it is not older than odom_age_s; pose and odom below use that same time.
                newest=self.inputs.lookup(self.map_frame,self.base_frame)
                if ros_now(self)-stamp_seconds(newest.header.stamp)>self.settings['health']['odom_age_s']:raise
                stamp=newest.header.stamp;pose=transform3(newest)
            self.trust_stamp=stamp_seconds(stamp)
            odom=transform3(self.inputs.lookup(self.odom_frame,self.base_frame,stamp))
            scan=self.inputs.messages.get('scan')
            if scan is not None and self.scan_stamp!=stamp_seconds(scan.header.stamp):
                try:laser=transform3(self.inputs.lookup(self.map_frame,scan.header.frame_id,scan.header.stamp))
                except ExtrapolationException:
                    # On the robot a scan often arrives before the EKF's odom TF for its stamp (scan ~30 ms old,
                    # TF up to ~25 ms behind, 2026-10-16): use the newest TF (a few mm off at coverage speed).
                    # Only a scan older than scan_age_s is a TF error.
                    if ros_now(self)-stamp_seconds(scan.header.stamp)>self.settings['health']['scan_age_s']:raise
                    laser=transform3(self.inputs.lookup(self.map_frame,scan.header.frame_id))
                ranges=np.asarray(scan.ranges);ids=np.flatnonzero(np.isfinite(ranges)&(ranges>=max(self.settings['localization']['min_scan_range_m'],scan.range_min))&(ranges<min(self.settings['localization']['max_scan_range_m'],scan.range_max)))
                every=scan.angle_min+ids*scan.angle_increment+laser[2]
                # Every endpoint, for the obstacle check ahead/behind on a drive-in stretch (spur_blocked).
                self.scan_xy=np.column_stack((laser[0]+ranges[ids]*np.cos(every),laser[1]+ranges[ids]*np.sin(every)))
                if len(ids)>self.settings['localization']['max_endpoints']:ids=ids[np.linspace(0,len(ids)-1,self.settings['localization']['max_endpoints']).astype(int)]
                angles=scan.angle_min+ids*scan.angle_increment+laser[2]
                endpoints=np.column_stack((laser[0]+ranges[ids]*np.cos(angles),laser[1]+ranges[ids]*np.sin(angles)))
                match=self.settings['localization']['match_distance_m']
                matched=0 if self.wall_tree is None or not len(endpoints) else int(np.sum(
                    self.wall_tree.query(endpoints,distance_upper_bound=match+1e-9)[0]<=match))
                self.scan_score=(len(endpoints),matched);self.scan_stamp=stamp_seconds(scan.header.stamp)
        except Exception as e:
            tf_ok=False
            # Name the failing lookup: every error here is reported as TF_UNAVAILABLE.
            self.get_logger().warning('Trust check failed: '+str(e).split('\n')[0],throttle_duration_sec=2.)
        self.pose=pose;self.odom_pose=odom
        age=(ros_now(self)-stamp_seconds(self.amcl.header.stamp)) if self.amcl else math.inf
        if now-self.amcl_received>self.settings['localization']['amcl_max_age_s']:age=math.inf
        self.trusted=self.trust.check(now,pose,odom,self.amcl.pose.covariance if self.amcl else None,
                                     *self.scan_score,age,
                                     self.inputs.fresh('scan',self.settings['health']['scan_age_s']) and self.inputs.fresh('odom',self.settings['health']['odom_age_s']) and self.live_map,tf_ok)
        reason=self.trust.reason if self.live_map else 'MAP_IDENTITY_UNVERIFIED'
        self.trust_pub.publish(json_msg({'trusted':self.trusted,'reason':reason}))
        if self.mission.generation:
            self.sample_seq+=1
            self.sample_pub.publish(json_msg({'generation':self.mission.generation,'sample_seq':self.sample_seq,'stamp':self.trust_stamp,
                'pose':pose or [0.,0.,0.],'trusted':self.trusted,'active':self.mission.state in ('CONNECT','SWEEP','DWELL','TURN','SPUR','PHASE_WAIT') and not self.manual}))

    def check_nav_active(self):
        if self.nav_pending or not self.nav_state.service_is_ready():
            if not self.nav_state.service_is_ready():self.nav_active=False
            return
        self.nav_pending=True
        def done(f):
            try:self.nav_active=bool(f.result().success)
            except Exception:self.nav_active=False
            self.nav_pending=False
        self.nav_state.call_async(Trigger.Request()).add_done_callback(done)

    def stationary_update(self):
        if self.inputs.stopped() and self.nomotion.service_is_ready() and not self.nomotion_pending:
            self.nomotion_pending=True
            f=self.nomotion.call_async(Empty.Request())
            f.add_done_callback(lambda _:setattr(self,'nomotion_pending',False))

    def ready(self,require_stopped=True,require_nav=False):
        now=time.monotonic()
        if self.manual:raise ValueError('Relinquish manual ownership first')
        if require_nav and not self.nav_active:raise ValueError('NAV2_NOT_ACTIVE: wait until the Nav2 planner and controller are active')
        if not self.trusted or not self.live_map:raise ValueError(self.trust.reason or 'MAP_UNVERIFIED')
        if self.inputs.reason():raise ValueError(self.inputs.reason())
        if now-self.gate_time>self.settings['health']['gate_age_s'] or not self.gate.get('graph_ok') or self.gate.get('fault'):
            raise ValueError('Safety gate unavailable, graph conflict or reset required')
        if require_stopped and not self.inputs.stopped():raise ValueError('Robot must be stationary')
        if not self.slot.idle:raise ValueError('Previous action is not yet terminal')

    def make_grid(self):
        return self.base

    def preview_service(self):
        self.idle_for_edit()
        if not self.plan_file:raise ValueError('PLAN_FILE_REQUIRED: launch app3_coverage_pi.launch.py with plan:=FILE')
        if not self.trusted:raise ValueError('Localize first')
        self.grid=self.make_grid();grid=self.grid;start=self.pose[:2];polygon=list(self.region)
        self.preview=None
        self.work=self.worker_pool.submit(lambda:self.plan_preview(grid,start,polygon))
        return 'Preview calculation queued; inspect /coverage/status and route before start'

    def plan_preview(self,grid,start,polygon):
        plan=load_plan(self.plan_file,grid,self.settings['runtime']['map_yaml'],self.map_frame,self.collision,
                       sensor=(self.offset,self.radius),turn_merge_m=self.settings['planning']['turn_merge_length_m'],
                       outline=self.outline,margin=self.settings['geometry']['planning_padding_m'])
        reachable=grid.reachable(start,self.collision,self.settings['recovery']['planner_start_tolerance_m'])
        check_base_station(plan,self.station,self.settings['planning']['plan_join_tolerance_m'])
        first=grid.cell(plan['points'][0])
        if not grid.valid(first) or not reachable[first]:raise ValueError('PLAN_START_UNREACHABLE from the robot position')
        # The meter counts what the plan can cover: floor the sensor can reach with the robot radius the plan was
        # made with (never less than NALA's collision radius). Cells only reachable closer to walls are not "missed".
        plan_radius=max(self.collision,plan['robot_radius'] or 0.)
        # A plan for an area only counts the floor inside that area.
        if plan['area']:polygon=plan['area']
        basis=reachable if plan_radius<=self.collision else grid.reachable(plan['points'][0],plan_radius,self.settings['recovery']['planner_start_tolerance_m'])
        if plan['passages']:
            # Areas with turning room the plan enters through a passage: reachable too (resume inside them, count
            # their floor).
            reachable=self.with_islands(grid,reachable,plan['points'],self.collision)
            basis=self.with_islands(grid,basis,plan['points'],plan_radius)
        return {'reachable':reachable,'denominator':self.coverable_cached(grid,basis,polygon,plan),
                'targets':plan['targets'],'connections':[[] for _ in plan['targets']],'start':start,
                'config_hash':self.settings.hash,'plan_file':str(self.plan_file),'plan_sha256':plan['plan_sha256'],
                'plan_robot_radius_m':plan_radius}

    def with_islands(self,grid,reachable,points,radius):
        """reachable plus every area with turning room (radius) that a plan point lies in."""
        out=reachable.copy();safe=grid.safe_centres(radius)
        for p in points:
            rc=grid.cell(p)
            if grid.valid(rc) and safe[rc] and not out[rc]:out|=grid.reachable(p,radius)
        return out

    def coverable_cached(self,grid,basis,polygon,plan):
        """The floor to cover (the slow part of the plan check: 18 s on the Pi 4 for a 68 m2 map), kept next to the
        plan file (<plan>.check.npz) and used again while the map, the plan, the settings, the reachable floor and
        the area are the same (a fingerprint of all of them; anything else is computed again)."""
        key=hashlib.sha256(json.dumps([grid.identity,plan['plan_sha256'],self.settings.hash,
                                       hashlib.sha256(np.packbits(basis).tobytes()).hexdigest(),
                                       [[round(float(v),6) for v in p] for p in polygon]]).encode()).hexdigest()
        cache=Path(self.plan_file).with_suffix('.check.npz')
        try:
            with np.load(cache) as old:
                if str(old['key'])==key and old['denominator'].shape==basis.shape:return old['denominator'].astype(bool)
        except (OSError,KeyError,ValueError):pass
        denominator=grid.coverable(grid.sensor_reachable(basis,self.offset),polygon,self.radius)
        # The floor the sensor reaches on the plan's drive-in stretches (outside the robot radius's reach).
        if plan.get('spurs'):
            area=grid.polygon_mask(polygon)
            for p,q in plan['spurs']:
                u=np.array([math.cos(spur_yaw(p,q,'spur')),math.sin(spur_yaw(p,q,'spur'))])
                for xy in densify([p,q],grid.resolution/2):
                    denominator|=grid.disk(np.asarray(xy)+self.offset*u,self.radius)&area
        try:np.savez_compressed(cache,key=np.array(key),denominator=denominator)
        except OSError as e:self.get_logger().warning('Plan check not saved: '+str(e))
        return denominator

    def start(self):
        self.ready(require_nav=True)
        self.check_located()
        if self.preview is None or self.work is not None:raise ValueError('A completed preview is required')
        if self.preview.get('config_hash')!=self.settings.hash:raise ValueError('PREVIEW_CONFIG_STALE: regenerate preview')
        if not self.config_ready():raise ValueError('CONFIG_HASH_MISMATCH')
        battery=self.inputs.battery()
        if battery is not None and battery<self.settings['battery']['start_ratio']:raise ValueError(f"Start requires battery >= {100*self.settings['battery']['start_ratio']:g}%")
        self.check_start_component()
        if self.settings['radiation']['enabled'] and not self.background_client.service_is_ready():
            raise ValueError('NO_RADIATION_NODE: /radiation/measure_background is not available')
        self.mission.start(time.monotonic(),self.timeout)
        self.definition={'generation':self.mission.generation,'map_id':self.base.identity,
                         'config_hash':self.settings.hash,'denominator':np.flatnonzero(self.preview['denominator']).tolist()}
        self.definition_pub.publish(json_msg(self.definition));self.measurement={};self.meter_time=0.
        self.covered_grid=None         # until the meter sends this task's measured floor
        self.targets=list(self.preview['targets']);self.current=None;self.pending_phase='NEXT';self.stuck=None;self.returning=False
        self.phase_since=time.monotonic();self.manual=False;self.background_done=False
        if self.settings['radiation']['enabled']:self.begin_background()
        return 'Task accepted; preparation checks precede motion'

    def check_located(self):
        """Start only with the robot on the base station, the right way round, as the locator sees it."""
        status=self.located.get('status')
        if time.monotonic()-self.located_time>self.settings['locator']['max_age_s']:
            raise ValueError('LOCATOR_UNAVAILABLE: the robot was not found on the map yet')
        if status=='BACKWARDS':raise ValueError('ROBOT_BACKWARDS: the robot is on the base station the wrong way round')
        if status=='TURNED':raise ValueError(f"ROBOT_TURNED: turn the robot {self.located.get('turn_deg')} deg (positive = to the left)")
        if status!='ON_BASE':raise ValueError('NOT_ON_BASE_STATION: put the robot on the base station')

    def begin_background(self):
        """The robot stands on the base station: the radiation node measures the background first."""
        if not self.background_client.service_is_ready():raise ValueError('NO_RADIATION_NODE: /radiation/measure_background is not available')
        self.mission.state='BACKGROUND';self.phase_since=time.monotonic();self.background_requested=None
        def done(future):
            if future.result() is not None and future.result().success:self.background_requested=time.monotonic()
        self.background_client.call_async(Trigger.Request()).add_done_callback(done)

    def background_ready(self):
        # Only a radiation state sent after the request counts (an older one may still say "ready").
        return bool(self.background_requested is not None and self.radiation_time>self.background_requested
                    and self.radiation.get('background_ready') and not self.radiation.get('measuring'))

    def backing_out_next(self):
        """The robot is in a pocket and the next thing it does is back out: paused while backing out, or while
        driving in (the rest of that drive-in was left out, pause())."""
        if getattr(self.current,'kind','')=='reverse':return True
        return self.current is None and bool(self.targets) and getattr(self.targets[0],'kind','')=='reverse'

    def check_start_component(self):
        # In a pocket there is never turning room: backing out has its own checks (begin_spur: on its line, measured
        # and clean floor, LiDAR clear behind). Robot 2026-10-20: a pause inside a pocket could never be resumed.
        if self.backing_out_next():return
        try:
            rc,_,_,_=self.grid.escape_start(self.pose[:2],self.collision,self.settings['recovery']['planner_start_tolerance_m'])
        except ValueError as error:
            raise ValueError('Actual pose exceeds planner_start_tolerance_m') from error
        if not self.preview['reachable'][rc]:
            raise ValueError('Actual pose belongs to a different reachable component')

    def pause(self,reason):
        if self.mission.state not in ACTIVE and self.mission.state!='PREPARING':return
        if self.current and self.mission.state not in ('TEST_PLAN','TEST') and self.in_drive_in():
            # Paused inside a pocket: resume backs out from where it stopped (the rest of the drive-in is left out).
            self.mission.events.append({'time':time.monotonic(),'reason':'SPUR_SHORTENED','cause':reason,
                                        'xy':[round(v,3) for v in self.pose[:2]]})
            self.current=None;self.sweep_points=None
        elif self.current and self.mission.state not in ('TEST_PLAN','TEST') and getattr(self.current,'kind','') not in ('turn',*SELF_DRIVEN):
            # Resume continues where the robot stopped on the piece (a corner turn keeps its headings left, a
            # drive-in stretch its line).
            rest=poly_target(self.plan_remainder(),'plan');rest.end_yaw=getattr(self.current,'end_yaw',None)
            rest.drive_in=getattr(self.current,'drive_in',None)
            self.current=rest;self.sweep_points=None
        self.slot.cancel();self.pending_phase=None;self.mission.pause(reason,time.monotonic())
        self.paused_at=time.monotonic()
        self.save_report()

    def auto_resumable(self,now):
        """Paused for a localization or late-data reason, recently: it continues by itself once trusted again."""
        return (self.mission.state=='PAUSED' and self.mission.reason in AUTO_RESUME and not self.manual
                and now-self.paused_at<=self.settings['recovery']['auto_resume_s'])

    def try_auto_resume(self,now):
        reason=self.mission.reason
        try:self.resume()
        except ValueError:return   # not yet: still localizing (resume checks trust, the gate and the robot)
        self.mission.events.append({'time':now,'reason':'AUTO_RESUMED','after':reason})

    def pause_service(self):
        if self.auto_resumable(time.monotonic()):
            # Pause while it recalculates: now a real pause that waits for Resume.
            self.mission.change('PAUSED','OPERATOR_PAUSE',time.monotonic());self.save_report()
        else:self.pause('OPERATOR_PAUSE')
        return 'Paused; stopping/cancellation is checked independently'

    def resume(self):
        self.ready(require_nav=True)
        if self.work is not None:raise ValueError('Previous planning worker is still finishing')
        if self.mission.state!='PAUSED':raise ValueError('Task is not paused')
        if self.mission.expired(time.monotonic()):raise ValueError('Mission deadline expired; start a new task')
        if self.mission.coverage_finished:raise ValueError('Nothing left to resume; start a new task')
        self.check_start_component()
        self.mission.resume();self.stuck=None
        if self.settings['radiation']['enabled'] and not self.background_done:
            self.begin_background();return 'Resume accepted; measuring the background again'
        if self.current:self.targets.insert(0,self.current)
        self.current=None;self.pending_phase='RETURN' if self.returning else 'NEXT'
        self.phase_since=time.monotonic();return 'Resume accepted; original mission deadline kept'

    def go_home(self):
        """Stop the plan and drive back to the base station (the rest of the plan is not covered)."""
        if self.mission.state not in ACTIVE and self.mission.state!='PAUSED':raise ValueError('No task to stop')
        if self.mission.state in ('TEST_PLAN','TEST'):raise ValueError('Not during a test navigation')
        if self.returning and self.mission.state!='PAUSED':return 'Already driving back to the base station'
        if self.manual:raise ValueError('Relinquish manual ownership first')
        if self.mission.state=='SPUR' or (getattr(self.current,'kind','') in SELF_DRIVEN and self.pose
                                           and not self.grid.segment_safe(self.pose[:2],self.pose[:2],self.collision)):
            raise ValueError('The robot is in a pocket: wait until it has backed out, then stop')
        rest=([self.plan_remainder()] if self.current else [])+[points_of(t) for t in self.targets]
        left=sum(length(p) for p in rest if len(p)>1)
        self.slot.cancel();self.targets=[];self.current=None;self.sweep_points=None
        now=time.monotonic()
        self.mission.events.append({'time':now,'reason':'STOPPED_BY_OPERATOR','plan_left_m':round(left,3)})
        if not self.mission.root_cause:self.mission.root_cause='OPERATOR_GO_HOME'
        self.returning=True;self.background_done=True
        if self.mission.state=='PAUSED':self.mission.resume()
        else:self.mission.state='PREPARING'
        self.pending_phase='RETURN';self.phase_since=now
        return 'Stopping the plan; driving back to the base station'

    def avoid_keepout(self,target):
        """The parts of a target outside the contaminated floor, each a target of its own (driven to first)."""
        pts=points_of(target)
        if self.keepout is None or not self.keepout.any():return [target]
        if getattr(target,'kind','') in SELF_DRIVEN:return [target]      # checked as a whole (begin_spur)
        p=getattr(target,'drive_in',None)
        if p is not None:
            k=next(i for i,x in enumerate(pts) if math.dist(x,p)<1e-6)
            into=densify(pts[k:],self.base.resolution)
            if any(self.base.valid(rc) and self.keepout[rc] for rc in map(self.base.cell,into)):
                # Contamination in the pocket: no drive-in (and no back-out); the route up to its start as usual.
                self.mission.events.append({'time':time.monotonic(),'reason':'SPUR_SKIPPED_CONTAMINATION','xy':list(p)})
                if self.targets and self.targets[0] is target and len(self.targets)>1 and getattr(self.targets[1],'kind','')=='reverse':
                    del self.targets[1]
                if k<1:return []
                head=poly_target(pts[:k+1],'plan');head.end_yaw=None;head.drive_in=None
                return self.avoid_keepout(head)
        dense=densify(pts,self.base.resolution) if len(pts)>1 else pts
        inside=[self.base.valid(rc) and bool(self.keepout[rc]) for rc in map(self.base.cell,dense)]
        if not any(inside):return [target]
        runs=[];run=[]
        for p,bad in zip(dense,inside):
            if bad:
                if run:runs.append(run);run=[]
            else:run.append(list(p))
        if run:runs.append(run)
        skipped=sum(math.dist(a,b) for (a,b),bad in zip(zip(dense,dense[1:]),inside[1:]) if bad)
        self.mission.blocked.append({'xy':list(dense[inside.index(True)]),'length_m':round(skipped,3),'radius':0.,
                                     'reason':'CONTAMINATION_AVOIDED','time':time.monotonic()})
        out=[]
        for i,r in enumerate(runs):
            n=poly_target(r if len(r)>1 else r*2,'plan');n.connect=True
            n.end_yaw=getattr(target,'end_yaw',None) if i==len(runs)-1 and not inside[-1] else None
            n.drive_in=getattr(target,'drive_in',None) if i==len(runs)-1 and not inside[-1] else None
            out.append(n)
        return out

    def cancel_service(self):
        self.slot.cancel();self.manual=False;self.pending_phase=None
        self.mission.change('CANCELED','OPERATOR_CANCEL',time.monotonic());self.save_report()
        return 'Canceled; no automatic retry'

    def manual_service(self,req,res):
        if req.data:
            self.pause('MANUAL_TAKEOVER')
            if not self.slot.idle or not self.inputs.stopped():
                res.success=False;res.message='Pause requested; wait until stopped/action terminal, then retry';return res
            if self.gate.get('fault') or self.inputs.reason():
                res.success=False;res.message='Safety reset/healthy sensors required';return res
        self.manual=req.data;self.mission.token+=1
        res.success=True;res.message='Fresh manual commands required' if req.data else 'Manual released; autonomy needs explicit resume'
        return res

    def goal_received(self,msg):
        if msg.header.frame_id==self.map_frame:self.test_goal=pose3(msg.pose)

    def test_navigation(self):
        self.idle_for_edit();self.ready(require_nav=True)
        if self.test_goal is None or math.dist(self.pose[:2],self.test_goal[:2])>self.settings['recovery']['short_test_max_m']:raise ValueError('Set a map-frame test_goal within 2 metres')
        self.grid=self.make_grid();self.mission.start(time.monotonic(),self.timeout);self.mission.coverage_finished=True
        self.plan_to(self.test_goal[:2],'TEST_PLAN',self.test_goal[2]);self.target_since=time.monotonic()
        return 'Supervised short navigation trial requested; it stops at the goal'

    def plan_to(self,xy,state,heading=None):
        goal=ComputePathToPose.Goal();goal.goal=pose_msg(self,xy,heading or 0.);goal.planner_id='GridBased';goal.use_start=False
        # NavFn cannot plan from inside its robot_radius band (collision radius + half a cell) near a wall: then
        # plan from the nearest safe point and drive the short way out first (robot 2026-10-19: BLOCKED_IN_PLACE
        # 2-3 cm too close to a wall after a detour around a box).
        self.escape=None
        try:
            _,point,_,escape=self.grid.escape_start(self.pose[:2],self.collision+self.base.resolution/2,
                                                    self.settings['recovery']['planner_start_tolerance_m'])
            if escape:
                self.escape=point;goal.use_start=True;goal.start=pose_msg(self,point,self.pose[2])
        except ValueError:pass       # too far from any safe point: the planner reports the failure as before
        self.path_end_yaw=heading;self.mission.state=state;self.phase_since=time.monotonic()
        self.slot.send(ComputePathToPose,'/compute_path_to_pose',goal)

    def follow(self,points,state,end_yaw=None,feedback=None,escape=None):
        """escape: the safe point the first segment leads to from a start a little too close to a wall
        (plan_to): that one short segment is checked for a clear line only."""
        if len(points)<2:raise ValueError('Controller path has fewer than two poses')
        # A piece ending with a drive-in: from its start P on the path is checked with the robot's outline (plus
        # the planning padding), straight, like when the plan was loaded; it never had turning-circle clearance.
        into=self.drive_in_from(points)
        for i,(a,b) in enumerate(zip(points,points[1:])):
            if i==0 and escape is not None and math.dist(b,escape)<=self.settings['recovery']['plan_endpoint_tolerance_m'] and self.grid.visible(a,b):continue
            if into is not None and i>=into:
                swept=self.grid.outline_swept(a,b,math.atan2(b[1]-a[1],b[0]-a[0]),self.outline,self.settings['geometry']['planning_padding_m'])
                if swept is None or (swept&~self.grid.free).any():raise ValueError('Unsafe drive-in path segment')
                continue
            if self.on_passage(a,b):
                swept=self.grid.outline_swept(a,b,math.atan2(b[1]-a[1],b[0]-a[0]),self.outline,self.settings['geometry']['planning_padding_m'])
                if swept is None or (swept&~self.grid.free).any():raise ValueError('Unsafe passage path segment')
                continue
            if not self.grid.segment_safe(a,b,self.collision):raise ValueError('Unsafe continuous path segment')
        self.path=points;goal=FollowPath.Goal();goal.path=path_msg(self,points,end_yaw)
        goal.controller_id='FollowPath';goal.goal_checker_id='goal_checker';goal.progress_checker_id='progress_checker'
        self.mission.state=state;self.phase_since=time.monotonic();self.motion_pose=list(self.pose);self.last_motion=time.monotonic()
        self.slot.send(FollowPath,'/follow_path',goal,feedback)

    def on_passage(self,a,b):
        """Segment a-b lies on one of the current piece's passages (plan_file: straight through an entrance narrower
        than the turning circle, checked with the outline), in its driving direction."""
        for p,q in getattr(self.current,'passages',None) or []:
            d=math.dist(p,q)
            if d<1e-9:continue
            u=((q[0]-p[0])/d,(q[1]-p[1])/d)
            def at(x):
                return (abs(-(x[0]-p[0])*u[1]+(x[1]-p[1])*u[0])<1e-6 and
                        -1e-6<=(x[0]-p[0])*u[0]+(x[1]-p[1])*u[1]<=d+1e-6)
            if at(a) and at(b) and (b[0]-a[0])*u[0]+(b[1]-a[1])*u[1]>=0:return True
        return False

    def drive_in_from(self,points):
        """Index in points where the current piece's drive-in starts (its P), or None. A path that starts inside the
        drive-in (rejoining after P) is a drive-in from its first point."""
        p=getattr(self.current,'drive_in',None)
        if p is None:return None
        for i,x in enumerate(points):
            if math.dist(x,p)<1e-6:return i
        return 0 if self.in_drive_in() else None

    def in_drive_in(self):
        """The robot is past the start P of the current piece's drive-in, into the pocket."""
        p=getattr(self.current,'drive_in',None)
        if p is None or self.pose is None:return False
        q=points_of(self.current)[-1];d=math.dist(p,q)
        if d<1e-9:return False
        along=((self.pose[0]-p[0])*(q[0]-p[0])+(self.pose[1]-p[1])*(q[1]-p[1]))/d
        return along>self.settings['spur']['end_tolerance_m']

    def end_drive_in(self,reason):
        """Stopped inside a pocket (blocked, timeout, a pause): the rest of the drive-in is left out and the robot backs
        out from here, the next target; never skip on deeper into a pocket or plan a way out with Nav2."""
        self.slot.cancel()
        self.mission.events.append({'time':time.monotonic(),'reason':'SPUR_SHORTENED','cause':reason,
                                    'xy':[round(v,3) for v in self.pose[:2]]})
        self.current=None;self.sweep_points=None
        self.mission.state='PREPARING';self.pending_phase='NEXT';self.phase_since=time.monotonic()

    def next_target(self):
        while self.targets:
            parts=self.avoid_keepout(self.targets[0])
            self.targets[0:1]=parts
            if parts:break
        self.current=self.targets.pop(0) if self.targets else None;self.sweep_points=None
        if self.current is None:self.finish_pass();return
        self.target_since=time.monotonic()
        if getattr(self.current,'kind','') in SELF_DRIVEN:self.begin_spur();return
        pts=points_of(self.current)
        # The robot normally stands where the previous piece ended; after a detour it is driven to the rest.
        if not getattr(self.current,'connect',False) and math.dist(self.pose[:2],pts[0])<=self.settings['planning']['plan_join_tolerance_m']:
            self.begin_sweep();return
        if not getattr(self.current,'connect',False) and self.rejoin(pts):return
        heading=math.atan2(pts[1][1]-pts[0][1],pts[1][0]-pts[0][0]) if len(pts)>1 and math.dist(pts[0],pts[1])>1e-9 else \
            (self.current.headings[0] if getattr(self.current,'headings',None) else None)
        self.plan_to(pts[0],'CONNECT_PLAN',heading)

    def rejoin(self,pts):
        """A little beside the piece (after a pause, a pose correction): back onto it while driving, without a planned
        connection. True when done; False -> the caller plans a connection as before."""
        cfg=self.settings['planning'];here=list(self.pose[:2])
        if getattr(self.current,'kind','')=='turn':
            if math.dist(here,pts[0])>cfg['rejoin_max_m']:return False
            self.begin_turn();return True                  # a corner turn is done where the robot stands
        dense=densify(pts,self.base.resolution)
        d=[math.dist(here,p) for p in dense];k=int(np.argmin(d))
        if d[k]>cfg['rejoin_max_m']:return False
        j=k;along=0.
        while j<len(dense)-1 and along<cfg['rejoin_ahead_m']:along+=math.dist(dense[j],dense[j+1]);j+=1
        if along<cfg['rejoin_ahead_m'] and math.dist(here,dense[-1])<=cfg['rejoin_max_m']:
            # Only the last few cm of the piece are left: done. Driving to its end point, a few cm away at a steep
            # angle, made Nav2 turn on the spot first (north-west corner of room Test, 2026-10-20).
            self.mission.events.append({'time':time.monotonic(),'reason':'REJOINED_AT_END','offset_m':round(d[k],3)})
            self.next_target();return True
        path=densify([here]+dense[j:],self.base.resolution)
        if len(path)<2 or not self.grid.segment_safe(here,dense[j],self.collision):return False
        self.mission.events.append({'time':time.monotonic(),'reason':'REJOINED','offset_m':round(d[k],3)})
        self.sweep_points=path;self.sweep_index=0
        self.follow(path,'SWEEP',getattr(self.current,'end_yaw',None));return True

    def update_sweep_index(self):
        # Nearest point of the piece ahead of the last one, so a piece that returns near itself is not skipped.
        if not self.sweep_points or self.pose is None:return
        p=np.asarray(self.pose[:2]);end=min(len(self.sweep_points),self.sweep_index+40)
        window=np.asarray(self.sweep_points[self.sweep_index:end])
        step=int(np.argmin(np.sum((window-p)**2,axis=1)))
        # Progress along the piece restarts target_s, so a long piece is not cut off. (FollowPath's
        # distance_to_goal starts from the nearest path point anywhere, which jumps on a loop past itself.)
        if step:self.sweep_index+=step;self.target_since=time.monotonic()

    def plan_remainder(self):
        if self.sweep_points:self.update_sweep_index();return self.sweep_points[self.sweep_index:]
        return densify(points_of(self.current),self.base.resolution)

    def plan_failure(self,reason):
        """Blocked while driving the plan: skip ahead past the obstacle, drive around it and continue exactly from there.

        While following a piece the rest of it is skipped by plan_detour_skip_m (more on repeated failures at
        the same point); a failed connection is first retried as it is. The new start is reached with the
        Nav2 planner, which routes around what the costmaps see. The skipped stretch is reported."""
        # Failing again and again without moving (an obstacle right next to the robot, so it can neither turn nor
        # plan a way out): pause instead of skipping the rest of the plan. Resume continues from here.
        # (Skipping on past an obstacle is fine; skipping blocked_in_place_skip_m of plan without moving is not.)
        here=list(self.pose[:2]) if self.pose else [0.,0.];rec=self.settings['recovery']
        if self.stuck and math.dist(self.stuck[0],here)<=rec['blocked_in_place_radius_m']:self.stuck[1]+=1
        else:self.stuck=[here,1,0.]
        if self.stuck[1]>rec['blocked_in_place_attempts'] or self.stuck[2]>=rec['blocked_in_place_skip_m']:
            if not self.mission.root_cause:self.mission.root_cause=reason
            self.pause('BLOCKED_IN_PLACE: '+reason);return
        following=bool(self.sweep_points)
        rest=self.plan_remainder();t=self.current
        self.slot.cancel()
        if not self.mission.root_cause:self.mission.root_cause=reason
        now=time.monotonic()
        key='plan:%.2f,%.2f'%tuple(rest[0]);count=self.mission.failures.get(key,0)+1;self.mission.failures[key]=count
        skip=self.settings['recovery']['plan_detour_skip_m']*(count if following else count-1)
        cut,along=0,0.
        while cut<len(rest)-1 and along<skip-1e-9:along+=math.dist(rest[cut],rest[cut+1]);cut+=1
        self.stuck[2]+=along
        event={'time':now,'target':key,'reason':reason}
        if cut:
            self.mission.blocked.append({'xy':list(rest[0]),'to':list(rest[cut]),'length_m':round(along,3),'radius':0.,
                                         'reason':'PLAN_STRETCH_SKIPPED','cause':reason,'time':now})
            event['skipped_m']=round(along,3)
        rest=rest[cut:]
        if len(rest)>1 or not cut:
            n=poly_target(rest,'plan');n.end_yaw=getattr(t,'end_yaw',None);n.connect=True;n.drive_in=getattr(t,'drive_in',None)
            self.targets.insert(0,n)
        self.mission.events.append(event)
        self.current=None;self.sweep_points=None;self.mission.state='PREPARING';self.pending_phase='NEXT';self.phase_since=now

    def begin_turn(self):
        """One continuous turn on the spot through the corner's headings (plan_file.merge_turns)."""
        self.turn_left=list(self.current.headings);self.mission.state='TURN'
        self.phase_since=self.target_since=self.last_motion=time.monotonic();self.motion_pose=list(self.pose)

    def turn_step(self,now):
        """Turn command for the corner turn, sent like Nav2's (the gate ramps and limits it). It slows down only
        before the last heading turned to in the same direction, so a corner is one smooth turn."""
        m=self.settings['motion'];tol=math.radians(m['turn_tolerance_deg']);yaw_now=self.pose[2]
        while self.turn_left and abs(wrap(self.turn_left[0]-yaw_now))<=tol:self.turn_left.pop(0)
        self.current.headings=list(self.turn_left)      # a pause keeps only what is left
        if not self.turn_left:
            self.turn_pub.publish(Twist());self.next_target();return
        first=wrap(self.turn_left[0]-yaw_now);same=abs(first);prev=self.turn_left[0]
        for h in self.turn_left[1:]:
            step=wrap(h-prev)
            if step*first<=0:break
            same+=abs(step);prev=h
        w=math.copysign(min(m['angular_rad_s'],math.sqrt(2*m['angular_accel_rad_s2']*same)),first)
        msg=Twist();msg.angular.z=w;self.turn_pub.publish(msg)
        if abs(wrap(yaw_now-self.motion_pose[2]))>=self.settings['recovery']['movement_rotation_rad']:
            self.motion_pose=list(self.pose);self.last_motion=now
        if now-self.last_motion>=self.settings['deadlines']['no_motion_s']:
            # The wheels do not turn it: leave the rest of this corner, never pause the task for it.
            self.mission.events.append({'time':now,'reason':'TURN_SKIPPED','headings_left':len(self.turn_left)})
            self.turn_left=[];self.turn_pub.publish(Twist());self.next_target()

    def begin_spur(self):
        """Backing out of a pocket (plan kind 'reverse': from Q straight back to P, facing in), driven by spur_step.
        Driving in is a normal piece that Nav2 follows (user 2026-10-20); only backing out is the supervisor's.
        It starts only when every cell the outline will pass is floor the meter has measured (or where the robot
        stood at P before driving in) and not contaminated."""
        t=self.current;q,p=points_of(t)[0],points_of(t)[-1];now=time.monotonic()
        cfg=self.settings['spur'];yaw_=spur_yaw(q,p,'reverse');u=(math.cos(yaw_),math.sin(yaw_))
        # The line is followed in the odometry frame (smooth; AMCL corrections jump a few cm and degrees: robot
        # 2026-10-20, steering on AMCL zigzagged +-15-20 deg). It is put into odom once, here.
        qo,po=self.to_odom(q),self.to_odom(p)
        yo=math.atan2(qo[1]-po[1],qo[0]-po[0]);uo=(math.cos(yo),math.sin(yo))
        xo,yo_,ho=self.odom_pose if self.odom_pose is not None else self.pose
        side=-(xo-qo[0])*uo[1]+(yo_-qo[1])*uo[0];err=abs(wrap(ho-yo))
        out_m=-((xo-qo[0])*uo[0]+(yo_-qo[1])*uo[1]);total=math.dist(q,p)
        x,y,h=self.pose
        self.spur={'a':list(qo),'b':list(po),'yaw':yo,'map_yaw':yaw_,'sign':-1.,'length':total}
        if out_m>=total:self.spur_done();return       # already out (a pause right at the end of backing out)
        rejoin=self.settings['planning']['rejoin_max_m']
        if abs(side)>rejoin or out_m<-rejoin or err>math.radians(cfg['max_heading_error_deg']):
            if self.grid.segment_safe([x,y],[x,y],self.collision):
                self.mission.events.append({'time':now,'reason':'SPUR_REVERSE_SKIPPED','xy':[round(x,3),round(y,3)]})
                self.next_target();return
            self.pause('SPUR_OFF_LINE: the robot is beside the line it drove into the pocket; drive it out by hand');return
        # The bare outline against the map (the plan was checked with padding + margin; a few cm of localization
        # error must not stop it: robot 2026-10-20, a false "wall behind"). Real obstacles: the LiDAR (spur_step).
        # The back-out ends up to end_tolerance_m PAST P (spur_step), never short of it on the pocket side.
        past=[p[0]-u[0]*cfg['end_tolerance_m'],p[1]-u[1]*cfg['end_tolerance_m']]
        swept=self.grid.outline_swept([x,y],past,yaw_,self.outline,0.)
        if swept is None or (swept&~self.grid.free).any():self.pause('SPUR_REVERSE_BLOCKED: a wall behind the robot');return
        if self.keepout is not None and (swept&self.keepout).any():
            self.pause('SPUR_REVERSE_CONTAMINATION: contaminated floor behind the robot');return
        stood=np.zeros(swept.shape,bool);cells=self.grid.outline_cells((p[0],p[1],yaw_),self.outline,0.)
        if cells is not None:stood[cells]=True
        covered=self.covered_grid if self.covered_grid is not None else np.zeros(swept.shape,bool)
        if (swept&~stood&~covered).any():
            self.pause('SPUR_REVERSE_NOT_MEASURED: floor behind the robot is not measured');return
        self.mission.state='SPUR';self.phase_since=self.target_since=self.last_motion=now;self.motion_pose=list(self.pose)

    def spur_done(self):
        """Backed out: turn on the spot to the next piece (TURN keeps this target, a pause resumes here) or go on."""
        self.reverse_pub.publish(Twist())
        if getattr(self.current,'end_yaw',None) is not None:
            self.current.headings=[self.current.end_yaw];self.begin_turn();return
        self.next_target()

    def to_odom(self,xy):
        """A map point in the odometry frame, by the current map and odom poses of the robot."""
        if self.odom_pose is None:return [float(xy[0]),float(xy[1])]
        (mx,my,mh),(ox,oy,oh)=self.pose,self.odom_pose;d=oh-mh;c,s=math.cos(d),math.sin(d)
        dx,dy=xy[0]-mx,xy[1]-my
        return [ox+c*dx-s*dy,oy+s*dx+c*dy]

    def spur_blocked(self,sign):
        """A LiDAR point within spur.clear_m ahead of (sign +1) or behind (-1) the outline, outline + padding wide."""
        if self.scan_xy is None or not len(self.scan_xy):return False
        x,y,h=self.pose;c,s=math.cos(h),math.sin(h);d=self.scan_xy-[x,y]
        lx=c*d[:,0]+s*d[:,1];ly=-s*d[:,0]+c*d[:,1]
        pad=self.settings['geometry']['planning_padding_m'];reach=pad+self.settings['spur']['clear_m']
        lo,hi=self.outline.min(axis=0),self.outline.max(axis=0)
        beside=(ly>=lo[1]-pad)&(ly<=hi[1]+pad)
        zone=(lx>hi[0])&(lx<=hi[0]+reach) if sign>0 else (lx<lo[0])&(lx>=lo[0]-reach)
        return bool((beside&zone).any())

    def spur_step(self,now):
        """Back straight out along the line, steering onto it on odometry; obstacles and contamination in the map."""
        sp=self.spur;cfg=self.settings['spur'];m=self.settings['motion']
        x,y,h=self.odom_pose if self.odom_pose is not None else self.pose;a=sp['a'];u=(math.cos(sp['yaw']),math.sin(sp['yaw']))
        mx,my,_=self.pose;mu=(math.cos(sp['map_yaw']),math.sin(sp['map_yaw']))
        along=(x-a[0])*u[0]+(y-a[1])*u[1];side=-(x-a[0])*u[1]+(y-a[1])*u[0];err=wrap(h-sp['yaw'])
        # Aim end_tolerance_m past P: the robot turns there, and stopping short would leave it on the pocket side,
        # closer to walls than the plan allows for turning (simulation 2026-10-20: the rod hit a wall).
        left=sp['length']+cfg['end_tolerance_m']+along
        if left<=cfg['end_tolerance_m']:self.spur_done();return
        cells=self.grid.outline_cells((mx-mu[0]*cfg['clear_m'],my-mu[1]*cfg['clear_m'],sp['map_yaw']),self.outline,
                                      self.settings['geometry']['planning_padding_m'])
        dirty=cells is not None and self.keepout is not None and bool(self.keepout[cells].any())
        if self.spur_blocked(-1) or dirty:
            self.reverse_pub.publish(Twist())
            self.pause('SPUR_REVERSE_CONTAMINATION: contaminated floor behind the robot' if dirty else
                       'SPUR_REVERSE_BLOCKED: an obstacle behind the robot');return
        v=-min(cfg['speed_m_s'],math.sqrt(2*m['linear_accel_m_s2']*left))
        w=-cfg['heading_gain']*err+cfg['track_gain']*side
        msg=Twist();msg.linear.x=v;msg.angular.z=max(-m['angular_rad_s'],min(m['angular_rad_s'],w));self.reverse_pub.publish(msg)
        if math.dist(self.pose[:2],self.motion_pose[:2])>=self.settings['recovery']['movement_translation_m']:
            self.motion_pose=list(self.pose);self.last_motion=now
        if now-self.last_motion>=self.settings['deadlines']['no_motion_s']:
            self.reverse_pub.publish(Twist());self.pause('NO_MOVEMENT')

    def begin_sweep(self):
        t=self.current
        if getattr(t,'kind','')=='turn':self.begin_turn();return
        if getattr(t,'kind','') in SELF_DRIVEN:self.begin_spur();return
        if length(points_of(t))<self.settings['planning']['stationary_target_length_m']:
            self.mission.state='DWELL';self.phase_since=time.monotonic();return
        # The exported path is already dense with its corners rounded where that keeps every cell: follow it
        # exactly and turn at its end to the next piece. The controller starts at the nearest path point.
        # A point every grid cell, so progress and a skipped stretch are measured along the path itself.
        pts=densify(points_of(t),self.base.resolution)
        self.sweep_points=pts;self.sweep_index=0
        self.follow(pts,'SWEEP',getattr(t,'end_yaw',None))

    def finish_pass(self):
        # Require meter acknowledgement of the last issued sample before finishing.
        self.phase_barrier=self.sample_seq+1;self.mission.state='PHASE_WAIT';self.phase_since=time.monotonic()

    def complete_coverage(self,reason):
        # NALA has no dock: the robot stops where the plan ends.
        self.returning=False
        self.mission.coverage_finished=True;self.mission.change('FINISHED',reason,time.monotonic());self.save_report()

    def fail(self,reason):
        self.slot.cancel();self.pending_phase=None;self.mission.change('FAILED',reason,time.monotonic());self.save_report()

    def begin_return(self):
        """Coverage is done: drive back to the base station with the Nav2 planner (forward only)."""
        self.returning=True;self.current=None;self.target_since=time.monotonic()
        self.plan_to(self.station[:2],'RETURN_PLAN',self.station[2])

    def target_failure(self,reason):
        if self.returning:self.pause('RETURN_FAILED: '+reason);return
        if self.current and self.in_drive_in():self.end_drive_in(reason);return
        if self.current:self.plan_failure(reason);return
        self.pause(reason)

    def save_report(self):
        if not self.mission.generation:return
        d={'generation':self.mission.generation,'state':self.mission.state,'reason':self.mission.reason,
           'root_cause':self.mission.root_cause,'events':self.mission.events,
           'temporary_blockages':self.mission.blocked,'target_failures':self.mission.failures,
           'region':'whole_map','collision_radius_m':self.collision,'sensor_offset_m':self.offset,
           'elapsed_s':time.monotonic()-self.mission.started,
           'config_hash':self.settings.hash,'effective_config':self.settings.values}
        if self.preview and self.preview.get('plan_file'):
            d['plan_file']=self.preview['plan_file'];d['plan_sha256']=self.preview['plan_sha256']
            d['coverable_robot_radius_m']=self.preview.get('plan_robot_radius_m')
        if self.preview:
            roi=self.base.polygon_mask(self.region)&self.base.free
            d['selected_free_m2']=int(roi.sum())*self.base.resolution**2
            d['unreachable_selected_m2']=int((roi&~self.preview['denominator']).sum())*self.base.resolution**2
        self.report_pub.publish(json_msg(d))
        self.output.mkdir(parents=True,exist_ok=True)
        (self.output/('task_'+self.mission.generation+'.json')).write_text(json.dumps(d,indent=2)+'\n')

    def action_result(self,name,status,result,error):
        phase=self.mission.state
        success=status==4 and not error and getattr(result,'error_code',0)==0
        if success and phase in DRIVING and self.path:
            miss=math.dist(self.pose[:2],self.path[-1])
            # Nav2 reached the end by its own pose, then AMCL moved the robot's pose a few cm (after corner turns).
            # A little off is accepted: the next piece rejoins smoothly from there (robot 2026-10-20: a new planned
            # connection in the south-west corner turned 267 deg on the spot; the return ended RETURN_FAILED).
            accept=(self.settings['planning']['rejoin_max_m'] if phase in ('CONNECT','SWEEP') else
                    self.settings['locator']['base_tolerance_m'] if phase=='RETURN' else 0.)
            if miss>self.settings['recovery']['endpoint_tolerance_m']:
                if miss<=accept:
                    self.mission.events.append({'time':time.monotonic(),'reason':'ENDPOINT_MISSED_ACCEPTED','state':phase,'offset_m':round(miss,3)})
                else:success=False;error='ACTION_SUCCEEDED_WITHOUT_REACHING_ENDPOINT'
        if not success:
            if phase in ('CONNECT','SWEEP','RETURN') and not error:error=f"FOLLOW_PATH_ERROR_{getattr(result,'error_code','')}"
            if phase in ('CONNECT_PLAN','CONNECT','SWEEP','RETURN_PLAN','RETURN'):self.target_failure(error or 'NAVIGATION_FAILED')
            else:self.pause(error or 'ACTION_FAILED')
            return
        if phase in ('CONNECT_PLAN','RETURN_PLAN','TEST_PLAN'):
            if result.path.header.frame_id!=self.map_frame:raise ValueError('Planner returned a non-map frame')
            points=[[p.pose.position.x,p.pose.position.y] for p in result.path.poses]
            if len(points)<2:
                # Planner may return one pose for a coincident goal; only accept if already there.
                if points and math.dist(self.pose[:2],points[0])<=self.settings['recovery']['plan_endpoint_tolerance_m']:points=[list(self.pose[:2]),points[0]]
                else:raise ValueError('Empty planner result')
            if math.dist(points[0],self.pose[:2])>self.settings['recovery']['planner_start_tolerance_m']:raise ValueError('Planner start does not match robot')
            expected={'CONNECT_PLAN':lambda:self.current.start,'RETURN_PLAN':lambda:self.station[:2],'TEST_PLAN':lambda:self.test_goal[:2]}[phase]()
            if math.dist(points[-1],expected)>self.settings['recovery']['plan_endpoint_tolerance_m']+1e-9:
                # The planner could only get near that point of the plan (blocked): skip further along the plan.
                if phase in ('CONNECT_PLAN','RETURN_PLAN'):self.target_failure('PLANNER_ENDPOINT_MISSES_TARGET');return
                raise ValueError('Planner endpoint misses requested target')
            points.insert(0,list(self.pose[:2]))
            try:self.follow(points,{'CONNECT_PLAN':'CONNECT','RETURN_PLAN':'RETURN','TEST_PLAN':'TEST'}[phase],self.path_end_yaw,escape=self.escape)
            except ValueError as e:
                # A detour planned around an obstacle may pass closer to a wall than the plan allows: that is a
                # failed connection (retry, then skip further), not a reason to stop.
                if phase in ('CONNECT_PLAN','RETURN_PLAN'):self.target_failure('CONNECT_PATH_REJECTED: '+str(e))
                else:raise
        elif phase=='CONNECT':self.begin_sweep()
        elif phase=='SWEEP':self.next_target()
        elif phase=='RETURN':self.complete_coverage('RETURNED_TO_BASE')
        elif phase=='TEST':self.mission.change('FINISHED');self.save_report()

    def request_estop(self):
        if self.stop_client.service_is_ready():self.stop_client.call_async(Trigger.Request())

    def tick(self):
        now=time.monotonic();tm=self.timing
        if tm['last'] is not None:tm['gap']=max(tm['gap'],now-tm['last'])
        tm['last']=now;tm['ticks']+=1
        self.update_trust(now)
        tm['trust']=max(tm['trust'],time.monotonic()-now)
        self.tick_body(now)
        tm['tick']=max(tm['tick'],time.monotonic()-now)
        self.log_timing(time.monotonic())

    def tick_body(self,now):
        if self.work is not None and self.work.done():
            work=self.work;self.work=None
            try:
                result=work.result()
                self.preview=result;self.mission.reason='PREVIEW_READY'
                self.output.mkdir(parents=True,exist_ok=True)
                audit={k:result[k] for k in ('config_hash','start','plan_file','plan_sha256','plan_robot_radius_m') if k in result}
                audit['effective_config']=self.settings.values
                (self.output/'preview_audit.json').write_text(json.dumps(audit,indent=2)+'\n')
            except Exception as e:
                self.mission.reason='PREVIEW_FAILED: '+str(e)
        if self.mission.expired(now):self.fail('MISSION_TIMEOUT')
        if self.slot.cancel_overdue(now) and not self.inputs.stopped():self.request_estop()
        if self.slot.acceptance_overdue(now):self.pause('ACTION_ACCEPT_TIMEOUT')
        phase=self.mission.state
        try:
            if phase=='SWEEP' and self.sweep_points:self.update_sweep_index()
            if phase=='BACKGROUND':
                if self.background_ready():
                    self.background_done=True;self.mission.state='PREPARING';self.phase_since=now
                elif now-self.phase_since>=self.settings['radiation']['background_timeout_s']:
                    self.pause('NO_SCINTILLATOR: no background readings within '
                               f"{self.settings['radiation']['background_timeout_s']:g} s")
            if self.keepout_changed:
                self.keepout_changed=False
                # New contamination on the rest of the piece being driven: stop and drive around it.
                if phase=='SWEEP' and self.sweep_points:
                    rest=self.plan_remainder()
                    if any(self.base.valid(rc) and self.keepout[rc] for rc in map(self.base.cell,rest)):
                        if self.in_drive_in():self.end_drive_in('CONTAMINATION_AHEAD');return
                        n=poly_target(rest,'plan');n.end_yaw=getattr(self.current,'end_yaw',None);n.connect=True
                        n.drive_in=getattr(self.current,'drive_in',None)
                        self.slot.cancel();self.targets.insert(0,n);self.current=None;self.sweep_points=None
                        self.mission.state='PREPARING';self.pending_phase='NEXT';self.phase_since=now
            # BACKGROUND: the robot stands on the base station, like PREPARING (no motion, no fault pauses).
            if phase in ACTIVE and phase!='BACKGROUND':
                fault=self.gate.get('fault') or self.inputs.reason()
                if now-self.gate_time>self.settings['health']['gate_age_s']:fault=fault or 'SAFETY_GATE_STALE'
                if not self.gate.get('graph_ok'):fault=fault or 'VELOCITY_GRAPH_CONFLICT'
                if not self.config_ready():fault=fault or 'CONFIG_HASH_MISMATCH'
                if not self.trusted:fault=fault or self.trust.reason
                if not self.mission.coverage_finished and (now-self.meter_time>self.settings['health']['meter_age_s'] or self.measurement.get('fault')):
                    fault=fault or 'COVERAGE_METER_STALE'
                if fault:self.pause(fault)
                elif phase in ('CONNECT','SWEEP','CONNECT_PLAN','RETURN','RETURN_PLAN') and now-self.target_since>=self.settings['deadlines']['target_s']:self.target_failure('TARGET_TIMEOUT')
                elif phase.endswith('_PLAN') and now-self.phase_since>=self.settings['deadlines']['planning_s']:
                    if phase in ('CONNECT_PLAN','RETURN_PLAN'):self.target_failure('PLAN_TIMEOUT')
                    else:self.pause('PLAN_TIMEOUT')
                elif phase=='TEST' and now-self.target_since>=self.settings['deadlines']['target_s']:self.pause('TEST_TIMEOUT')
                elif phase=='TURN':self.turn_step(now)
                elif phase=='SPUR':self.spur_step(now)
                elif phase in DRIVING:
                    if self.motion_pose is None or math.dist(self.pose[:2],self.motion_pose[:2])>=self.settings['recovery']['movement_translation_m'] or abs(wrap(self.pose[2]-self.motion_pose[2]))>=self.settings['recovery']['movement_rotation_rad']:
                        self.motion_pose=list(self.pose);self.last_motion=now
                        # A long connection to a plan piece is not cut off while the robot is moving.
                        if phase in ('CONNECT','RETURN'):self.target_since=now
                    if now-self.last_motion>=self.settings['deadlines']['no_motion_s']:
                        if phase in ('CONNECT','SWEEP','RETURN'):self.target_failure('NO_MOVEMENT')
                        else:self.pause('NO_MOVEMENT')
            if self.auto_resumable(now):self.try_auto_resume(now)
            # Actions can complete during a pause: token filtering prevents state resurrection.
            while self.slot.events:
                self.action_result(*self.slot.events.pop(0))
            phase=self.mission.state
            if phase=='PREPARING' and now-self.phase_since>self.settings['deadlines']['prepare_s']:self.pause('PREPARATION_TIMEOUT')
            if self.mission.state=='PREPARING' and self.pending_phase and self.slot.idle and self.inputs.stopped():
                if self.trusted and self.gate.get('healthy') and not self.inputs.reason() and \
                        self.measurement.get('generation')==self.mission.generation and now-self.meter_time<self.settings['health']['meter_age_s']:
                    pending=self.pending_phase;self.pending_phase=None;self.check_start_component()
                    if pending=='RETURN':self.begin_return()
                    else:self.next_target()
            elif phase=='PHASE_WAIT':
                if self.measurement.get('last_active_sample_seq',-1)>=self.phase_barrier and self.inputs.stopped():
                    self.mission.events.append({'time':now,'state':phase,'reason':'PLAN_COMPLETE'});self.begin_return()
                elif now-self.phase_since>self.settings['health']['meter_age_s']:self.pause('METER_PHASE_ACK_TIMEOUT')
            elif phase=='DWELL' and now-self.phase_since>self.settings['deadlines']['dwell_s']:self.next_target()
        except (ValueError,RuntimeError,KeyError) as e:
            self.pause('EXECUTION_CHECK: '+str(e))
        phase=self.mission.state
        owner='MANUAL' if self.manual else ('NAV' if phase in DRIVING else 'NONE')
        self.lease_pub.publish(json_msg({'config_hash':self.settings.hash,'owner':owner,'epoch':self.mission.generation+':'+str(self.mission.token)}))
        data={'config_hash':self.settings.hash,'generation':self.mission.generation,'state':phase,'reason':self.mission.reason,'root_cause':self.mission.root_cause,
              'trusted':self.trusted,'trust_reason':self.trust.reason,'preview_ready':self.preview is not None,
              'located':self.located.get('status','') if now-self.located_time<=self.settings['locator']['max_age_s'] else '',
              'turn_deg':self.located.get('turn_deg'),'auto_resume':self.auto_resumable(now),
              'fraction':self.measurement.get('fraction',0.),'owner':owner,'nav_active':self.nav_active,
              'skipped_stretches':sum(b.get('reason')=='PLAN_STRETCH_SKIPPED' for b in self.mission.blocked)}
        self.state_pub.publish(json_msg(data));diagnostic(self,self.diag,'coverage_supervisor',phase,data,int(phase in ('PAUSED','FAILED')))

    def publish_visuals(self):
        markers=[]
        def line(points,name,color):
            m=Marker();m.header.frame_id=self.map_frame;m.header.stamp=self.get_clock().now().to_msg()
            m.ns=name;m.id=0;m.type=Marker.LINE_STRIP;m.action=Marker.ADD;m.pose.orientation.w=1.
            m.scale.x=.025;m.color.r,m.color.g,m.color.b=color;m.color.a=1.
            from geometry_msgs.msg import Point
            m.points=[Point(x=float(p[0]),y=float(p[1]),z=.03) for p in points];markers.append(m)
        line(self.region+self.region[:1],'region',(1.,.7,0.))
        if self.current:line(points_of(self.current),'target',(1.,0.,1.))
        else:line([],'target',(1.,0.,1.))
        text=Marker();text.header.frame_id=self.map_frame;text.ns='status';text.id=0;text.type=Marker.TEXT_VIEW_FACING
        text.pose.orientation.w=1.;text.pose.position.x=self.base.origin[0];text.pose.position.y=self.base.origin[1]
        text.pose.position.z=.3;text.scale.z=.16;text.color.r=text.color.g=text.color.b=text.color.a=1.
        text.text=f'{self.mission.state} {100*self.measurement.get("fraction",0.):.1f}%\n{self.mission.reason or self.trust.reason}'
        markers.append(text);self.markers.publish(MarkerArray(markers=markers))
        if self.preview:
            points=[]
            for t,c in zip(self.preview['targets'],self.preview['connections']):points.extend(c+points_of(t))
            self.route_pub.publish(path_msg(self,points))


def _shutdown_supervisor(node,executor):
    node.manual=False
    node.slot.cancel()
    node.lease_pub.publish(json_msg({'owner':'NONE','epoch':'shutdown','config_hash':node.settings.hash}))
    deadline=time.monotonic()+node.settings['deadlines']['action_cancel_s']
    while rclpy.ok() and not node.slot.idle and time.monotonic()<deadline:
        executor.spin_once(timeout_sec=0.05)
    if not node.slot.idle:
        node.get_logger().error('Action cancellation was not confirmed before shutdown')
        if node.stop_client.service_is_ready():node.stop_client.call_async(Trigger.Request())
    node.save_report()
    node.worker_pool.shutdown(wait=False,cancel_futures=True)


def main(args=None):
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
    node=Supervisor()
    previous={sig:signal.getsignal(sig) for sig in (signal.SIGINT,signal.SIGTERM)}
    def interrupt(_signum,_frame):raise KeyboardInterrupt
    for sig in previous:signal.signal(sig,interrupt)
    executor=None
    try:
        executor=run_until_stopped(node)
    finally:
        node.inputs.close()
        _shutdown_supervisor(node,executor or rclpy.executors.SingleThreadedExecutor(context=node.context))
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for sig,handler in previous.items():signal.signal(sig,handler)
    # Every shutdown step is done (lease released, report saved). End here: Python's own clean-up at exit then crashed
    # the supervisor (exit code -11) on the robot and in the simulation, 2026-10-21, after all of the above.
    sys.stdout.flush();sys.stderr.flush();os._exit(0)
