"""Mission supervisor: drives an exported coverage_tool plan with Nav2 from the base station and back.
Explicit commands, bounded actions, no automatic launch motion. Autodrive is forward only: every piece is
followed forwards and the robot turns on the spot between pieces. After the plan the Nav2 planner drives the
robot back to the base station (RETURN_PLAN, RETURN)."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import signal
import json
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
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
from .plan_file import check_base_station, load_plan
from .ros_common import (Inputs, ActionSlot, LATCHED, LATEST, decode, json_msg,
                         diagnostic, pose_msg, path_msg, pose3, transform3, yaw, ros_now, stamp_seconds)

ACTIVE={'CONNECT_PLAN','CONNECT','SWEEP','DWELL','PHASE_WAIT','RETURN_PLAN','RETURN','TEST_PLAN','TEST'}
DRIVING=('CONNECT','SWEEP','RETURN','TEST')


class Supervisor(Node):
    def __init__(self):
        super().__init__('coverage_supervisor')
        self.settings=node_settings(self);cfg=self.settings
        # A reviewed coverage_tool export is what the robot drives.
        self.plan_file=self.declare_parameter('plan_file','').value
        self.map_frame=cfg['runtime']['map_frame'];self.odom_frame=cfg['runtime']['odom_frame'];self.base_frame=cfg['runtime']['base_frame']
        self.base=Grid.load(cfg['runtime']['map_yaml']);self.grid=self.base
        # Coverage starts and ends at the base station.
        self.station_file=cfg['runtime']['base_station_yaml'] or str(base_station.default_path(cfg['runtime']['map_yaml']))
        self.station=base_station.load(self.station_file,self.map_frame)
        base_station.check(self.base,self.station,cfg.collision)
        self.returning=False
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
        self.create_subscription(String,'/coverage/meter_config',self.meter_config_received,LATCHED)
        self.create_subscription(PoseStamped,'/coverage/test_goal',self.goal_received,10)
        self.create_service(SetBool,'/control/manual',self.manual_service)
        for name,fn in [('preview',self.preview_service),('start',self.start),
                        ('pause',self.pause_service),('resume',self.resume),('cancel',self.cancel_service),
                        ('test_navigation',self.test_navigation)]:
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

    def amcl_pose(self,msg):self.amcl=msg;self.amcl_received=time.monotonic()

    def publish_initial_pose(self):
        """Tell AMCL the robot is on the base station, until AMCL reports a pose. An operator can still correct
        it with RViz's 2D Pose Estimate."""
        if self.amcl is not None:self.initial_timer.cancel();return
        loc=self.settings['localization'];msg=PoseWithCovarianceStamped()
        msg.header.frame_id=self.map_frame;msg.header.stamp=self.get_clock().now().to_msg()
        msg.pose.pose=pose_msg(self,self.station[:2],self.station[2]).pose
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
            pose=transform3(self.inputs.lookup(self.map_frame,self.base_frame,stamp))
            odom=transform3(self.inputs.lookup(self.odom_frame,self.base_frame,stamp))
            scan=self.inputs.messages.get('scan')
            if scan is not None and self.scan_stamp!=stamp_seconds(scan.header.stamp):
                laser=transform3(self.inputs.lookup(self.map_frame,scan.header.frame_id,scan.header.stamp))
                ranges=np.asarray(scan.ranges);ids=np.flatnonzero(np.isfinite(ranges)&(ranges>=max(self.settings['localization']['min_scan_range_m'],scan.range_min))&(ranges<min(self.settings['localization']['max_scan_range_m'],scan.range_max)))
                if len(ids)>self.settings['localization']['max_endpoints']:ids=ids[np.linspace(0,len(ids)-1,self.settings['localization']['max_endpoints']).astype(int)]
                angles=scan.angle_min+ids*scan.angle_increment+laser[2]
                endpoints=np.column_stack((laser[0]+ranges[ids]*np.cos(angles),laser[1]+ranges[ids]*np.sin(angles)))
                obstacles=self.base.world(np.argwhere(self.base.cells==100))
                matched=0
                for p in endpoints:
                    if len(obstacles) and np.min(np.sum((obstacles-p)**2,axis=1))<=self.settings['localization']['match_distance_m']**2:matched+=1
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
            self.sample_pub.publish(json_msg({'generation':self.mission.generation,'sample_seq':self.sample_seq,'stamp':ros_now(self)-self.settings['localization']['tf_delay_s'],
                'pose':pose or [0.,0.,0.],'trusted':self.trusted,'active':self.mission.state in ('CONNECT','SWEEP','DWELL','PHASE_WAIT') and not self.manual}))

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
                       sensor=(self.offset,self.radius))
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
        return {'reachable':reachable,'denominator':grid.coverable(grid.sensor_reachable(basis,self.offset),polygon,self.radius),
                'targets':plan['targets'],'connections':[[] for _ in plan['targets']],'start':start,
                'config_hash':self.settings.hash,'plan_file':str(self.plan_file),'plan_sha256':plan['plan_sha256'],
                'plan_robot_radius_m':plan_radius}

    def start(self):
        self.ready(require_nav=True)
        if self.preview is None or self.work is not None:raise ValueError('A completed preview is required')
        if self.preview.get('config_hash')!=self.settings.hash:raise ValueError('PREVIEW_CONFIG_STALE: regenerate preview')
        if not self.config_ready():raise ValueError('CONFIG_HASH_MISMATCH')
        battery=self.inputs.battery()
        if battery is not None and battery<self.settings['battery']['start_ratio']:raise ValueError(f"Start requires battery >= {100*self.settings['battery']['start_ratio']:g}%")
        self.check_start_component()
        self.mission.start(time.monotonic(),self.timeout)
        self.definition={'generation':self.mission.generation,'map_id':self.base.identity,
                         'config_hash':self.settings.hash,'denominator':np.flatnonzero(self.preview['denominator']).tolist()}
        self.definition_pub.publish(json_msg(self.definition));self.measurement={};self.meter_time=0.
        self.targets=list(self.preview['targets']);self.current=None;self.pending_phase='NEXT';self.stuck=None;self.returning=False
        self.phase_since=time.monotonic();self.manual=False
        return 'Task accepted; preparation checks precede motion'

    def check_start_component(self):
        try:
            rc,_,_=self.grid.project_start(self.pose[:2],self.collision,self.settings['recovery']['planner_start_tolerance_m'])
        except ValueError as error:
            raise ValueError('Actual pose exceeds planner_start_tolerance_m') from error
        if not self.preview['reachable'][rc]:
            raise ValueError('Actual pose belongs to a different reachable component')

    def pause(self,reason):
        if self.mission.state not in ACTIVE and self.mission.state!='PREPARING':return
        if self.current and self.mission.state not in ('TEST_PLAN','TEST'):
            # Resume continues where the robot stopped on the piece.
            rest=poly_target(self.plan_remainder(),'plan');rest.end_yaw=getattr(self.current,'end_yaw',None)
            self.current=rest;self.sweep_points=None
        self.slot.cancel();self.pending_phase=None;self.mission.pause(reason,time.monotonic())
        self.save_report()

    def pause_service(self):self.pause('OPERATOR_PAUSE');return 'Paused; stopping/cancellation is checked independently'

    def resume(self):
        self.ready(require_nav=True)
        if self.work is not None:raise ValueError('Previous planning worker is still finishing')
        if self.mission.state!='PAUSED':raise ValueError('Task is not paused')
        if self.mission.expired(time.monotonic()):raise ValueError('Mission deadline expired; start a new task')
        if self.mission.coverage_finished:raise ValueError('Nothing left to resume; start a new task')
        self.check_start_component()
        self.mission.resume();self.stuck=None
        if self.current:self.targets.insert(0,self.current)
        self.current=None;self.pending_phase='RETURN' if self.returning else 'NEXT'
        self.phase_since=time.monotonic();return 'Resume accepted; original mission deadline kept'

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
        self.path_end_yaw=heading;self.mission.state=state;self.phase_since=time.monotonic()
        self.slot.send(ComputePathToPose,'/compute_path_to_pose',goal)

    def follow(self,points,state,end_yaw=None,feedback=None):
        if len(points)<2:raise ValueError('Controller path has fewer than two poses')
        for a,b in zip(points,points[1:]):
            if not self.grid.segment_safe(a,b,self.collision):raise ValueError('Unsafe continuous path segment')
        self.path=points;goal=FollowPath.Goal();goal.path=path_msg(self,points,end_yaw)
        goal.controller_id='FollowPath';goal.goal_checker_id='goal_checker';goal.progress_checker_id='progress_checker'
        self.mission.state=state;self.phase_since=time.monotonic();self.motion_pose=list(self.pose);self.last_motion=time.monotonic()
        self.slot.send(FollowPath,'/follow_path',goal,feedback)

    def next_target(self):
        self.current=self.targets.pop(0) if self.targets else None;self.sweep_points=None
        if self.current is None:self.finish_pass();return
        self.target_since=time.monotonic()
        pts=points_of(self.current)
        # The robot normally stands where the previous piece ended; after a detour it is driven to the rest.
        if not getattr(self.current,'connect',False) and math.dist(self.pose[:2],pts[0])<=self.settings['planning']['plan_join_tolerance_m']:
            self.begin_sweep();return
        heading=math.atan2(pts[1][1]-pts[0][1],pts[1][0]-pts[0][0]) if len(pts)>1 else None
        self.plan_to(pts[0],'CONNECT_PLAN',heading)

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
            n=poly_target(rest,'plan');n.end_yaw=getattr(t,'end_yaw',None);n.connect=True
            self.targets.insert(0,n)
        self.mission.events.append(event)
        self.current=None;self.sweep_points=None;self.mission.state='PREPARING';self.pending_phase='NEXT';self.phase_since=now

    def begin_sweep(self):
        t=self.current
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
            if math.dist(self.pose[:2],self.path[-1])>self.settings['recovery']['endpoint_tolerance_m']:
                success=False;error='ACTION_SUCCEEDED_WITHOUT_REACHING_ENDPOINT'
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
            try:self.follow(points,{'CONNECT_PLAN':'CONNECT','RETURN_PLAN':'RETURN','TEST_PLAN':'TEST'}[phase],self.path_end_yaw)
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
        now=time.monotonic()
        self.update_trust(now)
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
            if phase in ACTIVE:
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
                elif phase in DRIVING:
                    if self.motion_pose is None or math.dist(self.pose[:2],self.motion_pose[:2])>=self.settings['recovery']['movement_translation_m'] or abs(wrap(self.pose[2]-self.motion_pose[2]))>=self.settings['recovery']['movement_rotation_rad']:
                        self.motion_pose=list(self.pose);self.last_motion=now
                        # A long connection to a plan piece is not cut off while the robot is moving.
                        if phase in ('CONNECT','RETURN'):self.target_since=now
                    if now-self.last_motion>=self.settings['deadlines']['no_motion_s']:
                        if phase in ('CONNECT','SWEEP','RETURN'):self.target_failure('NO_MOVEMENT')
                        else:self.pause('NO_MOVEMENT')
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


def _shutdown_supervisor(node):
    node.manual=False
    node.slot.cancel()
    node.lease_pub.publish(json_msg({'owner':'NONE','epoch':'shutdown','config_hash':node.settings.hash}))
    deadline=time.monotonic()+node.settings['deadlines']['action_cancel_s']
    while rclpy.ok() and not node.slot.idle and time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=0.05)
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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        _shutdown_supervisor(node)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for sig,handler in previous.items():signal.signal(sig,handler)
