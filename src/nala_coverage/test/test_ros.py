"""Actual rclpy pub/sub/action tests on loopback in domain 91; never robot inputs."""
import os
os.environ['ROS_DOMAIN_ID']='91'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
os.environ['RMW_IMPLEMENTATION']='rmw_cyclonedds_cpp'
os.environ['CYCLONEDDS_URI']='<CycloneDDS><Domain Id="any"><General><Interfaces><NetworkInterface name="lo" multicast="false"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
import math
import time
import threading
import json
import numpy as np
import pytest
pytest.importorskip('rclpy', reason='ROS 2 Python runtime is not installed')
pytest.importorskip('nav2_msgs', reason='Nav2 messages are not installed')
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer,CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist,TransformStamped,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,OccupancyGrid
from nav2_msgs.action import FollowPath,ComputePathToPose
from sensor_msgs.msg import LaserScan,BatteryState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from nala_coverage.ros_common import LATEST,LATCHED,json_msg,ActionSlot,path_msg
from nala_coverage.gate_node import SafetyGate
from nala_coverage.supervisor import Supervisor
from nala_coverage.meter import CoverageMeter
from nala_coverage.state import Mission
from nala_coverage.params import node_settings


@pytest.fixture
def ros(tmp_path):
    from PIL import Image
    a=np.full((80,80),254,np.uint8);a[[0,-1],:]=0;a[:,[0,-1]]=0
    Image.fromarray(a).save(tmp_path/'map.pgm')
    (tmp_path/'map.yaml').write_text('image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    # The base station next to the map, where the fake robot starts.
    (tmp_path/'base_station.yaml').write_text('frame_id: map\nx: 2.0\ny: 2.0\nyaw: 0.0\n')
    import shutil,yaml
    from nala_coverage.params import default_config_dir
    # All nodes of a test share one Python process (one GIL), so a busy node can delay the others by about a
    # second; on the robot each node is its own process. Allow that here with longer message ages.
    config=tmp_path/'config';shutil.copytree(default_config_dir(),config)
    raw=yaml.safe_load((config/'coverage.yaml').read_text())
    raw['/coverage_config']['ros__parameters']['health'].update(scan_age_s=3.,odom_age_s=3.,gate_age_s=3.,meter_age_s=3.,lease_age_s=3.)
    (config/'coverage.yaml').write_text(yaml.safe_dump(raw))
    rclpy.init(args=['--ros-args','-p','config_dir:='+str(config),'-p','map_yaml:='+str(tmp_path/'map.yaml'),'-p','output_dir:='+str(tmp_path/'reports')])
    executor=MultiThreadedExecutor(num_threads=4);nodes=[]
    def add(node):nodes.append(node);executor.add_node(node);return node
    def pump(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:executor.spin_once(timeout_sec=.01)
    yield add,pump,executor,tmp_path
    executor.shutdown()
    for n in nodes:
        if hasattr(n,'worker_pool'):n.worker_pool.shutdown(wait=True,cancel_futures=True)
        executor.remove_node(n);n.destroy_node()
    rclpy.shutdown()


class FakeInputs(Node):
    def __init__(self):
        super().__init__('isolated_fake_robot')
        self.settings=node_settings(self)
        self.battery=.8;self.x=2.;self.y=2.;self.yaw=0.;self.pose_bad=False
        self.publish_scan=True;self.battery_enabled=True;self.publish_trust=True;self.request_owner='NONE';self.epoch='test'
        self.outputs=[];self.final=(0.,0.)
        types={'scan':LaserScan,'odom':Odometry,'battery_state':BatteryState,'tf':TFMessage,'amcl_pose':PoseWithCovarianceStamped}
        self.pubs={name:self.create_publisher(typ,'/'+name,10) for name,typ in types.items()}
        self.trust=self.create_publisher(String,'/coverage/trust',LATEST)
        self.lease=self.create_publisher(String,'/coverage/lease',LATEST)
        self.nav=self.create_publisher(Twist,'/cmd_vel_nav',LATEST)
        self.map_pub=self.create_publisher(OccupancyGrid,'/map',LATCHED)
        self.create_subscription(Twist,'/cmd_vel',self.command,10)
        # Stands in for Nav2's lifecycle manager: the navigation servers are active.
        self.create_service(Trigger,'/lifecycle_manager_navigation/is_active',self.nav_active)
        self.create_timer(.05,self.tick)
    def nav_active(self,req,res):res.success=True;return res
    def command(self,msg):self.outputs.append((time.monotonic(),msg.linear.x,msg.angular.z));self.final=(msg.linear.x,msg.angular.z)
    def tick(self):
        stamp=self.get_clock().now().to_msg()
        od=Odometry();od.header.stamp=stamp;od.header.frame_id='odom';od.child_frame_id='base_footprint'
        od.pose.pose.position.x=self.x;od.pose.pose.position.y=self.y;od.pose.pose.orientation.w=1.;self.pubs['odom'].publish(od)
        if self.battery_enabled:
            bat=BatteryState();bat.header.stamp=stamp;bat.percentage=self.battery;self.pubs['battery_state'].publish(bat)
        if self.publish_scan:
            scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='laser';scan.angle_min=-math.pi
            scan.angle_increment=2*math.pi/180;scan.range_min=.1;scan.range_max=12.
            # Distance to the walls of the 4 m box (vectorised: a slow fake scan goes stale on a busy PC).
            a=scan.angle_min+np.arange(180)*scan.angle_increment;c,s=np.cos(a),np.sin(a)
            dist=np.full((4,180),np.inf)
            with np.errstate(divide='ignore',invalid='ignore'):
                for row,(boundary,coordinate,velocity) in enumerate([(3.95,self.x,c),(.05,self.x,c),(3.95,self.y,s),(.05,self.y,s)]):
                    d=(boundary-coordinate)/velocity
                    dist[row]=np.where((np.abs(velocity)>1e-9)&(d>0),d,np.inf)
            scan.ranges=dist.min(axis=0).astype(float).tolist();self.pubs['scan'].publish(scan)
        transforms=[]
        for parent,child,x,y in [('map','odom',0.,0.),('odom','base_footprint',self.x,self.y),('base_footprint','base_link',0.,0.),('base_link','laser',0.,0.)]:
            t=TransformStamped();t.header.stamp=stamp;t.header.frame_id=parent;t.child_frame_id=child
            t.transform.translation.x=x;t.transform.translation.y=y;t.transform.rotation.w=1.;transforms.append(t)
        self.pubs['tf'].publish(TFMessage(transforms=transforms))
        am=PoseWithCovarianceStamped();am.header.stamp=stamp;am.header.frame_id='map';am.pose.pose=od.pose.pose
        am.pose.covariance[0]=am.pose.covariance[7]=.0025 if not self.pose_bad else .04
        am.pose.covariance[35]=math.radians(5)**2;self.pubs['amcl_pose'].publish(am)
        if self.publish_trust:self.trust.publish(json_msg({'trusted':True}))
        if self.request_owner is not None:self.lease.publish(json_msg({'config_hash':self.settings.hash,'owner':self.request_owner,'epoch':self.epoch}))
    def publish_map(self,grid):
        m=OccupancyGrid();m.header.frame_id='map';m.info.resolution=grid.resolution
        m.info.height,m.info.width=grid.cells.shape;m.info.origin.orientation.w=1.;m.data=grid.cells.ravel().tolist();self.map_pub.publish(m)


def test_gate_idle_silence_and_nav_timeout(ros):
    add,pump,_,_=ros;gate=add(SafetyGate());fake=add(FakeInputs());pump(.8)
    assert gate.graph_ok and not fake.outputs
    fake.request_owner='NAV';pump(.2)
    for _ in range(8):fake.nav.publish(Twist(linear=__import__('geometry_msgs.msg',fromlist=['Vector3']).Vector3(x=.3)));pump(.06)
    assert any(v>0 for _,v,_ in fake.outputs)
    pump(.8);assert fake.outputs[-1][1:]==(0.,0.)
    count=len(fake.outputs);pump(.3);assert len(fake.outputs)==count


def test_operator_stop_latches_until_reset_without_any_hardware_estop(ros):
    add,pump,_,_=ros;gate=add(SafetyGate());fake=add(FakeInputs());pump(.6)
    res=gate.stop_service(Trigger.Request(),Trigger.Response())
    assert res.success and gate.policy.fault=='OPERATOR_OR_ACTION_STOP'
    fake.request_owner='NAV';pump(.2)
    for _ in range(5):fake.nav.publish(Twist());pump(.06)
    assert not any(v for _,v,_ in fake.outputs)
    assert not gate.reset(Trigger.Request(),Trigger.Response()).success   # owner still NAV
    fake.request_owner='NONE';pump(.3)
    assert gate.reset(Trigger.Request(),Trigger.Response()).success and not gate.policy.fault


def test_nav2_cannot_drive_backwards_through_the_gate(ros):
    add,pump,_,_=ros;gate=add(SafetyGate());fake=add(FakeInputs());pump(.8)
    fake.request_owner='NAV';pump(.2)
    from geometry_msgs.msg import Vector3
    for _ in range(10):fake.nav.publish(Twist(linear=Vector3(x=-.1),angular=Vector3(z=.3)));pump(.06)
    assert fake.outputs and all(v>=0. for _,v,_ in fake.outputs) and any(w>0 for _,_,w in fake.outputs)


def test_gate_dropout_extra_publisher_and_optional_battery(ros):
    add,pump,_,_=ros;gate=add(SafetyGate());fake=add(FakeInputs());pump(.7)
    fake.battery_enabled=False;pump(.3)
    # The MCU's battery is not monitored unless battery.enabled: no battery messages is fine.
    assert not gate.inputs.reason() and gate.inputs.battery() is None
    fake.publish_scan=False;pump(gate.settings['health']['scan_age_s']+.3);assert gate.inputs.reason()=='SCAN_STALE'
    rogue=fake.create_publisher(Twist,'/cmd_vel',10);pump(.6)
    assert not gate.graph_ok
    fake.destroy_publisher(rogue)


def test_supervisor_launch_no_motion_and_continuous_trust(ros):
    add,pump,_,_=ros;gate=add(SafetyGate());sup=add(Supervisor());meter=add(CoverageMeter());fake=add(FakeInputs())
    fake.request_owner=None;fake.publish_trust=False;fake.publish_map(sup.base);pump(3.4)
    assert sup.trusted, sup.trust.reason
    assert sup.mission.state=='IDLE' and gate.policy.owner=='NONE' and not fake.outputs
    fake.pose_bad=True;pump(.3)
    assert not sup.trusted and sup.trust.reason=='POSITION_UNCERTAIN'
    assert not meter.generation


def test_fake_action_late_result_cannot_mutate_new_generation(ros):
    add,pump,executor,_=ros;server_node=add(Node('fake_action_server'));client=add(Node('action_client_test'));m=Mission();m.start(time.monotonic());slot=ActionSlot(client,m)
    entered=threading.Event();release=threading.Event()
    def execute(handle):
        entered.set();release.wait(2.)
        if handle.is_cancel_requested:handle.canceled()
        else:handle.succeed()
        return FollowPath.Result()
    server=ActionServer(server_node,FollowPath,'/follow_path',execute_callback=execute,
                        cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=ReentrantCallbackGroup())
    pump(.4)
    goal=FollowPath.Goal();goal.path=path_msg(client,[[1.,1.],[1.1,1.]])
    slot.send(FollowPath,'/follow_path',goal);pump(.3);assert entered.is_set()
    slot.cancel();m.pause('OPERATOR',time.monotonic());m.change('CANCELED');m.start(time.monotonic())
    release.set();pump(.5)
    assert slot.idle and not slot.events and m.state=='PREPARING'
    server.destroy()


def test_paused_task_keeps_its_total_deadline(ros):
    add,pump,_,_=ros;gate=add(SafetyGate());sup=add(Supervisor());fake=add(FakeInputs());fake.request_owner=None;fake.publish_trust=False
    fake.publish_map(sup.base);pump(3.)
    sup.mission.start(time.monotonic()-1801);sup.mission.state='PAUSED'
    sup.mission.blocked.append({'xy':[1,1],'radius':.25});pump(.3)
    assert sup.mission.state=='FAILED' and sup.mission.reason=='MISSION_TIMEOUT'
    assert sup.mission.blocked and not fake.outputs


def test_meter_rejects_planned_path_and_untrusted_samples(ros):
    add,pump,_,_=ros;meter=add(CoverageMeter());fake=add(Node('meter_test_client'))
    defs=fake.create_publisher(String,'/coverage/definition',LATCHED)
    samples=fake.create_publisher(String,'/coverage/sample',LATEST)
    routes=fake.create_publisher(__import__('nav_msgs.msg',fromlist=['Path']).Path,'/coverage/route',LATCHED)
    defs.publish(json_msg({'generation':'g','config_hash':meter.settings.hash,'map_id':meter.base.identity,'denominator':np.flatnonzero(meter.base.free).tolist(),'keepout':[]}));pump(.3)
    routes.publish(path_msg(fake,[[1.,1.],[3.,3.]]));pump(.2)
    assert not meter.meter.covered.any()
    samples.publish(json_msg({'generation':'g','stamp':fake.get_clock().now().nanoseconds/1e9,'pose':[1.,1.,0.],'trusted':False,'active':True}));pump(.2)
    assert not meter.meter.covered.any()
    samples.publish(json_msg({'generation':'g','stamp':fake.get_clock().now().nanoseconds/1e9,'pose':[1.,1.,0.],'trusted':True,'active':True}));pump(.2)
    assert meter.meter.covered.any()


def test_idle_mapping_requires_explicit_manual_grant(ros):
    from rcl_interfaces.msg import ParameterValue
    # Core gate defaults NONE; no keyboard message alone can claim ownership.
    add,pump,_,_=ros;gate=add(SafetyGate());fake=add(FakeInputs());pump(.7)
    keyboard=fake.create_publisher(Twist,'/cmd_vel_remote',LATEST)
    msg=Twist();msg.linear.x=.25
    for _ in range(4):keyboard.publish(msg);pump(.1)
    assert not fake.outputs and gate.policy.owner=='NONE'


def test_matching_nodes_still_reject_stale_preview(ros):
    add,_,_,_=ros;sup=add(Supervisor())
    sup.ready=lambda **_:None
    sup.gate={'config_hash':sup.settings.hash};sup.meter_config_hash=sup.settings.hash
    sup.preview={'config_hash':'reviewed-older-config'}
    with pytest.raises(ValueError,match='PREVIEW_CONFIG_STALE'):sup.start()
    assert sup.mission.state=='IDLE'


def test_config_mismatch_stops_nav_commands(ros):
    add,pump,_,_=ros;gate=add(SafetyGate());fake=add(FakeInputs());pump(.7)
    fake.request_owner=None;gate.policy.lease('NAV','nav-test',time.monotonic())
    gate.last_lease=time.monotonic();gate.peer_hash='wrong-config';gate.trusted=True;gate.last_trust=time.monotonic()
    gate.policy.receive('NAV',(.1,0.),time.monotonic());gate.tick()
    assert not any(v for _,v,_ in fake.outputs)


def test_closed_boundary_loop_is_followed_not_treated_as_a_point(ros):
    from nala_coverage.planning import poly_target
    add,_,_,_=ros;sup=add(Supervisor());sup.pose=[2.,2.,0.]
    sup.current=poly_target([[2.,2.],[2.2,2.],[2.2,2.2],[2.,2.]],'boundary')
    calls=[];sup.follow=lambda points,state,*_:calls.append((points,state))
    sup.begin_sweep()
    assert calls and calls[0][1]=='SWEEP' and len(calls[0][0])>3


def test_new_meter_generation_discards_previous_phase_acknowledgement(ros):
    add,_,_,_=ros;meter=add(CoverageMeter());meter.sample_seq=500;meter.last_active_sample_seq=500
    meter.definition(json_msg({'generation':'new','config_hash':meter.settings.hash,'map_id':meter.base.identity,
                               'denominator':np.flatnonzero(meter.base.free).tolist(),'keepout':[]}))
    assert meter.generation=='new' and meter.last_active_sample_seq==0 and meter.sample_seq==0


def run_plan(ros,poses,drive=None,block_at=None,base_station=None):
    """Drive an exported plan through fake planner and controller servers.

    FollowPath moves the fake robot along exactly the path it is given. With `block_at` the first goal that
    reaches that point stops there and aborts, like the controller's collision check in front of an obstacle."""
    import yaml
    add,pump,_,tmp_path=ros
    plan=tmp_path/'plan.yaml'
    data={'frame_id':'map','poses':[{'x':x,'y':y,'orientation':{'z':0.,'w':1.}} for x,y in poses],
          'settings':{'geometry':{'sensor_offset_m':.26,'coverage_disk_radius_m':.6}}}
    if drive:data['drive_path']=drive
    if base_station:data['base_station']=dict(zip(('x','y','yaw'),base_station))
    plan.write_text(yaml.safe_dump(data))
    gate=add(SafetyGate());sup=add(Supervisor());meter=add(CoverageMeter());fake=add(FakeInputs())
    fake.request_owner=None;fake.publish_trust=False;fake.publish_map(sup.base);sup.plan_file=str(plan)
    follows=[];plans=[];block=[block_at]
    def planner(handle):
        g=handle.request.goal.pose.position;plans.append([round(g.x,3),round(g.y,3)])
        result=ComputePathToPose.Result();result.path=path_msg(fake,[[fake.x,fake.y],[g.x,g.y]])
        handle.succeed();return result
    def follow(handle):
        pts=[[p.pose.position.x,p.pose.position.y] for p in handle.request.path.poses]
        q=handle.request.path.poses[-1].pose.orientation
        follows.append((gate.policy.owner,pts,math.atan2(2*q.w*q.z,1-2*q.z*q.z)))
        walk=[pts[0]]+[(np.asarray(a)+(np.asarray(b)-a)*i/max(1,math.ceil(math.dist(a,b)/.02))).tolist()
                       for a,b in zip(pts,pts[1:]) for i in range(1,max(1,math.ceil(math.dist(a,b)/.02))+1)]
        for xy in walk:
            if block[0] and math.dist(xy,block[0])<.015:
                block[0]=None;time.sleep(.3);handle.abort();return FollowPath.Result(error_code=106)
            fake.x,fake.y=xy;time.sleep(.02)
        time.sleep(.2);handle.succeed();return FollowPath.Result()
    # The fake Nav2 servers sleep while they 'drive': give them their own node and executor thread, so they
    # never hold the threads that deliver the fake sensors to the supervisor and the gate.
    nav=Node('fake_nav2');nav_executor=MultiThreadedExecutor(num_threads=2);nav_executor.add_node(nav)
    servers=[ActionServer(nav,ComputePathToPose,'/compute_path_to_pose',execute_callback=planner,callback_group=ReentrantCallbackGroup()),
             ActionServer(nav,FollowPath,'/follow_path',execute_callback=follow,
                          cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=ReentrantCallbackGroup())]
    threading.Thread(target=nav_executor.spin,daemon=True).start()
    end=time.monotonic()+15.
    while not (sup.trusted and sup.config_ready()) and time.monotonic()<end:pump(.2)
    assert sup.trusted,sup.trust.reason
    # Built synchronously: the GIL-bound worker is starved by this test's tight executor loop.
    sup.grid=sup.make_grid();sup.preview=sup.plan_preview(sup.grid,sup.pose[:2],sup.region)
    # Sensor messages were not processed while the preview was built; wait until they are fresh again.
    end=time.monotonic()+10.
    while not (sup.trusted and not sup.inputs.reason()) and time.monotonic()<end:pump(.2)
    sup.start()
    end=time.monotonic()+30.
    while sup.mission.state not in ('FINISHED','FAILED','PAUSED') and time.monotonic()<end:pump(.2)
    for server in servers:server.destroy()
    nav_executor.shutdown();nav.destroy_node()
    return sup,follows,plans,plan


def test_older_plan_is_split_at_corners_and_followed_exactly(ros):
    poses=[[2.,2.],[2.6,2.],[2.6,2.6],[2.,2.6]]
    sup,follows,plans,plan=run_plan(ros,poses)
    assert sup.mission.state=='FINISHED' and sup.mission.reason=='RETURNED_TO_BASE',(sup.mission.state,sup.mission.reason,sup.mission.events)
    # The robot starts on the plan (at the base station), so the only planner call is the way back to the
    # base station. One FollowPath goal per piece between stop corners, each on exactly the plan's points and
    # ending with the heading of the next piece (turn on the spot); then the way back.
    assert plans==[[2.,2.]] and all(owner=='NAV' for owner,_,_ in follows)
    # Pieces are followed with a point every grid cell (0.05 m) along the plan's lines.
    assert [[np.round(p[0],3).tolist(),np.round(p[-1],3).tolist(),len(p)] for _,p,_ in follows[:3]]==[[poses[0],poses[1],13],[poses[1],poses[2],13],[poses[2],poses[3],13]]
    assert np.allclose([y for _,_,y in follows[:3]],[math.pi/2,math.pi,math.pi],atol=1e-6)
    # The way back ends at the base station with its heading.
    assert len(follows)==4 and np.allclose(follows[3][1][-1],[2.,2.]) and abs(follows[3][2])<1e-6
    assert any(e.get('reason')=='PLAN_COMPLETE' for e in sup.mission.events)
    report=json.loads((sup.output/('task_'+sup.mission.generation+'.json')).read_text())
    assert report['plan_file']==str(plan) and not report['temporary_blockages']


def test_drivable_path_is_followed_in_one_piece_without_stops(ros):
    # A drivable path with a rounded corner and no stop corners: one goal with all of its points.
    arc=[[2.4+.2*math.sin(a),2.2-.2*math.cos(a)] for a in np.linspace(0,math.pi/2,8)]
    points=[[2.+.1*i,2.] for i in range(4)]+arc+[[2.6,2.3+.1*i] for i in range(3)]
    sup,follows,plans,_=run_plan(ros,[[2.,2.],[3.,3.]],drive={'format':1,'points':points,'stops':[]})
    assert sup.mission.state=='FINISHED' and sup.mission.reason=='RETURNED_TO_BASE',(sup.mission.state,sup.mission.reason,sup.mission.events)
    assert plans==[[2.,2.]] and len(follows)==2
    followed=np.asarray(follows[0][1])
    hits=[int(np.argmin(np.linalg.norm(followed-p,axis=1))) for p in points]
    assert hits==sorted(hits) and all(np.min(np.linalg.norm(followed-p,axis=1))<1e-6 for p in points)


def test_blocked_piece_is_skipped_past_the_obstacle_and_rejoined(ros):
    poses=[[2.,2.],[3.5,2.]]
    sup,follows,plans,_=run_plan(ros,poses,block_at=[2.5,2.])
    assert sup.mission.state=='FINISHED' and sup.mission.reason=='RETURNED_TO_BASE',(sup.mission.state,sup.mission.reason,sup.mission.events)
    # The blocked goal ends at 2.5; 0.5 m of the plan is skipped and the planner drives around to 3.0,
    # from where the rest is followed exactly. Then the planner drives back to the base station.
    assert len(plans)==2 and math.dist(plans[0],[3.,2.])<.06 and plans[1]==[2.,2.],(plans,sup.mission.events)
    rest=follows[-2][1]
    assert math.dist(rest[0],plans[0])<1e-3 and np.allclose(rest[-1],[3.5,2.])
    [b]=sup.mission.blocked
    assert b['reason']=='PLAN_STRETCH_SKIPPED' and math.dist(b['xy'],[2.5,2.])<.06 and abs(b['length_m']-.5)<.06
    assert sup.mission.root_cause.startswith('FOLLOW_PATH_ERROR_106')


def test_pause_mid_piece_resumes_where_the_robot_stopped(ros):
    from nala_coverage.planning import poly_target
    add,_,_,_=ros;sup=add(Supervisor());sup.plan_file='plan.yaml'
    points=[[1.+.05*i,1.] for i in range(31)]
    sup.mission.start(time.monotonic());sup.mission.state='SWEEP'
    sup.current=poly_target(points,'plan');sup.current.end_yaw=1.;sup.sweep_points=points;sup.sweep_index=0
    sup.pose=[1.52,1.03,0.]
    sup.pause('OPERATOR_PAUSE')
    assert sup.mission.state=='PAUSED' and np.allclose(sup.current.points[0],[1.5,1.]) and sup.current.end_yaw==1.
    assert np.allclose(sup.current.points[-1],[2.5,1.])


def test_plan_mode_counts_floor_reachable_with_the_plans_robot_radius(ros):
    import yaml
    add,_,_,tmp_path=ros;sup=add(Supervisor())
    def denominator(**extra):
        plan=tmp_path/'plan.yaml'
        geometry={'sensor_offset_m':.26,'coverage_disk_radius_m':.6,**extra}
        plan.write_text(yaml.safe_dump({'frame_id':'map','poses':[{'x':x,'y':2.,'orientation':{'z':0.,'w':1.}} for x in (1.5,2.5)],
                                        'settings':{'geometry':geometry}}))
        sup.plan_file=str(plan);grid=sup.make_grid()
        preview=sup.plan_preview(grid,[2.,2.],sup.region)
        return preview['denominator'],preview['plan_robot_radius_m']
    own,r_own=denominator()
    planned,r_planned=denominator(robot_radius_m=1.)
    # With a 1.0 m plan radius the centre stays 1.0 m from the walls and the sensor (0.26 m ahead) 0.74 m:
    # the 0.6 m disk no longer reaches the edge cells.
    assert r_own==sup.collision and r_planned==1.
    assert planned.sum()<own.sum() and not (planned&~own).any()


def test_area_plan_counts_only_the_floor_inside_its_area(ros):
    import yaml
    add,_,_,tmp_path=ros;sup=add(Supervisor())
    def denominator(area=None):
        plan=tmp_path/'plan.yaml'
        data={'frame_id':'map','poses':[{'x':x,'y':2.,'orientation':{'z':0.,'w':1.}} for x in (1.5,2.5)],
              'settings':{'geometry':{'sensor_offset_m':.26,'coverage_disk_radius_m':.6}}}
        if area:data['area_polygon']=area
        plan.write_text(yaml.safe_dump(data));sup.plan_file=str(plan)
        return sup.plan_preview(sup.make_grid(),[2.,2.],sup.region)['denominator']
    whole=denominator();half=denominator([[0.,0.],[2.,0.],[2.,4.],[0.,4.]])
    assert 0<half.sum()<whole.sum() and not (half&~whole).any()
    xs=sup.base.world(np.argwhere(half))[:,0]
    assert xs.max()<=2.+1e-6


def test_start_waits_until_nav2_navigation_is_active(ros):
    add,_,_,_=ros;sup=add(Supervisor())
    sup.nav_active=False
    with pytest.raises(ValueError,match='NAV2_NOT_ACTIVE'):sup.ready(require_nav=True)


def test_progress_along_a_piece_restarts_the_target_deadline(ros):
    add,_,_,_=ros;sup=add(Supervisor());sup.plan_file='plan.yaml'
    # A loop that passes close to its own start: progress is only searched forward along the piece.
    loop=[[1.+.05*i,1.] for i in range(21)]+[[2.,1.+.05*i] for i in range(1,11)]+[[2.-.05*i,1.5] for i in range(1,19)]+[[1.1,1.5-.05*i] for i in range(1,9)]
    sup.sweep_points=loop;sup.sweep_index=0;sup.target_since=0.
    sup.pose=[1.5,1.,0.];sup.update_sweep_index()
    assert sup.sweep_index==10 and sup.target_since>0.
    sup.pose=[1.12,1.12,0.]  # back near the start, but at the end of the loop: not a jump back
    for _ in range(10):sup.update_sweep_index()
    assert sup.sweep_index>=len(loop)-3


def test_repeated_failures_in_place_pause_instead_of_skipping_the_plan(ros):
    from nala_coverage.planning import poly_target
    add,_,_,_=ros;sup=add(Supervisor());sup.plan_file='plan.yaml'
    points=[[1.+.05*i,1.] for i in range(61)]
    sup.mission.start(time.monotonic());sup.pose=[1.,1.,0.]
    sup.targets=[poly_target(points,'plan')];sup.next_target=lambda:None
    for attempt in range(20):
        if sup.mission.state=='PAUSED':break
        sup.current=sup.targets.pop(0) if sup.targets else sup.current;sup.sweep_points=None
        sup.mission.state='CONNECT_PLAN';sup.plan_failure('NAVIGATION_FAILED')
    assert sup.mission.state=='PAUSED' and sup.mission.reason.startswith('BLOCKED_IN_PLACE')
    # It stopped skipping once blocked_in_place_skip_m (2 m) of the 2 m piece was gone... or before.
    skipped=sum(b['length_m'] for b in sup.mission.blocked)
    assert skipped<=sup.settings['recovery']['blocked_in_place_skip_m']+.5
    # What was not skipped yet is kept for resume.
    assert np.allclose(points_of_rest(sup)[-1],[4.,1.]) and len(points_of_rest(sup))>2


def points_of_rest(sup):
    from nala_coverage.planning import points_of
    return points_of(sup.current)


def test_supervisor_sends_the_base_station_to_amcl_until_it_answers(ros):
    add,pump,_,_=ros;sup=add(Supervisor());listener=add(Node('initial_pose_listener'))
    got=[];listener.create_subscription(PoseWithCovarianceStamped,'/initialpose',got.append,10)
    pump(.5);sup.publish_initial_pose();pump(.5)
    assert got and got[-1].header.frame_id=='map'
    p=got[-1].pose
    assert (p.pose.position.x,p.pose.position.y)==(2.,2.) and p.covariance[0]==pytest.approx(sup.settings['localization']['base_station_sigma_m']**2)
    sup.amcl=PoseWithCovarianceStamped();count=len(got);sup.publish_initial_pose();pump(.3)
    assert len(got)==count


def test_plan_starting_away_from_the_base_station_is_driven_to_first(ros):
    # The robot stands on the base station (2, 2); coverage starts at (2.6, 2.0).
    poses=[[2.6,2.],[3.2,2.]]
    sup,follows,plans,_=run_plan(ros,poses,base_station=(2.,2.,0.))
    assert sup.mission.state=='FINISHED' and sup.mission.reason=='RETURNED_TO_BASE',(sup.mission.state,sup.mission.reason,sup.mission.events)
    # Planner to the start, then the plan, then the planner back to the base station.
    assert plans==[[2.6,2.],[2.,2.]],plans
    assert np.allclose(follows[1][1][0],poses[0]) and np.allclose(follows[1][1][-1],poses[1])
    assert np.allclose(follows[-1][1][-1],[2.,2.])


def test_failed_return_pauses_and_resume_drives_back_again(ros):
    add,_,_,_=ros;sup=add(Supervisor());sup.plan_file='plan.yaml'
    sup.mission.start(time.monotonic());sup.pose=[3.,3.,0.]
    calls=[];sup.plan_to=lambda xy,state,heading=None:(calls.append((list(xy),state,heading)),setattr(sup.mission,'state',state))
    sup.begin_return()
    assert calls==[([2.,2.],'RETURN_PLAN',0.)] and sup.returning
    sup.target_failure('NO_MOVEMENT')
    assert sup.mission.state=='PAUSED' and sup.mission.reason=='RETURN_FAILED: NO_MOVEMENT'
    # Resume goes back to the return, not to the (finished) plan.
    sup.ready=lambda **_:None;sup.check_start_component=lambda:None
    sup.resume()
    assert sup.pending_phase=='RETURN' and not sup.mission.coverage_finished


def test_plan_for_another_base_station_is_refused(ros):
    import yaml
    add,_,_,tmp_path=ros;sup=add(Supervisor())
    plan=tmp_path/'plan.yaml'
    plan.write_text(yaml.safe_dump({'frame_id':'map','poses':[{'x':x,'y':2.,'orientation':{'z':0.,'w':1.}} for x in (2.5,3.)],
                                    'base_station':{'x':2.5,'y':2.,'yaw':0.},
                                    'settings':{'geometry':{'sensor_offset_m':.26,'coverage_disk_radius_m':.6}}}))
    sup.plan_file=str(plan)
    with pytest.raises(ValueError,match='PLAN_BASE_MISMATCH'):sup.plan_preview(sup.make_grid(),[2.,2.],sup.region)
