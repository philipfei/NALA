import hashlib
import math
from pathlib import Path
import numpy as np
import pytest
from nala_coverage.geometry import Grid,validate_polygon
from nala_coverage.measurement import Meter
from nala_coverage.state import Mission
from nala_coverage.health import Trust,Ownership


def room(size=30,res=.1,origin=(0.,0.,0.)):
    a=np.zeros((size,size),np.int8);a[[0,-1],:]=100;a[:,[0,-1]]=100
    return Grid.from_cells(a,res,origin)


def test_map_threshold_and_image_y(tmp_path):
    from PIL import Image
    Image.fromarray(np.array([[0,205],[254,255]],np.uint8)).save(tmp_path/'map.pgm')
    (tmp_path/'map.yaml').write_text('image: map.pgm\nresolution: 0.05\norigin: [1, 2, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n')
    g=Grid.load(tmp_path/'map.yaml')
    assert g.cells.tolist()==[[0,0],[100,-1]]
    assert g.world((0,0))==pytest.approx([1.025,2.025])


@pytest.mark.parametrize('angle',[0.,.2,math.pi/2,-math.pi/3])
def test_world_roundtrip_rotated(angle):
    g=room(origin=(-1.,-4.,angle))
    for rc in [(0,0),(10,20),(29,29)]:assert g.cell(g.world(rc))==rc


def test_exact_capsule_corner_collision():
    g=room();g.cells[10,10]=100
    # Endpoints individually clear, but diagonal segment grazes the obstacle corner.
    a=[.85,1.15];b=[1.15,.85]
    assert g.segment_safe(a,a,.1) and g.segment_safe(b,b,.1)
    assert not g.segment_safe(a,b,.1)
    assert not g.segment_safe([.11,.5],[.11,2.],.20)


def test_unknown_blocks_and_passage_clearance():
    g=room();g.cells[:,15]=100;g.cells[13:17,15]=0
    assert not g.reachable([.8,1.5],.20)[:,20].any()  # 0.4m opening is not strict clearance
    g.cells[12:18,15]=0
    assert g.reachable([.8,1.5],.20)[:,20].any()
    g.cells[12:18,15]=-1
    assert not g.reachable([.8,1.5],.20)[:,20].any()


def test_polygon_rejects_bowtie_and_zero_area():
    for p in [[[0,0],[1,1],[0,1],[1,0]],[[0,0],[1,0],[2,0]],[[0,0],[0,0],[1,1]]]:
        with pytest.raises(ValueError):validate_polygon(p)
    assert len(validate_polygon([[0,0],[1,0],[1,1],[0,0]]))==3


def test_denominator_is_not_robot_centre_area_or_box():
    g=room();reach=g.reachable([1.,1.],.20);roi=[[0,0],[3,0],[3,3],[0,3]]
    den=g.coverable(reach,roi,.25)
    assert den.sum()>reach.sum()
    assert den.sum()<=g.free.sum()<g.cells.size
    m=Meter(g,den);assert not m.denominator.flags.writeable


def test_disk_never_paints_through_wall():
    g=room(res=.05);g.cells[:,15]=100
    disk=g.disk(g.world((15,14)),.25)
    assert not disk[:,16:].any()


def test_actual_meter_not_planned_goal():
    g=room(res=.05);den=g.free;meter=Meter(g,den)
    assert meter.report()['covered_m2']==0
    meter.sample([.4,.4,0],1.)
    expected=g.disk([.4,.4],.25)&den
    assert np.array_equal(meter.covered,expected)


def test_sampling_gap_does_not_bridge():
    g=room(80,.05);m=Meter(g,g.free)
    m.sample([.5,.5,0],1.);m.sample([2.,.5,0],20.)
    assert not m.covered[g.cell([1.2,.5])]
    before=m.covered.copy();m.sample([3.,3.,0],20.1)
    assert np.array_equal(m.covered,before)  # jump paints neither bridge nor endpoint


def test_untrusted_manual_and_reversed_time_no_paint():
    g=room();m=Meter(g,g.free)
    m.sample([1.,1.,0],10.,False);m.sample([1.,1.,0],11.,True,False)
    assert not m.covered.any()
    m.sample([1.,1.,0],12.);before=m.covered.copy();m.sample([2.,2.,0],11.)
    assert np.array_equal(before,m.covered)


def test_mission_generation_deadline_and_late_callback():
    m=Mission();m.start(100.);key=m.action_id()
    m.failures['plan:1.00,1.00']=2;m.blocked.append({'xy':[1,1],'reason':'PLAN_STRETCH_SKIPPED'})
    m.pause('OPERATOR',103);assert not m.current(key)
    generation=m.generation;m.resume()
    assert m.generation==generation and m.blocked and m.failures
    assert m.expired(1900.)
    m.change('FAILED','MISSION_TIMEOUT');m.start(2000.)
    assert m.generation!=generation and not m.blocked and not m.failures


def test_autodrive_is_forward_only_and_manual_may_reverse():
    p=Ownership();p.lease('NAV','a',0);p.receive('NAV',(-.1,.3),0)
    for i in range(10):
        p.lease('NAV','a',i*.05);p.receive('NAV',(-.1,.3),i*.05);v=p.tick(i*.05,True,False)
        assert v[0]==0. and v[1]>0         # Nav2 asked to reverse: only the turn passes
    p.lease('MANUAL','b',1);p.receive('MANUAL',(-.1,0.),1)
    for i in range(10):
        p.lease('MANUAL','b',1+i*.05);p.receive('MANUAL',(-.1,0.),1+i*.05);v=p.tick(1+i*.05,True,False)
    assert v[0]<0                           # manual driving may reverse


def test_command_timeout_stops():
    p=Ownership();p.lease('NAV','a',0);p.receive('NAV',(.12,0),0)
    assert p.tick(0,True,False)[0]>0
    assert p.tick(1,True,True)==(0.,0.)
    assert p.tick(1.1,True,True) is None


def test_meter_paints_at_the_sensor_ahead_of_the_robot():
    g=room(80,.05);m=Meter(g,g.free,radius=.3,offset=.26)
    m.sample([1.,2.,0.],1.)
    assert np.array_equal(m.covered,g.disk([1.26,2.],.3)&g.free)
    assert not m.covered[g.cell([.72,2.])] and m.covered[g.cell([1.5,2.])]
    # Turning on the spot swings the sensor around the robot centre and paints the arc.
    m.sample([1.,2.,.4],2.);m.sample([1.,2.,.8],3.);m.sample([1.,2.,1.2],4.);m.sample([1.,2.,1.57],5.)
    assert m.covered[g.cell([1.,2.5])]


def test_sensor_reachable_reaches_offset_ahead_of_any_centre():
    g=room(80,.05);reach=np.zeros(g.cells.shape,bool);reach[g.cell([2.,2.])]=True
    s=g.sensor_reachable(reach,.26)
    assert s[g.cell([2.25,2.])] and s[g.cell([2.,1.75])] and not s[g.cell([2.35,2.])]
    assert np.array_equal(g.sensor_reachable(reach,0.),reach)


def good_cov():
    c=np.zeros((6,6));c[0,0]=c[1,1]=.05**2;c[5,5]=math.radians(5)**2
    return c.ravel().tolist()


def test_trust_continuous_same_thresholds_reacquisition():
    t=Trust();c=good_cov()
    for now in [0.,1.,2.]:ready=t.check(now,[1,1,0],[0,0,0],c,100,65,.1)
    assert ready
    assert not t.check(2.1,[1,1,0],[0,0,0],c,100,64,.1)
    assert t.reason=='SCAN_MAP_MISMATCH'
    assert not t.check(2.2,[1,1,0],[0,0,0],c,100,65,.1)
    assert t.check(4.3,[1,1,0],[0,0,0],c,100,65,.1)


@pytest.mark.parametrize('problem',['position','heading','endpoints','age','jump'])
def test_runtime_trust_loss(problem):
    t=Trust();c=good_cov();args=[3.,[1,1,0],[0,0,0],c,100,100,.1]
    t.check(0.,[1,1,0],[0,0,0],c,100,100,.1)
    if problem=='position':args[3][0]=.101**2
    if problem=='heading':args[3][35]=math.radians(10.1)**2
    if problem=='endpoints':args[4]=59;args[5]=59
    if problem=='age':args[6]=2.01
    if problem=='jump':args[1]=[1.101,1,0]
    assert not t.check(*args)


def test_gate_idle_silence_and_handoff():
    p=Ownership();assert p.tick(0,True,True) is None
    p.lease('NAV','a',1);p.receive('NAV',(.5,2.),1)
    for i in range(20):p.lease('NAV','a',1+i*.05);p.receive('NAV',(.5,2),1+i*.05);v=p.tick(1+i*.05,True,False)
    assert v[0]<=.12 and v[1]<=.40
    p.lease('NONE','b',2);assert p.command is None
    assert p.tick(2,True,False)==(0.,0.)   # the hand-off stops the robot first...
    assert p.tick(2.5,True,True)==(0.,0.) and p.tick(2.6,True,True) is None   # ...then goes silent
    p.lease('NAV','c',3);assert p.tick(3,True,True) is None


def test_trajectory_never_bridges_untrusted_interval_or_sampling_gap():
    g=room();m=Meter(g,g.free)
    m.sample([1.,1.,0.],1.)
    m.sample([1.01,1.,0.],1.1)
    assert len(m.trajectory_edges)==1
    m.sample([1.02,1.,0.],1.15,trusted=False)
    m.sample([1.03,1.,0.],1.2)
    assert len(m.trajectory_edges)==1 and m.trajectory_segment_start==2
    m.sample([1.04,1.,0.],2.)
    assert len(m.trajectory_edges)==1 and m.trajectory_segment_start==3
