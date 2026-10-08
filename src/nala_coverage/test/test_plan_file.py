import hashlib
import math
import numpy as np
import pytest
import yaml
from PIL import Image
from nala_coverage.geometry import Grid
from nala_coverage.plan_file import load_plan, map_image_sha256


@pytest.fixture
def room_map(tmp_path):
    a=np.full((80,80),254,np.uint8);a[[0,-1],:]=0;a[:,[0,-1]]=0
    Image.fromarray(a).save(tmp_path/'map.pgm')
    (tmp_path/'map.yaml').write_text('image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    return tmp_path/'map.yaml'


def write_plan(tmp_path,points,**extra):
    poses=[{'x':x,'y':y,'yaw':0.,'orientation':{'z':0.,'w':1.},'kind':'lane'} for x,y in points]
    path=tmp_path/'plan.yaml';path.write_text(yaml.safe_dump({'frame_id':'map','poses':poses,**extra}))
    return path


def test_safe_plan_is_split_into_ordered_targets(tmp_path,room_map):
    grid=Grid.load(room_map)
    path=write_plan(tmp_path,[[1.,1.],[1.,1.],[3.,1.],[3.,3.]],map_image_sha256=map_image_sha256(room_map))
    plan=load_plan(path,grid,room_map,'map',.2)
    assert plan['points']==[[1.,1.],[3.,1.],[3.,3.]] and plan['stops']==[1] and not plan['drivable']
    # An older plan without a drivable path stops at every corner of 20 degrees or more.
    first,second=plan['targets']
    assert first.points==[[1.,1.],[3.,1.]] and abs(first.end_yaw-math.pi/2)<1e-9
    assert second.points==[[3.,1.],[3.,3.]] and second.end_yaw is None
    assert all(t.kind=='plan' for t in plan['targets'])
    assert plan['plan_sha256']==hashlib.sha256(path.read_bytes()).hexdigest()


def test_drivable_path_is_used_and_split_at_its_stops(tmp_path,room_map):
    grid=Grid.load(room_map)
    points=[[1.+.1*i,1.] for i in range(11)]+[[2.,1.+.1*i] for i in range(1,11)]
    path=write_plan(tmp_path,[[1.,1.],[3.,3.]],drive_path={'format':1,'points':points,'stops':[10]})
    plan=load_plan(path,grid,room_map,'map',.2)
    assert plan['drivable'] and plan['points']==points and plan['stops']==[10]
    first,second=plan['targets']
    assert first.points==points[:11] and abs(first.end_yaw-math.pi/2)<1e-9 and second.points==points[10:]
    # Without stops it is one piece, followed exactly from start to end.
    path=write_plan(tmp_path,[[1.,1.],[3.,3.]],drive_path={'format':1,'points':points,'stops':[]})
    [only]=load_plan(path,grid,room_map,'map',.2)['targets']
    assert only.points==points


def test_stop_corners_ignore_gentle_bends():
    from nala_coverage.plan_file import stop_corners
    pts=[[0.,0.],[1.,0.],[2.,.1],[2.,1.],[3.,1.]]
    assert stop_corners(pts)==[2,3]


def test_plan_hugging_a_wall_is_rejected(tmp_path,room_map):
    grid=Grid.load(room_map)
    with pytest.raises(ValueError,match='PLAN_TOO_CLOSE_TO_OBSTACLES: 1 of 2'):
        load_plan(write_plan(tmp_path,[[1.,1.],[2.,1.],[2.,.2]]),grid,room_map,'map',.2)


def test_plan_from_another_map_or_frame_is_rejected(tmp_path,room_map):
    grid=Grid.load(room_map)
    with pytest.raises(ValueError,match='PLAN_MAP_MISMATCH'):
        load_plan(write_plan(tmp_path,[[1.,1.],[2.,1.]],map_image_sha256='0'*64),grid,room_map,'map',.2)
    with pytest.raises(ValueError,match='PLAN_FRAME_MISMATCH'):
        load_plan(write_plan(tmp_path,[[1.,1.],[2.,1.]],frame_id='odom'),grid,room_map,'map',.2)
    with pytest.raises(ValueError,match='fewer than two'):
        load_plan(write_plan(tmp_path,[[1.,1.]]),grid,room_map,'map',.2)


def test_densify_keeps_every_vertex_and_bounds_spacing():
    from nala_coverage.planning import densify
    points=[[0.,0.],[.25,0.],[.25,.05],[1.,.05]]
    dense=densify(points,.1)
    gaps=np.linalg.norm(np.diff(dense,axis=0),axis=1)
    assert all(any(np.allclose(p,q) for q in dense) for p in points) and np.allclose(dense[-1],points[-1])
    assert gaps.max()<=.1+1e-9 and gaps.min()>0 and len(dense)==1+3+1+8
    assert densify(dense,.1)==dense  # A retry re-densifies the unreached rest.


def test_plan_robot_radius_is_read_from_the_export(tmp_path,room_map):
    grid=Grid.load(room_map)
    def radius(**extra):return load_plan(write_plan(tmp_path,[[1.,1.],[2.,1.]],**extra),grid,room_map,'map',.2)['robot_radius']
    assert radius(settings={'geometry':{'robot_radius_m':.23}})==.23
    assert radius() is None and radius(settings={'geometry':{'robot_radius_m':-1}}) is None


def test_plan_made_for_another_sensor_is_rejected(tmp_path,room_map):
    grid=Grid.load(room_map)
    def load(**geometry):
        return load_plan(write_plan(tmp_path,[[1.,1.],[2.,1.]],settings={'geometry':geometry}),grid,room_map,'map',.2,
                         sensor=(.26,.6))
    assert load(sensor_offset_m=.26,coverage_disk_radius_m=.6)['points']==[[1.,1.],[2.,1.]]
    with pytest.raises(ValueError,match='PLAN_ROBOT_MISMATCH: .*sensor_offset_m'):
        load(coverage_disk_radius_m=.6)              # a SIMBA-style plan: sensor at the robot centre
    with pytest.raises(ValueError,match='PLAN_ROBOT_MISMATCH: .*coverage_disk_radius_m'):
        load(sensor_offset_m=.26,coverage_disk_radius_m=.4)
