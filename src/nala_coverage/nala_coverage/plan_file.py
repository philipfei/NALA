"""Exported coverage_tool plans: load, check against the loaded map, split into targets."""
from pathlib import Path
import hashlib
import json
import math
import yaml
from .planning import poly_target


def map_image_sha256(map_yaml):
    map_yaml=Path(map_yaml).resolve()
    return hashlib.sha256((map_yaml.parent/yaml.safe_load(map_yaml.read_text())['image']).read_bytes()).hexdigest()


def _heading(a,b):return math.atan2(b[1]-a[1],b[0]-a[0])


def stop_corners(points,min_deg=20.):
    """Corners of an older plan without a drivable path where the robot has to stop and turn."""
    out=[]
    for i in range(1,len(points)-1):
        turn=abs(math.remainder(_heading(points[i],points[i+1])-_heading(points[i-1],points[i]),2*math.pi))
        if math.degrees(turn)>=min_deg:out.append(i)
    return out


def pieces(points,stops):
    """The path split at its stop corners; each piece ends with the heading of the next one (turn on the spot there)."""
    cuts=[0]+sorted(i for i in set(stops) if 0<i<len(points)-1)+[len(points)-1]
    out=[]
    for a,b in zip(cuts,cuts[1:]):
        t=poly_target([list(p) for p in points[a:b+1]],'plan')
        t.end_yaw=_heading(points[b],points[b+1]) if b<len(points)-1 else None
        out.append(t)
    return out


def check_base_station(plan,station,tolerance):
    """Refuse a plan made for another base station (PLAN_BASE_MISMATCH). Plans without one are accepted."""
    if plan['base_station'] is not None and math.dist(plan['base_station'][:2],station[:2])>tolerance:
        raise ValueError(f"PLAN_BASE_MISMATCH: the plan was made for base station {plan['base_station'][:2]}, "
                         f'the base station is now {list(station[:2])}; plan again')


def load_plan(path,grid,map_yaml,map_frame,collision,sensor=None):
    """Return the plan's driven path and its pieces between stop corners, or raise ValueError naming the problem.

    A coverage_tool export with `drive_path` gives the exact path to follow (corners rounded where that
    keeps every cell) and the corners where the robot stops and turns. For an older export the poses are
    joined by straight lines and every corner of 20 degrees or more is a stop.

    `sensor` = (offset, radius) of this robot's coverage sensor: a plan made for another sensor covers
    different floor, so it is rejected (PLAN_ROBOT_MISMATCH)."""
    path=Path(path)
    if not path.is_file():raise ValueError('PLAN_FILE_MISSING: '+str(path))
    text=path.read_text()
    data=json.loads(text) if path.suffix=='.json' else yaml.safe_load(text)
    if not isinstance(data,dict) or not isinstance(data.get('poses'),list):raise ValueError('PLAN_FILE_INVALID: no poses list')
    if data.get('frame_id','map')!=map_frame:raise ValueError(f"PLAN_FRAME_MISMATCH: plan uses {data.get('frame_id')}, map uses {map_frame}")
    # Plans exported before the fingerprint existed rely on the clearance check below.
    expected=data.get('map_image_sha256')
    if expected and expected!=map_image_sha256(map_yaml):raise ValueError('PLAN_MAP_MISMATCH: the plan was made on a different map image')
    if sensor is not None:
        geometry=(data.get('settings') or {}).get('geometry') or {}
        for key,value in zip(('sensor_offset_m','coverage_disk_radius_m'),sensor):
            planned=geometry.get(key,0. if key=='sensor_offset_m' else None)
            if planned is not None and abs(float(planned)-float(value))>1e-6:
                raise ValueError(f'PLAN_ROBOT_MISMATCH: the plan was made with {key}={planned}, this robot has {value}; '
                                 're-plan in coverage_tool with this robot\'s settings')
    drive=data.get('drive_path')
    raw=drive['points'] if isinstance(drive,dict) and drive.get('points') else [[p['x'],p['y']] for p in data['poses']]
    points=[];index={}
    for i,p in enumerate(raw):
        xy=[float(p[0]),float(p[1])]
        if not all(math.isfinite(v) for v in xy):raise ValueError('PLAN_FILE_INVALID: non-finite pose')
        if not points or math.dist(points[-1],xy)>1e-6:points.append(xy)
        index[i]=len(points)-1
    if len(points)<2:raise ValueError('PLAN_FILE_INVALID: fewer than two distinct poses')
    unsafe=[i for i,(a,b) in enumerate(zip(points,points[1:])) if not grid.segment_safe(a,b,collision)]
    if unsafe:
        x,y=points[unsafe[0]]
        raise ValueError(f'PLAN_TOO_CLOSE_TO_OBSTACLES: {len(unsafe)} of {len(points)-1} segments come within '
                         f'{collision:g} m of a wall/unknown cell (first near x={x:.2f}, y={y:.2f}); '
                         're-plan in coverage_tool with a larger robot radius')
    if isinstance(drive,dict) and drive.get('points'):
        stops=[index[int(i)] for i in drive.get('stops',[]) if int(i) in index]
    else:
        stops=stop_corners(points)
    # The robot radius the plan was made with: cells only reachable closer to walls were never part of it.
    try:robot_radius=float(data['settings']['geometry']['robot_radius_m'])
    except (KeyError,TypeError,ValueError):robot_radius=None
    if robot_radius is not None and not (math.isfinite(robot_radius) and robot_radius>0):robot_radius=None
    # The base station the plan starts from (coverage_tool --base-station); None for older exports.
    station=data.get('base_station')
    try:station=[float(station[k]) for k in ('x','y','yaw')] if station is not None else None
    except (KeyError,TypeError,ValueError):raise ValueError('PLAN_FILE_INVALID: base_station needs x, y and yaw') from None
    # The area the plan covers (coverage_tool --area); None = all reachable floor.
    area=data.get('area_polygon') or None
    try:area=[[float(p[0]),float(p[1])] for p in area] if area else None
    except (TypeError,ValueError,IndexError):raise ValueError('PLAN_FILE_INVALID: area_polygon needs [x, y] corners') from None
    if area is not None and len(area)<3:raise ValueError('PLAN_FILE_INVALID: area_polygon needs at least 3 corners')
    return {'points':points,'stops':stops,'targets':pieces(points,stops),'drivable':isinstance(drive,dict),
            'robot_radius':robot_radius,'base_station':station,'area':area,
            'plan_sha256':hashlib.sha256(text.encode()).hexdigest()}
