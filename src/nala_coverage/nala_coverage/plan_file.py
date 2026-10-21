"""Exported coverage_tool plans: load, check against the loaded map, split into targets."""
from pathlib import Path
import hashlib
import json
import math
import yaml
from .planning import poly_target, points_of


def map_image_sha256(map_yaml):
    map_yaml=Path(map_yaml).resolve()
    return hashlib.sha256((map_yaml.parent/yaml.safe_load(map_yaml.read_text())['image']).read_bytes()).hexdigest()


def _heading(a,b):return math.atan2(b[1]-a[1],b[0]-a[0])


# Drive-in stretches into a pocket (coverage_tool coverage/spurs.py): 'spur' drives straight in forwards, 'reverse'
# straight back out backwards along the same line. Driving in is a normal piece (Nav2 follows it like any other;
# user 2026-10-20); only backing out is driven by the supervisor (state SPUR).
SPUR_KINDS=('spur','reverse')
# A passage (coverage_tool coverage/passages.py): straight through an entrance narrower than the turning circle into
# an area with turning room, driven forwards by Nav2 like any piece. Checked with the outline, as drive-ins.
OUTLINE_KINDS=SPUR_KINDS+('passage',)


def spur_yaw(a,b,kind):
    """The robot's heading on a stretch from a to b checked with the outline: driving forwards (drive-in, passage),
    or facing into the pocket while backing out ('reverse')."""
    return _heading(b,a) if kind=='reverse' else _heading(a,b)


def stop_corners(points,min_deg=20.):
    """Corners of an older plan without a drivable path where the robot has to stop and turn."""
    out=[]
    for i in range(1,len(points)-1):
        turn=abs(math.remainder(_heading(points[i],points[i+1])-_heading(points[i-1],points[i]),2*math.pi))
        if math.degrees(turn)>=min_deg:out.append(i)
    return out


def pieces(points,stops,kinds=None):
    """The path split at its stop corners; each piece ends with the heading of the next one (turn on the spot there).
    kinds: the planner's kind of the stretch arriving at each point (drive_path 'kinds'), or None (older exports)."""
    cuts=[0]+sorted(i for i in set(stops) if 0<i<len(points)-1)+[len(points)-1]
    out=[]
    for a,b in zip(cuts,cuts[1:]):
        k=kinds[a+1:b+1] if kinds else None
        back=bool(k) and len(k)==1 and k[0]=='reverse'
        t=poly_target([list(p) for p in points[a:b+1]],'reverse' if back else 'plan')
        # No turn at the end of a drive-in: the robot backs out the same way it faces.
        into=bool(k) and k[-1]=='spur'
        t.end_yaw=_heading(points[b],points[b+1]) if b<len(points)-1 and not into else None
        t.kinds=k
        # Where its drive-in starts (P): the stretch from there is checked with the outline (supervisor.follow).
        t.drive_in=list(points[a+k.index('spur')]) if into else None
        # The passages driven in this piece (checked with the outline by supervisor.follow).
        t.passages=[(list(points[a+j]),list(points[a+j+1])) for j,kind in enumerate(k or []) if kind=='passage']
        out.append(t)
    return out


def merge_turns(targets,max_length):
    """One turn on the spot per corner (user, 2026-10-19): plan pieces no longer than max_length (the aimed goals'
    5 cm moves, which Nav2's 5 cm goal tolerance treats as reached anyway) are no separate goals. Together with the
    stop before them they become one 'turn' target at that corner: the headings to turn through in order (the
    aims' headings, then the heading of the next piece). The piece before it ends without its own final turn.
    With the planner's kinds, only corner sweeps ('aim') turn: another short step joins its neighbours into one
    piece (no stop, no turn)."""
    out=[]
    for t in targets:
        if getattr(t,'kind','') in SPUR_KINDS:
            # Drive-in and back-out stretches stay their own targets, whatever their length.
            if out:out[-1].join_next=False
            out.append(t);continue
        pts=points_of(t)
        short=out and _length(pts)<=max_length
        kinds=getattr(t,'kinds',None)
        if short and kinds and kinds[-1]!='aim':
            # A few-cm step that is no corner sweep (e.g. a lane following a wall's pixel staircase, 5 cm aside;
            # room Test 2026-10-20): no turn towards it, and no stop either: the piece before, the step and the
            # next piece become one piece (the planner's own, safe stretches), which Nav2 follows in one go.
            last=out[-1]
            if getattr(last,'kind','') in SPUR_KINDS:
                out.append(t);continue
            if getattr(last,'kind','')=='turn':
                if last.headings:last.headings.pop()
                if t.end_yaw is not None:last.headings.append(t.end_yaw)
            else:
                out[-1]=_joined(last,t);out[-1].join_next=True
            continue
        if not short and out and getattr(out[-1],'join_next',False):
            out[-1]=_joined(out[-1],t);continue
        if out:out[-1].join_next=False
        if short and getattr(out[-1],'kind','') in SPUR_KINDS:
            out.append(t);continue
        if short and getattr(out[-1],'kind','')!='turn':
            prev=out[-1];end=points_of(prev)[-1]
            turn=poly_target([list(end),list(end)],'turn');turn.headings=[];turn.end_yaw=None
            if prev.end_yaw is not None:turn.headings.append(prev.end_yaw)
            prev.end_yaw=None;out.append(turn)
        if short:
            if t.end_yaw is not None:out[-1].headings.append(t.end_yaw)
            continue
        out.append(t)
    return out


def _joined(a,b):
    """Pieces a and b (b starting where a ends) as one piece with b's final heading."""
    pa,pb=points_of(a),points_of(b)
    t=poly_target([list(p) for p in pa]+[list(p) for p in pb[1:]],'plan')
    t.end_yaw=b.end_yaw
    t.kinds=(getattr(a,'kinds',None) or [])+(getattr(b,'kinds',None) or []) or None
    t.drive_in=getattr(b,'drive_in',None)
    t.passages=(getattr(a,'passages',None) or [])+(getattr(b,'passages',None) or [])
    return t


def _length(pts):
    return sum(math.dist(a,b) for a,b in zip(pts,pts[1:]))


def check_base_station(plan,station,tolerance):
    """Refuse a plan made for another base station (PLAN_BASE_MISMATCH). Plans without one are accepted."""
    if plan['base_station'] is not None and math.dist(plan['base_station'][:2],station[:2])>tolerance:
        raise ValueError(f"PLAN_BASE_MISMATCH: the plan was made for base station {plan['base_station'][:2]}, "
                         f'the base station is now {list(station[:2])}; plan again')


def spur_unsafe(grid,a,b,kind,outline,margin):
    """A spur stretch where the robot's outline (+ margin) would touch a wall or unknown floor."""
    if outline is None:return True
    swept=grid.outline_swept(a,b,spur_yaw(a,b,kind),outline,margin)
    return swept is None or bool((swept&~grid.free).any())


def load_plan(path,grid,map_yaml,map_frame,collision,sensor=None,turn_merge_m=None,outline=None,margin=0.):
    """Return the plan's driven path and its pieces between stop corners, or raise ValueError naming the problem.

    A coverage_tool export with `drive_path` gives the exact path to follow (corners rounded where that
    keeps every cell) and the corners where the robot stops and turns. For an older export the poses are
    joined by straight lines and every corner of 20 degrees or more is a stop.

    `sensor` = (offset, radius) of this robot's coverage sensor: a plan made for another sensor covers
    different floor, so it is rejected (PLAN_ROBOT_MISMATCH). `turn_merge_m`: merge_turns() with that length.
    Drive-in stretches (kinds 'spur'/'reverse') are driven straight without turning, so they are checked with the
    robot's `outline` widened by `margin` instead of the turning circle (`collision`)."""
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
    raw_kinds=drive.get('kinds') if isinstance(drive,dict) and isinstance(drive.get('kinds'),list) and len(drive['kinds'])==len(raw) else None
    points=[];index={};kinds=[]
    for i,p in enumerate(raw):
        xy=[float(p[0]),float(p[1])]
        if not all(math.isfinite(v) for v in xy):raise ValueError('PLAN_FILE_INVALID: non-finite pose')
        if not points or math.dist(points[-1],xy)>1e-6:
            points.append(xy)
            if raw_kinds:kinds.append(str(raw_kinds[i]))
        index[i]=len(points)-1
    if len(points)<2:raise ValueError('PLAN_FILE_INVALID: fewer than two distinct poses')
    unsafe=[i for i,(a,b) in enumerate(zip(points,points[1:]))
            if (spur_unsafe(grid,a,b,kinds[i+1],outline,margin) if kinds and kinds[i+1] in OUTLINE_KINDS
                else not grid.segment_safe(a,b,collision))]
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
    targets=pieces(points,stops,kinds or None)
    if turn_merge_m is not None:targets=merge_turns(targets,turn_merge_m)
    spurs=[(points[i-1],points[i]) for i in range(1,len(points)) if kinds and kinds[i]=='spur']
    passages=[(points[i-1],points[i]) for i in range(1,len(points)) if kinds and kinds[i]=='passage']
    return {'points':points,'stops':stops,'targets':targets,'drivable':isinstance(drive,dict),'spurs':spurs,
            'passages':passages,
            'robot_radius':robot_radius,'base_station':station,'area':area,
            'plan_sha256':hashlib.sha256(text.encode()).hexdigest()}
