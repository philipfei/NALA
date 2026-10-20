"""Pure continuously evaluated localization trust and command ownership policies."""
import math
import numpy as np
from .params import load_settings
from .geometry import wrap


class Trust:
    def __init__(self, settings=None):
        self.cfg=(settings or load_settings())['localization']
        self.since=None;self.previous=None;self.reason='LOCALIZATION_UNINITIALIZED'

    def check(self, now, pose, odom, covariance, endpoints, matched, amcl_age,
              sensors_ok=True, tf_ok=True):
        reason=''
        if not sensors_ok:reason='SENSOR_STALE'
        elif not tf_ok or pose is None or odom is None:reason='TF_UNAVAILABLE'
        elif not np.isfinite([*pose,*odom]).all():reason='TF_INVALID'
        elif covariance is None or not np.isfinite(covariance).all():reason='AMCL_INVALID'
        elif amcl_age<-.1 or amcl_age>self.cfg['amcl_max_age_s']:reason='AMCL_STALE'
        else:
            cov=np.asarray(covariance).reshape(6,6)
            sigma=math.sqrt(max(0,float(np.linalg.eigvalsh(cov[:2,:2]).max())))
            yaw_sigma=math.sqrt(max(0,float(cov[5,5])))
            # The scan decides (user, 2026-10-19): AMCL's own sigma was wrong both ways on the robot (0.017 m while
            # 0.6 m off; over 0.10 m while right, in turns on the spot). Above the limit the robot only stops when
            # the live scan does not fit the map well, or when the sigma is uncertain_hard_factor x the limit.
            scan_fits=endpoints>=self.cfg['min_endpoints'] and matched/endpoints>=self.cfg['matched_ratio']
            hard=self.cfg['uncertain_hard_factor']
            pos_limit=self.cfg['position_sigma_m'];yaw_limit=math.radians(self.cfg['heading_sigma_deg'])
            # A tightly converged filter can report a variance a hair below zero (rounding): that is zero.
            if np.any(np.diag(cov)<-self.cfg['covariance_rounding']):reason='AMCL_INVALID'
            elif sigma>hard*pos_limit+1e-10 or (sigma>pos_limit+1e-10 and not scan_fits):reason='POSITION_UNCERTAIN'
            elif yaw_sigma>hard*yaw_limit+1e-10 or (yaw_sigma>yaw_limit+1e-10 and not scan_fits):reason='HEADING_UNCERTAIN'
            elif endpoints<self.cfg['min_endpoints']:reason='TOO_FEW_SCAN_ENDPOINTS'
            elif matched/endpoints<self.cfg['matched_ratio']:reason='SCAN_MAP_MISMATCH'
        if not reason and self.previous:
            p,o,t=self.previous
            if now<=t:reason='TIME_REVERSED'
            else:
                # Compare changes in map->odom, not unaligned map and odom displacements.
                def transform(p,o):
                    a=wrap(p[2]-o[2]);c,s=math.cos(a),math.sin(a)
                    return (p[0]-c*o[0]+s*o[1],p[1]-s*o[0]-c*o[1],a)
                a,b=transform(p,o),transform(pose,odom)
                # Rotation around the robot must not magnify a correction by distant odom origin.
                predicted_angle=wrap(p[2]+wrap(odom[2]-o[2]))
                c,s=math.cos(a[2]),math.sin(a[2]);dx,dy=odom[0]-o[0],odom[1]-o[1]
                predicted=(p[0]+c*dx-s*dy,p[1]+s*dx+c*dy)
                if math.dist(predicted,pose[:2])>self.cfg['jump_translation_m'] or abs(wrap(pose[2]-predicted_angle))>math.radians(self.cfg['jump_rotation_deg']):
                    reason='LOCALIZATION_JUMP'
        if reason:self.since=None;self.previous=None
        else:
            if self.since is None:self.since=now
            self.previous=(list(pose),list(odom),now)
        self.reason=reason or ('LOCALIZATION_SETTLING' if now-self.since<self.cfg['settle_s'] else '')
        return not self.reason


class Ownership:
    """One grant epoch; no cached command survives a source hand-off."""
    def __init__(self, linear=.12, angular=.40, acceleration=.15, angular_acceleration=.60, settings=None, turn_start=0.):
        self.cfg=settings or load_settings()
        self.linear=linear;self.angular=angular;self.acceleration=acceleration;self.angular_acceleration=angular_acceleration
        self.turn_start=turn_start
        self.owner='NONE';self.epoch='';self.command=None;self.command_time=-math.inf;self.reverse=False
        self.lease_time=-math.inf;self.output=(0.,0.);self.stopping=False
        self.stop_at=0.;self.fault='';self.events=[]

    def lease(self, owner, epoch, now):
        if owner not in ('NONE','MANUAL','NAV'):return
        if (owner,epoch)!=(self.owner,self.epoch):
            if self.owner in ('NAV','MANUAL'):self.stopping=True;self.stop_at=now
            self.owner=owner;self.epoch=epoch;self.command=None;self.command_time=-math.inf;self.reverse=False
        self.lease_time=now

    def receive(self, source, command, now):
        """source REVERSE: the supervisor reversing out of a pocket (spur) while it holds NAV ownership."""
        if not all(math.isfinite(x) for x in command):return
        if source==self.owner:
            self.command=command;self.command_time=now;self.reverse=False
        elif source=='REVERSE' and self.owner=='NAV':
            self.command=command;self.command_time=now;self.reverse=True

    def latch(self, reason):
        if not self.fault:self.fault=reason
        if reason and (not self.events or self.events[-1]!=reason):self.events.append(reason)

    def tick(self, now, healthy, stopped, dt=.05):
        valid=healthy and not self.fault and now-self.lease_time<=self.cfg['health']['lease_age_s']
        valid &= self.owner in ('NAV','MANUAL') and self.command is not None and now-self.command_time<=self.cfg['health']['command_age_s']
        if valid:
            v,w=self.command
            # Autodrive (Nav2) only drives forwards and turns; only manual driving and the supervisor's spurs reverse.
            if self.owner=='NAV' and self.reverse:
                # Only backwards, and slowly (spur.speed_m_s): the supervisor checked the floor behind is measured.
                v=max(-self.cfg['spur']['speed_m_s'],min(0.,v));w=min(self.angular,max(-self.angular,w))
            elif self.owner=='NAV':
                v=min(self.linear,max(0.,v))
                # A turn on the spot from STANDSTILL asks at least turn_start: Nav2 (RPP) asks only measured rate +
                # a small step, below the wheels' breakaway (robot, 2026-10-16/19). Still ramped below. Once the
                # robot turns, Nav2's own command passes, so it can slow down at the heading: lifting always made
                # the robot overshoot and turn extra rounds (simulation 2026-10-19: 1500-2700 deg instead of 900).
                if v==0. and w!=0. and stopped:w=math.copysign(max(abs(w),self.turn_start),w)
                w=min(self.angular,max(-self.angular,w))
            else:v=min(self.cfg['motion']['manual_forward_m_s'],max(-self.cfg['motion']['manual_reverse_m_s'],v));w=min(self.cfg['motion']['manual_angular_rad_s'],max(-self.cfg['motion']['manual_angular_rad_s'],w))
            # Idle keyboard zeros do not create a permanent zero stream.
            if abs(v)+abs(w)<1e-9:
                valid=False
            else:
                ov,ow=self.output
                v=min(ov+self.acceleration*dt,max(ov-self.acceleration*dt,v));w=min(ow+self.angular_acceleration*dt,max(ow-self.angular_acceleration*dt,w))
                self.output=(v,w);self.stopping=True;self.stop_at=now
                return self.output
        if self.stopping:
            self.output=(0.,0.)
            if stopped and now-self.stop_at>=self.cfg['health']['stop_burst_s']:self.stopping=False
            return (0.,0.)
        return None
