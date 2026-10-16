"""Path helpers for exported coverage plans: targets, lengths and densification."""
from dataclasses import dataclass
import hashlib
import json
import math
import numpy as np


@dataclass
class Target:
    start: list
    end: list
    key: str


def target(a,b):
    endpoints=sorted(tuple(round(float(v),4) for v in p) for p in (a,b))
    return Target(list(a),list(b),hashlib.sha256(repr(endpoints).encode()).hexdigest()[:20])


def length(points):
    a=np.asarray(points,float)
    return float(np.linalg.norm(np.diff(a,axis=0),axis=1).sum()) if len(a)>1 else 0.


def turns(points):
    d=np.diff(np.asarray(points,float),axis=0);d=d[np.linalg.norm(d,axis=1)>1e-8]
    if len(d)<2:return 0.
    a=np.arctan2(d[:,1],d[:,0])
    return float(np.abs(np.arctan2(np.sin(np.diff(a)),np.cos(np.diff(a)))).sum())


def poly_target(points,kind='sweep'):
    points=np.asarray(points,float).tolist()
    t=target(points[0],points[-1]);t.points=points;t.kind=kind
    t.key=hashlib.sha256(json.dumps([kind,points],separators=(',',':')).encode()).hexdigest()[:20]
    return t


def points_of(t):return getattr(t,'points',None) or [t.start,t.end]


def densify(points,spacing):
    """Points at least every `spacing` metres along the polyline, keeping every vertex."""
    out=[list(points[0])]
    for a,b in zip(points,points[1:]):
        n=max(1,math.ceil(math.dist(a,b)/spacing-1e-6))  # Tolerance: densifying twice must not add points.
        out.extend((np.asarray(a,float)+(np.asarray(b,float)-a)*i/n).tolist() for i in range(1,n+1))
    return out
