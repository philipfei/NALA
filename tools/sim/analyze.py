"""Summarise one run: plan tracking, box contacts, reverse driving, turns, coverage, skipped stretches and a plot.

    python3 analyze.py runs/<name>      -> runs/<name>/summary.json and runs/<name>/plot.png
"""
import csv
import json
import sys
from pathlib import Path
import numpy as np
import yaml
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def seg_dist(p, a, b):
    ab = b - a
    t = np.clip(((p - a) @ ab) / max(ab @ ab, 1e-12), 0, 1)
    return np.linalg.norm(p - (a + t * ab))


def main(run):
    run = Path(run)
    res = json.load(open(run / 'result.json'))
    plan = yaml.safe_load(open(res['plan']))
    poses = np.array([[p['x'], p['y']] for p in plan['poses']])
    # A plan with a drivable path is followed along that path: measure against it.
    drive = plan.get('drive_path') or {}
    line = np.array(drive['points']) if drive.get('points') else poses
    stops = line[drive['stops']] if drive.get('stops') else np.zeros((0, 2))
    rows = list(csv.DictReader(open(run / 'truth.csv')))
    traj = np.array([[float(r['x']), float(r['y'])] for r in rows])
    clear = np.array([float(r['clearance']) for r in rows])
    moving = np.flatnonzero(np.linalg.norm(np.diff(traj, axis=0), axis=1) > 1e-4)
    report = json.load(open(run / 'output' / res['report'])) if res.get('report') else {}
    # Over the whole run: the first plan pose may be dropped as passed, and the drive to the plan
    # start only adds a few incidental hits.
    reach = np.array([np.min(np.linalg.norm(traj - p, axis=1)) for p in poses])
    # Distance from the driven path to the line it should follow (drivable path, else the poses).
    seg_a, seg_b = line[:-1], line[1:]

    def to_line(p):
        ab = seg_b - seg_a
        t = np.clip(np.sum((p - seg_a) * ab, axis=1) / np.maximum(np.sum(ab * ab, axis=1), 1e-12), 0, 1)
        return float(np.min(np.linalg.norm(p - (seg_a + t[:, None] * ab), axis=1)))
    near_start = np.flatnonzero(np.linalg.norm(traj - line[0], axis=1) < .15)
    on_path = traj[near_start[0]:] if len(near_start) else traj
    dev = np.array([to_line(p) for p in on_path[::3]])
    m = yaml.safe_load(open(res['map']))
    img = np.array(Image.open(Path(res['map']).parent / m['image']))[::-1]
    r0 = m['resolution']
    ox, oy = m['origin'][:2]
    yaw = np.unwrap([float(r['yaw']) for r in rows])
    v = np.array([abs(float(r['v'])) for r in rows])
    # Distance driven against the heading (autodrive must never reverse): displacement opposite the heading.
    step = np.diff(traj, axis=0)
    along = step[:, 0] * np.cos(yaw[:-1]) + step[:, 1] * np.sin(yaw[:-1])
    reverse_m = float(-along[along < -1e-4].sum())
    spins, acc = 0, 0.
    for k in range(1, len(rows)):
        if v[k] < .01:
            acc += abs(yaw[k] - yaw[k - 1])
        else:
            spins += acc > np.pi / 2
            acc = 0.
    blockages = report.get('temporary_blockages', [])
    summary = {
        'final_state': res['final'].get('state'), 'reason': res['final'].get('reason'),
        'mission_s': res.get('mission_s'), 'meter_fraction': round(res['final'].get('fraction', 0), 4),
        'box_contacts': int(sum(1 for i in range(1, len(rows)) if rows[i]['contact'] == '1' and rows[i - 1]['contact'] == '0')),
        'reverse_distance_m': round(reverse_m, 3),
        'min_clearance_m': round(float(clear[moving].min()) if len(moving) else float(clear.min()), 3),
        'distance_m': round(float(np.linalg.norm(np.diff(traj, axis=0), axis=1).sum()), 2),
        'total_turn_rad': round(float(np.abs(np.diff(yaw)).sum()), 1),
        'in_place_turns_over_90deg': int(spins),
        'poses_within_0.10m': f'{int((reach <= .10).sum())}/{len(poses)}',
        'poses_within_0.25m': f'{int((reach <= .25).sum())}/{len(poses)}',
        'worst_pose_miss_m': round(float(reach.max()), 3), 'worst_pose_index': int(reach.argmax()),
        'distance_to_path_median_m': round(float(np.median(dev)), 3),
        'distance_to_path_p95_m': round(float(np.percentile(dev, 95)), 3),
        'distance_to_path_max_m': round(float(dev.max()), 3),
        'drivable_path': bool(drive.get('points')), 'stop_corners': int(len(stops)),
        'skipped_poses': [[round(x, 3) for x in b['xy']] + [b.get('cause', b.get('reason'))] for b in blockages],
        'target_failures': report.get('target_failures'),
        'events': [(e.get('reason'), e.get('target')) for e in report.get('events', [])],
        'resumes': res.get('resumes'), 'error': res.get('error'),
        'timeline': res['timeline'],
    }
    json.dump(summary, open(run / 'summary.json', 'w'), indent=1)
    fig, ax = plt.subplots(figsize=(6, 9))
    ax.imshow(img, cmap='gray', origin='lower', extent=[ox, ox + img.shape[1] * r0, oy, oy + img.shape[0] * r0])
    ax.plot(line[:, 0], line[:, 1], 'r-', lw=.8, label='drivable path' if drive.get('points') else 'plan')
    if len(stops):
        ax.plot(stops[:, 0], stops[:, 1], 'o', mfc='none', mec='red', ms=5, label='stop corner')
    ax.plot(traj[:, 0], traj[:, 1], 'b-', lw=1, alpha=.8, label='driven (ground truth)')
    for o in res['obstacles']:
        x, y, r = map(float, o.split(','))
        ax.add_patch(plt.Circle((x, y), r, color='orange', label='obstacle (not in map)'))
    for b in blockages:
        ax.plot(*b['xy'], 'kx', ms=10, mew=2, label='skipped pose')
    bad = reach > .25
    ax.plot(poses[bad, 0], poses[bad, 1], 'o', mfc='none', mec='m', ms=9, label='pose missed > 0.25 m')
    ax.plot(*res['start'][:2], 'g^', ms=9, label='robot start')
    h, lab = ax.get_legend_handles_labels()
    u = dict(zip(lab, h))
    ax.legend(u.values(), u.keys(), fontsize=7, loc='upper right')
    ax.set_title(f"{run.name}: {summary['final_state']} {summary['reason']}  {summary['mission_s']} s", fontsize=9)
    fig.savefig(run / 'plot.png', dpi=110, bbox_inches='tight')
    print(json.dumps({k: v for k, v in summary.items() if k not in ('timeline', 'events')}, indent=1))


if __name__ == '__main__':
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    main(sys.argv[1])
