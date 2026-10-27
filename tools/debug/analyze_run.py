"""Summarize one app 3 run folder (maps/<map>/runs/<time>/): pauses, weaving while driving, supervisor CPU timing.
Only reads files; no ROS needed. Works on the PC (after scp of the run folder) and on the Pi.

    python3 tools/debug/analyze_run.py maps/RDG_NM_v4/runs/2026-10-26_164108 [--plan path/to/plan.yaml]

Pauses: from task_*.json events ("Recalculating..." on the screen = a pause that ends with AUTO_RESUMED).
Driving: from coverage_*.json actual_trajectory (trusted map poses, ~10 Hz) against the plan's drive_path.
  |w| = turn rate while driving (> 0.06 m/s); heading error / cross-track = to the nearest plan segment.
Timing: the supervisor's 'timing:' lines in launch.log (load average, longest gap between its callbacks).
Robot runs 2026-10-26 (RDG_NM_v4, lookahead 0.2 m, differential AMCL), run 164108: 40 POSITION_UNCERTAIN,
12 SCAN_STALE; |w| > 0.1 in 36 % of driving, heading error p90 12.9 deg; load 10.5, gap median 475 ms.
"""
import argparse
import collections
import glob
import json
import math
import os
import re

import numpy as np
import yaml


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def one(pattern):
    files = glob.glob(pattern)
    return files[0] if files else None


def pauses(task):
    events = task['events']
    counts = collections.Counter(e['reason'] for e in events if e.get('state') == 'PAUSED')
    times = collections.defaultdict(list)
    # A pause can start while another one is open (e.g. SCAN_STALE during SPUR_OFF_LINE): AUTO_RESUMED names its
    # pause in 'after', so match by reason.
    start = {}
    for e in events:
        if e.get('state') == 'PAUSED':
            start.setdefault(e['reason'], e['time'])
        elif e.get('reason') == 'AUTO_RESUMED' and e.get('after') in start:
            times[e['after']].append(e['time'] - start.pop(e['after']))
        elif e.get('state') is not None:
            start = {}
    print(f"Task: {task['state']} / {task['reason']}, {task['elapsed_s']:.0f} s")
    print('Pauses (count, auto-resumed: count, total s, median s):')
    for reason, n in counts.most_common():
        t = times.get(reason, [])
        extra = f'  {len(t)} auto, {sum(t):.0f} s, median {np.median(t):.1f} s' if t else ''
        print(f'  {n:3d}  {reason[:60]}{extra}')


def driving(coverage, plan_file):
    points = np.array(yaml.safe_load(open(plan_file))['drive_path']['points'], float)
    a, d = points[:-1], np.diff(points, axis=0)
    length2 = np.maximum((d ** 2).sum(1), 1e-12)
    t, x, y, h = np.array(coverage['actual_trajectory']).T
    dt = np.diff(t)
    ok = (dt > 0) & (dt < 0.35)
    v = np.where(ok, np.hypot(np.diff(x), np.diff(y)) / np.where(ok, dt, 1), 0)
    w = np.where(ok, wrap(np.diff(h)) / np.where(ok, dt, 1), 0)
    drive = ok & (v > 0.06)
    heading, cross = [], []
    for i in np.flatnonzero(drive):
        p = np.array([x[i + 1], y[i + 1]])
        s = np.clip(((p - a) * d).sum(1) / length2, 0, 1)
        dist = np.hypot(*(p - a - d * s[:, None]).T)
        k = dist.argmin()
        e = wrap(h[i + 1] - math.atan2(d[k, 1], d[k, 0]))
        if abs(e) < math.radians(60):
            heading.append(abs(math.degrees(e)))
            cross.append(dist[k])
    print(f'Driving: {dt[drive].sum():.0f} s, median speed {np.median(v[drive]):.3f} m/s, plan {plan_file}')
    print(f'  |w| > 0.1 / 0.2 / 0.3 rad/s: ' + ' / '.join(
        f'{100 * np.mean(np.abs(w[drive]) > lim):.1f} %' for lim in (0.1, 0.2, 0.3)))
    print(f'  heading error to the plan: median {np.median(heading):.1f} deg, p90 {np.percentile(heading, 90):.1f} deg')
    print(f'  cross-track to the plan: median {100 * np.median(cross):.1f} cm, p90 {100 * np.percentile(cross, 90):.1f} cm')
    print(f'  trajectory gaps > 0.6 s (not trusted / not active): {(dt > 0.6).sum()}, {dt[dt > 0.6].sum():.0f} s')


def timing(launch_log):
    rows = []
    for line in open(launch_log, errors='replace'):
        m = re.search(r'timing: state (\w+) .*tick max (\d+) ms .*gap max (\d+) ms.* load ([\d.]+)', line)
        if m and m.group(1) != 'IDLE':
            rows.append((int(m.group(2)), int(m.group(3)), float(m.group(4))))
    if not rows:
        print('Timing: no timing lines in launch.log')
        return
    tick, gap, load = np.array(rows).T
    print(f'Supervisor timing ({len(rows)} windows of 5 s, not IDLE): load median {np.median(load):.1f}, '
          f'max {load.max():.1f}; tick max median {np.median(tick):.0f} ms; gap max median {np.median(gap):.0f} ms, '
          f'p90 {np.percentile(gap, 90):.0f} ms, > 500 ms in {100 * np.mean(gap > 500):.0f} % of windows')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('run_dir')
    parser.add_argument('--plan', help='plan .yaml (default: the task report plan, looked up in this map folder)')
    args = parser.parse_args()
    task_file = one(os.path.join(args.run_dir, 'task_*.json'))
    coverage_file = one(os.path.join(args.run_dir, 'coverage_*.json'))
    if task_file:
        task = json.load(open(task_file))
        pauses(task)
        plan = args.plan or os.path.join(args.run_dir, '..', '..', 'plans', os.path.basename(task.get('plan_file') or ''))
        if coverage_file and os.path.isfile(plan):
            driving(json.load(open(coverage_file)), plan)
        else:
            print(f'Driving: skipped (no coverage report or plan {plan})')
    else:
        print('No task report in this run folder')
    if os.path.isfile(os.path.join(args.run_dir, 'launch.log')):
        timing(os.path.join(args.run_dir, 'launch.log'))


if __name__ == '__main__':
    main()
