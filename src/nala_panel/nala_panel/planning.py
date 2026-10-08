"""Plan a coverage path on the robot (no Qt): coverage_tool/plan_cli.py in a child process.

A child process keeps the screen responsive, can be cancelled, and keeps coverage_tool's own
`coverage` package apart from the system package of the same name (python3-coverage).
"""
import os
from pathlib import Path
import queue
import signal
import subprocess
import sys
import threading
import yaml
from nala_coverage.geometry import validate_polygon


def check_inputs(grid, collision, start, area):
    """'' if planning can start, else the reason (shown at once, not after a long planning run)."""
    if start is not None and not grid.segment_safe(start, start, collision):
        return f'The start point is closer than {collision:.2f} m to a wall: the robot cannot turn there'
    if area:
        try:
            validate_polygon(area)
        except ValueError as error:
            return f'Area: {error}'
    return ''


def command(repo_dir, map_yaml, station_file, out_yaml, out_png, start=None, area=None):
    """The plan_cli.py command. start=None: coverage starts at the base station."""
    repo_dir = Path(repo_dir)
    cmd = [sys.executable, '-u', str(repo_dir / 'coverage_tool' / 'plan_cli.py'), str(map_yaml),
           '--base-station', str(station_file), '--config-dir', str(repo_dir / 'config'),
           '-o', str(out_yaml), '--png', str(out_png)]
    if start is not None:
        cmd += ['--start', *(f'{v:.4f}' for v in start)]
    if area:
        cmd += ['--area', *(f'{v:.4f}' for point in area for v in point)]
    return cmd


class PlanRun:
    """One planning run. Read new output lines with lines(); check done() and ok()."""

    def __init__(self, cmd, out_yaml, out_png):
        self.outputs = (Path(out_yaml), Path(out_png))
        self.queue = queue.Queue()
        self.cancelled = False
        self.process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                        start_new_session=True)
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for line in self.process.stdout:
            self.queue.put(line.rstrip())

    def lines(self):
        out = []
        while not self.queue.empty():
            out.append(self.queue.get())
        return out

    def done(self):
        return self.process.poll() is not None

    def ok(self):
        return self.done() and self.process.returncode == 0 and not self.cancelled and self.outputs[0].is_file()

    def stats(self):
        """The planner statistics from the saved plan."""
        return (yaml.safe_load(self.outputs[0].read_text()) or {}).get('stats') or {}

    def cancel(self):
        """Stop planning and remove what it wrote."""
        self.cancelled = True
        if not self.done():
            os.killpg(self.process.pid, signal.SIGTERM)
            self.process.wait()
        self.discard()

    def discard(self):
        for path in self.outputs:
            path.unlink(missing_ok=True)


# Plan statistics the panel shows as checks: (key, test, text). Same checks as the README asks for.
CHECKS = (
    ('coverage_of_reachable_pct', lambda v: v >= 99.99, 'covers all reachable floor'),
    ('footprint_hits', lambda v: v == 0, 'robot outline touches nothing'),
    ('drive_unsafe_segments', lambda v: v == 0, 'no unsafe path segment'),
    ('drive_lost_cells', lambda v: v == 0, 'drivable path loses no covered floor'),
)


def check_results(stats):
    """[(ok, text)] for the plan checks; a missing statistic counts as failed."""
    return [(key in stats and test(stats[key]), text) for key, test, text in CHECKS]
