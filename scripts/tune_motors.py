#!/usr/bin/env python3
"""Tune the MCU wheel speed controller over the UART (firmware v2.2.0 setting mode). Run on the Pi.

Stop app 1 / app 2 first: base_node uses the same UART.

    python3 ~/NALA/scripts/tune_motors.py show                 # all controller parameters
    python3 ~/NALA/scripts/tune_motors.py set kp 2 ki 10       # change them in the MCU RAM (lost at power-off)
    python3 ~/NALA/scripts/tune_motors.py save                 # write the RAM values to the MCU EEPROM (kept)
    python3 ~/NALA/scripts/tune_motors.py reload               # back to the values in the EEPROM
    python3 ~/NALA/scripts/tune_motors.py defaults             # the firmware defaults (config.h), in RAM
    python3 ~/NALA/scripts/tune_motors.py step --vx 0.15       # one speed step, metrics per wheel
    python3 ~/NALA/scripts/tune_motors.py sweep [--floor]      # steps in 6 directions at several speeds
    python3 ~/NALA/scripts/tune_motors.py ff                   # measure the feed-forward table (wheels in the air)

The MCU accepts a change only while the robot stands still (else it answers "busy").
Every motion asks for a confirmation first; --yes skips it (for SSH runs after a check by a person).
Settings of the tests: config/tune_motors.yaml. Robot numbers: config/base.yaml.
"""

import argparse
import math
import os
import queue
import subprocess
import sys
import threading
import time

import serial
import yaml

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'src', 'nala_base'))
from nala_base import mcu_protocol as mp  # noqa: E402
from nala_base.step_metrics import body_to_wheels, step_metrics, wheel_speeds  # noqa: E402

OP_NAMES = {mp.OP_GET: 'get', mp.OP_SET: 'set', mp.OP_SAVE: 'save', mp.OP_DEFAULTS: 'defaults',
            mp.OP_RELOAD: 'reload'}
IDX = {name: i for i, name in enumerate(mp.PARAM_NAMES)}
N = mp.FF_TABLE_N
FF_SPEED, FF_PWM = IDX['ff_speed0'], IDX['ff_pwm0']


class LinkError(Exception):
    pass


class Link:
    """UART to the MCU: a sender thread (drive command at a fixed rate) and one reader thread.

    Every write goes through write_frame() (one lock), so two frames never interleave. Only the
    reader thread reads; it puts parameter replies into a queue and keeps the state frames.
    Reads are not under the lock: a blocking read must never delay the keep-alive commands.
    """

    def __init__(self, port, baud, rate):
        self.serial = serial.Serial(port, baud, timeout=0.05, write_timeout=0.2)
        self.write_lock = threading.Lock()
        self.cmd_lock = threading.Lock()      # taken before write_lock when both are needed
        self.cmd = (0.0, 0.0, 0.0)
        self.period = 1.0 / rate
        self.replies = queue.Queue()
        self.frames = []                      # (t, seq, flags, counts), appended by the reader only
        self.crc_errors = 0
        self.read_error = None
        self.running = True
        self.threads = [threading.Thread(target=self._read, daemon=True),
                        threading.Thread(target=self._send, daemon=True)]
        for thread in self.threads:
            thread.start()

    def write_frame(self, frame):
        with self.write_lock:
            self.serial.write(frame)

    def _send(self):
        next_time = time.monotonic()
        while self.running:
            try:
                with self.cmd_lock:           # so stop() can never be overtaken by an old command
                    self.write_frame(mp.encode_command(*self.cmd))
            except (serial.SerialException, OSError):
                pass                          # the main thread notices it at its next request
            next_time += self.period
            time.sleep(max(0.0, next_time - time.monotonic()))

    def _read(self):
        buffer = b''
        while self.running:
            try:
                buffer += self.serial.read(self.serial.in_waiting or 1)
            except (serial.SerialException, OSError) as error:
                self.read_error = error
                return
            states, replies, buffer, bad = mp.split_frames(buffer)
            self.crc_errors += bad
            now = time.monotonic()
            for seq, flags, counts in states:
                self.frames.append((now, seq, flags, counts))
            for reply in replies:
                self.replies.put(reply)

    def drive(self, vx, vy, wz):
        with self.cmd_lock:
            self.cmd = (vx, vy, wz)

    def stop(self):
        """Zero command now (not only at the next keep-alive)."""
        with self.cmd_lock:
            self.cmd = (0.0, 0.0, 0.0)
            self.write_frame(mp.encode_command(0.0, 0.0, 0.0))

    def request(self, op, idx=0, value=0.0, timeout=0.2, attempts=3):
        """One parameter request at a time; returns (status, value). Raises LinkError without a reply."""
        if self.read_error:
            raise LinkError(f'UART read failed: {self.read_error}')
        for _ in range(attempts):
            while not self.replies.empty():
                self.replies.get_nowait()     # drop late replies of an earlier attempt
            self.write_frame(mp.encode_param(op, idx, value))
            end = time.monotonic() + timeout
            while (left := end - time.monotonic()) > 0:
                try:
                    r_op, r_idx, status, r_value = self.replies.get(timeout=left)
                except queue.Empty:
                    break
                if r_op == op and r_idx == idx:
                    return status, r_value
        raise LinkError(f'no reply from the MCU to "{OP_NAMES.get(op, op)} {idx}"')

    def flags(self):
        return self.frames[-1][2] if self.frames else None

    def close(self):
        try:
            self.stop()
        except (serial.SerialException, OSError):
            pass
        self.running = False
        for thread in self.threads:
            thread.join(timeout=0.5)
        self.serial.close()


def fmt(value, digits=3):
    return '-' if value is None else f'{value:.{digits}g}'


def get_all(link, cfg):
    values = []
    for i, name in enumerate(mp.PARAM_NAMES):
        status, value = link.request(mp.OP_GET, i, timeout=cfg['reply_timeout'], attempts=cfg['attempts'])
        if status != mp.STATUS_OK:
            raise LinkError(f'get {name}: {mp.STATUS_TEXT.get(status, status)}')
        values.append(value)
    return values


def set_param(link, cfg, name, value, attempts=None, timeout=None):
    try:
        status, now = link.request(mp.OP_SET, IDX[name], value, timeout=timeout or cfg['reply_timeout'],
                                   attempts=attempts or cfg['attempts'])
    except LinkError as error:
        raise LinkError(f'set {name}: {error}') from None
    if status != mp.STATUS_OK:
        raise LinkError(f'set {name} {value}: {mp.STATUS_TEXT.get(status, status)} (still {fmt(now, 4)})')
    return now


def table_valid(values):
    speeds = values[FF_SPEED:FF_SPEED + N]
    return speeds[0] == 0.0 and all(b > a for a, b in zip(speeds, speeds[1:]))


def print_flags(link):
    flags = link.flags()
    if flags is None:
        print('No state frame from the MCU.')
        return
    print('EEPROM: ' + ('RAM = EEPROM block' if flags & mp.FLAG_EEPROM else
                        'RAM differs from the EEPROM (unsaved changes, or defaults)'))
    if flags & mp.FLAG_FF_INVALID:
        print('FF TABLE INVALID: the MCU ignores drive commands until it is fixed.')


def cmd_show(link, robot, cfg, args):
    v = get_all(link, cfg)
    print(f'kp          {fmt(v[IDX["kp"]], 4):>8}   % pwm per rad/s')
    print(f'ki          {fmt(v[IDX["ki"]], 4):>8}   % pwm per rad/s per s')
    print(f'i_limit     {fmt(v[IDX["i_limit"]], 4):>8}   % pwm')
    print(f'accel_limit {fmt(v[IDX["accel_limit"]], 4):>8}   rad/s^2 (0 = off)')
    print('ff_speed    ' + ' '.join(f'{x:6.2f}' for x in v[FF_SPEED:FF_SPEED + N]) + '   rad/s')
    print('ff_pwm      ' + ' '.join(f'{x:6.2f}' for x in v[FF_PWM:FF_PWM + N]) + '   % pwm')
    if not table_valid(v):
        print('The FF table is not valid (ff_speed0 must be 0, the speeds must increase).')
    print_flags(link)


def cmd_set(link, robot, cfg, args):
    if len(args.pairs) % 2:
        raise SystemExit('set needs NAME VALUE pairs, for example: set kp 2 ki 10')
    pairs = list(zip(args.pairs[::2], args.pairs[1::2]))
    for name, _ in pairs:
        if name not in IDX:
            raise SystemExit(f'Unknown parameter {name}. Names: {", ".join(mp.PARAM_NAMES)}')
    for name, text in pairs:
        print(f'{name} = {fmt(set_param(link, cfg, name, float(text)), 4)}')
    if any(name.startswith('ff_') for name, _ in pairs) and not table_valid(get_all(link, cfg)):
        print('WARNING: the FF table is not valid now: the MCU will ignore drive commands until it is fixed.')
    print('In RAM only. "save" keeps it after power-off.')


def simple_op(op, text):
    def run(link, robot, cfg, args):
        timeout = cfg['save_timeout'] if op == mp.OP_SAVE else cfg['reply_timeout']
        status, _ = link.request(op, 0, timeout=timeout, attempts=cfg['attempts'])
        if status != mp.STATUS_OK:
            raise LinkError(f'{OP_NAMES[op]}: {mp.STATUS_TEXT.get(status, status)}')
        print(text)
    return run


def confirm(text, args):
    print(text)
    if args.yes:
        return
    if input('Continue? [y/N] ').strip().lower() != 'y':
        raise SystemExit('Cancelled.')


def run_step(link, robot, cfg, vx, vy, wz, seconds):
    """Zero, step, zero. Returns (targets, metrics per wheel or None, samples, t_on, t_off)."""
    start = len(link.frames)
    link.stop()
    time.sleep(cfg['pre_time'])
    t_on = time.monotonic()
    link.drive(vx, vy, wz)
    time.sleep(seconds)
    t_off = time.monotonic()
    link.stop()
    time.sleep(cfg['post_time'])
    frames = link.frames[start:]
    flags = 0
    for frame in frames:
        flags |= frame[2]
    if flags & mp.FLAG_FF_INVALID:
        print('WARNING: FF table invalid: the MCU ignored the command.')
    if flags & mp.FLAG_TIMEOUT:
        print('WARNING: the MCU reported a command timeout during the step.')
    samples = wheel_speeds(frames, robot['rad_per_count'], robot['period'])
    targets = body_to_wheels(vx, vy, wz, robot['radius'], robot['k'])
    times = [t for t, _ in samples]
    metrics = [step_metrics(times, [s[m] for _, s in samples], targets[m], t_on, t_off)
               if abs(targets[m]) > 0.01 else None for m in range(4)]
    return targets, metrics, samples, t_on, t_off


def print_metrics(targets, metrics):
    print('wheel  target  dead   rise   overshoot  settle  steady (error)')
    for m, (target, r) in enumerate(zip(targets, metrics)):
        if r is None:
            print(f'M{m + 1}     {target:6.2f}  (no target)')
            continue
        error = None if r['mean'] is None else (r['mean'] / target - 1) * 100
        print(f'M{m + 1}     {target:6.2f}  {fmt(r["dead"], 2):>5}  {fmt(r["rise"], 2):>5}  '
              f'{fmt(r["overshoot"], 3):>7} %  {fmt(r["settle"], 2):>6}  '
              f'{fmt(r["mean"], 3)} +- {fmt(r["std"], 2)} ({fmt(error, 2)} %)')
    print('(times in s from the step, speeds in rad/s; dead = to 10 %, rise = 10 -> 90 %, settle = within 10 %)')


def describe(vx, vy, wz, seconds):
    return (f'vx {vx:+.3f} m/s, vy {vy:+.3f} m/s, wz {wz:+.3f} rad/s for {seconds:.1f} s '
            f'(on the floor: {math.hypot(vx, vy) * seconds:.2f} m, {math.degrees(wz * seconds):.0f} deg)')


def cmd_step(link, robot, cfg, args):
    seconds = args.time or cfg['step_time']
    confirm('Step: ' + describe(args.vx, args.vy, args.wz, seconds) + '. Wheels lifted or floor clear?', args)
    targets, metrics, samples, t_on, _ = run_step(link, robot, cfg, args.vx, args.vy, args.wz, seconds)
    print_metrics(targets, metrics)
    if args.csv:
        with open(args.csv, 'w') as f:
            f.write('t,M1,M2,M3,M4,target_M1,target_M2,target_M3,target_M4\n')
            for t, speeds in samples:
                on = t_on <= t <= t_on + seconds
                f.write(','.join(f'{x:.4f}' for x in [t - t_on] + speeds + [x if on else 0.0 for x in targets]) + '\n')
        print(f'Wrote {args.csv}')


def cmd_sweep(link, robot, cfg, args):
    k = robot['k']
    tests = []
    for v in cfg['sweep_speeds']:
        seconds = cfg['step_time']
        if args.floor:
            seconds = min(seconds, cfg['floor_max_dist'] / v)
        tests += [('forward', v, 0, 0, seconds), ('back', -v, 0, 0, seconds),
                  ('left', 0, v, 0, seconds), ('right', 0, -v, 0, seconds),
                  ('ccw', 0, 0, v / k, seconds), ('cw', 0, 0, -v / k, seconds)]
    total = sum(t[4] + cfg['pre_time'] + cfg['post_time'] for t in tests)
    far = max(abs(t[1]) * t[4] for t in tests)
    confirm(f'Sweep: {len(tests)} steps at {cfg["sweep_speeds"]} m/s rim speed (forward, back, left, right, '
            f'ccw, cw), about {total:.0f} s. Each pair comes back; the longest step is {far:.2f} m. '
            'Wheels lifted or floor clear?', args)
    rows = []
    for name, vx, vy, wz, seconds in tests:
        print(f'== {name}: ' + describe(vx, vy, wz, seconds))
        targets, metrics, _, _, _ = run_step(link, robot, cfg, vx, vy, wz, seconds)
        print_metrics(targets, metrics)
        rows.append((name, max(abs(x) for x in targets), [r for r in metrics if r]))
    print('\nSummary (worst wheel; steady error = mean over the wheels)')
    print('test       target  dead   rise   overshoot  settle  steady error')
    for name, target, rs in rows:
        def worst(key, rs=rs):
            values = [r[key] for r in rs]
            return None if None in values or not values else max(values)
        errors = [abs(r['mean']) / target - 1 for r in rs if r['mean'] is not None]
        error = sum(errors) / len(errors) * 100 if errors else None
        print(f'{name:9}  {target:6.2f}  {fmt(worst("dead"), 2):>5}  {fmt(worst("rise"), 2):>5}  '
              f'{fmt(worst("overshoot"), 3):>7} %  {fmt(worst("settle"), 2):>6}  {fmt(error, 2)} %')


def suggest_ff(table_speed, table_pwm, measured):
    """PWM that gives each table speed, from the measured curve (pwm_i -> measured_i), linear."""
    curve = sorted(zip(measured, table_pwm))
    out = [table_pwm[0]]                                  # the dead-zone point is not measured here
    for s in table_speed[1:]:
        lower = [c for c in curve if c[0] <= s]
        upper = [c for c in curve if c[0] > s]
        if not upper:
            out.append(100.0)
            continue
        (s0, p0), (s1, p1) = (lower[-1] if lower else (0.0, table_pwm[0])), upper[0]
        out.append(min(100.0, max(0.0, p0 + (p1 - p0) * (s - s0) / (s1 - s0))))
    return out


def cmd_ff(link, robot, cfg, args):
    values = get_all(link, cfg)
    speeds, pwms = values[FF_SPEED:FF_SPEED + N], values[FF_PWM:FF_PWM + N]
    hold = cfg['ff_hold_time']
    confirm(f'FF test: kp and ki set to 0 (feed-forward only), each table speed for {hold:.1f} s, up to '
            f'{speeds[-1]:.1f} rad/s = {speeds[-1] * robot["radius"]:.2f} m/s. '
            'ARE THE WHEELS LIFTED OFF THE GROUND?', args)
    original = {name: values[IDX[name]] for name in ('kp', 'ki')}
    measured = []
    try:
        for name in original:
            set_param(link, cfg, name, 0.0)
        print('point  speed   pwm     measured M1..M4 [rad/s]          mean')
        for i in range(1, N):
            start = len(link.frames)
            t_on = time.monotonic()
            link.drive(speeds[i] * robot['radius'], 0.0, 0.0)
            time.sleep(hold)
            samples = wheel_speeds(link.frames[start:], robot['rad_per_count'], robot['period'])
            late = [s for t, s in samples if t >= t_on + hold / 2]
            mean = [sum(s[m] for s in late) / len(late) for m in range(4)] if late else [0.0] * 4
            measured.append(sum(mean) / 4)
            print(f'{i:5}  {speeds[i]:5.2f}  {pwms[i]:6.2f}  ' + ' '.join(f'{x:6.2f}' for x in mean)
                  + f'   {measured[-1]:6.2f}')
    finally:
        restore(link, cfg, original)
    new = suggest_ff(speeds, pwms, [0.0] + measured)
    print('\nSuggested ff_pwm (same speeds; the dead-zone point ff_pwm0 is kept):')
    print('ff_pwm  ' + ' '.join(f'{x:6.2f}' for x in new))
    print('Apply with:  python3 ~/NALA/scripts/tune_motors.py set '
          + ' '.join(f'ff_pwm{i} {new[i]:.2f}' for i in range(1, N)))


def restore(link, cfg, original):
    """Stop, then put the original values back: one try each, short timeout. Never hangs."""
    failed = None
    try:
        link.stop()
        time.sleep(0.1)                       # the MCU handles the zero command first (else: busy)
        for name, value in original.items():
            set_param(link, cfg, name, value, attempts=1, timeout=cfg['reply_timeout'])
        print('Restored ' + ', '.join(f'{n} = {fmt(v, 4)}' for n, v in original.items()) + '.')
    except (LinkError, serial.SerialException, OSError) as error:
        failed = error
    if failed:
        print(f'ERROR: Restore failed, please power cycle the MCU ({failed}). Original values: '
              + ', '.join(f'{n} = {fmt(v, 4)}' for n, v in original.items()), file=sys.stderr)
        raise SystemExit(1)


def base_node_running():
    return subprocess.run(['pgrep', '-f', 'nala_base/base_node'], capture_output=True).returncode == 0


def main():
    with open(os.path.join(REPO, 'config', 'base.yaml')) as f:
        base = yaml.safe_load(f)['base_node']['ros__parameters']
    with open(os.path.join(REPO, 'config', 'tune_motors.yaml')) as f:
        cfg = yaml.safe_load(f)['tune_motors']
    robot = {'radius': base['wheel_radius'], 'k': base['half_track_width'] + base['half_wheelbase'],
             'rad_per_count': 2 * math.pi / base['encoder_counts_per_rev'], 'period': base['feedback_period']}

    parser = argparse.ArgumentParser(description='Tune the MCU wheel speed controller (setting mode).')
    parser.add_argument('--port', default=base['serial_port'])
    parser.add_argument('--yes', action='store_true', help='do not ask before a motion')
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('show', help='all parameters').set_defaults(func=cmd_show)
    p = sub.add_parser('set', help='set parameters in RAM: NAME VALUE [NAME VALUE ...]')
    p.add_argument('pairs', nargs='+')
    p.set_defaults(func=cmd_set)
    sub.add_parser('save', help='write RAM to the EEPROM').set_defaults(
        func=simple_op(mp.OP_SAVE, 'Saved in the MCU EEPROM (kept after power-off).'))
    sub.add_parser('reload', help='RAM = the EEPROM values').set_defaults(
        func=simple_op(mp.OP_RELOAD, 'Loaded the EEPROM values into RAM.'))
    sub.add_parser('defaults', help='RAM = the firmware defaults (config.h)').set_defaults(
        func=simple_op(mp.OP_DEFAULTS, 'Loaded the firmware defaults into RAM ("save" to keep them).'))
    p = sub.add_parser('step', help='one speed step')
    p.add_argument('--vx', type=float, default=0.0, help='m/s forward')
    p.add_argument('--vy', type=float, default=0.0, help='m/s left')
    p.add_argument('--wz', type=float, default=0.0, help='rad/s counter-clockwise')
    p.add_argument('--time', type=float, help='s (default: step_time in config/tune_motors.yaml)')
    p.add_argument('--csv', help='write the wheel speeds to this file')
    p.set_defaults(func=cmd_step)
    p = sub.add_parser('sweep', help='steps in 6 directions at several speeds')
    p.add_argument('--floor', action='store_true', help='short steps (floor_max_dist) for driving on the floor')
    p.set_defaults(func=cmd_sweep)
    sub.add_parser('ff', help='measure the feed-forward table (wheels in the air)').set_defaults(func=cmd_ff)
    args = parser.parse_args()

    if base_node_running():
        raise SystemExit('base_node is running (app 1 or 2). Stop it first: it uses the same UART.')
    link = None
    code = 0
    try:
        link = Link(args.port, base['baud_rate'], base['command_rate'])
        time.sleep(0.2)
        if not link.frames:
            raise LinkError(f'no state frames on {args.port}. Is the MCU powered and connected?')
        try:
            link.request(mp.OP_GET, 0, timeout=cfg['reply_timeout'], attempts=cfg['attempts'])
        except LinkError:
            raise LinkError('the MCU sends state frames but does not answer parameter requests: '
                            'its firmware is older than v2.2.0 (no setting mode).') from None
        args.func(link, robot, cfg, args)
    except KeyboardInterrupt:
        print('\nCtrl+C: stopped.')
        code = 130
    except (LinkError, serial.SerialException, OSError) as error:
        print(f'ERROR: {error}', file=sys.stderr)
        code = 2
    finally:
        if link:
            link.close()
    return code


if __name__ == '__main__':
    sys.exit(main())
