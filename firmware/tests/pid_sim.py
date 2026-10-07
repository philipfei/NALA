"""Closed-loop simulation of one wheel: OLD control law (v0) vs NEW law (v1).

Plant (assumed, NOT measured):  d(w)/dt = (G * pwm_eff - w) / tau
    G    : steady-state gain [rad/s per 1 % pwm]. The original feed-forward (4 pwm per rad/s) implies G ~ 0.25.
    tau  : mechanical/electrical time constant [s] (unknown -> swept)
    db   : static-friction dead band [% pwm]: below it the wheel does not move (swept)
Sensor : encoder counting edges of channel A, 1536 counts/rev, speed = counts in the last period / period.
Actuator: PWM output; v0 uses integer percent, v1 uses 8-bit duty (1/255).

Scenario: speed step 0 -> REF at t=0.5 s, hold, stop (command 0) at t=3.5 s, watch until t=6 s.
Run:  python pid_sim.py
"""
import math, itertools, sys

CPR = 1536.0                      # counts per wheel revolution
FF = 4.0                          # feed-forward [% pwm per rad/s] (from the original code)
REF = 3.0                         # rad/s
T_STEP, T_STOP, T_END = 0.5, 3.5, 6.0
H = 0.001                         # simulation step 1 ms


def plant_step(w, pwm, G, tau, db):
    eff = 0.0
    if abs(pwm) > db:
        eff = math.copysign(abs(pwm) - db, pwm)
    return w + H * (G * eff - w) / tau


def default_ref(t):
    return REF if T_STEP <= t < T_STOP else 0.0


def simulate(law, T, G, tau, db, params, ref_fn=default_ref):
    """Return (t, w_true) arrays. `law` is a controller object with .command(ref) and .tick(meas, dt)->pwm."""
    n = int(T_END / H)
    w = 0.0; theta = 0.0; count_prev = 0
    pwm = 0.0; ref = 0.0
    tick_every = int(round(T / H)); ws = []
    for k in range(n):
        t = k * H
        new_ref = ref_fn(t)
        if new_ref != ref:                      # a new command arrives
            ref = new_ref
            pwm = law.command(ref)
        if k > 0 and k % tick_every == 0:
            count = math.floor(theta * CPR / (2 * math.pi))
            meas = (count - count_prev) * 2 * math.pi / CPR / T     # quantised speed over the last period
            count_prev = count
            pwm = law.tick(meas, T)
        w = plant_step(w, pwm, G, tau, db)
        theta += w * H
        ws.append(w)
    return ws


# ---------------------------------------------------------------- v0 law (as in the original code)
class OldLaw:
    def __init__(self, kp=0.74, ki=3.7, kd=0.0644):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.I = 0.0; self.e_prev = 0.0; self.ref = 0.0; self.pwm = 0
    def command(self, ref):                      # calc_angular_speed(): pwm reset to the feed-forward, PID state kept
        self.ref = ref; self.pwm = int(ref * FF); return self.pwm
    def tick(self, meas, dt):
        e = meas - self.ref                      # err = measured - target
        self.I += e * dt
        d = (e - self.e_prev) / dt
        u = self.kp * e + self.ki * self.I + self.kd * d
        self.e_prev = e
        self.pwm = max(-100, min(100, self.pwm - int(u)))   # int() truncates toward zero
        return self.pwm


# ---------------------------------------------------------------- v1 law
class NewLaw:
    def __init__(self, kp, ki, imax=25.0):
        self.kp, self.ki, self.imax = kp, ki, imax
        self.I = 0.0; self.ref = 0.0; self.pwm = 0.0
    @staticmethod
    def q(p):                                    # 8-bit PWM duty: 1/255 resolution
        return round(p * 2.55) / 2.55
    def command(self, ref):
        if ref == 0.0 or (ref > 0) != (self.ref > 0):
            self.I = 0.0                         # stop or direction change: forget the old integral
        self.ref = ref
        self.pwm = self.q(max(-100, min(100, FF * ref + self.I))) if ref != 0.0 else 0.0
        return self.pwm
    def tick(self, meas, dt):
        if self.ref == 0.0:                      # target zero: output off, integrator cleared
            self.I = 0.0; self.pwm = 0.0; return 0.0
        e = self.ref - meas                      # textbook sign
        u_unsat = FF * self.ref + self.kp * e + self.I
        if abs(u_unsat) < 100 or (u_unsat > 0) != (e > 0):      # anti-windup: do not integrate into saturation
            self.I = max(-self.imax, min(self.imax, self.I + self.ki * e * dt))
        self.pwm = self.q(max(-100, min(100, FF * self.ref + self.kp * e + self.I)))
        return self.pwm


# ---------------------------------------------------------------- metrics
def metrics(ws):
    n_stop = int(T_STOP / H); n_step = int(T_STEP / H)
    run = ws[n_step:n_stop]; stop = ws[n_stop:]
    peak = max(run)
    overshoot = max(0.0, (peak - REF) / REF * 100)
    # settling: last time |w - REF| > 0.15 rad/s during the run phase
    band = 0.15; last_out = 0
    for i, v in enumerate(run):
        if abs(v - REF) > band: last_out = i
    settle = (last_out + 1) * H
    # steady-state ripple in the last second of the run phase
    tail = run[-1000:]
    ripple = max(tail) - min(tail)
    # after the stop command: time until |w| < 0.1 for good, count of direction reversals (ringing), max reverse speed
    last_big = 0
    for i, v in enumerate(stop):
        if abs(v) > 0.1: last_big = i
    t_stop = (last_big + 1) * H
    rev = 0; sign = 0
    for v in stop:
        if abs(v) > 0.15:
            s = 1 if v > 0 else -1
            if sign and s != sign: rev += 1
            sign = s
    max_rev = -min(0.0, min(stop))
    return dict(over=overshoot, settle=settle, ripple=ripple, t_stop=t_stop, rev=rev, max_rev=max_rev)


GRID_G = (0.15, 0.25, 0.40)
GRID_TAU = (0.05, 0.10, 0.20, 0.40)
GRID_DB = (0.0, 6.0)


def sweep(make_law, T):
    out = []
    for G, tau, db in itertools.product(GRID_G, GRID_TAU, GRID_DB):
        ws = simulate(make_law(), T, G, tau, db, None)
        m = metrics(ws); m.update(G=G, tau=tau, db=db); out.append(m)
    return out


def summarize(name, res):
    def agg(key, f): return f(r[key] for r in res)
    bad = [r for r in res if r['rev'] >= 2 or r['ripple'] > 0.6 or r['t_stop'] > 1.0]
    print(f"{name:<44} overshoot max {agg('over', max):6.1f}%  settle max {agg('settle', max):4.2f}s  "
          f"ripple max {agg('ripple', max):4.2f}  stop-time max {agg('t_stop', max):4.2f}s  "
          f"reversals max {agg('rev', max):2d}  | bad cases {len(bad):2d}/{len(res)}")
    return bad


if __name__ == "__main__":
    print("OLD law (v0 structure, int truncation, integral kept) -------------------------")
    for T in (0.1, 0.05):
        summarize(f"old law, period {int(T*1000)} ms, gains 0.74/3.7/0.0644", sweep(OldLaw, T))
    print("\nNEW law (positional PI + feed-forward + anti-windup), period 50 ms ----------------")
    best = None
    for kp, ki in itertools.product((0.5, 1.0, 1.5, 2.0, 3.0), (2, 4, 8, 12, 16)):
        res = sweep(lambda: NewLaw(kp, ki), 0.05)
        score = (sum(1 for r in res if r['rev'] >= 2 or r['ripple'] > 0.6 or r['t_stop'] > 1.0),
                 max(r['over'] for r in res), max(r['settle'] for r in res))
        print(f"  Kp {kp:3.1f}  Ki {ki:4.1f}: ", end="")
        summarize("", res)
        if best is None or score < best[0]: best = (score, kp, ki)
    print("\nbest (fewest bad cases, then smallest overshoot, then fastest settle): Kp %.1f  Ki %.1f" % (best[1], best[2]))


# ---------------------------------------------------------------- extra scenarios for the chosen gains
def extra_scenarios(make_law, T, label):
    import random
    random.seed(3)
    jitter = {}
    def r_down(t):   return 0.0 if t < 0.5 else (3.0 if t < 3.0 else 1.5)           # 0 -> 3 -> 1.5
    def r_rev(t):    return 0.0 if t < 0.5 else (3.0 if t < 3.0 else -3.0)          # 3 -> -3 (direction change)
    def r_jit(t):                                                                     # 3 rad/s +-0.5 changing every 50 ms (nav noise)
        k = int(t / 0.05)
        if k not in jitter: jitter[k] = 3.0 + random.uniform(-0.5, 0.5)
        return 0.0 if t < 0.5 else jitter[k]
    print(f"\nExtra scenarios, {label}: worst case over the plant grid")
    for name, fn, tail_ref in (("3 -> 1.5 rad/s", r_down, 1.5), ("3 -> -3 rad/s (reversal)", r_rev, -3.0), ("jittering command 3 +-0.5", r_jit, None)):
        worst_ripple = 0.0; worst_final_err = 0.0
        for G, tau, db in itertools.product(GRID_G, GRID_TAU, GRID_DB):
            ws = simulate(make_law(), T, G, tau, db, None, ref_fn=fn)
            tail = ws[-1000:]
            worst_ripple = max(worst_ripple, max(tail) - min(tail))
            if tail_ref is not None:
                worst_final_err = max(worst_final_err, abs(sum(tail) / len(tail) - tail_ref))
        print(f"  {name:<28} ripple in last 1 s: {worst_ripple:5.2f} rad/s" + (f"   |  mean error vs target: {worst_final_err:5.2f}" if tail_ref is not None else ""))
