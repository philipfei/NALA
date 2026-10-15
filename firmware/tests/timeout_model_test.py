"""Python model of the command-timeout logic in main.c (handle_command + control_step).
Time is simulated in ms; control ticks every 50 ms. Checks the TIMING/LOGIC, not the AVR build.
v2.2.0: a command handled since the last step counts only the periods after it (at least 1)."""
PERIOD = 50; TIMEOUT_MS = 200
TICKS = (TIMEOUT_MS + PERIOD - 1) // PERIOD          # CMD_TIMEOUT_TICKS

class Fw:
    def __init__(self):
        self.age = 0; self.active = 0; self.target_zero = True; self.reports = []; self.stop_time = None
        self.fresh = 0; self.before = 0
    def frame(self, v, t, pending=0):                # handle_command(); pending = tick_periods at that moment
        self.age = 0; self.active = int(any(v)); self.target_zero = not any(v); self.stop_time = None
        self.fresh = 1; self.before = pending
    def tick(self, t, periods=1):                    # control_step() timeout part
        if self.active:
            if self.fresh:
                age = periods - self.before if periods > self.before + 1 else 1
            else:
                age = self.age + periods
            self.age = min(255, age)
            if self.age > TICKS:
                self.target_zero = True; self.active = 0; self.reports.append(t); self.stop_time = t
        self.fresh = 0

def run(frames, until=3000):
    fw = Fw(); fi = sorted(frames)
    for t in range(0, until + 1):
        while fi and fi[0][0] == t: fw.frame(fi[0][1], t); fi.pop(0)   # frames processed before the tick at same ms
        if t > 0 and t % PERIOD == 0: fw.tick(t)
    return fw

res = []
def check(name, ok, info=""):
    print(("PASS " if ok else "FAIL ") + name + (("  " + info) if info else "")); res.append(ok)

# 1. stop time lies in [TIMEOUT, TIMEOUT + PERIOD] after the last frame, for any phase of the frame inside the period
worst_lo, worst_hi, ok = 10**9, 0, True
for phase in range(0, PERIOD):                       # frame at 1000+phase ms (every phase inside one period)
    t0 = 1000 + phase
    fw = run([(t0, (5, 0, 0))])
    dt = fw.stop_time - t0
    worst_lo = min(worst_lo, dt); worst_hi = max(worst_hi, dt)
    ok &= (TIMEOUT_MS <= dt <= TIMEOUT_MS + PERIOD)
check(f"stop happens {TIMEOUT_MS}..{TIMEOUT_MS+PERIOD} ms after the last frame (all {PERIOD} phases)", ok, f"observed {worst_lo}..{worst_hi} ms")
# 2. exactly one report
fw = run([(1050, (5, 0, 0))]); check("reported exactly once", len(fw.reports) == 1, str(fw.reports))
# 3. frames arriving every 50 ms (20 Hz) or every 150 ms keep it alive
fw = run([(t, (5, 0, 0)) for t in range(100, 3000, 50)]); check("20 Hz heartbeat -> never times out", fw.reports == [])
fw = run([(t, (5, 0, 0)) for t in range(100, 3000, 150)]); check("150 ms heartbeat -> never times out", fw.reports == [])
fw = run([(t, (5, 0, 0)) for t in range(100, 3000, 300)]); check("300 ms spacing (too slow) -> times out between frames", len(fw.reports) > 0)
# 4. a zero command never times out
fw = run([(100, (0, 0, 0))]); check("zero command never times out", fw.reports == [] )
# 5. moving then explicit zero -> no timeout report
fw = run([(100, (5, 0, 0)), (300, (0, 0, 0))]); check("explicit stop frame cancels the timeout", fw.reports == [])
# 6. resume after timeout
fw = run([(100, (5, 0, 0)), (1500, (5, 0, 0))])
check("timeout, then a new frame resumes, then times out again", len(fw.reports) == 2, str(fw.reports))
# 7. late main loop (3 periods elapsed at once) still counts correctly
f = Fw(); f.frame((5, 0, 0), 0); f.tick(300, periods=3); f.tick(600, periods=3)
check("late loop: 6 periods counted at once triggers the stop", f.reports == [600])
# 9. v2.2.0: a command that waited in its slot during a 7-period pause (EEPROM save) is not a timeout
f = Fw(); f.frame((5, 0, 0), 350, pending=7); f.tick(350, periods=7)
check("command handled after a 7-period pause -> no timeout", f.reports == [] and f.age == 1, f"age {f.age}")
for t in (400, 450, 500, 550): f.tick(t)
check("... and it still times out 200 ms after it was handled, without new frames", f.reports == [550], str(f.reports))
# 10. a 7-period pause AFTER a command counts in full
f = Fw(); f.frame((5, 0, 0), 0, pending=0); f.tick(350, periods=7)
check("7-period pause after the last command -> timeout at once", f.reports == [350], str(f.reports))
# 8. age saturates, no overflow
f = Fw(); f.frame((5, 0, 0), 0); f.active = 1
for _ in range(300): f.age = min(255, f.age + 1)
check("age saturates at 255", f.age == 255)
print(f"\n{sum(res)}/{len(res)} passed"); raise SystemExit(0 if all(res) else 1)
