"""Serial feedback line budget at 38400 baud 8N1 (10 bits per byte), firmware v1.1.0."""
BAUD = 38400; MS_PER_CHAR = 10 * 1000 / BAUD
def line(vals): return "c %d %d %d %d\n" % tuple(vals)
worst = max((line(v) for v in [(-2147483648,)*4, (2147483647,)*4, (12345, -6789, 0, 100000)]), key=len)
n = len(worst); t = n * MS_PER_CHAR
print(f"worst-case line: {n} chars -> {t:.1f} ms at {BAUD} baud")
for period_ms, label in ((50, "every 50 ms (20 Hz)"), (100, "every 100 ms (10 Hz)")):
    load = t / period_ms * 100
    print(f"  feedback {label:<22}: line load {load:5.1f} %  -> {'FITS, buffer drains each period' if t < period_ms else 'DOES NOT FIT, backlog grows'}")
# queue simulation: TX buffer 128 bytes, lines queued every period, drained at 1 byte per MS_PER_CHAR, drop if no room
def simulate(period_ms, extra_echo=False, secs=20):
    buf = 0.0; dropped = 0; sent = 0; t = 0.0; peak = 0
    while t < secs * 1000:
        buf = max(0.0, buf - period_ms / MS_PER_CHAR)            # drain during one period
        if 128 - buf >= n: buf += n; sent += 1
        else: dropped += 1
        if extra_echo:                                           # command echo (~30 chars) at 20 Hz
            if 128 - buf >= 30: buf += 30
        peak = max(peak, buf); t += period_ms
    return sent, dropped, peak
s, d, pk = simulate(50); print(f"  simulated 20 s, period  50 ms: lines sent {s:4d}, dropped {d:3d}, peak buffer {pk:5.1f} of 128 bytes")
s, d, pk = simulate(50, extra_echo=True); print(f"  20 Hz feedback + command echo ON at 20 Hz: sent {s}, dropped {d}, peak {pk:.0f}/128 bytes")
