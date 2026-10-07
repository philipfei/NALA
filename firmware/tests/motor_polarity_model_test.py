"""Model of motor_functions.c (v1) vs the v0 behaviour. Checks the direction-pin level and the PWM compare value."""
def v0(m):                      # original: m integer percent
    mm = -m
    if mm < 0: level, mm = 0, -mm
    else: level = 1
    return level, int((100 - mm) / 100.0 * 0xFF)

def v1(m, invert):              # NALA_v1 (float percent)
    level = 1 if m <= 0 else 0
    if invert: level = 1 - level
    d = abs(m) * 2.55 + 0.5
    d = min(d, 255.0)
    return level, 255 - int(d)

INV = {1: 0, 2: 1, 3: 1, 4: 0}  # MOTORn_DIR_INVERT in config.h
ok = True
for motor, inv in INV.items():
    diff_level = diff_ocr = 0
    for m in range(-100, 101):
        l0, o0 = v0(m); l1, o1 = v1(float(m), inv)
        if m != 0:                                   # at power 0 the direction is irrelevant
            if inv == 0 and l0 != l1: diff_level += 1
            if inv == 1 and l0 == l1: diff_level += 1
        if abs(o0 - o1) > 1: diff_ocr += 1
    good = (diff_level == 0 and diff_ocr == 0); ok &= good
    print(("PASS " if good else "FAIL ") + f"M{motor}: direction {'INVERTED' if inv else 'same as v0'} for all 200 non-zero powers; PWM within 1 count of v0 for every integer percent")
# resolution: v1 resolves finer than 1 %
steps = len({v1(x / 10.0, 0)[1] for x in range(0, 1001)})
print(("PASS " if steps > 200 else "FAIL ") + f"v1 output has {steps} distinct duty levels over 0..100 % (v0: 101)")
raise SystemExit(0 if ok and steps > 200 else 1)
