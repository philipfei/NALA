"""Model test of the v2.2.0 setting mode (Python copy of the logic in main.c).

Checks handle_param() (ops, index, range, NaN/Inf, busy rule), the feed-forward table states
(unchecked after an edit, checked at the next drive command and at save), the EEPROM block
(layout version, CRC-16 XMODEM, rejected blocks) and the target ramp with the clamped
control time step. Run: python3 param_model_test.py
"""
import math
import struct

N_FF = 10
NAMES = ['kp', 'ki', 'i_limit', 'accel_limit'] + [f'ff_speed{i}' for i in range(N_FF)] + [f'ff_pwm{i}' for i in range(N_FF)]
P_ACCEL, P_FF_SPEED, P_FF_PWM, NUM = 3, 4, 14, 24
OP_GET, OP_SET, OP_SAVE, OP_DEFAULTS, OP_RELOAD = range(5)
OK, BAD_INDEX, OUT_OF_RANGE, BUSY, BAD_TABLE, BAD_OP, NO_EEPROM = range(7)
FF_OK, FF_UNCHECKED, FF_INVALID = range(3)
LAYOUT_VERSION = 1
PERIOD = 0.05
DEFAULTS = ([1.0, 8.0, 25.0, 0.0] + [0.0, 3.5, 6.6, 8.1, 9.4, 10.7, 11.5, 11.85, 12.2, 13.0]
            + [9.0, 15.0, 21.5, 29.5, 37.5, 45.5, 54.0, 63.0, 72.0, 100.0])           # config.h


def f32(x):                                   # AVR float = IEEE 754 single
    return struct.unpack('<f', struct.pack('<f', x))[0]


def param_max(idx):                           # param_max() in main.c
    if idx >= P_FF_PWM:
        return 100.0
    if idx >= P_FF_SPEED:
        return 20.0
    return [50.0, 200.0, 100.0, 200.0][idx]


def in_range(idx, v):                         # param_in_range(): NaN and +-Inf fail
    return math.isfinite(v) and 0.0 <= v <= param_max(idx)


def table_valid(p):                           # ff_table_valid()
    s = p[P_FF_SPEED:P_FF_SPEED + N_FF]
    return s[0] == 0.0 and all(s[i] > s[i - 1] for i in range(1, N_FF))


def crc16_xmodem(data):                       # _crc_xmodem_update() of avr-libc, init 0
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def block(params, version=LAYOUT_VERSION):    # param_block_t as bytes (AVR: no padding)
    body = bytes([version]) + struct.pack(f'<{NUM}f', *params)
    return body + struct.pack('<H', crc16_xmodem(body))


class Mcu:
    def __init__(self, eeprom=None):
        self.p = [f32(x) for x in DEFAULTS]
        self.ff = FF_OK
        self.eeprom = eeprom                  # bytes of the EEPROM block, or None (erased)
        self.in_eeprom = self.load_eeprom()
        self.goal = [0.0] * 4
        self.w = [0.0] * 4

    def load_eeprom(self):                    # load_eeprom()
        b = self.eeprom
        if b is None or len(b) != 1 + 4 * NUM + 2:
            return 0
        values = list(struct.unpack(f'<{NUM}f', b[1:-2]))
        if b[0] != LAYOUT_VERSION or struct.unpack('<H', b[-2:])[0] != crc16_xmodem(b[:-2]) or not table_valid(values):
            return 0
        if not all(in_range(i, v) for i, v in enumerate(values)):
            return 0
        self.p, self.ff = values, FF_OK
        return 1

    def stopped(self):
        return all(g == 0.0 for g in self.goal) and all(w == 0.0 for w in self.w)

    def param(self, op, idx, value=0.0):      # handle_param() -> (status, value after the op)
        value = f32(value)
        st = OK
        if op > OP_RELOAD:
            st = BAD_OP
        elif op in (OP_GET, OP_SET) and idx >= NUM:
            st = BAD_INDEX
        elif op != OP_GET and not self.stopped():
            st = BUSY
        elif op == OP_SET:
            if not in_range(idx, value):
                st = OUT_OF_RANGE
            else:
                self.p[idx] = value
                self.in_eeprom = 0
                if idx >= P_FF_SPEED:
                    self.ff = FF_UNCHECKED
        elif op == OP_SAVE:
            if not table_valid(self.p):
                st = BAD_TABLE
            else:
                self.eeprom = block(self.p)
                self.ff, self.in_eeprom = FF_OK, 1
        elif op == OP_DEFAULTS:
            self.p, self.ff, self.in_eeprom = [f32(x) for x in DEFAULTS], FF_OK, 0
        elif op == OP_RELOAD:
            if self.load_eeprom():
                self.in_eeprom = 1
            else:
                st = NO_EEPROM
        return st, (self.p[idx] if idx < NUM else 0.0)

    def command(self, vx):                    # handle_command(), forward only (enough for these checks)
        moving = vx != 0
        if moving and self.ff == FF_UNCHECKED:
            self.ff = FF_OK if table_valid(self.p) else FF_INVALID
        if self.ff == FF_INVALID:
            vx, moving = 0, False
        self.goal = [vx / 1000 / 0.040] * 4
        for m in range(4):
            if abs(self.goal[m]) < 0.01:
                self.goal[m] = self.w[m] = 0.0

    def step(self, periods=1):                # control_step(): ramp with dt_ctrl
        dt_ctrl = PERIOD if periods > 1 else periods * PERIOD
        for m in range(4):
            goal, prev, nxt = self.goal[m], self.w[m], self.goal[m]
            if goal != 0.0 and self.p[P_ACCEL] > 0.0:
                st = self.p[P_ACCEL] * dt_ctrl
                nxt = min(max(nxt, prev - st), prev + st)
            self.w[m] = nxt

    def flags(self):
        return (self.in_eeprom << 2) | ((self.ff == FF_INVALID) << 3)


res = []


def check(name, ok, info=''):
    print(('PASS ' if ok else 'FAIL ') + name + (('  ' + str(info)) if info and not ok else ''))
    res.append(bool(ok))


def main():
    # Names and CRC
    check('24 parameters, Pi order', len(NAMES) == NUM and NAMES[P_FF_SPEED] == 'ff_speed0' and NAMES[P_FF_PWM] == 'ff_pwm0')
    check('crc16 xmodem("123456789") = 0x31C3', crc16_xmodem(b'123456789') == 0x31C3)
    check('EEPROM block size 99 bytes', len(block(DEFAULTS)) == 99)
    check('defaults are a valid table', table_valid(DEFAULTS))

    # Boot
    check('erased EEPROM -> defaults, flag bit2 = 0', (lambda m: m.p == [f32(x) for x in DEFAULTS] and m.flags() == 0)(Mcu()))
    good = [f32(x) for x in DEFAULTS]
    good[0] = f32(2.5)
    check('valid EEPROM block wins at boot, bit2 = 1', (lambda m: m.p[0] == 2.5 and m.flags() == 4)(Mcu(block(good))))
    broken = bytearray(block(good))
    broken[10] ^= 0x01
    check('one flipped bit -> CRC fails -> defaults', Mcu(bytes(broken)).p[0] == 1.0)
    check('other layout version -> defaults', Mcu(block(good, version=2)).p[0] == 1.0)
    nan_block = list(good)
    nan_block[1] = float('nan')
    check('NaN in the block -> defaults', Mcu(block(nan_block)).p[1] == 8.0)
    bad_table = list(good)
    bad_table[P_FF_SPEED + 3] = bad_table[P_FF_SPEED + 2]
    check('non-increasing table in the block -> defaults', Mcu(block(bad_table)).p[0] == 1.0)

    # Ops
    m = Mcu()
    check('get kp', m.param(OP_GET, 0) == (OK, 1.0))
    check('set ki 12', m.param(OP_SET, 1, 12.0) == (OK, 12.0))
    check('set NaN -> out of range, unchanged', m.param(OP_SET, 1, float('nan')) == (OUT_OF_RANGE, 12.0))
    check('set +Inf -> out of range', m.param(OP_SET, 0, float('inf'))[0] == OUT_OF_RANGE)
    check('set -Inf -> out of range', m.param(OP_SET, 0, float('-inf'))[0] == OUT_OF_RANGE)
    check('set negative -> out of range', m.param(OP_SET, 0, -0.1)[0] == OUT_OF_RANGE)
    check('set above max -> out of range', m.param(OP_SET, 0, 50.01)[0] == OUT_OF_RANGE)
    check('index 24 -> bad index', m.param(OP_GET, 24)[0] == BAD_INDEX)
    check('op 5 -> bad op', m.param(5, 0)[0] == BAD_OP)
    check('reload without EEPROM -> no EEPROM', m.param(OP_RELOAD, 0)[0] == NO_EEPROM)
    check('save, then bit2 = 1', m.param(OP_SAVE, 0)[0] == OK and m.flags() == 4)
    m.param(OP_SET, 1, 3.0)
    check('a set after a save clears bit2', m.flags() == 0)
    check('reload brings the saved ki 12 back', m.param(OP_RELOAD, 1) == (OK, 12.0) and m.flags() == 4)
    check('defaults -> ki 8, bit2 = 0', m.param(OP_DEFAULTS, 1) == (OK, 8.0) and m.flags() == 0)

    # Busy rule
    m = Mcu()
    m.command(100)
    check('get while moving is allowed', m.param(OP_GET, 0)[0] == OK)
    check('set while moving -> busy', m.param(OP_SET, 0, 2.0) == (BUSY, 1.0))
    check('save while moving -> busy', m.param(OP_SAVE, 0)[0] == BUSY)
    m.command(0)
    check('set after a stop is allowed', m.param(OP_SET, 0, 2.0) == (OK, 2.0))

    # FF table edits
    m = Mcu()
    m.param(OP_SET, P_FF_SPEED + 1, 7.0)          # now 0, 7.0, 6.6 ...: not increasing
    check('editing a point is never refused', m.ff == FF_UNCHECKED and m.flags() == 0)
    m.command(0)
    check('a zero command does not check the table', m.ff == FF_UNCHECKED)
    check('save refuses the broken table', m.param(OP_SAVE, 0)[0] == BAD_TABLE)
    m.command(100)
    check('drive with the broken table is ignored, bit3 = 1', m.goal == [0.0] * 4 and m.flags() == 8)
    m.param(OP_SET, P_FF_SPEED + 1, 3.5)          # fixed again
    m.command(100)
    check('after the fix the next drive works, bit3 = 0', m.goal[0] > 0 and m.flags() == 0)

    # Ramp and the clamped time step
    m = Mcu()
    m.command(0)
    m.param(OP_SET, P_ACCEL, 10.0)                # 10 rad/s^2 -> 0.5 rad/s per period
    m.command(200)                                # goal 5 rad/s
    m.step()
    check('ramp: first step 0.5 rad/s', abs(m.w[0] - 0.5) < 1e-6, m.w[0])
    m.step(periods=7)
    check('after a 7-period pause the ramp moves one period only (1.0, not 4.0)', abs(m.w[0] - 1.0) < 1e-6, m.w[0])
    m.command(0)
    check('zero command stops at once, ramp or not', m.w == [0.0] * 4)
    m.param(OP_SET, P_ACCEL, 0.0)
    m.command(200)
    m.step()
    check('accel_limit 0 = jump to the goal', abs(m.w[0] - 5.0) < 1e-6, m.w[0])

    print(f'\n{sum(res)}/{len(res)} passed')
    raise SystemExit(0 if all(res) else 1)


if __name__ == '__main__':
    main()
