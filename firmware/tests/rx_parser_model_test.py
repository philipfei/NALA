"""Line-by-line Python model of ISR(USART0_RX_vect) in main.c (NALA_v0) + test vectors.
It checks the PROTOCOL LOGIC (not the AVR build). Run: python rx_parser_model_test.py"""
H1, H2, H2E, PAYLOAD, ECHO_LEN = 0x80, 0x86, 0x87, 3, 1

def s8(x):  # (int8_t) cast
    return x - 256 if x > 127 else x

class Rx:
    def __init__(self, echo_default=0):
        self.reading = 0; self.count = 0; self.len = 0; self.is_echo = 0; self.prev = 0
        self.buf = [0] * PAYLOAD; self.cmd_val = [0] * PAYLOAD; self.cmd_ready = 0
        self.echo_enabled = echo_default; self.echo_ack = 0; self.cmds = []; self.acks = []
    def isr(self, b):
        if self.reading:
            self.buf[self.count] = s8(b); self.count += 1
            if self.count >= self.len:
                if self.is_echo:
                    if self.buf[0] == 0 or self.buf[0] == 1:
                        self.echo_enabled = self.buf[0]; self.echo_ack = 1 + self.buf[0]
                else:
                    self.cmd_val = list(self.buf); self.cmd_ready = 1
                self.reading = 0; self.count = 0; self.prev = 0
            return
        if self.prev == H1 and (b == H2 or b == H2E):
            self.reading = 1; self.count = 0
            self.is_echo = int(b == H2E)
            self.len = ECHO_LEN if self.is_echo else PAYLOAD
        self.prev = b
    def feed(self, data):
        for b in data:
            self.isr(b)
            if self.cmd_ready: self.cmds.append(tuple(self.cmd_val)); self.cmd_ready = 0   # main loop consumes
            if self.echo_ack: self.acks.append("on" if self.echo_ack == 2 else "off"); self.echo_ack = 0
        return self

def check(name, got, want):
    ok = got == want; print(("PASS " if ok else "FAIL ") + name + ("" if ok else f"  got={got} want={want}")); return ok

r = []
r.append(check("velocity frame Vx=0.10",           Rx().feed([0x80,0x86,10,0,0]).cmds, [(10,0,0)]))
r.append(check("negative values (int8)",           Rx().feed([0x80,0x86,0xF6,0x05,0x9C]).cmds, [(-10,5,-100)]))
r.append(check("junk before frame",                Rx().feed([1,2,3,0x80,0x00,0x80,0x86,1,2,3]).cmds, [(1,2,3)]))
r.append(check("two back-to-back frames",          Rx().feed([0x80,0x86,1,2,3,0x80,0x86,4,5,6]).cmds, [(1,2,3),(4,5,6)]))
r.append(check("payload may contain 0x80 0x87 (no retrigger)", Rx().feed([0x80,0x86,0x80,0x87,0x01]).cmds, [(-128,-121,1)]))
r.append(check("...and that did NOT toggle echo",  Rx().feed([0x80,0x86,0x80,0x87,0x01]).echo_enabled, 0))
r.append(check("echo ON frame",                    Rx().feed([0x80,0x87,0x01]).echo_enabled, 1))
x = Rx(echo_default=1).feed([0x80,0x87,0x00])
r.append(check("echo OFF frame (default was on)",  (x.echo_enabled, x.acks), (0, ["off"])))
r.append(check("ack text for ON",                  Rx().feed([0x80,0x87,0x01]).acks, ["on"]))
x = Rx().feed([0x80,0x87,0x05])
r.append(check("invalid E=5 ignored, no ack",      (x.echo_enabled, x.acks), (0, [])))
x = Rx().feed([0x80,0x87,0x05,0x80,0x86,7,8,9])
r.append(check("frame after invalid echo frame still works", x.cmds, [(7,8,9)]))
x = Rx().feed([0x80,0x87,0x01,0x80,0x86,1,1,1,0x80,0x87,0x00])
r.append(check("echo on -> command -> echo off",   (x.cmds, x.acks, x.echo_enabled), ([(1,1,1)], ["on","off"], 0)))
# Original protocol compatibility: any stream the ORIGINAL parser accepted gives the same velocity commands.
def orig(data):
    reading = 0; count = 0; prev = 0; buf = [0]*5; out = []
    for b in data:
        if reading:
            buf[count] = s8(b); count += 1
            if count >= 3: out.append(tuple(buf[:3])); reading = 0; count = 0; prev = 0
            continue
        if prev == 0x80 and b == 0x86: reading = 1; count = 0
        prev = b
    return out
import random; random.seed(1); same = True
for _ in range(20000):
    n = random.randint(1, 40)
    data = [random.choice([0x80, 0x86, random.choice([v for v in range(256) if v != 0x87])]) for _ in range(n)]   # 0x87 excluded -> echo frame never starts
    if Rx().feed(data).cmds != orig(data): same = False; print("MISMATCH", data); break
r.append(check("20000 random streams without 0x87: identical to ORIGINAL parser", same, True))
print(f"\n{sum(r)}/{len(r)} passed"); raise SystemExit(0 if all(r) else 1)
