"""Model test of the v2.2.0 UART frames (Python copy of the C code in main.c).

Checks the CRC-8 (poly 0x07, init 0), the receive state machine of the command frame and
the parameter request (IDLE -> HEADER_ACK -> PAYLOAD -> complete, one slot per id), the
command decoding and the byte layout of the state frame and the parameter reply. The Pi
side (src/nala_base/nala_base/mcu_protocol.py) must give the same bytes: checked below.
Run: python3 uart_frame_model_test.py
"""
import os
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'src', 'nala_base'))
from nala_base import mcu_protocol as pi  # noqa: E402  (the Pi code, for the byte comparison)

SYNC, ID_CMD, ID_STATE, CMD_PAYLOAD, STATE_LEN = 0xA5, 0x01, 0x02, 7, 21
ID_PARAM, ID_PARAM_REPLY, REPLY_LEN = 0x03, 0x04, 10
IDLE, HEADER_ACK, PAYLOAD = range(3)


def crc8(data):                       # same loop as crc8() in main.c
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


class Rx:
    """USART0_RX_vect + handle_command() + handle_param() (decoding only).

    process=True: the main loop handles each frame at once. process=False: the main loop is
    blocked (EEPROM save); frames only land in their slot, a newer one replaces an older one
    of the same id. main_loop() then handles what is in the slots, like for (;;) in main().
    """

    def __init__(self, process=True):
        self.state, self.count, self.buf, self.cmds, self.params = IDLE, 0, [0] * CMD_PAYLOAD, [], []
        self.process, self.slot = process, {ID_CMD: None, ID_PARAM: None}

    def isr(self, b):
        if self.state == IDLE:
            if b == SYNC:
                self.state = HEADER_ACK
        elif self.state == HEADER_ACK:
            if b in (ID_CMD, ID_PARAM):
                self.id, self.count, self.state = b, 0, PAYLOAD
            elif b != SYNC:
                self.state = IDLE
        else:
            self.buf[self.count] = b
            self.count += 1
            if self.count >= CMD_PAYLOAD:
                self.slot[self.id] = list(self.buf)
                self.state = IDLE
                if self.process:
                    self.main_loop()

    def main_loop(self):
        if self.slot[ID_CMD] is not None:
            self.handle_command(self.slot[ID_CMD])
            self.slot[ID_CMD] = None
        if self.slot[ID_PARAM] is not None:
            self.handle_param(self.slot[ID_PARAM])
            self.slot[ID_PARAM] = None

    def handle_param(self, frame):                      # frame = op, idx, 4 value bytes, crc
        b = [ID_PARAM] + frame
        if crc8(b[:CMD_PAYLOAD]) != b[CMD_PAYLOAD]:
            return
        self.params.append((b[1], b[2], struct.unpack('<f', bytes(b[3:7]))[0]))

    def handle_command(self, frame):                    # frame = 6 data bytes + crc
        b = [ID_CMD] + frame
        if crc8(b[:CMD_PAYLOAD]) != b[CMD_PAYLOAD]:
            return
        v = []
        for i in range(3):
            u = b[1 + 2 * i] | (b[2 + 2 * i] << 8)       # (uint16_t)
            v.append(u - 0x10000 if u & 0x8000 else u)  # (int16_t)
        self.cmds.append(tuple(v))

    def feed(self, data):
        for b in data:
            self.isr(b)
        return self.cmds


def command(vx, vy, w):                                 # what the Pi sends (mm/s, mrad/s)
    body = bytes([ID_CMD]) + struct.pack('<hhh', vx, vy, w)
    return bytes([SYNC]) + body + bytes([crc8(body)])


def state_frame(seq, boot, timed_out, counts):          # like send_state_frame() in main.c
    f = [SYNC, ID_STATE, seq & 0xFF, (boot | (timed_out << 1)) & 0xFF]
    for c in counts:
        c &= 0xFFFFFFFF                                  # (uint32_t)M_count[m]
        f += [c & 0xFF, (c >> 8) & 0xFF, (c >> 16) & 0xFF, (c >> 24) & 0xFF]
    return bytes(f + [crc8(f[1:])])


def param_request(op, idx, value):                      # what the Pi sends
    body = bytes([ID_PARAM, op, idx]) + struct.pack('<f', value)
    return bytes([SYNC]) + body + bytes([crc8(body)])


def param_reply(op, idx, status, value):                # like send_param_reply() in main.c
    f = [SYNC, ID_PARAM_REPLY, op, idx, status] + list(struct.pack('<f', value))
    return bytes(f + [crc8(f[1:])])


def fed(data, process=True):                            # a receiver after these bytes
    rx = Rx(process)
    rx.feed(data)
    return rx


def check(name, got, want):
    ok = got == want
    print(('PASS ' if ok else 'FAIL ') + name + ('' if ok else f': got {got}, want {want}'))
    return ok


bad = bytearray(command(100, 0, 0))
bad[4] ^= 0x01
results = [
    check('crc8("123456789") = 0xF4', crc8(b'123456789'), 0xF4),
    check('command length 9', len(command(0, 0, 0)), 9),
    check('command decode', Rx().feed(command(300, -100, 0)), [(300, -100, 0)]),
    check('two back-to-back commands', Rx().feed(command(1, 2, 3) + command(-4, 5, -6)), [(1, 2, 3), (-4, 5, -6)]),
    check('junk before the frame', Rx().feed(b'\x00\x01test\n' + command(7, 8, 9)), [(7, 8, 9)]),
    check('0xA5 in the data does not restart the frame',
          Rx().feed(command(0x01A5, -23131, 0x01A5)), [(0x01A5, -23131, 0x01A5)]),   # -23131 = 0xA5A5
    check('0xA5 0xA5 0x01 ...: the second 0xA5 is the header', Rx().feed(bytes([SYNC]) + command(10, 0, 0)), [(10, 0, 0)]),
    check('0xA5 then a wrong id: back to idle, next frame is found',
          Rx().feed(bytes([SYNC, 0x02]) + command(11, 0, 0)), [(11, 0, 0)]),
    check('wrong crc is ignored, next frame is used', Rx().feed(bytes(bad) + command(12, 0, 0)), [(12, 0, 0)]),
    check('state frame length', len(state_frame(0, 1, 0, [0, 0, 0, 0])), STATE_LEN),
    check('state frame layout (little endian, signed counts)',
          struct.unpack('<BBBBiiii', state_frame(200, 0, 1, [1536, -20, 2**31 - 1, -2**31])[:20]),
          (SYNC, ID_STATE, 200, 2, 1536, -20, 2**31 - 1, -2**31)),
    check('state frame crc covers id .. counts', crc8(state_frame(7, 1, 0, [1, 2, 3, 4])[1:]), 0),
    # Setting mode (v2.2.0)
    check('param request length 9 = command length (6 data + crc after sync + id, no padding)',
          (len(param_request(1, 0, 1.0)), len(command(0, 0, 0))), (9, 9)),
    check('param request decode (op, idx, float32 LE)', fed(param_request(1, 3, 12.5)).params, [(1, 3, 12.5)]),
    check('command and param request in one stream',
          (lambda r: (r.cmds, r.params))(fed(command(5, 0, 0) + param_request(0, 2, 0.0) + command(6, 0, 0))),
          ([(5, 0, 0), (6, 0, 0)], [(0, 2, 0.0)])),
    check('main loop blocked (EEPROM save): commands after a request do not overwrite it',
          (lambda r: (r.main_loop(), (r.cmds, r.params))[1])(
              fed(param_request(0, 1, 0.0) + command(0, 0, 0) + command(0, 0, 0), process=False)),
          ([(0, 0, 0)], [(0, 1, 0.0)])),
    check('0xA5 inside the float value is data',
          fed(param_request(1, 0, struct.unpack('<f', bytes([0xA5, 0xA5, 0x03, 0x41]))[0])).params[0][:2], (1, 0)),
    check('param reply length and layout', struct.unpack('<BBBBBf', param_reply(2, 0, 4, 9.0)[:9]),
          (SYNC, ID_PARAM_REPLY, 2, 0, 4, 9.0)),
    check('param reply crc covers id .. value', crc8(param_reply(0, 5, 0, 3.5)[1:]), 0),
    # The Pi code gives the same bytes
    check('Pi encode_command == model', pi.encode_command(0.3, -0.1, 1.0), command(300, -100, 1000)),
    check('Pi encode_param == model', pi.encode_param(1, 7, 0.125), param_request(1, 7, 0.125)),
    check('Pi split_frames reads the model reply',
          pi.split_frames(param_reply(0, 5, 0, 3.5) + state_frame(9, 0, 0, [1, 2, 3, 4]))[:2],
          ([(9, 0, [1, 2, 3, 4])], [(0, 5, 0, 3.5)])),
]
print(f'{sum(results)}/{len(results)} passed')
raise SystemExit(0 if all(results) else 1)
