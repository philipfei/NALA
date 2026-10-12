"""Model test of the v2.1.0 UART frames (Python copy of the C code in main.c).

Checks the CRC-8 (poly 0x07, init 0), the receive state machine of the command frame
(IDLE -> HEADER_ACK -> PAYLOAD -> complete), the command decoding and the byte layout
of the state frame. The Pi side (src/nala_base/nala_base/mcu_protocol.py) must give
the same bytes. Run: python3 uart_frame_model_test.py
"""
import struct

SYNC, ID_CMD, ID_STATE, CMD_PAYLOAD, STATE_LEN = 0xA5, 0x01, 0x02, 7, 21
IDLE, HEADER_ACK, PAYLOAD = range(3)


def crc8(data):                       # same loop as crc8() in main.c
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


class Rx:
    """USART0_RX_vect + handle_command()."""

    def __init__(self):
        self.state, self.count, self.buf, self.cmds = IDLE, 0, [0] * CMD_PAYLOAD, []

    def isr(self, b):
        if self.state == IDLE:
            if b == SYNC:
                self.state = HEADER_ACK
        elif self.state == HEADER_ACK:
            if b == ID_CMD:
                self.count, self.state = 0, PAYLOAD
            elif b != SYNC:
                self.state = IDLE
        else:
            self.buf[self.count] = b
            self.count += 1
            if self.count >= CMD_PAYLOAD:
                self.handle_command(list(self.buf))
                self.state = IDLE

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
]
print(f'{sum(results)}/{len(results)} passed')
raise SystemExit(0 if all(results) else 1)
