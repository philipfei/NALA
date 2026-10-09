"""Model test of the v2.0.0 I2C frames (Python copy of the C code in main.c).

Checks the CRC-8 (poly 0x07, init 0) with the standard test vector, the byte layout of
the command (Pi -> MCU) and of the state frame (MCU -> Pi), and the int16 / int32
sign handling. The Pi side (src/nala_base/nala_base/mcu_protocol.py) must give the
same bytes. Run: python3 i2c_frame_model_test.py
"""
import struct

STATE_LEN, CMD_LEN, CMD_ID = 19, 8, 0x01


def crc8(data):                       # same loop as crc8() in main.c
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def build_state_frame(seq, boot, timed_out, counts):   # like build_state_frame() in main.c
    f = [seq & 0xFF, (boot | (timed_out << 1)) & 0xFF]
    for c in counts:
        c &= 0xFFFFFFFF                                  # (uint32_t)M_count[m]
        f += [c & 0xFF, (c >> 8) & 0xFF, (c >> 16) & 0xFF, (c >> 24) & 0xFF]
    return bytes(f + [crc8(f)])


def decode_command(b):                                   # like handle_command() in main.c
    if len(b) != CMD_LEN or b[0] != CMD_ID or crc8(b[:7]) != b[7]:
        return None
    v = []
    for i in range(3):
        u = b[1 + 2 * i] | (b[2 + 2 * i] << 8)            # (uint16_t)
        v.append(u - 0x10000 if u & 0x8000 else u)       # (int16_t)
    return [x / 1000.0 for x in v]


def check(name, got, want):
    ok = got == want
    print(('PASS ' if ok else 'FAIL ') + name + ('' if ok else f': got {got}, want {want}'))
    return ok


results = [
    check('crc8("123456789") = 0xF4', crc8(b'123456789'), 0xF4),
    check('state frame length', len(build_state_frame(0, 1, 0, [0, 0, 0, 0])), STATE_LEN),
    check('state frame layout (little endian, signed counts)',
          struct.unpack('<BBiiii', build_state_frame(200, 0, 1, [1536, -20, 2**31 - 1, -2**31])[:18]),
          (200, 2, 1536, -20, 2**31 - 1, -2**31)),
    check('state frame crc covers the first 18 bytes',
          crc8(build_state_frame(7, 1, 0, [1, 2, 3, 4])), 0),   # crc over data + its crc = 0
    check('command decode (mm/s, mrad/s)',
          decode_command(bytes([CMD_ID, 0x2C, 0x01, 0x9C, 0xFF, 0x00, 0x00]) +
                         bytes([crc8(bytes([CMD_ID, 0x2C, 0x01, 0x9C, 0xFF, 0x00, 0x00]))])),
          [0.3, -0.1, 0.0]),
    check('command with a wrong crc is ignored',
          decode_command(bytes([CMD_ID, 0, 0, 0, 0, 0, 0, 0x55])), None),
    check('command with a wrong id is ignored',
          decode_command(bytes([0x02, 0, 0, 0, 0, 0, 0, crc8(bytes([0x02, 0, 0, 0, 0, 0, 0]))])), None),
]
print(f'{sum(results)}/{len(results)} passed')
raise SystemExit(0 if all(results) else 1)
