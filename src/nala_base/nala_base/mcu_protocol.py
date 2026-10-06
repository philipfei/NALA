"""Pi <-> MCU UART protocol (no ROS). See docs/mcu_protocol.md.

Axes are the ROS axes: vx forward, vy left (m/s), wz counter-clockwise (rad/s).
"""

import struct

START = b'\x80\x86'   # velocity frame start flags
SCALE = 0.02          # m/s or rad/s per count (the firmware divides by 50)
MAX_COUNT = 127       # int8, so at most 127 * 0.02 = 2.54


def to_count(value):
    """Speed -> int8 count, rounded and clamped to +-MAX_COUNT."""
    count = round(value / SCALE)
    return max(-MAX_COUNT, min(MAX_COUNT, count))


def encode_command(vx, vy, wz):
    """Return the 5-byte velocity frame: 0x80 0x86 Vx Vy w."""
    return START + struct.pack('bbb', to_count(vx), to_count(vy), to_count(wz))


def parse_line(line):
    """Parse one text line from the MCU (without the newline).

    Returns:
      ('wheels', [m1, m2, m3, m4])  measured wheel speeds in rad/s
      ('echo', [vx, vy, wz])        command echo (only when echo is on in the MCU)
      ('text', line)                any other text, e.g. 'cmd timeout', 'test'
      None                          empty line
    """
    line = line.strip()
    if not line:
        return None
    try:
        values = [float(v) for v in line.split()]
    except ValueError:
        return ('text', line)
    if len(values) == 4:
        return ('wheels', values)
    if len(values) == 3:
        return ('echo', values)
    return ('text', line)
