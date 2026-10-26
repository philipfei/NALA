"""Scintillator serial protocol (no ROS). See docs/scintillator.md.

19200 baud, 8N1, RTS/CTS. One text line per second, fields separated by TAB, ending in CR LF:
  0 <TAB> <TAB> 21-10-2026 13:57:39 <TAB> 0.000 <TAB> cps <TAB> 100 <TAB> % <TAB> 4 <TAB> cps <TAB> 47 <TAB> %
Only the value before the second 'cps' field is used (user, 2026-10-26); every other field is ignored.
"""

UNIT = 'cps'


def split_lines(buffer):
    """Cut a byte buffer into lines.

    Returns (lines, rest): lines are complete lines without CR LF; rest is the unused tail (an incomplete line).
    """
    *lines, rest = buffer.split(b'\n')
    return [line.rstrip(b'\r') for line in lines], rest


def parse_line(line):
    """The count rate (counts/s) of one line, or None if the line has no second cps value."""
    fields = [field.strip() for field in line.decode('ascii', errors='replace').split('\t')]
    units = [i for i, field in enumerate(fields) if field == UNIT]
    if len(units) < 2:
        return None
    try:
        return float(fields[units[1] - 1])
    except ValueError:
        return None
