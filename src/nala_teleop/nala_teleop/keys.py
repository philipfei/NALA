"""Keyboard layout of the teleop (no ROS). Same keys as teleop_twist_keyboard, without the speed keys."""

HELP = """
Hold a key to drive. Keep this terminal focused.
---------------------------
   u    i    o        i = forward, , = backward
   j    k    l        j / l = turn left / right
   m    ,    .        u o m . = drive and turn at the same time

Shift = drive without turning (J / L = move sideways):
   U    I    O
   J    K    L
   M    <    >

k or any other key : stop
Ctrl+C : stop and quit
Speeds are fixed: config/teleop.yaml
"""

# key: (x direction, y direction, turn direction). Values -1, 0 or 1.
KEYS = {
    'i': (1, 0, 0),
    'o': (1, 0, -1),
    'j': (0, 0, 1),
    'l': (0, 0, -1),
    'u': (1, 0, 1),
    ',': (-1, 0, 0),
    '.': (-1, 0, 1),
    'm': (-1, 0, -1),
    'O': (1, -1, 0),
    'I': (1, 0, 0),
    'J': (0, 1, 0),
    'L': (0, -1, 0),
    'U': (1, 1, 0),
    '<': (-1, 0, 0),
    '>': (-1, -1, 0),
    'M': (-1, 1, 0),
}


def twist_for_key(key, linear, angular):
    """Return (vx, vy, wz) for a key. Unknown keys give zero (stop)."""
    x, y, turn = KEYS.get(key, (0, 0, 0))
    return x * linear, y * linear, turn * angular
