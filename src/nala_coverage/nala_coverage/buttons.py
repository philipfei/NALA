"""Game controller button helpers (no ROS): /joy buttons lists."""


def pressed(buttons, index):
    """True if button `index` is held. A list that is too short counts as "not held"."""
    return 0 <= index < len(buttons) and bool(buttons[index])


def rising(previous, buttons, index):
    """True when button `index` goes from not held (previous /joy) to held (this /joy)."""
    return pressed(buttons, index) and not pressed(previous, index)
