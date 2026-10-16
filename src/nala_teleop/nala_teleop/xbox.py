"""Xbox controller teleop logic (no ROS): /joy buttons and axes -> body speed command (vx, vy, wz).

Hold LB (the enable button) to drive. Like teleop_twist_joy before: speed = stick axis x scale, and nothing is
published while LB is not held, so the keyboard teleop can drive. Safety rules (user decision 2026-10-16):
- LB pressed: the command is zero until all used stick axes were 0.0 once (joy_node gives exactly 0.0
  inside its deadzone). A stick held off-centre while LB is pressed never makes the robot jump.
- LB released: one zero command, then nothing.
- No /joy for joy_timeout while LB is held (controller lost, joy_node stopped): one zero command. Then the
  robot stays stopped until LB is released and pressed again.
A missing button or axis index in a /joy message counts as "not pressed" / "no command" (zero).
"""

ZERO = (0.0, 0.0, 0.0)

RELEASED = 'released'   # LB not held: publish nothing
WAITING = 'waiting'     # LB held, the sticks were not centred yet: zero
DRIVING = 'driving'     # LB held, sticks were centred once: drive
LOST = 'lost'           # no /joy for joy_timeout while LB was held: zero until LB is released


def _value(values, index):
    """values[index], or None if the message is too short."""
    return values[index] if 0 <= index < len(values) else None


class XboxTeleop:

    def __init__(self, enable_button, axes, scales, joy_timeout):
        """axes, scales: (vx, vy, wz) stick axis indexes and speeds at full stick (m/s, m/s, rad/s)."""
        self.enable_button = enable_button
        self.axes = axes
        self.scales = scales
        self.joy_timeout = joy_timeout
        self.state = RELEASED
        self.last_joy = None

    def update(self, buttons, axes, now):
        """One /joy message at time now (s). Returns the command to publish, or None (publish nothing)."""
        self.last_joy = now
        held = bool(_value(buttons, self.enable_button))
        if not held:
            was = self.state
            self.state = RELEASED
            return ZERO if was in (WAITING, DRIVING) else None
        if self.state in (RELEASED, LOST):
            if self.state == RELEASED:
                self.state = WAITING   # LB just pressed
            return ZERO
        values = [_value(axes, index) for index in self.axes]
        if None in values:
            return ZERO
        if self.state == WAITING:
            if all(value == 0.0 for value in values):
                self.state = DRIVING
            return ZERO
        return tuple(value * scale for value, scale in zip(values, self.scales))

    def timeout(self, now):
        """Watchdog, called by a timer. Returns ZERO once when /joy stopped while LB was held, else None."""
        if self.state not in (WAITING, DRIVING) or now - self.last_joy <= self.joy_timeout:
            return None
        self.state = LOST
        return ZERO
