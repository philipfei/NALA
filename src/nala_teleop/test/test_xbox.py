from nala_teleop.xbox import ZERO, XboxTeleop

LB = 4
AXES = (1, 0, 3)   # left stick up/down, left stick left/right, right stick left/right


def teleop():
    return XboxTeleop(enable_button=LB, axes=AXES, scales=(0.45, 0.45, 0.5), joy_timeout=0.2)


def buttons(lb):
    b = [0] * 11
    b[LB] = int(lb)
    return b


def axes(forward=0.0, left=0.0, turn=0.0):
    a = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]   # triggers (2, 5) are 1.0 when released
    a[1], a[0], a[3] = forward, left, turn
    return a


def test_nothing_is_published_without_lb():
    t = teleop()
    assert t.update(buttons(False), axes(forward=1.0), 0.0) is None
    assert t.update(buttons(False), axes(), 0.05) is None


def test_held_stick_at_lb_press_does_not_drive_until_centred():
    t = teleop()
    assert t.update(buttons(True), axes(forward=1.0), 0.0) == ZERO
    assert t.update(buttons(True), axes(forward=1.0), 0.05) == ZERO
    assert t.update(buttons(True), axes(forward=0.6, turn=0.2), 0.10) == ZERO
    assert t.update(buttons(True), axes(), 0.15) == ZERO         # centred once
    assert t.update(buttons(True), axes(forward=1.0), 0.20) == (0.45, 0.0, 0.0)


def test_axis_mapping_scales_and_signs():
    t = teleop()
    t.update(buttons(True), axes(), 0.0)
    t.update(buttons(True), axes(), 0.05)
    assert t.update(buttons(True), axes(forward=-0.5), 0.10) == (-0.225, 0.0, 0.0)
    assert t.update(buttons(True), axes(left=1.0), 0.15) == (0.0, 0.45, 0.0)       # strafe left positive
    assert t.update(buttons(True), axes(turn=-1.0), 0.20) == (0.0, 0.0, -0.5)      # turn right negative


def test_release_sends_one_zero_and_the_next_press_waits_again():
    t = teleop()
    t.update(buttons(True), axes(), 0.0)
    t.update(buttons(True), axes(), 0.05)
    assert t.update(buttons(True), axes(forward=1.0), 0.10) == (0.45, 0.0, 0.0)
    assert t.update(buttons(False), axes(forward=1.0), 0.15) == ZERO
    assert t.update(buttons(False), axes(forward=1.0), 0.20) is None
    assert t.update(buttons(True), axes(forward=1.0), 0.25) == ZERO
    assert t.update(buttons(True), axes(forward=1.0), 0.30) == ZERO


def test_watchdog_stops_once_and_needs_a_new_lb_press():
    t = teleop()
    t.update(buttons(True), axes(), 0.0)
    t.update(buttons(True), axes(), 0.05)
    assert t.update(buttons(True), axes(forward=1.0), 0.10) == (0.45, 0.0, 0.0)
    assert t.timeout(0.25) is None                  # 0.15 s without /joy: still fine
    assert t.timeout(0.31) == ZERO                  # > 0.2 s: one zero
    assert t.timeout(0.40) is None
    # /joy comes back with LB and the stick still held: no driving, not even after centring.
    assert t.update(buttons(True), axes(forward=1.0), 0.45) == ZERO
    assert t.update(buttons(True), axes(), 0.50) == ZERO
    assert t.update(buttons(True), axes(forward=1.0), 0.55) == ZERO
    # Release (nothing to send: the stop was sent), press again, centre, drive.
    assert t.update(buttons(False), axes(), 0.60) is None
    assert t.update(buttons(True), axes(), 0.65) == ZERO
    assert t.update(buttons(True), axes(), 0.70) == ZERO
    assert t.update(buttons(True), axes(forward=1.0), 0.75) == (0.45, 0.0, 0.0)


def test_watchdog_is_quiet_while_lb_is_not_held():
    t = teleop()
    assert t.timeout(5.0) is None
    t.update(buttons(False), axes(), 0.0)
    assert t.timeout(5.0) is None


def test_short_messages_count_as_not_pressed_or_zero():
    t = teleop()
    assert t.update([0, 0], axes(forward=1.0), 0.0) is None          # LB index missing
    t.update(buttons(True), axes(), 0.05)
    t.update(buttons(True), axes(), 0.10)
    assert t.update(buttons(True), [0.0, 1.0], 0.15) == ZERO         # turn axis missing
