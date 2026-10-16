from nala_coverage.buttons import pressed, rising

B = 1


def test_pressed_and_short_lists():
    assert pressed([0, 1, 0], B)
    assert not pressed([1, 0, 0], B)
    assert not pressed([1], B)          # too short: not held
    assert not pressed([], B)


def test_rising_edge_only_once_per_press():
    stream = [[0, 0], [0, 1], [0, 1], [0, 1], [0, 0], [0, 1]]
    edges = [rising(prev, now, B) for prev, now in zip([[]] + stream, stream)]
    assert edges == [False, True, False, False, False, True]


def test_other_buttons_do_not_count():
    assert not rising([0, 0, 0, 0, 0], [1, 0, 1, 1, 1], B)
