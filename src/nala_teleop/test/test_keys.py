from nala_teleop.keys import twist_for_key


def test_forward_and_backward():
    assert twist_for_key('i', 1.0, 0.5) == (1.0, 0, 0)
    assert twist_for_key(',', 1.0, 0.5) == (-1.0, 0, 0)


def test_turn_left_is_positive():
    assert twist_for_key('j', 1.0, 0.5) == (0, 0, 0.5)
    assert twist_for_key('l', 1.0, 0.5) == (0, 0, -0.5)


def test_shift_strafes_left_positive():
    assert twist_for_key('J', 1.0, 0.5) == (0, 1.0, 0)
    assert twist_for_key('L', 1.0, 0.5) == (0, -1.0, 0)


def test_speed_keys_and_unknown_keys_stop():
    for key in 'qzwxeck \x1b':
        assert twist_for_key(key, 1.0, 0.5) == (0, 0, 0)
