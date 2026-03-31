from teleop_fetch.operator_buttons import rising_edge


def test_rising_edge_pressed():
    assert rising_edge(0.0, 1.0, 0.5)
    assert rising_edge(0.0, 0.6, 0.5)
    assert rising_edge(0.5, 1.0, 0.5)


def test_rising_edge_not_pressed():
    assert not rising_edge(1.0, 1.0, 0.5)
    assert not rising_edge(0.0, 0.4, 0.5)
    assert not rising_edge(0.6, 0.2, 0.5)
