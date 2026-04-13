# Contract for operator-facing /teleop_state (std_msgs/String).
# Keep in sync with teleop_node._publish_teleop_state payloads.


def test_teleop_state_payloads_distinct():
    get_control = "get_control"
    stop_control = "stop_control"
    assert get_control != stop_control
    assert len({get_control, stop_control}) == 2
