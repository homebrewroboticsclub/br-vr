# DATA_NODE correlation merge into HBR metadata (optional ROS params).

import sys
import types
import unittest

from teleop_fetch.correlation_metadata import merge_data_node_correlation_metadata


def _install_fake_rospy(get_param_impl):
    fake = types.ModuleType("rospy")
    fake.core = types.SimpleNamespace(is_initialized=lambda: True)
    fake.get_param = get_param_impl
    sys.modules["rospy"] = fake


def _remove_fake_rospy():
    sys.modules.pop("rospy", None)


class TestDataNodeCorrelationMetadata(unittest.TestCase):
    def test_merge_sets_keys_from_ros_params(self):
        def _gp(name, default=""):
            m = {
                "/kyr_proxy/robot_id": "robot_001",
                "/teleop_fetch/current_kyr_session_id": "sess-abc",
                "/x402_server/raid_robot_id": "raid-uuid-1",
            }
            return m.get(name, default)

        _install_fake_rospy(_gp)
        try:
            metadata = {}
            merge_data_node_correlation_metadata(metadata, "ds1")
        finally:
            _remove_fake_rospy()

        self.assertEqual(metadata["kyrRobotId"], "robot_001")
        self.assertEqual(metadata["kyrSessionId"], "sess-abc")
        self.assertEqual(metadata["raidRobotUuid"], "raid-uuid-1")

    def test_merge_skips_when_rospy_not_initialized(self):
        fake = types.ModuleType("rospy")
        fake.core = types.SimpleNamespace(is_initialized=lambda: False)

        def _no_param(*_a, **_k):
            self.fail("get_param must not be called when ROS is not initialized")

        fake.get_param = _no_param
        sys.modules["rospy"] = fake
        try:
            metadata = {}
            merge_data_node_correlation_metadata(metadata, "ds1")
        finally:
            _remove_fake_rospy()
        self.assertEqual(metadata, {})


if __name__ == "__main__":
    unittest.main()
