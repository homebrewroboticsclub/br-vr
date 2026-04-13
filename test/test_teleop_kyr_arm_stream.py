#!/usr/bin/env python3
"""
Integration rostest: teleop_fetch + KYR proxy arm stream to /bus_servo/set_position.

Emulates grant, Quest joints (L_X rising edge), and fast_ik output on arm_servo_targets.
"""

from __future__ import annotations

import json
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import Pose, PoseArray
from ros_robot_controller.msg import BusServoPosition, SetBusServosPosition
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from teleop_fetch.srv import EndSession, ReceiveGrant


class TestTeleopKyrArmStream(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("test_teleop_kyr_arm_stream", anonymous=False, log_level=rospy.DEBUG)

    def setUp(self):
        self._lock = threading.Lock()
        self._bus_count = 0
        self._kyr_in_count = 0
        self._last_bus = None

    def _on_bus(self, msg):
        with self._lock:
            self._bus_count += 1
            self._last_bus = msg

    def _on_kyr_in(self, msg):
        with self._lock:
            self._kyr_in_count += 1

    def _wait_service(self, name: str, timeout: float = 60.0):
        rospy.wait_for_service(name, timeout=timeout)

    def _grant_payload(self, session_suffix: str, scope_inner: dict) -> str:
        return json.dumps(
            {
                "session_id": f"rostest_session_{session_suffix}",
                "robot_id": "robot_001",
                "task_id": "rostest",
                "operator_pubkey": "rostest_operator",
                "valid_until_sec": int(time.time()) + 3600,
                "scope_json": json.dumps(scope_inner),
            },
            separators=(",", ":"),
        )

    def _publish_poses(self, pub_poses):
        pa = PoseArray()
        pa.header = Header(stamp=rospy.Time.now(), frame_id="base_link")
        for _ in range(3):
            p = Pose()
            p.orientation.w = 1.0
            pa.poses.append(p)
        pub_poses.publish(pa)

    def _arm_and_publish_targets(self, pub_joints, pub_arm, servo_id: int, pos: int):
        js = JointState()
        js.name = ["L_X", "L_Y"]
        js.position = [0.0, 0.0]
        pub_joints.publish(js)
        time.sleep(0.05)
        pub_joints.publish(js)
        time.sleep(0.05)
        js2 = JointState()
        js2.name = ["L_X", "L_Y"]
        js2.position = [1.0, 0.0]
        pub_joints.publish(js2)
        time.sleep(0.2)

        cmd = SetBusServosPosition()
        cmd.position = [BusServoPosition(id=servo_id, position=pos)]
        pub_arm.publish(cmd)

    def _wait_bus_after(self, count_after: int, timeout_sec: float = 15.0):
        """Wait until /bus_servo/set_position count exceeds count_after (strictly greater)."""
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            with self._lock:
                if self._bus_count > count_after:
                    return
            time.sleep(0.05)
        with self._lock:
            self.assertGreater(
                self._bus_count,
                count_after,
                f"timeout waiting for new /bus_servo/set_position (have {self._bus_count}, need > {count_after})",
            )

    def test_arm_stream_star_scope_then_empty_scope_normalized(self):
        rospy.Subscriber("/bus_servo/set_position", SetBusServosPosition, self._on_bus, queue_size=10)
        rospy.Subscriber("/kyr/bus_servo_in", SetBusServosPosition, self._on_kyr_in, queue_size=10)

        self._wait_service("/teleop_fetch/receive_grant", timeout=90.0)
        self._wait_service("/teleop_fetch/end_session", timeout=30.0)

        pub_poses = rospy.Publisher("/quest/poses", PoseArray, queue_size=1, latch=True)
        pub_joints = rospy.Publisher("/quest/joints", JointState, queue_size=10, latch=False)
        pub_arm = rospy.Publisher("/teleop_fetch/arm_servo_targets", SetBusServosPosition, queue_size=1)

        self._publish_poses(pub_poses)

        rg = rospy.ServiceProxy("/teleop_fetch/receive_grant", ReceiveGrant)
        end = rospy.ServiceProxy("/teleop_fetch/end_session", EndSession)

        # --- Phase 1: explicit allowed_actions * ---
        with self._lock:
            self._bus_count = 0
            self._kyr_in_count = 0

        grant_a = self._grant_payload("a", {"allowed_actions": ["*"]})
        res = rg(grant_payload=grant_a, signature="rostest_dummy_sig_a")
        self.assertTrue(res.success, msg=res.message)
        time.sleep(0.5)

        with self._lock:
            bus_before_arm = self._bus_count
        self._arm_and_publish_targets(pub_joints, pub_arm, 13, 500)
        self._wait_bus_after(bus_before_arm)

        with self._lock:
            self.assertGreater(self._kyr_in_count, 0, "expected /kyr/bus_servo_in message")
            self.assertTrue(self._last_bus.position, "non-empty servo list")

        # --- Reset session for phase 2 ---
        er = end(reason="rostest_between_phases")
        self.assertTrue(er.success, msg=er.message)
        time.sleep(0.8)

        # --- Phase 2: empty scope object {} -> SessionModule normalizes to allow teleop ---
        with self._lock:
            self._bus_count = 0

        grant_b = self._grant_payload("b", {})
        res2 = rg(grant_payload=grant_b, signature="rostest_dummy_sig_b")
        self.assertTrue(res2.success, msg=res2.message)
        time.sleep(0.5)

        with self._lock:
            bus_before_arm_b = self._bus_count
        self._arm_and_publish_targets(pub_joints, pub_arm, 14, 400)
        self._wait_bus_after(bus_before_arm_b)


if __name__ == "__main__":
    rostest.rosrun("teleop_fetch", "test_teleop_kyr_arm_stream", TestTeleopKyrArmStream)
