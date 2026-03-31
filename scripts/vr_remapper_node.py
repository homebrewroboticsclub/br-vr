#!/usr/bin/env python3
"""
VR Remapper: axis mapping + calibration (R_A) + scale.

Reference pose: arms in front, slightly lower (reference_pose).
Operator brings hands to similar position, presses R_A — offset is computed.
output = mapped_vr + offset; output *= scale

SCALE (0.0001..100) — sensitivity, live update from UI.
"""

import copy
import rospy
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


def _controller_to_body_link(x, y, z, is_left):
    """Mapping: Quest -> body_link. Change swap/signs here."""
    if is_left:
        return (z, -x, y)
    else:
        return (z, -x, y)


class VRRemapperNode:
    def __init__(self):
        rospy.init_node('vr_remapper', anonymous=False)
        self.quest_poses = None
        self.r_a_pressed = False

        # Reference pose from robot logs
        ref_left = rospy.get_param('~reference_pose/left', [0.143281, 0.103784, 0.020140])
        ref_right = rospy.get_param('~reference_pose/right', [0.124819, -0.087679, 0.016086])
        self.ref_left = [float(ref_left[0]), float(ref_left[1]), float(ref_left[2])]
        self.ref_right = [float(ref_right[0]), float(ref_right[1]), float(ref_right[2])]

        self.offset_left = [0.0, 0.0, 0.0]
        self.offset_right = [0.0, 0.0, 0.0]
        self.has_calibration = False

        self.scale = rospy.get_param('~scale', 0.25)

        self.pub = rospy.Publisher('/teleop_fetch/quest_poses_remapped', PoseArray, queue_size=1)
        rospy.Subscriber('/quest/poses', PoseArray, self._quest_cb)
        rospy.Subscriber('/quest/joints', JointState, self._joints_cb)
        rospy.Subscriber('/teleop_fetch/scale', Float64, self._scale_cb)
        rospy.loginfo('vr_remapper: map + R_A calibration + scale. ref_left=%s ref_right=%s',
                      self.ref_left, self.ref_right)

    def _scale_cb(self, msg):
        v = float(msg.data)
        if 0.0001 <= v <= 100.0:
            self.scale = v

    def _joints_cb(self, msg):
        if msg.name and msg.position and len(msg.name) == len(msg.position):
            joints = dict(zip(msg.name, msg.position))
            if joints.get('R_A', 0) > 0.5:
                if not self.r_a_pressed:
                    self.r_a_pressed = True
                    self._do_calibration()
            else:
                self.r_a_pressed = False

    def _do_calibration(self):
        if not self.quest_poses or len(self.quest_poses.poses) < 3:
            rospy.logwarn('Calibration: no poses')
            return
        p_left = self.quest_poses.poses[1].position
        p_right = self.quest_poses.poses[2].position
        ml = _controller_to_body_link(float(p_left.x), float(p_left.y), float(p_left.z), True)
        mr = _controller_to_body_link(float(p_right.x), float(p_right.y), float(p_right.z), False)
        self.offset_left = [self.ref_left[i] - ml[i] for i in range(3)]
        self.offset_right = [self.ref_right[i] - mr[i] for i in range(3)]
        self.has_calibration = True
        rospy.loginfo('Calibration: R_A. offset_left=%s offset_right=%s', self.offset_left, self.offset_right)

    def _quest_cb(self, msg):
        self.quest_poses = msg

    def _process_hand(self, p, is_left):
        x, y, z = _controller_to_body_link(float(p.x), float(p.y), float(p.z), is_left)
        if self.has_calibration:
            off = self.offset_left if is_left else self.offset_right
            x, y, z = x + off[0], y + off[1], z + off[2]
        x *= self.scale
        y *= self.scale
        z *= self.scale
        return x, y, z

    def _publish(self, event):
        if not self.quest_poses or len(self.quest_poses.poses) < 3:
            return
        out = copy.deepcopy(self.quest_poses)
        p = self.quest_poses.poses[1].position
        out.poses[1].position.x, out.poses[1].position.y, out.poses[1].position.z = \
            self._process_hand(p, is_left=True)
        p = self.quest_poses.poses[2].position
        out.poses[2].position.x, out.poses[2].position.y, out.poses[2].position.z = \
            self._process_hand(p, is_left=False)
        self.pub.publish(out)

    def run(self):
        rospy.Timer(rospy.Duration(0.02), self._publish)
        rospy.spin()


def main():
    try:
        node = VRRemapperNode()
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
