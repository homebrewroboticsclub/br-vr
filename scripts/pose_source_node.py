#!/usr/bin/env python3
"""
Pose source: merges VR and Manual.
- Manual: /teleop_fetch/manual_poses (body_link, unchanged)
- VR: /teleop_fetch/quest_poses_remapped (already remapped by vr_remapper from /quest/poses)
Publishes to /teleop_fetch/poses for fast_ik.
"""

import rospy
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String


class PoseSourceNode:
    def __init__(self):
        rospy.init_node('pose_source', anonymous=False)
        self.mode = 'vr'
        self.quest_poses_remapped = None  # from vr_remapper (controller remap only)
        self.manual_poses = None

        self.pub = rospy.Publisher('/teleop_fetch/poses', PoseArray, queue_size=1)
        rospy.Subscriber('/teleop_fetch/quest_poses_remapped', PoseArray, self._quest_cb)
        rospy.Subscriber('/teleop_fetch/manual_poses', PoseArray, self._manual_cb)
        rospy.Subscriber('/teleop_fetch/pose_mode', String, self._mode_cb)

        self.timer = rospy.Timer(rospy.Duration(0.02), self._publish)  # 50 Hz
        rospy.loginfo('pose_source: VR (remapped) + Manual -> /teleop_fetch/poses')

    def _quest_cb(self, msg):
        self.quest_poses_remapped = msg

    def _manual_cb(self, msg):
        self.manual_poses = msg

    def _mode_cb(self, msg):
        self.mode = msg.data if msg.data in ('vr', 'manual') else self.mode

    def _publish(self, event):
        if self.mode == 'manual' and self.manual_poses and len(self.manual_poses.poses) >= 3:
            self.pub.publish(self.manual_poses)  # body_link as-is
        elif self.quest_poses_remapped and len(self.quest_poses_remapped.poses) >= 3:
            self.pub.publish(self.quest_poses_remapped)  # already remapped by vr_remapper


def main():
    try:
        node = PoseSourceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
