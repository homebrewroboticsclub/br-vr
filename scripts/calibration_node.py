#!/usr/bin/env python3
"""
Calibration node: on R_A button press, capture T-pose and publish TeleopCalibration.
Operator stretches arms in T-pose (like robot at init), presses R_A.
Computes offset (raw VR positions) and scale (from hand distance).
"""

import rospy
import math
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import JointState
from ainex_interfaces.msg import TeleopCalibration
from geometry_msgs.msg import Point


class CalibrationNode:
    def __init__(self):
        rospy.init_node('teleop_calibration', anonymous=False)
        self.poses = None
        self.joints = {}
        self.r_a_pressed = False
        self.robot_arm_span = rospy.get_param('~robot_arm_span', 0.4)  # T-pose hand distance (m)

        self.pub = rospy.Publisher('/teleop_fetch/calibration', TeleopCalibration, queue_size=1, latch=True)
        rospy.Subscriber('/quest/poses', PoseArray, self._poses_cb)
        rospy.Subscriber('/quest/joints', JointState, self._joints_cb)

        rospy.loginfo('Calibration: stretch arms in T-pose, press R_A to calibrate')

    def _poses_cb(self, msg):
        self.poses = msg

    def _joints_cb(self, msg):
        if msg.name and msg.position and len(msg.name) == len(msg.position):
            self.joints = dict(zip(msg.name, msg.position))
        if self.joints.get('R_A', 0) > 0.5:
            if not self.r_a_pressed:
                self.r_a_pressed = True
                self._do_calibration()
        else:
            self.r_a_pressed = False

    def _do_calibration(self):
        if not self.poses or len(self.poses.poses) < 3:
            rospy.logwarn('Calibration: no poses, need /quest/poses')
            return
        left = self.poses.poses[1].position
        right = self.poses.poses[2].position
        dx = (left.x - right.x) or 0
        dy = (left.y - right.y) or 0
        dz = (left.z - right.z) or 0
        human_span = math.sqrt(dx*dx + dy*dy + dz*dz)
        if human_span < 0.01:
            rospy.logwarn('Calibration: hands too close, spread arms in T-pose')
            return
        scale = self.robot_arm_span / human_span
        msg = TeleopCalibration()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'quest'
        msg.offset_left = Point(x=left.x, y=left.y, z=left.z)
        msg.offset_right = Point(x=right.x, y=right.y, z=right.z)
        msg.scale = scale
        self.pub.publish(msg)
        rospy.loginfo('Calibration: T-pose captured. scale=%.4f, left=(%.3f,%.3f,%.3f) right=(%.3f,%.3f,%.3f)',
                      scale, left.x, left.y, left.z, right.x, right.y, right.z)


def main():
    try:
        node = CalibrationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
