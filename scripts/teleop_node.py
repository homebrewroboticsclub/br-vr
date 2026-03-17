#!/usr/bin/env python3
"""
teleop_fetch - unified VR teleoperation node.
Single point of publication to bus_servo.
"""

import rospy
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import JointState
from ainex_interfaces.msg import HeadState
from ros_robot_controller.msg import SetBusServosPosition

from teleop_fetch.config import load_config
from teleop_fetch.vr_adapter import (
    VRData,
    pose_array_to_vr_data,
    joint_state_to_dict,
    update_vr_data_from_joints,
)
from teleop_fetch.head_controller import (
    compute_head_targets,
    create_head_state_msg,
)
from teleop_fetch.start_stop_controller import (
    build_arm_start_positions_msg,
    build_reset_grippers_msg,
)


class TeleopNode:
    def __init__(self):
        rospy.init_node('teleop_fetch', anonymous=False)
        self.config = load_config()

        # State: 'idle' or 'controlling'
        self.arm_control_state = 'idle'
        self.button_left_x_pressed = False
        self.button_left_y_pressed = False

        # VR data cache
        self.vr_data = VRData()

        # Publishers - single point for bus_servo
        self.servo_pub = rospy.Publisher(
            self.config['servo_topic'],
            SetBusServosPosition,
            queue_size=1,
        )
        self.head_pan_pub = rospy.Publisher(
            self.config['head']['pan_topic'],
            HeadState,
            queue_size=1,
        )
        self.head_tilt_pub = rospy.Publisher(
            self.config['head']['tilt_topic'],
            HeadState,
            queue_size=1,
        )

        # Subscribers
        rospy.Subscriber(
            self.config['poses_topic'],
            PoseArray,
            self._pose_callback,
            queue_size=10,
        )
        rospy.Subscriber(
            self.config['joints_topic'],
            JointState,
            self._joints_callback,
            queue_size=10,
        )
        rospy.Subscriber(
            self.config['arm_servo_targets_topic'],
            SetBusServosPosition,
            self._arm_targets_callback,
            queue_size=10,
        )

        # Initial: set arms to start position
        self._publish_arm_start_position()
        rospy.loginfo('teleop_fetch initialized, state=idle. X=enable, Y=disable')

    def _pose_callback(self, msg):
        data = pose_array_to_vr_data(msg)
        self.vr_data.head_pose = data.head_pose
        self.vr_data.head_orientation = data.head_orientation
        self.vr_data.left_hand_pose = data.left_hand_pose
        self.vr_data.right_hand_pose = data.right_hand_pose

        # Head control (always active)
        self._process_head_control()

        # X/Y buttons for start/stop
        self._process_xy_buttons()

    def _joints_callback(self, msg):
        joint_dict = joint_state_to_dict(msg)
        update_vr_data_from_joints(self.vr_data, joint_dict)
        self._process_xy_buttons()

    def _process_xy_buttons(self):
        """X = enable, Y = disable (no calibration for now)."""
        left_x = self.vr_data.left_x
        left_y = self.vr_data.left_y

        if left_x > 0.5 and not self.button_left_x_pressed:
            self.button_left_x_pressed = True
            if self.arm_control_state == 'idle':
                self.arm_control_state = 'controlling'
                rospy.loginfo('Arm control ENABLED (X pressed)')
        elif left_x <= 0.5:
            self.button_left_x_pressed = False

        if left_y > 0.5 and not self.button_left_y_pressed:
            self.button_left_y_pressed = True
            if self.arm_control_state == 'controlling':
                self._stop_arm_control()
        elif left_y <= 0.5:
            self.button_left_y_pressed = False

    def _process_head_control(self):
        pan, tilt = compute_head_targets(
            self.vr_data.head_orientation,
            self.config['head'],
        )
        if pan is not None and tilt is not None:
            duration = self.config['head']['movement_duration']
            pan_msg = create_head_state_msg(pan, duration)
            tilt_msg = create_head_state_msg(tilt, duration)
            self.head_pan_pub.publish(pan_msg)
            self.head_tilt_pub.publish(tilt_msg)

    def _stop_arm_control(self):
        self.arm_control_state = 'idle'
        rospy.loginfo('Arm control DISABLED (Y pressed)')
        self._publish_arm_start_position()
        self._reset_head_to_base()
        self._reset_grippers()

    def _publish_arm_start_position(self):
        msg = build_arm_start_positions_msg(self.config, duration=0.1)
        self.servo_pub.publish(msg)
        rospy.loginfo('Published arm start positions')

    def _reset_head_to_base(self):
        pan_msg = create_head_state_msg(0.0, self.config['head']['movement_duration'])
        tilt_msg = create_head_state_msg(0.0, self.config['head']['movement_duration'])
        self.head_pan_pub.publish(pan_msg)
        self.head_tilt_pub.publish(tilt_msg)
        rospy.loginfo('Head reset to base')

    def _reset_grippers(self):
        msg = build_reset_grippers_msg(self.config)
        self.servo_pub.publish(msg)
        rospy.loginfo('Grippers reset')

    def _arm_targets_callback(self, msg):
        """Forward arm targets from fast_ik to bus_servo when controlling."""
        if self.arm_control_state != 'controlling':
            return
        self.servo_pub.publish(msg)

    def run(self):
        rospy.spin()


def main():
    try:
        node = TeleopNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo('teleop_fetch stopped')


if __name__ == '__main__':
    main()
