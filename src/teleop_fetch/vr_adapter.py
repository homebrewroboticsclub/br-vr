"""
VR Input Adapter: PoseArray + JointState -> VRData (internal format).
"""

from dataclasses import dataclass
from typing import Optional


@dataclass
class VRData:
    """
    Internal format for VR input data.
    poses[0]=head, poses[1]=left_hand, poses[2]=right_hand (relative-to-head)
    """
    head_pose: Optional[object] = None  # geometry_msgs/Pose
    head_orientation: Optional[object] = None  # geometry_msgs/Quaternion
    left_hand_pose: Optional[object] = None
    right_hand_pose: Optional[object] = None
    # Joints: L_grip, L_index, R_grip, R_index, L_X, L_Y, R_A, R_B (0..1)
    left_grip: float = 0.0
    left_index: float = 0.0
    right_grip: float = 0.0
    right_index: float = 0.0
    left_x: float = 0.0
    left_y: float = 0.0
    right_a: float = 0.0
    right_b: float = 0.0


def pose_array_to_vr_data(pose_array) -> VRData:
    """
    Extract VRData from PoseArray.
    poses[0]=head, [1]=left_hand, [2]=right_hand
    """
    data = VRData()
    if len(pose_array.poses) >= 3:
        data.head_pose = pose_array.poses[0].position
        data.head_orientation = pose_array.poses[0].orientation
        data.left_hand_pose = pose_array.poses[1]
        data.right_hand_pose = pose_array.poses[2]
    return data


def joint_state_to_dict(joint_state) -> dict:
    """
    Convert JointState to dict {name: position}.
    """
    if not joint_state.name or not joint_state.position:
        return {}
    if len(joint_state.name) != len(joint_state.position):
        return {}
    return dict(zip(joint_state.name, joint_state.position))


def update_vr_data_from_joints(data: VRData, joint_dict: dict) -> None:
    """
    Update VRData with joint values from JointState.
    """
    data.left_grip = joint_dict.get('L_grip', 0.0)
    data.left_index = joint_dict.get('L_index', 0.0)
    data.right_grip = joint_dict.get('R_grip', 0.0)
    data.right_index = joint_dict.get('R_index', 0.0)
    data.left_x = joint_dict.get('L_X', 0.0)
    data.left_y = joint_dict.get('L_Y', 0.0)
    data.right_a = joint_dict.get('R_A', 0.0)
    data.right_b = joint_dict.get('R_B', 0.0)
