"""
Head control: quaternion -> euler, send HeadState to pan/tilt controllers.
Uses ainex_interfaces/HeadState for compatibility with ainex_controller.
"""

import numpy as np
from ainex_interfaces.msg import HeadState


def quaternion_to_euler(x, y, z, w):
    """
    Convert quaternion to Euler angles (roll, pitch, yaw).
    """
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)
    else:
        pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return [roll, pitch, yaw]


def compute_head_targets(orientation, head_config):
    """
    Compute pan/tilt from head orientation quaternion.
    axis_mapping: "pitch_roll" = pitch->pan, roll->tilt (Quest/teleop_fetch)
                  "yaw_pitch" = yaw->pan, pitch->tilt (standard)
    """
    if orientation is None:
        return None, None
    euler = quaternion_to_euler(
        orientation.x, orientation.y, orientation.z, orientation.w)
    roll, pitch, yaw = euler[0], euler[1], euler[2]
    sens = head_config['sensitivity']
    max_pan = head_config['max_pan']
    max_tilt = head_config['max_tilt']
    mapping = head_config.get('axis_mapping', 'pitch_roll')
    if mapping == 'yaw_pitch':
        pan_src = -yaw
        tilt_src = -pitch
    else:
        # pitch_roll: matches Quest
        pan_src = -pitch
        tilt_src = -roll
    target_pan = np.clip(pan_src * sens, -max_pan, max_pan)
    target_tilt = np.clip(tilt_src * sens, -max_tilt, max_tilt)
    return target_pan, target_tilt


def create_head_state_msg(position, duration):
    """Create HeadState message."""
    msg = HeadState()
    msg.position = position
    msg.duration = duration
    return msg
