"""
Start/Stop controller: X = enable, Y = disable.
set_arms_to_start_position, reset_head_to_base, reset_grippers.
"""

from ros_robot_controller.msg import SetBusServosPosition, BusServoPosition


def build_arm_start_positions_msg(config, duration=0.1):
    """
    Build SetBusServosPosition message with arm start positions from config.
    """
    msg = SetBusServosPosition()
    msg.duration = duration
    positions = []
    for side in ('left', 'right'):
        for servo_id, pos in config['arm_start_positions'][side].items():
            bp = BusServoPosition()
            bp.id = int(servo_id)
            bp.position = int(pos)
            positions.append(bp)
    msg.position = positions
    return msg


def build_reset_grippers_msg(config, duration=0.5):
    """
    Build SetBusServosPosition message to reset grippers to safe center.
    Uses closed position as "reset" for safety.
    """
    msg = SetBusServosPosition()
    msg.duration = duration
    left_id = config['servo_ids']['left_gripper']
    right_id = config['servo_ids']['right_gripper']
    left_closed = config['gripper']['left']['closed']
    right_closed = config['gripper']['right']['closed']
    msg.position = [
        BusServoPosition(id=left_id, position=left_closed),
        BusServoPosition(id=right_id, position=right_closed),
    ]
    return msg
