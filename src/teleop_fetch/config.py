"""
Configuration loader for teleop_fetch.
Loads parameters from rosparam (loaded from teleop.yaml).
"""

import rospy


def load_config():
    """
    Load teleop config from ROS parameters.
    Expects config to be loaded via rosparam in launch file.
    Uses private namespace (~) for node-specific params.
    """
    def p(name, default):
        return rospy.get_param('~' + name, default)

    config = {}

    # VR input
    config['poses_topic'] = p('vr_input/poses_topic', '/quest/poses')
    config['joints_topic'] = p('vr_input/joints_topic', '/quest/joints')

    # Robot scale
    config['robot_scale'] = {
        'human_arm_length': p('robot_scale/human_arm_length', 0.8),
        'robot_arm_length': p('robot_scale/robot_arm_length', 0.2),
        'neck_offset': p('robot_scale/neck_offset', 0.25),
        'big_head_x_offset': p('robot_scale/big_head_x_offset', 0.15),
        'robot_z_offset': p('robot_scale/robot_z_offset', 0.087448),
        'hand_center_outer_offset': p('robot_scale/hand_center_outer_offset', 0.050),
    }

    # Servo IDs
    config['servo_ids'] = {
        'left_arm': p('servo_ids/left_arm', [13, 15, 17, 19]),
        'right_arm': p('servo_ids/right_arm', [14, 16, 18, 20]),
        'left_gripper': p('servo_ids/left_gripper', 21),
        'right_gripper': p('servo_ids/right_gripper', 22),
    }

    # Gripper limits
    config['gripper'] = {
        'left': {'closed': p('gripper/left/closed', 500), 'open': p('gripper/left/open', 100)},
        'right': {'closed': p('gripper/right/closed', 400), 'open': p('gripper/right/open', 800)},
    }

    # Arm start positions
    config['arm_start_positions'] = {
        'left': p('arm_start_positions/left', {13: 874, 15: 833, 17: 502, 19: 44, 21: 500}),
        'right': p('arm_start_positions/right', {14: 126, 16: 167, 18: 498, 20: 956, 22: 500}),
    }

    # Head
    config['head'] = {
        'sensitivity': p('head/sensitivity', 1.0),
        'max_pan': p('head/max_pan', 2.0),
        'max_tilt': p('head/max_tilt', 2.0),
        'movement_duration': p('head/movement_duration', 0.2),
        'axis_mapping': p('head/axis_mapping', 'pitch_roll'),
        'pan_topic': p('head/pan_topic', '/head_pan_controller/command'),
        'tilt_topic': p('head/tilt_topic', '/head_tilt_controller/command'),
    }

    # Output
    config['servo_topic'] = p('servo_topic', '/ros_robot_controller/bus_servo/set_position')
    config['arm_servo_targets_topic'] = p('arm_servo_targets_topic', '/teleop_fetch/arm_servo_targets')
    config['teleop_state_topic'] = p('teleop_state_topic', '/teleop_state')

    # If True (default), arm/start/stop servo commands go to /kyr/bus_servo_in; else direct to servo_topic (bench / no KYR).
    config['use_kyr_servo_gateway'] = p('use_kyr_servo_gateway', True)
    # If False, skip KYR grant lifecycle: behave like legacy teleop (always allow arm stream gating via L_X only).
    config['teleop_require_kyr_session'] = p('teleop_require_kyr_session', True)

    # Arm stream to KYR: if True, need rising edge on operator_arm joint after ACTIVE (Quest L_X).
    config['arm_stream_requires_lx'] = p('arm_stream_requires_lx', True)
    config['joint_name_lx'] = p('operator_arm/joint_name_lx', 'L_X')
    config['joint_name_ly'] = p('operator_arm/joint_name_ly', 'L_Y')
    # After first L_Y disarms arms, second L_Y ends KYR session and triggers operator SOL payment.
    config['end_session_on_second_ly'] = p('end_session_on_second_ly', True)

    # Operator presence watchdog (see DOC/VR_APP_TELEOP_ROS_CONTRACT.md)
    config['operator_presence_watchdog_enabled'] = p('operator_presence/watchdog_enabled', True)
    config['operator_presence_timeout_sec'] = float(p('operator_presence/timeout_sec', 15.0))
    config['operator_presence_check_hz'] = float(p('operator_presence/check_hz', 1.0))
    config['operator_presence_lifecycle_topic'] = str(
        p('operator_presence/lifecycle_topic', '/quest/teleop_lifecycle') or ''
    ).strip()
    config['operator_presence_pause_max_sec'] = float(p('operator_presence/pause_max_sec', 600.0))

    return config
