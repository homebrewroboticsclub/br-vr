#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import JointState
from teleop_fetch.msg import HeadCommand
from ros_robot_controller.msg import SetBusServosPosition, BusServoPosition


class TeleopFetcher:
    """
    ROS node for teleoperation of Fetch robot based on VR headset data.
    Controls robot head and arms based on operator's head position and controller data.
    
    New arm control system:
    - Controller Y -> sho_pitch (shoulder forward-backward)
    - Controller Z -> sho_roll (shoulder up-down)
    - Controller X -> el_yaw (forearm rotation)
    - Controller X tilt -> el_pitch (forearm bend)
    
    Calibration is performed separately for each arm.
    """
    
    def __init__(self):
        rospy.init_node('teleop_fetch', anonymous=True)
        
        # Sensitivity parameters
        self.head_sensitivity = rospy.get_param('~head_sensitivity', 1.0)
        self.max_head_pan = rospy.get_param('~max_head_pan', 2.0)  # Maximum head pan
        self.max_head_tilt = rospy.get_param('~max_head_tilt', 2.0)  # Maximum head tilt
        self.movement_duration = rospy.get_param('~movement_duration', 0.2)  # Movement duration
        
        # Publishers for head control
        self.head_pan_pub = rospy.Publisher('/head_pan_controller/command', HeadCommand, queue_size=1)
        self.head_tilt_pub = rospy.Publisher('/head_tilt_controller/command', HeadCommand, queue_size=1)
        
        # Publisher for robot arm control
        self.arms_pub = rospy.Publisher('/ros_robot_controller/bus_servo/set_position', SetBusServosPosition, queue_size=1)
        
        # Subscribers for VR headset data
        rospy.Subscriber('/quest/poses', PoseArray, self.pose_callback, queue_size=10)
        rospy.Subscriber('/quest/joints', JointState, self.joints_callback, queue_size=10)
        
        # Current robot head positions
        self.current_head_pan = 0.0
        self.current_head_tilt = 0.0
        
        # Base head positions for reset
        self.head_base_pan = 0.0
        self.head_base_tilt = 0.0
        
        # Operator head position data
        self.operator_head_pose = None
        self.operator_head_orientation = None
        
        # VR controller state data
        self.vr_controllers_state = {
            'left_grip': 0.0,
            'left_index': 0.0,
            'left_x': 0.0,
            'left_y': 0.0,
            'right_grip': 0.0,
            'right_index': 0.0,
            'right_a': 0.0,
            'right_b': 0.0
        }
        
        # Controller orientation data (for tilt calculation)
        self.left_controller_orientation = None
        self.right_controller_orientation = None
        
        # Arm and head control states
        self.arm_control_state = 'idle'  # 'idle', 'calibrating', 'controlling'
        self.head_control_enabled = False  # Head control enabled/disabled
        
        # Gripper states (for remembering last position)
        # Center position = 500, limits ±200
        self.left_gripper_state = 0.5   # 0.0 = closed, 1.0 = open (inverted)
        self.right_gripper_state = 0.5   # 0.0 = closed, 1.0 = open
        # Calibration system for each arm separately
        self.calibration_data = {
            'left_hand_base': None,   # Left hand base position during calibration
            'right_hand_base': None,  # Right hand base position during calibration
            'left_controller_base': None,  # Left controller base position
            'right_controller_base': None,  # Right controller base position
            'head_base': None         # Head base position during calibration
        }
        
        # Calibration values for each arm
        self.left_arm_calibration = {
            'sho_pitch_base': 874,    # l_sho_pitch base position
            'sho_roll_base': 833,     # l_sho_roll base position
            'el_yaw_base': 44,        # l_el_yaw base position
            'el_pitch_base': 502     # l_el_pitch base position
        }
        
        self.right_arm_calibration = {
            'sho_pitch_base': 126,    # r_sho_pitch base position
            'sho_roll_base': 167,     # r_sho_roll base position
            'el_yaw_base': 956,       # r_el_yaw base position
            'el_pitch_base': 498     # r_el_pitch base position
        }
        
        # Button press tracking to prevent repeated triggers
        self.button_states = {
            'left_x_pressed': False,
            'left_y_pressed': False
        }
        
        # Scaling (robot is 5x smaller than operator)
        self.scale_factor = 0.2  # 1/5 = 0.2
        
        # Sensitivity coefficients for new control system
        # Y -> sho_pitch, Z -> sho_roll, X -> el_yaw, X tilt -> el_pitch
        self.arm_sensitivity = {
            'y_to_sho_pitch': 90,      # Controller Y -> sho_pitch
            'z_to_sho_roll': 90,       # Controller Z -> sho_roll
            'x_to_el_yaw': 90,         # Controller X -> el_yaw
            'tilt_x_to_el_pitch': 35   # Controller X tilt -> el_pitch
        }
        
        # Start positions for robot arms (original values with correct IDs)
        self.arm_start_positions = {
            # Right arm (correct IDs from URDF)
            14: 126,   # r_sho_pitch - right shoulder forward-backward
            16: 167,   # r_sho_roll - right shoulder up-down
            18: 498,   # r_el_pitch - right forearm rotation
            20: 956,   # r_el_yaw - right forearm bend
            22: 500,   # r_gripper - right gripper
            
            # Left arm (correct IDs from URDF)
            13: 874,   # l_sho_pitch - left shoulder forward-backward
            15: 833,   # l_sho_roll - left shoulder up-down
            17: 502,   # l_el_pitch - left forearm rotation
            19: 44,    # l_el_yaw - left forearm bend
            21: 500    # l_gripper - left gripper
        }
        
        rospy.loginfo("TeleopFetcher node initialized")
        rospy.loginfo(f"Head sensitivity: {self.head_sensitivity}")
        rospy.loginfo(f"Maximum head pan: ±{self.max_head_pan}")
        rospy.loginfo(f"Maximum head tilt: ±{self.max_head_tilt}")
        rospy.loginfo("Subscribed to topics:")
        rospy.loginfo("  - /quest/poses: operator head and hand position data")
        rospy.loginfo("  - /quest/joints: VR controller button and joystick data")
        rospy.loginfo("Ready to receive data from Quest VR headset")
        rospy.loginfo(f"Initial arm control state: {self.arm_control_state}")
        rospy.loginfo(f"Head control: {'enabled' if self.head_control_enabled else 'disabled'}")
        rospy.loginfo("Press X on left controller for calibration")
        
        # Set initial arm pose
        self.set_arms_to_start_position()
    
    def pose_callback(self, pose_array):
        """
        Process operator head and hand position data.
        poses[0] = head (abs, frame_id: "unity_world")
        poses[1] = left hand (relative-to-head)
        poses[2] = right hand (relative-to-head)
        """
        if len(pose_array.poses) < 3:
            rospy.logwarn("Received incomplete PoseArray")
            return
        
        # Extract operator head data
        head_pose = pose_array.poses[0]
        self.operator_head_pose = head_pose.position
        self.operator_head_orientation = head_pose.orientation
        
        # Process head control
        self.process_head_control()
        
        # Process arm control
        left_hand_pose = pose_array.poses[1]
        right_hand_pose = pose_array.poses[2]
        
        # Save controller orientation for tilt calculation
        self.left_controller_orientation = left_hand_pose.orientation
        self.right_controller_orientation = right_hand_pose.orientation
        
        # Save last hand data for calibration
        self.last_left_hand_pose = left_hand_pose
        self.last_right_hand_pose = right_hand_pose
        
        # TEMPORARILY DISABLED: arm control
        # self.process_arms_control(left_hand_pose, right_hand_pose)
    
    def joints_callback(self, joint_state):
        """
        Process operator arm joint data.
        Handles VR controller button and joystick data.
        
        Expected data:
        - L_grip, L_index: left controller (grip and trigger)
        - R_grip, R_index: right controller (grip and trigger)
        - L_X, L_Y: left buttons (X and Y buttons)
        - R_A, R_B: right buttons (A and B buttons)
        """
        if joint_state.name and joint_state.position:
            joint_dict = dict(zip(joint_state.name, joint_state.position))
            
            # Extract controller data
            left_grip = joint_dict.get('L_grip', 0.0)
            left_index = joint_dict.get('L_index', 0.0)
            right_grip = joint_dict.get('R_grip', 0.0)
            right_index = joint_dict.get('R_index', 0.0)
            
            # Extract button data
            left_x = joint_dict.get('L_X', 0.0)
            left_y = joint_dict.get('L_Y', 0.0)
            right_a = joint_dict.get('R_A', 0.0)
            right_b = joint_dict.get('R_B', 0.0)
            
            # Log received data (rate limited)
            rospy.loginfo_throttle(2, 
                f"VR controllers - Left: grip={left_grip:.2f}, index={left_index:.2f}, "
                f"buttons X={left_x:.2f}, Y={left_y:.2f} | "
                f"Right: grip={right_grip:.2f}, index={right_index:.2f}, "
                f"buttons A={right_a:.2f}, B={right_b:.2f}"
            )
            
        # Process commands for robot arm control
            self.process_vr_controller_input(
                left_grip, left_index, left_x, left_y,
                right_grip, right_index, right_a, right_b
            )
    
    def process_head_control(self):
        """
        Process head control based on operator head position.
        Converts operator head orientation to robot head control commands.
        """
        if self.operator_head_orientation is None:
            return
        
        # Head control enabled only when arm control is active
        #if not self.head_control_enabled:
        #    return
        
        # Extract Euler angles from quaternion
        # Y controls head rotation left-right (-1 to 1)
        # X controls head tilt up-down (-1 to 1)
        euler_angles = self.quaternion_to_euler(
            self.operator_head_orientation.x,
            self.operator_head_orientation.y,
            self.operator_head_orientation.z,
            self.operator_head_orientation.w
        )
        
        # Convert angles to control commands
        # Y -> pan (left-right rotation)
        # X -> tilt (up-down tilt)
        y_rotation = euler_angles[1]  # Yaw (rotation around Z)
        x_rotation = euler_angles[0]  # Pitch (tilt around Y)
        
        # Apply sensitivity and limits with inversion
        # Invert control: operator turns left -> robot turns right
        # Add base positions for reset
        target_pan = self.head_base_pan + np.clip(-y_rotation * self.head_sensitivity, -self.max_head_pan, self.max_head_pan)
        target_tilt = self.head_base_tilt + np.clip(-x_rotation * self.head_sensitivity, -self.max_head_tilt, self.max_head_tilt)
        
        # Send head control commands
        self.send_head_command(target_pan, target_tilt)
    
    def quaternion_to_euler(self, x, y, z, w):
        """
        Convert quaternion to Euler angles (roll, pitch, yaw).
        """
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return [roll, pitch, yaw]
    
    def send_head_command(self, pan, tilt):
        """
        Send robot head control commands.
        Message structure: HeadCommand(position, duration)
        """
        # Create head control messages
        pan_msg = HeadCommand()
        pan_msg.position = pan
        pan_msg.duration = self.movement_duration
        
        tilt_msg = HeadCommand()
        tilt_msg.position = tilt
        tilt_msg.duration = self.movement_duration
        
        # Publish commands
        self.head_pan_pub.publish(pan_msg)
        self.head_tilt_pub.publish(tilt_msg)
        
        # Update current positions
        self.current_head_pan = pan
        self.current_head_tilt = tilt
        
        # Log commands (rate limited)
        rospy.loginfo_throttle(0.5, f"Head commands - Pan: {pan:.3f}, Tilt: {tilt:.3f}, Duration: {self.movement_duration}")
    
    def process_vr_controller_input(self, left_grip, left_index, left_x, left_y, 
                                   right_grip, right_index, right_a, right_b):
        """
        Process input data from VR controllers.
        
        Args:
            left_grip, left_index: left controller (grip and trigger)
            left_x, left_y: left buttons (X and Y buttons)
            right_grip, right_index: right controller (grip and trigger)
            right_a, right_b: right buttons (A and B buttons)
        """
        # Save current controller state
        self.vr_controllers_state.update({
            'left_grip': left_grip,
            'left_index': left_index,
            'left_x': left_x,
            'left_y': left_y,
            'right_grip': right_grip,
            'right_index': right_index,
            'right_a': right_a,
            'right_b': right_b
        })
        
        # Process buttons for arm and head control
        # Button X (left) - calibration/start control
        if left_x > 0.5 and not self.button_states['left_x_pressed']:
            rospy.loginfo(f"Button X pressed! Current state: {self.arm_control_state}")
            self.button_states['left_x_pressed'] = True
            if self.arm_control_state == 'idle':
                rospy.loginfo("Starting calibration...")
                self.start_arm_calibration()
            elif self.arm_control_state == 'calibrating':
                rospy.loginfo("Finishing calibration...")
                self.finish_arm_calibration()
        elif left_x <= 0.5:
            self.button_states['left_x_pressed'] = False
        
        # Button Y (left) - stop control
        if left_y > 0.5 and not self.button_states['left_y_pressed']:
            self.button_states['left_y_pressed'] = True
            if self.arm_control_state == 'controlling':
                self.stop_arm_control()
        elif left_y <= 0.5:
            self.button_states['left_y_pressed'] = False
        
        # TEMPORARILY DISABLED: gripper control
        # if self.arm_control_state == 'controlling':
        #     self.control_grippers(left_grip, right_grip, left_index, right_index)
    
    def process_arms_control(self, left_hand_pose, right_hand_pose):
        """
        Process robot arm control based on new logic:
        - Controller Y -> sho_pitch
        - Controller Z -> sho_roll
        - Controller X -> el_yaw
        - Controller X tilt -> el_pitch
        """
        if self.arm_control_state != 'controlling':
            return
        
        if self.calibration_data['left_hand_base'] is None or self.calibration_data['right_hand_base'] is None:
            return
        
        # Calculate relative offsets from calibration positions
        left_offset = self.calculate_hand_offset(left_hand_pose, self.calibration_data['left_hand_base'])
        right_offset = self.calculate_hand_offset(right_hand_pose, self.calibration_data['right_hand_base'])
        
        # Get controller tilts
        left_tilt = self.get_controller_tilt(self.left_controller_orientation, self.calibration_data['left_controller_base'])
        right_tilt = self.get_controller_tilt(self.right_controller_orientation, self.calibration_data['right_controller_base'])
        
        # Log data for debugging
        rospy.loginfo_throttle(1, 
            f"New control - Left: Y={left_offset['y']:.3f}->sho_pitch, Z={left_offset['z']:.3f}->sho_roll, "
            f"X={left_offset['x']:.3f}->el_yaw, tilt={left_tilt:.3f}->el_pitch | "
            f"Right: Y={right_offset['y']:.3f}->sho_pitch, Z={right_offset['z']:.3f}->sho_roll, "
            f"X={right_offset['x']:.3f}->el_yaw, tilt={right_tilt:.3f}->el_pitch"
        )
        
        # Convert to servo commands using new logic
        self.convert_to_new_servo_commands(left_offset, right_offset, left_tilt, right_tilt)
    
    def set_arms_to_start_position(self):
        """
        Set robot arms to start position.
        Sends commands to all arm servos with specified positions.
        """
        rospy.loginfo("Setting robot arms to start position...")
        
        # Create message for setting servo positions
        arm_msg = SetBusServosPosition()
        arm_msg.duration = 0.1  # Movement time in seconds
        
        # Create list of positions for all arm servos
        positions = []
        for servo_id, position in self.arm_start_positions.items():
            servo_pos = BusServoPosition()
            servo_pos.id = servo_id
            servo_pos.position = position
            positions.append(servo_pos)
            
            rospy.loginfo(f"Servo ID{servo_id}: position {position}")
        
        arm_msg.position = positions
        
        # Send command
        self.arms_pub.publish(arm_msg)
        rospy.loginfo("Start pose arm command sent")
        rospy.loginfo(f"Set {len(positions)} servos")
        
        # Wait for movement to complete
        rospy.sleep(arm_msg.duration + 0.5)  # Small delay for completion
        rospy.loginfo("Start arm pose set")
    
    def start_arm_calibration(self):
        """
        Start arm calibration. Operator must position arms in start pose.
        """
        rospy.loginfo("=== ARM CALIBRATION START ===")
        rospy.loginfo(f"Current state: {self.arm_control_state}")
        
        self.arm_control_state = 'calibrating'
        self.head_control_enabled = False  # Disable head control during calibration
        
        # Reset head to base position
        self.reset_head_to_base()
        
        rospy.loginfo("=== ARM CALIBRATION ===")
        rospy.loginfo("Position arms in start pose and press X to finish calibration")
        rospy.loginfo("Waiting for arm data...")
    
    def finish_arm_calibration(self):
        """
        Finish calibration and start arm control.
        """
        rospy.loginfo("=== CALIBRATION FINISH ===")
        rospy.loginfo(f"Current state: {self.arm_control_state}")
        
        # Save current arm positions as base
        # Arm data should be received in last pose_callback
        rospy.loginfo("Saving calibration data...")
        
        # Check that we have arm and controller data
        if (hasattr(self, 'last_left_hand_pose') and hasattr(self, 'last_right_hand_pose') and
            self.left_controller_orientation is not None and self.right_controller_orientation is not None):
            rospy.loginfo("Arm and controller data found, saving calibration...")
            self.calibration_data['left_hand_base'] = self.last_left_hand_pose
            self.calibration_data['right_hand_base'] = self.last_right_hand_pose
            self.calibration_data['left_controller_base'] = self.left_controller_orientation
            self.calibration_data['right_controller_base'] = self.right_controller_orientation
            self.calibration_data['head_base'] = self.operator_head_pose
            
            self.arm_control_state = 'controlling'
            self.head_control_enabled = True  # Enable head control
            rospy.loginfo("=== CALIBRATION COMPLETE ===")
            rospy.loginfo("New arm control system activated:")
            rospy.loginfo("  - Controller Y -> sho_pitch")
            rospy.loginfo("  - Controller Z -> sho_roll")
            rospy.loginfo("  - Controller X -> el_yaw")
            rospy.loginfo("  - Controller X tilt -> el_pitch")
            rospy.loginfo("Press Y to stop")
        else:
            rospy.logwarn("No arm or controller data for calibration. Try again.")
            rospy.logwarn("Ensure VR headset is connected and transmitting data")
            self.arm_control_state = 'idle'
    
    def stop_arm_control(self):
        """
        Stop arm and head control, return them to start pose.
        """
        self.arm_control_state = 'idle'
        self.head_control_enabled = False  # Disable head control
        rospy.loginfo("=== STOPPING ARM AND HEAD CONTROL ===")
        rospy.loginfo("Returning arms to start pose...")
        
        # Return arms to start pose
        self.set_arms_to_start_position()
        
        # Reset grippers to closed position
        self.reset_grippers()
        
        # Clear calibration data
        self.calibration_data = {
            'left_hand_base': None,
            'right_hand_base': None,
            'left_controller_base': None,
            'right_controller_base': None,
            'head_base': None
        }
        
        rospy.loginfo("Done. Press X for new calibration")
    
    def reset_head_to_base(self):
        """
        Reset head to base position.
        """
        rospy.loginfo("Resetting head to base position...")
        self.send_head_command(self.head_base_pan, self.head_base_tilt)
        self.current_head_pan = self.head_base_pan
        self.current_head_tilt = self.head_base_tilt
        rospy.loginfo(f"Head reset: pan={self.head_base_pan:.2f}, tilt={self.head_base_tilt:.2f}")
    
    def reset_grippers(self):
        """
        Reset grippers to center position (500).
        """
        rospy.loginfo("Resetting grippers to center position...")
        
        # Reset gripper states to center position
        self.left_gripper_state = 0.5
        self.right_gripper_state = 0.5
        
        # Send commands to center position (500)
        arm_msg = SetBusServosPosition()
        arm_msg.duration = 0.5
        
        positions = [
            BusServoPosition(id=21, position=500),   # Left gripper at center
            BusServoPosition(id=22, position=500)    # Right gripper at center
        ]
        
        arm_msg.position = positions
        self.arms_pub.publish(arm_msg)
        rospy.loginfo("Grippers reset to center position (500)")
    
    def calculate_hand_offset(self, current_pose, base_pose):
        """
        Calculate hand offset relative to calibration position.
        
        Args:
            current_pose: current hand position
            base_pose: calibration hand position
            
        Returns:
            dict: offsets along x, y, z axes
        """
        if base_pose is None:
            return {'x': 0, 'y': 0, 'z': 0}
        
        offset = {
            'x': current_pose.position.x - base_pose.position.x,
            'y': current_pose.position.y - base_pose.position.y,
            'z': current_pose.position.z - base_pose.position.z
        }
        
        return offset
    
    def get_controller_tilt(self, current_orientation, base_orientation):
        """
        Calculate controller tilt along X axis relative to calibration position.
        
        Args:
            current_orientation: current controller orientation
            base_orientation: calibration controller orientation
            
        Returns:
            float: tilt along X axis in radians
        """
        if base_orientation is None or current_orientation is None:
            return 0.0
        
        # Convert quaternions to Euler angles
        current_euler = self.quaternion_to_euler(
            current_orientation.x, current_orientation.y, 
            current_orientation.z, current_orientation.w
        )
        base_euler = self.quaternion_to_euler(
            base_orientation.x, base_orientation.y,
            base_orientation.z, base_orientation.w
        )
        
        # Calculate tilt difference along X axis (pitch)
        tilt_difference = current_euler[0] - base_euler[0]  # pitch (forward-backward tilt)
        
        return tilt_difference
    
    def convert_to_new_servo_commands(self, left_offset, right_offset, left_tilt, right_tilt):
        """
        Convert offsets and tilts to servo commands using new logic.
        
        Args:
            left_offset: left hand offset (x, y, z)
            right_offset: right hand offset (x, y, z)
            left_tilt: left controller tilt
            right_tilt: right controller tilt
        """
        # Left arm: Y->sho_pitch, Z->sho_roll, X->el_yaw, tilt->el_pitch
        left_angles = {
            'sho_pitch': self.left_arm_calibration['sho_pitch_base'] + int(left_offset['y'] * self.arm_sensitivity['y_to_sho_pitch']),
            'sho_roll': self.left_arm_calibration['sho_roll_base'] + int(left_offset['z'] * self.arm_sensitivity['z_to_sho_roll']),
            'el_yaw': self.left_arm_calibration['el_yaw_base'] + int(left_offset['x'] * self.arm_sensitivity['x_to_el_yaw']),
            'el_pitch': self.left_arm_calibration['el_pitch_base'] + int(left_tilt * self.arm_sensitivity['tilt_x_to_el_pitch'])
        }
        
        # Right arm: Y->sho_pitch, Z->sho_roll, X->el_yaw, tilt->el_pitch (mirrored)
        right_angles = {
            'sho_pitch': self.right_arm_calibration['sho_pitch_base'] - int(right_offset['y'] * self.arm_sensitivity['y_to_sho_pitch']),
            'sho_roll': self.right_arm_calibration['sho_roll_base'] - int(right_offset['z'] * self.arm_sensitivity['z_to_sho_roll']),
            'el_yaw': self.right_arm_calibration['el_yaw_base'] - int(right_offset['x'] * self.arm_sensitivity['x_to_el_yaw']),
            'el_pitch': self.right_arm_calibration['el_pitch_base'] - int(right_tilt * self.arm_sensitivity['tilt_x_to_el_pitch'])
        }
        
        # Limit angles to bounds
        left_angles = self.limit_servo_angles(left_angles, 'left')
        right_angles = self.limit_servo_angles(right_angles, 'right')
        
        # Send commands
        self.send_arm_commands(left_angles, right_angles)
    
    def limit_servo_angles(self, angles, hand_side):
        """
        Limit servo angles to hand-specific bounds.
        
        Args:
            angles: dictionary of angles
            hand_side: 'left' or 'right'
            
        Returns:
            dict: limited angles
        """
        limited_angles = {}
        
        for key, angle in angles.items():
            if key == 'sho_roll' and hand_side == 'left':
                # ID15 (l_sho_roll): maximum 800
                limited_angles[key] = int(max(100, min(800, angle)))
            elif key == 'sho_roll' and hand_side == 'right':
                # ID16 (r_sho_roll): minimum 200
                limited_angles[key] = int(max(200, min(900, angle)))
            else:
                # Other servos - standard limits
                limited_angles[key] = int(max(100, min(900, angle)))
        
        return limited_angles
    
    def convert_to_servo_commands(self, left_offset, right_offset):
        """
        Convert arm offsets to servo commands.
        
        Args:
            left_offset: left arm offset
            right_offset: right arm offset
        """
        # Simple inverse kinematics
        # Apply scaling
        left_scaled = {
            'x': left_offset['x'] * self.scale_factor,
            'y': left_offset['y'] * self.scale_factor,
            'z': left_offset['z'] * self.scale_factor
        }
        
        right_scaled = {
            'x': right_offset['x'] * self.scale_factor,
            'y': right_offset['y'] * self.scale_factor,
            'z': right_offset['z'] * self.scale_factor
        }
        
        # Convert to servo angles (simplified model)
        left_angles = self.calculate_servo_angles(left_scaled, 'left')
        right_angles = self.calculate_servo_angles(right_scaled, 'right')
        
        # Send commands
        self.send_arm_commands(left_angles, right_angles)
    
    def calculate_servo_angles(self, offset, hand_side):
        """
        Calculate servo angles based on hand offset.
        
        Args:
            offset: hand offset (x, y, z) in meters
            hand_side: 'left' or 'right'
            
        Returns:
            dict: angles for servos (0-1000)
        """
        # Improved inverse kinematics
        # X -> shoulder rotation forward-backward (sho_pitch)
        # Y -> shoulder lift up-down (sho_roll)
        # Z -> forearm rotation (el_yaw)
        
        # Coefficients for converting offsets to angles (radians per meter)
        # 1 radian ≈ 57.3 degrees, 1000 units = 2π radians
        scale_x = 200 * self.arm_sensitivity.get('x', 90)  # X sensitivity
        scale_y = 200 * self.arm_sensitivity.get('y', 90)  # Y sensitivity
        scale_z = 100 * self.arm_sensitivity.get('z', 90)  # Z sensitivity
        
        # Debug info
        rospy.loginfo_throttle(2, 
            f"Kinematics {hand_side}: offset={offset}, scale_x={scale_x}, scale_y={scale_y}, scale_z={scale_z}"
        )
        
        # Base positions (start positions from config)
        if hand_side == 'left':
            base_sho_pitch = 874  # l_sho_pitch start position
            base_sho_roll = 833   # l_sho_roll start position
            base_el_pitch = 502   # l_el_pitch start position
            base_el_yaw = 44      # l_el_yaw start position
        else:
            base_sho_pitch = 126  # r_sho_pitch start position
            base_sho_roll = 167   # r_sho_roll start position
            base_el_pitch = 498   # r_el_pitch start position
            base_el_yaw = 956     # r_el_yaw start position
        
        angles = {}
        
        if hand_side == 'left':
            # Left arm
            angles['sho_pitch'] = base_sho_pitch + int(offset['x'] * scale_x)  # l_sho_pitch
            angles['sho_roll'] = base_sho_roll + int(offset['y'] * scale_y)    # l_sho_roll
            angles['el_pitch'] = base_el_pitch  # l_el_pitch (not used yet)
            angles['el_yaw'] = base_el_yaw + int(offset['z'] * scale_z)        # l_el_yaw
        else:
            # Right arm (mirrored)
            angles['sho_pitch'] = base_sho_pitch - int(offset['x'] * scale_x)  # r_sho_pitch
            angles['sho_roll'] = base_sho_roll - int(offset['y'] * scale_y)   # r_sho_roll
            angles['el_pitch'] = base_el_pitch  # r_el_pitch (not used yet)
            angles['el_yaw'] = base_el_yaw - int(offset['z'] * scale_z)       # r_el_yaw
        
        # Limit angles to servo-specific bounds
        # ID15 (l_sho_roll): maximum 800
        if 'sho_roll' in angles and hand_side == 'left':
            angles['sho_roll'] = int(max(100, min(800, angles['sho_roll'])))
        # ID16 (r_sho_roll): minimum 200
        elif 'sho_roll' in angles and hand_side == 'right':
            angles['sho_roll'] = int(max(200, min(900, angles['sho_roll'])))
        # Other servos - standard limits
        else:
            for key in angles:
                if key != 'sho_roll':  # sho_roll already processed above
                    angles[key] = int(max(100, min(900, angles[key])))
        
        # Additional check: ensure all values are positive integers
        for key in angles:
            angles[key] = int(max(0, angles[key]))
        
        return angles
    
    def send_arm_commands(self, left_angles, right_angles):
        """
        Send arm control commands.
        
        Args:
            left_angles: angles for left arm
            right_angles: angles for right arm
        """
        arm_msg = SetBusServosPosition()
        arm_msg.duration = 0.1  # Fast update
        
        positions = []
        
        # Left arm (correct IDs from URDF)
        if left_angles:
            positions.append(BusServoPosition(id=13, position=int(max(0, left_angles['sho_pitch']))))   # l_sho_pitch
            positions.append(BusServoPosition(id=15, position=int(max(0, left_angles['sho_roll']))))    # l_sho_roll
            positions.append(BusServoPosition(id=17, position=int(max(0, left_angles['el_pitch']))))  # l_el_pitch
            positions.append(BusServoPosition(id=19, position=int(max(0, left_angles['el_yaw']))))     # l_el_yaw
        
        # Right arm (correct IDs from URDF)
        if right_angles:
            positions.append(BusServoPosition(id=14, position=int(max(0, right_angles['sho_pitch']))))  # r_sho_pitch
            positions.append(BusServoPosition(id=16, position=int(max(0, right_angles['sho_roll']))))  # r_sho_roll
            positions.append(BusServoPosition(id=18, position=int(max(0, right_angles['el_pitch']))))  # r_el_pitch
            positions.append(BusServoPosition(id=20, position=int(max(0, right_angles['el_yaw']))))    # r_el_yaw
        
        arm_msg.position = positions
        self.arms_pub.publish(arm_msg)
        
        # Log commands for debugging
        rospy.loginfo_throttle(1, f"Arm commands - Left: {left_angles}, Right: {right_angles}")
    
    def control_grippers(self, left_grip, right_grip, left_index, right_index):
        """
        Control arm grippers based on controller grip and index values.
        
        Args:
            left_grip: left controller grip value (0.0-1.0)
            right_grip: right controller grip value (0.0-1.0)
            left_index: left controller index value (0.0-1.0)
            right_index: right controller index value (0.0-1.0)
        """
        if self.arm_control_state != 'controlling':
            return
        
        # Gripper control logic:
        # index = 1.0 -> close gripper (decrease angle)
        # grip = 1.0 -> open gripper (increase angle)
        # If buttons released, gripper stays in last position
        
        # Left gripper
        if left_index > 0.5:  # Close
            self.left_gripper_state = max(0.0, self.left_gripper_state - 0.1)
        elif left_grip > 0.5:  # Open
            self.left_gripper_state = min(1.0, self.left_gripper_state + 0.1)
        
        # Right gripper
        if right_index > 0.5:  # Close
            self.right_gripper_state = max(0.0, self.right_gripper_state - 0.1)
        elif right_grip > 0.5:  # Open
            self.right_gripper_state = min(1.0, self.right_gripper_state + 0.1)
        
        # Convert state to servo angles
        # Center position = 500, limits ±200
        # Left gripper inverted: 0.0 = open (700), 1.0 = closed (300)
        # Right gripper: 0.0 = closed (300), 1.0 = open (700)
        left_gripper_pos = int(500 + (1.0 - self.left_gripper_state) * 200)  # Inverted
        right_gripper_pos = int(500 + self.right_gripper_state * 200)  # Normal
        
        # Limit angles to ±200 from center position
        left_gripper_pos = max(300, min(700, left_gripper_pos))
        right_gripper_pos = max(300, min(700, right_gripper_pos))
        
        # Send commands to grippers
        arm_msg = SetBusServosPosition()
        arm_msg.duration = 0.1
        
        positions = [
            BusServoPosition(id=21, position=left_gripper_pos),   # Left gripper
            BusServoPosition(id=22, position=right_gripper_pos)   # Right gripper
        ]
        
        arm_msg.position = positions
        self.arms_pub.publish(arm_msg)
    
    def run(self):
        """
        Main node loop.
        """
        rospy.loginfo("TeleopFetcher node started")
        rospy.spin()


if __name__ == '__main__':
    try:
        teleop_fetcher = TeleopFetcher()
        teleop_fetcher.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("TeleopFetcher node stopped")
