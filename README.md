# Teleop Fetch

ROS package for Fetch robot teleoperation using Quest VR headset.

## Features

- Robot head control based on operator head position
- Automatic arm start pose setup on launch
- **Calibration and arm control via VR controllers**
- **Inverse kinematics with 1:5 scaling**
- **Arm gripper control**
- Ready for VR teleoperation
- Configurable sensitivity parameters

## Dependencies

- ROS (tested with ROS Noetic)
- Python 3
- NumPy
- geometry_msgs
- sensor_msgs
- std_msgs

## Installation

1. Ensure ROS is installed
2. Copy the package to your workspace
3. Run `catkin_make` or `catkin build`

## Usage

### Launching the node

```bash
# Direct launch
rosrun teleop_fetch fetcher.py

# Or via launch file
roslaunch teleop_fetch teleop_fetch.launch
```

### Parameters

- `head_sensitivity` (default: 1.0) - head control sensitivity
- `max_head_pan` (default: 2.0) - maximum head rotation left/right
- `max_head_tilt` (default: 2.0) - maximum head tilt up/down
- `movement_duration` (default: 0.2) - head movement time

### Topics

#### Input topics:
- `/quest/poses` (geometry_msgs/PoseArray) - operator head and hand position data
- `/quest/joints` (sensor_msgs/JointState) - operator arm joint data

#### Output topics:
- `/head_pan_controller/command` (teleop_fetch/HeadCommand) - head pan commands {position, duration}
- `/head_tilt_controller/command` (teleop_fetch/HeadCommand) - head tilt commands {position, duration}
- `/ros_robot_controller/bus_servo/set_position` (ros_robot_controller/SetBusServosPosition) - arm control commands

## Code structure

Code is organized in `TeleopFetcher` class with methods:

- `pose_callback()` - process head and hand position data
- `joints_callback()` - process arm joint data
- `process_head_control()` - head control logic
- `process_arms_control()` - arm control logic
- `set_arms_to_start_position()` - set arm start pose

## Arm control

### Servo start positions

The following positions are automatically set when the node launches:

**Right arm:**
- ID14: 126 - r_sho_pitch (right shoulder forward-backward)
- ID16: 167 - r_sho_roll (right shoulder up-down)
- ID18: 498 - r_el_pitch (right forearm bend)
- ID20: 956 - r_el_yaw (right forearm rotation)
- ID22: 500 - r_gripper (right gripper)

**Left arm:**
- ID13: 874 - l_sho_pitch (left shoulder forward-backward)
- ID15: 833 - l_sho_roll (left shoulder up-down)
- ID17: 502 - l_el_pitch (left forearm bend)
- ID19: 44 - l_el_yaw (left forearm rotation)
- ID21: 500 - l_gripper (left gripper)

### Testing

To test arm start pose setup:

```bash
rosrun teleop_fetch test_arm_setup.py
```

## VR arm control

### Calibration system

1. **Start calibration**: Press **X** button on left controller
2. **Positioning**: Position arms in start pose (matching robot initial pose)
3. **Finish calibration**: Press **X** again
4. **Control**: Move arms and head - robot will replicate movements with 1:5 scaling
5. **Stop**: Press **Y** to stop control and return to start pose

**Note**: Head control is automatically enabled/disabled with arm control

### Gripper control

- **Close**: Press **index** (trigger) button on controller
- **Open**: Press **grip** button on controller
- **Memory**: If buttons released, gripper stays in last position
- **Center position**: 500 (on init and reset)
- **Movement limits**: ±200 from center (300-700)
- **Inversion**: Left gripper works inverted

### Inverse kinematics

The system uses simplified inverse kinematics:
- **X-offset** → shoulder rotation forward-backward
- **Y-offset** → shoulder lift up-down
- **Z-offset** → forearm rotation

Scaling: robot is 5x smaller than operator (coefficient 0.2)

## Future extensions

Code is ready for adding more complex inverse kinematics and additional control features.
