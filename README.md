# Teleop Fetch - ROS Teleoperation Package

> **Related Project**: This ROS package is designed to work with the [Robot MR Control VR application](../robots-mr-main) for Quest VR headsets.

## Overview

ROS 1 Noetic package for teleoperation of Fetch-based robots (Brewie) using VR control system. This package receives pose and joint data from a VR headset and translates them into robot commands for head movement, arm control, and gripper operation.

## Features

- **Head control** synchronized with operator's head position
- **Automatic arm initialization** to starting pose on startup
- **VR controller-based calibration** and arm control
- **Inverse kinematics** with 1:5 scaling for intuitive control
- **Gripper control** via VR controller buttons
- **Configurable sensitivity** parameters
- Ready for integration with [VR teleoperation application](../robots-mr-main)

## Dependencies

### ROS Packages
- ROS Noetic (tested)
- `geometry_msgs`
- `sensor_msgs`
- `std_msgs`
- `ros_robot_controller` (for servo control)

### Python
- Python 3
- NumPy

## Installation

1. **Prerequisites**: Ensure ROS Noetic is installed

2. **Clone the package** to your catkin workspace:
   ```bash
   cd ~/catkin_ws/src
   # Copy this package here
   ```

3. **Build the workspace**:
   ```bash
   cd ~/catkin_ws
   catkin_make
   # or
   catkin build
   ```

4. **Source the workspace**:
   ```bash
   source ~/catkin_ws/devel/setup.bash
   ```

## Usage

### Prerequisites

Ensure the [VR application](../robots-mr-main) is installed on the Quest headset and both devices are on the same network.

### Launching the Node

**Direct launch:**
```bash
rosrun teleop_fetch fetcher.py
```

**Using launch file:**
```bash
roslaunch teleop_fetch teleop_fetch.launch
```

### Parameters

You can configure the following parameters:

- `head_sensitivity` (default: 1.0) - Head control sensitivity multiplier
- `max_head_pan` (default: 2.0) - Maximum head rotation left/right (radians)
- `max_head_tilt` (default: 2.0) - Maximum head tilt up/down (radians)
- `movement_duration` (default: 0.2) - Time duration for head movements (seconds)

**Example with custom parameters:**
```bash
rosrun teleop_fetch fetcher.py _head_sensitivity:=1.5 _max_head_pan:=2.5
```

### Topics

#### Subscribed Topics (Input from VR):
- `/quest/poses` (`geometry_msgs/PoseArray`) - Operator's head and hand position data from VR headset
- `/quest/joints` (`sensor_msgs/JointState`) - Operator's hand joint data from VR controllers

#### Published Topics (Output to Robot):
- `/head_pan_controller/command` (`teleop_fetch/HeadCommand`) - Head rotation commands {position, duration}
- `/head_tilt_controller/command` (`teleop_fetch/HeadCommand`) - Head tilt commands {position, duration}
- `/ros_robot_controller/bus_servo/set_position` (`ros_robot_controller/SetBusServosPosition`) - Arm and gripper control commands

## Code Structure

The code is organized in the `TeleopFetcher` class with the following methods:

- `__init__()` - Initialize node, publishers, subscribers, and parameters
- `pose_callback()` - Process head and hand position data from VR
- `joints_callback()` - Process hand joint data from VR controllers
- `process_head_control()` - Convert head movements to robot head commands
- `process_arms_control()` - Implement inverse kinematics for arm control
- `set_arms_to_start_position()` - Initialize arms to starting pose

## Arm Control Details

### Starting Servo Positions

When the node starts, the following servo positions are automatically set:

**Right arm:**
- ID14: 126 - `r_sho_pitch` (right shoulder pitch - forward/backward)
- ID16: 167 - `r_sho_roll` (right shoulder roll - up/down)  
- ID18: 498 - `r_el_pitch` (right elbow pitch - forearm bend)
- ID20: 956 - `r_el_yaw` (right elbow yaw - forearm rotation)
- ID22: 500 - `r_gripper` (right gripper - neutral position)

**Left arm:**
- ID13: 874 - `l_sho_pitch` (left shoulder pitch - forward/backward)
- ID15: 833 - `l_sho_roll` (left shoulder roll - up/down)
- ID17: 502 - `l_el_pitch` (left elbow pitch - forearm bend)  
- ID19: 44 - `l_el_yaw` (left elbow yaw - forearm rotation)
- ID21: 500 - `l_gripper` (left gripper - neutral position)

### Testing Arm Setup

To test the starting arm pose configuration:

```bash
rosrun teleop_fetch test_arm_setup.py
```

## VR Teleoperation Workflow

### Calibration System

1. **Start calibration**: Press **X** button on the left VR controller
2. **Position arms**: Set your arms in the starting pose corresponding to the robot's initial position
3. **Complete calibration**: Press **X** button again to lock calibration
4. **Active control**: Move your arms and head - the robot will replicate movements with 1:5 scaling
5. **Stop control**: Press **Y** button on the left controller to stop and return to starting pose

**Note**: Head control is automatically enabled/disabled together with arm control.

### Gripper Control

- **Close gripper**: Press **index trigger** button on the VR controller
- **Open gripper**: Press **grip** button on the VR controller
- **Position memory**: Gripper maintains last position when buttons are released
- **Neutral position**: 500 (set on initialization and reset)
- **Movement range**: ±200 from neutral (300-700)
- **Note**: Left gripper control is inverted

### Inverse Kinematics

The system uses simplified inverse kinematics with the following mappings:

- **X-displacement** → Shoulder pitch (forward/backward rotation)
- **Y-displacement** → Shoulder roll (up/down lift)
- **Z-displacement** → Elbow yaw (forearm rotation)

**Scaling factor**: 0.2 (robot is 5 times smaller than operator movements)

## System Integration

### Complete Setup

1. **Robot side**:
   ```bash
   # Start ROSBridge for VR communication
   roslaunch rosbridge_server rosbridge_websocket.launch
   
   # Start teleoperation node
   roslaunch teleop_fetch teleop_fetch.launch
   
   # Start robot controllers
   # (your robot-specific launch files)
   ```

2. **VR headset side**:
   - Launch the [VR application](../robots-mr-main)
   - Connect to robot's IP address and port (default: 9090)
   - Follow calibration and control procedures

## Message Types

### HeadCommand.msg
```
float64 position
float64 duration
```

## Troubleshooting

**Robot doesn't respond**:
- Check if ROS topics are being published: `rostopic list`
- Verify data is coming from VR: `rostopic echo /quest/poses`
- Check node is running: `rosnode list`

**Arm movements are incorrect**:
- Re-run calibration in VR application
- Verify starting positions are correct
- Check inverse kinematics scaling factor

**Gripper not working**:
- Confirm servo IDs are correct
- Check servo communication
- Verify button mappings in VR application

## Future Extensions

The code architecture is designed to support:
- More complex inverse kinematics algorithms
- Additional control features and modes
- Integration with different robot models
- Advanced motion planning

## Related Projects

- **[Robot MR Control](../robots-mr-main)** - Unity VR application for Meta Quest headsets (example implementation for Brewie robot)

## Contributing

[Your contribution guidelines]

## License

[Your License Here]

## Authors

Angels Control Team

## Version

1.0.0
