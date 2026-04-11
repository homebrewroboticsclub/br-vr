# Teleop Fetch

ROS package for dual-arm robot teleoperation using a Quest VR headset on ROS1 Noetic.

**Documentation:** [DOC/README.md](DOC/README.md) (architecture, datasets, operator session contracts). **Agents:** [DEV_AI.md](DEV_AI.md).

## Ecosystem entry point

**Full stack launch (KYR + x402 + this package), RAID, and dashboard ports** — metapackage **`br_bringup`**: **[../br_bringup/README.md](../br_bringup/README.md)**. **Agent index (whole workspace):** **[../br_bringup/DEV_AI.md](../br_bringup/DEV_AI.md)**.

> `teleop_fetch` is the historical/internal package name. The code is robot-agnostic and not tied to any specific vendor robot; Ainex is simply the first robot family with a full configuration and MoveIt integration (see the `ainex_moveit` config package).

## Features

- **Robot head control based on operator head position**
- **Automatic arm start pose setup on launch**
- **Calibration and arm control via VR controllers**
- **Inverse kinematics with 1:5 scaling**
- **Arm gripper control**
- Ready for VR teleoperation
- Configurable sensitivity parameters
- **Extended dual‑arm support**:
  - Unified head/arms/grippers start/stop with X/Y buttons
  - Single publisher to `/ros_robot_controller/bus_servo/set_position`
  - Dataset recording and upload API for `.hbr` datasets

## Dependencies

- ROS (tested with ROS Noetic)
- Python 3
- NumPy
- `rospy`, `geometry_msgs`, `sensor_msgs`, `std_msgs`
- `ainex_interfaces` (HeadState) or equivalent head/arm interfaces
- `ros_robot_controller` (for bus servo control) or an adapter layer
- `robot`, `my_package` (for MoveIt/IK and robot-specific config)

## Installation

1. Ensure ROS is installed.
2. Copy the package to your workspace.
3. Run `catkin_make` or `catkin build`.

## Usage

### Launching the node

Minimal teleop node (head + arms + start/stop):

```bash
roslaunch teleop_fetch teleop_fetch.launch
```

Full VR teleop stack (robot + MoveIt + IK + teleop):

```bash
# Full stack: robot, move_group, fast_ik, teleop
roslaunch teleop_fetch teleop.launch

# With RViz for debugging
roslaunch teleop_fetch teleop_debug.launch
```

### Parameters (legacy Fetch head control)

- `head_sensitivity` (default: 1.0) - head control sensitivity
- `max_head_pan` (default: 2.0) - maximum head rotation left/right
- `max_head_tilt` (default: 2.0) - maximum head tilt up/down
- `movement_duration` (default: 0.2) - head movement time

### Topics (core)

#### Input topics:

- `/quest/poses` (`geometry_msgs/PoseArray`) - operator head and hand position data
- `/quest/joints` (`sensor_msgs/JointState`) - operator arm joint and button data

#### Output topics:

- `/head_pan_controller/command` (`teleop_fetch/HeadCommand` or `ainex_interfaces/HeadState`) - head pan commands
- `/head_tilt_controller/command` (`teleop_fetch/HeadCommand` or `ainex_interfaces/HeadState`) - head tilt commands
- `/ros_robot_controller/bus_servo/set_position` (`ros_robot_controller/SetBusServosPosition`) - arm control commands

#### Ainex-specific topics:

- `/teleop_fetch/quest_poses_remapped` (`PoseArray`) - VR poses after mapping + calibration + scale
- `/teleop_fetch/poses` (`PoseArray`) - merged poses (VR/manual)
- `/teleop_fetch/scale` (`Float64`) - sensitivity 0.0001..100 from UI
- `/teleop_fetch/arm_servo_targets` (`SetBusServosPosition`) - IK outputs from `fast_ik_node`
- `/teleop_state` (**latched**): see `DOC/ARCHITECTURE.md`. In short: after grant, by default waits for rising edge on **`~operator_arm/joint_name_lx`** on `~vr_input/joints_topic`; **`~arm_stream_requires_lx:=false`** — arm stream right after ACTIVE. IK: `/teleop_fetch/teleop_state`.
- `/record_sessions` (`std_msgs/String(JSON)`) - dataset lifecycle events

## Code structure

Legacy Fetch implementation is organized around the `TeleopFetcher` class:

- `pose_callback()` - process head and hand position data
- `joints_callback()` - process arm joint data
- `process_head_control()` - head control logic
- `process_arms_control()` - arm control logic
- `set_arms_to_start_position()` - set arm start pose

New Ainex teleop stack adds:

- `teleop_node.py` — main VR teleop node (head, arms, start/stop).
- `vr_remapper_node.py` — axis mapping, R_A calibration, scale.
- `pose_source_node.py` — multiplexing VR and manual poses.
- `dataset_recorder_node.py`, `dataset_upload_server.py`, `episode_recorder.py` — `.hbr` dataset pipeline.

## Arm control

### Servo start positions (legacy Fetch)

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

1. **Start calibration**: Press **X** button on left controller.
2. **Positioning**: Position arms in start pose (matching robot initial pose).
3. **Finish calibration**: Press **X** again.
4. **Control**: Move arms and head — robot will replicate movements with 1:5 scaling.
5. **Stop**: Press **Y** to stop control and return to start pose.

**Note**: Head control is automatically enabled/disabled with arm control.

### Gripper control

- **Close**: Press **index** (trigger) button on controller.
- **Open**: Press **grip** button on controller.
- **Memory**: If buttons are released, gripper stays in the last position.
- **Center position**: 500 (on init and reset).
- **Movement limits**: ±200 from center (300–700).
- **Inversion**: Left gripper works inverted.

### Inverse kinematics

The system uses simplified inverse kinematics:

- **X-offset** → shoulder rotation forward-backward.
- **Y-offset** → shoulder lift up-down.
- **Z-offset** → forearm rotation.

Scaling: robot is 5x smaller than operator (coefficient 0.2).

## Dataset recording & docs

For dataset recording semantics and `.hbr` container format, see:

- [DOC/README.md](DOC/README.md) — full documentation index
- [ARCHITECTURE.md](DOC/ARCHITECTURE.md) — abstraction levels, mappings, data flows
- [PROJECT_STATE.md](DOC/PROJECT_STATE.md) — package status
- [TODO.md](DOC/TODO.md) — known issues / backlog
- [TELEOP_DATAS.md](DOC/TELEOP_DATAS.md) — headset event and upload payload contract
- [HBR.md](DOC/HBR.md) — `.hbr` container format and storage requirements

## Example robot config (Ainex)

The first fully supported dual‑arm configuration is for the Ainex robot. The MoveIt and robot description package lives separately in:

- [`ainex_moveit`](https://github.com/homebrewroboticsclub/ainex_moveit)

Other robots can be integrated by providing a similar MoveIt + description package and wiring topics to match `teleop_fetch` expectations.
