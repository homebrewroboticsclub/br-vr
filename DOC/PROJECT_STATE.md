# Project State — VR Teleop

**Version:** 2.0 beta  
**Date:** 2026-03-31

## Overview

Unified VR teleoperation for dual-arm robots: head, arms, grippers, X/Y start/stop. A single publisher to `bus_servo`.

---

## Packages and their status

### teleop_fetch (main teleop package)

| Component                       | Description                                                    | Status       |
|---------------------------------|----------------------------------------------------------------|--------------|
| `teleop_node.py`               | Main node: head, arms, start/stop                             | ✅            |
| `vr_remapper_node.py`          | Axis mapping, R_A calibration, scale                          | ✅ beta 1.0  |
| `pose_source_node.py`          | VR (remapped) + manual → `/teleop_fetch/poses`                | ✅            |
| `head_controller.py`           | Pan/tilt from head orientation                                | ✅            |
| `start_stop_controller.py`     | X = enable arms, Y = disable                                  | ✅            |
| `dataset_recorder_node.py`     | `/record_sessions` start/stop, robot-side capture, `.hbr` finalize | ✅ beta 1.1 |
| `dataset_upload_server.py`     | `POST /upload_dataset` on port 9191                           | ✅ beta 1.1  |
| `episode_recorder.py`          | Session manager + HBR writer integration                      | ✅ beta 1.1  |
| `sensors/base_*.py`, `sensors/ros_*.py` | Typed sensor abstractions (camera/IMU/joints)        | ✅ beta 1.1  |
| `config/teleop.yaml`           | Servo IDs, arm start, head                                    | ✅            |
| `config/vr_remapper.yaml`      | Reference pose, scale                                         | ✅            |
| `config/dataset_recorder.yaml` | Dataset topics, storage paths, upload API                     | ✅ beta 1.1  |
| `web/teleop_debug.html`        | 3D visualization, scale, manual drag                         | ✅            |

**Launch:** `roslaunch teleop_fetch teleop.launch`

**CI / headless:** `catkin run_tests teleop_fetch` runs nosetests plus `rostest` `test/teleop_kyr_arm_stream.test` (KYR proxy + emulated grant + `arm_servo_targets`; the test sets `servo_command_out_topic` to `/bus_servo/set_position` so no hardware node is required). On the robot, `kyr_proxy` defaults to **`/ros_robot_controller/bus_servo/set_position`**. KYR `SessionModule` normalizes empty grant `scope_json` so `bus_servo` is allowed (see `ARCHITECTURE.md`).

---

### my_package (fast_ik_node)

| Component             | Description                                      | Status |
|-----------------------|--------------------------------------------------|--------|
| `fast_ik_node.cpp`   | IK for both arms, gripper, joint→servo mapping   | ✅      |
| `config/fast_ik.yaml`| Gripper, MoveIt groups, left_hand config         | ✅      |
| Publishes             | `/teleop_fetch/arm_servo_targets`               | ✅      |
| Publishes             | `/teleop_fetch/debug_target_poses`              | ✅      |

**Note:** Axis mapping, calibration and scale live in `vr_remapper`. `fast_ik` receives poses in `body_link`.

---

### robot

| Component                         | Description              | Status |
|-----------------------------------|--------------------------|--------|
| `planning_context`, `move_group`, `SRDF` | MoveIt, kinematics | ✅      |
| `robot_description`              | URDF, `ainex_description` | ✅    |

---

### ainex_interfaces

| Component  | Description                     | Status |
|-----------|---------------------------------|--------|
| HeadState | Message type for the head       | ✅      |
| HeadCommand | Compatible with HeadState    | ✅      |

---

### ros_robot_controller

| Component            | Description                   | Status |
|----------------------|-------------------------------|--------|
| SetBusServosPosition | Bus servo command message     | ✅      |
| `/ros_robot_controller/bus_servo/set_position` | Driver input (`SetBusServosPosition`); KYR proxy publishes here after policy | ✅      |

---

## Data architecture

See [ARCHITECTURE.md](ARCHITECTURE.md) for abstraction levels, flows, and diagrams.

Short version:  
`/quest/poses` → `vr_remapper` (map + R_A calibration + scale) → `pose_source` → `fast_ik` (IK) → `teleop_fetch` → `/kyr/bus_servo_in` → `kyr_proxy` → `/ros_robot_controller/bus_servo/set_position`.

Dataset branch:  
`/record_sessions` + robot sensors + `/upload_dataset` → dataset recorder → `.hbr`.

---

## Configuration

| File                       | Key parameters                              |
|----------------------------|---------------------------------------------|
| `config/teleop.yaml`      | `servo_ids`, `arm_start_positions`, head, VR topics |
| `config/vr_remapper.yaml` | Reference pose (left/right), scale          |
| `config/fast_ik.yaml` (my_package) | Gripper, MoveIt groups, left_hand  |

---

## Calibration (beta 1.0)

- **R_A on right joystick:** Operator moves arms into a natural pose (in front, slightly down). `vr_remapper` computes `offset = reference_pose - mapped_vr`. The reference robot pose is defined in `vr_remapper.yaml`.
- **SCALE:** Sensitivity 0.0001..100, topic `/teleop_fetch/scale`, updated live from the UI.

---

## Debugging

- **RViz:** `roslaunch teleop_fetch teleop_debug.launch`
- **Web viz:** rosbridge + `teleop_debug.html`
- **Topics:** `/teleop_fetch/debug_target_poses`, `/teleop_fetch/teleop_state`, `/visualization_marker`

---

## Topic `/teleop_fetch/teleop_state`

**Type:** `ainex_interfaces/TeleopState`

Published by `fast_ik_node` on each pose update. Contains IK status and errors.

| Field                    | Type            | Description                                                                  |
|--------------------------|-----------------|------------------------------------------------------------------------------|
| header                   | std_msgs/Header | stamp, frame_id                                                              |
| left_arm_ok              | bool            | IK succeeded for the left arm                                               |
| right_arm_ok             | bool            | IK succeeded for the right arm                                              |
| left_arm_out_of_bounds   | bool            | Target out of reach; arm follows closest point (clamp_to_workspace)        |
| right_arm_out_of_bounds  | bool            | Same for right arm                                                          |
| errors                   | string[]        | Text messages: `"Left arm IK failed"`, `"Right arm IK failed"` on failures  |

---

## Known issues

See [TODO.md](TODO.md).
