# DATA_NODE_SPEC.md

Implementation status (teleop_fetch side):

- `.hbr` directory generation is implemented in `episode_recorder.py`.
- Robot stream output is finalized on `stop` event from `/record_sessions`.
- Operator stream is attached after `POST /upload_dataset`.
- `lerobot_manifest/*` files are generated for downstream conversion.

Current v1 video representation keeps raw image frames in `video/cam_main_frames.jsonl` and creates `video/cam_main.mp4` placeholder for compatibility. A dedicated transcoding stage can be added in DATA_NODE.

## Overview

This document specifies the `.hbr` dataset format for robot teleoperation and the supporting services required to store, index, and export these datasets into native LeRobot / GR00T-compatible formats.

Key design decisions:

- One operator recording (one start/stop on the VR headset) = one `.hbr` dataset.
- Multiple `.hbr` datasets can share the same `taskName` and will later be merged into a larger LeRobot dataset.
- `.hbr` is a directory-based container with strict structure and metadata, so an agent can reliably:
    - ingest data from the robot and VR headset,
    - store it in S3-compatible storage,
    - export to LeRobot dataset format.

All interfaces, field names, comments, and UI strings are in English.

***

## 1. .hbr Directory Layout

Each recording (dataset) produced by an operator is stored as a directory named:

```text
<hbr_root> = <datasetId>.hbr/
```

Recommended naming:

```text
<datasetId>.hbr
  ├─ metadata.json
  ├─ robot/
  │   ├─ robot_state.bin
  │   ├─ imu.bin          # optional if not in robot_state
  │   └─ motors.bin       # optional if not in robot_state
  ├─ operator/
  │   ├─ operator_state.bin
  │   └─ events.jsonl
  ├─ video/
  │   ├─ cam_main.mp4
  │   └─ cam_aux_0.mp4    # optional additional cameras
  └─ lerobot_manifest/
      ├─ info.json        # LeRobot metadata for this single-episode dataset
      ├─ episodes.jsonl   # episode-level info for this dataset (one episode)
      ├─ tasks.jsonl      # optional: task-level aggregation
      └─ mapping.json     # mapping from .hbr fields to LeRobot feature keys
```


### 1.1 Core concepts

- **Dataset**: one `.hbr` directory, single operator recording.
- **Task**: semantic task, e.g. `"Approach target A"`. Many `.hbr` datasets may refer to the same task.
- **Episode**: in LeRobot sense, each `.hbr` is exactly one episode (`episode_index = 0` inside this `.hbr` dataset).

***

## 2. metadata.json

Top-level metadata describing this `.hbr` recording:

```json
{
  "hbrVersion": "1.0.0",
  "createdUtcIso": "2026-03-17T14:12:33.123456Z",
  "datasetId": "a4e6a0b6-3d1a-4b0f-9d5a-2f4b32b89c01",
  "source": "unity_quest_dataset",
  "taskName": "Approach point A",
  "label": "Approach to target object",
  "robotType": "ainex_v1",
  "robotId": "ainex-01",
  "operatorId": "operator-0001",
  "sourceWsUrl": "ws://192.168.1.100:9090",
  "sourceSendHz": 30.0,
  "startedLocalUnixTimeNs": 1710000000000000000,
  "endedLocalUnixTimeNs": 1710000005000000000,
  "startedEstimatedRosUnixTimeNs": 1710000000100000000,
  "endedEstimatedRosUnixTimeNs": 1710000005100000000,
  "rosTimeWasSynchronizedAtStart": true,
  "rosTimeWasSynchronizedAtEnd": true,
  "durationSec": 5.0,
  "headsetBuildVersion": "quest_build_00123",
  "robotSoftwareVersion": "teleop_stack_1.0.0",
  "notes": "Optional free-form notes for operator or QA",
  "video": {
    "mainCameraName": "cam_main",
    "frameRateHz": 30.0,
    "encoding": "h264",
    "hasAudio": false
  },
  "schema": {
    "robotStateFrameLayout": "v1",
    "operatorStateFrameLayout": "v1"
  }
}
```


***

## 3. Frame Time Header (common for all binary streams)

All binary frame streams (`robot_state.bin`, `operator_state.bin`, `imu.bin`, `motors.bin`) use a common header per frame:

```text
struct FrameTimeHeaderV1 {
  int64  localUnixTimeNs;        // host local wall-clock time in nanoseconds
  float64 localMonotonicSec;     // local monotonic clock in seconds since some epoch
  int64  estimatedRosUnixTimeNs; // ROS time mapped into Unix time, nanoseconds
  float32 rosClockOffsetSec;     // ros_time - local_time, seconds
  float32 syncRttSec;            // RTT of last time sync exchange, seconds
  uint8  rosTimeSynchronized;    // 0 = false, 1 = true
  uint8  _padding[7];            // reserved / alignment
}
```

Binary files are little-endian, packed structures, fixed record size per layout version.

***

## 4. robot/robot_state.bin

`robot_state.bin` is a sequence of fixed-size frames:

```text
[ FrameTimeHeaderV1 ][ RobotStatePayloadV1 ]
[ FrameTimeHeaderV1 ][ RobotStatePayloadV1 ]
...
```


### 4.1 RobotStatePayloadV1

Example layout (this can be tuned per robot but must be strictly documented):

```text
struct RobotStatePayloadV1 {
  // General robot state
  uint8  controlMode;         // e.g. 0=idle, 1=teleop, 2=autonomous
  uint8  estopEngaged;        // 0/1
  uint8  batteryPercentage;   // 0-100
  uint8  reserved0;

  float32 basePosX;           // base pose in some fixed frame
  float32 basePosY;
  float32 basePosZ;
  float32 baseRotX;
  float32 baseRotY;
  float32 baseRotZ;
  float32 baseRotW;

  // IMU summary (if not using separate imu.bin)
  float32 imuOrientationX;
  float32 imuOrientationY;
  float32 imuOrientationZ;
  float32 imuOrientationW;
  float32 imuAngVelX;
  float32 imuAngVelY;
  float32 imuAngVelZ;
  float32 imuLinAccX;
  float32 imuLinAccY;
  float32 imuLinAccZ;

  // Joints: N joints, fixed max N, unused = NaN
  float32 jointPosition[32];
  float32 jointVelocity[32];
  float32 jointEffort[32];

  // Optional per-joint temperatures / currents (if known)
  float32 jointTemperature[32];
  float32 jointCurrent[32];
}
```

The actual number of active joints is defined in a separate per-robot specification (`FORMAT-hbr.md`) and referenced by `robotType`.

***

## 5. operator/operator_state.bin

`operator_state.bin` is also a sequence:

```text
[ FrameTimeHeaderV1 ][ OperatorStatePayloadV1 ]
...
```


### 5.1 OperatorStatePayloadV1

```text
enum InputMode : uint8 {
  INPUT_MODE_CONTROLLERS = 0,
  INPUT_MODE_HAND_TRACKING = 1,
  INPUT_MODE_MIXED = 2
};

struct Vec3 {
  float32 x;
  float32 y;
  float32 z;
};

struct Quat {
  float32 x;
  float32 y;
  float32 z;
  float32 w;
};

struct ControllerStateV1 {
  Vec3    position;          // in VR tracking space
  Quat    orientation;
  float32 triggerValue;      // 0..1
  float32 gripValue;         // 0..1
  uint32  buttonMask;        // bitfield of digital buttons
};

struct HeadStateV1 {
  Vec3 position;
  Quat orientation;
};

struct OperatorStatePayloadV1 {
  uint8  inputMode;          // InputMode
  uint8  reserved0[3];

  HeadStateV1       head;
  ControllerStateV1 left;
  ControllerStateV1 right;

  // Optional finger joints etc.
  float32 joints[32];        // normalized 0..1 finger joints or other inputs
}
```


***

## 6. operator/events.jsonl

`events.jsonl` contains one JSON object per line. Typical events:

```json
{"type":"mark","timeUnixNs":1710000000123456789,"label":"start_approach"}
{"type":"mark","timeUnixNs":1710000000456789012,"label":"aligned_with_target"}
{"type":"task_result","timeUnixNs":1710000000500000000,"status":"success","details":"Object reached"}
{"type":"button","timeUnixNs":1710000000200000000,"source":"right","button":"A","value":1}
{"type":"safety","timeUnixNs":1710000000300000000,"code":"FALL_DETECTED","severity":"CRITICAL"}
```

Core fields:

- `type`: `"mark" | "task_result" | "button" | "safety" | ...`
- `timeUnixNs`: `int64`, same time base as `FrameTimeHeaderV1.localUnixTimeNs`.
- Additional fields depend on `type`, documented in `FORMAT-hbr.md`.

***

## 7. video/*

Video files contain the visual stream(s) from the robot camera(s). For v1 we standardize:

- `video/cam_main.mp4` – main forward camera.
- Additional cameras optional: `cam_aux_0.mp4`, `cam_aux_1.mp4`, etc.

Frame timing is reconstructed using:

- `metadata.json.video.frameRateHz`
- Alignment with `robot_state.bin` / `operator_state.bin` via timestamps and the known recording start time.

***

## 8. lerobot_manifest/

This subdirectory describes how to interpret `.hbr` as a LeRobot dataset.

### 8.1 info.json

Minimal example for a single-episode dataset:

```json
{
  "version": "2.1",
  "datasetId": "a4e6a0b6-3d1a-4b0f-9d5a-2f4b32b89c01",
  "source": "unity_quest_dataset",
  "taskName": "Approach point A",
  "label": "Approach to target object",
  "robotType": "ainex_v1",
  "numEpisodes": 1,
  "numFrames": 150,
  "fps": 30.0,
  "features": {
    "observation.images.cam_main": {
      "type": "video",
      "path": "video/cam_main.mp4"
    },
    "observation.state": {
      "type": "tensor",
      "shape": [ -1, 128 ],
      "description": "robot joint states, imu, base pose"
    },
    "observation.operator": {
      "type": "tensor",
      "shape": [ -1, 64 ],
      "description": "VR head and controllers state"
    },
    "action": {
      "type": "tensor",
      "shape": [ -1, 32 ],
      "description": "target joint commands (if available)"
    }
  }
}
```


### 8.2 episodes.jsonl

Per-episode metadata; here a single line, since each `.hbr` is one episode:

```json
{
  "episodeIndex": 0,
  "datasetId": "a4e6a0b6-3d1a-4b0f-9d5a-2f4b32b89c01",
  "taskName": "Approach point A",
  "label": "Approach to target object",
  "numFrames": 150,
  "startUnixTimeNs": 1710000000000000000,
  "endUnixTimeNs": 1710000005000000000
}
```


### 8.3 tasks.jsonl (optional)

Aggregated across multiple `.hbr` datasets; generated by DATA_NODE when merging multiple datasets of the same task. Example for single `.hbr`:

```json
{
  "taskName": "Approach point A",
  "numDatasets": 1,
  "totalEpisodes": 1,
  "totalFrames": 150
}
```


### 8.4 mapping.json

Describes mapping of `.hbr` binary layouts to LeRobot feature columns:

```json
{
  "version": "1.0.0",
  "robotType": "ainex_v1",
  "robotStateLayout": "RobotStatePayloadV1",
  "operatorStateLayout": "OperatorStatePayloadV1",
  "features": {
    "observation.images.cam_main": {
      "source": "video",
      "path": "video/cam_main.mp4"
    },
    "observation.state": {
      "source": "robot/robot_state.bin",
      "fields": [
        "basePosX", "basePosY", "basePosZ",
        "baseRotX", "baseRotY", "baseRotZ", "baseRotW",
        "imuOrientationX", "imuOrientationY", "imuOrientationZ", "imuOrientationW",
        "imuAngVelX", "imuAngVelY", "imuAngVelZ",
        "imuLinAccX", "imuLinAccY", "imuLinAccZ",
        "jointPosition[0..N-1]",
        "jointVelocity[0..N-1]",
        "jointEffort[0..N-1]"
      ]
    },
    "observation.operator": {
      "source": "operator/operator_state.bin",
      "fields": [
        "head.position", "head.orientation",
        "left.position", "left.orientation", "left.triggerValue", "left.gripValue",
        "right.position", "right.orientation", "right.triggerValue", "right.gripValue",
        "joints[0..M-1]"
      ]
    },
    "action": {
      "source": "robot/robot_state.bin",
      "fields": [
        "jointTargetPosition[0..N-1]"
      ],
      "optional": true
    }
  }
}
```


***

## 9. Robot-side Sensor Abstractions

To allow swapping robots without breaking `.hbr`, the robot-side code should use typed sensor classes.

### 9.1 BaseCamera

```python
class BaseCamera:
    name: str
    topic: str
    frame_rate_hz: float

    def start_recording(self, dataset_id: str) -> None:
        """Begin capturing frames for the given dataset."""

    def stop_recording(self) -> None:
        """Stop capturing and flush any buffers."""
```


### 9.2 BaseIMU

```python
class BaseIMU:
    name: str
    topic: str

    def start_recording(self, dataset_id: str) -> None:
        ...

    def stop_recording(self) -> None:
        ...
```

The IMU callback should populate fields matching the `RobotStatePayloadV1` or the separate `imu.bin` layout.

### 9.3 BaseMotor / JointSensor

```python
class BaseJointSensor:
    joint_names: list[str]

    def start_recording(self, dataset_id: str) -> None:
        ...

    def stop_recording(self) -> None:
        ...
```

Concrete implementations for each robot define:

- ROS topics,
- units conversion,
- mapping to `jointPosition[]`, `jointVelocity[]`, `jointEffort[]`, etc.


### 9.4 EpisodeRecorder (robot side)

Pseudo-interface:

```python
class EpisodeRecorder:
    def __init__(self, dataset_id: str, task_name: str, label: str, config: dict):
        ...

    def start(self) -> None:
        """Start capturing robot-side data into .hbr buffers."""

    def stop(self) -> None:
        """Stop capturing and finalize robot/*.bin and video/* files."""

    def attach_camera(self, camera: BaseCamera) -> None:
        ...

    def attach_imu(self, imu: BaseIMU) -> None:
        ...

    def attach_joint_sensor(self, js: BaseJointSensor) -> None:
        ...
```


***

## 10. Runtime Flow: VR → Robot → .hbr

### 10.1 VR control messages (ROS topic XXXX)

- Operator accepts a task in VR.
- VR sends a start message to topic `XXXX` with:
    - `flag = start`
    - `datasetId = <uuid>`
    - `taskName`, `label` (optional if only on HTTP payload).
- Robot:
    - instantiates `EpisodeRecorder(datasetId, taskName, label, ...)`,
    - starts all sensor recordings at maximum frequency.
- VR sends stop message with:
    - `flag = stop`
    - same `datasetId`.
- Robot:
    - stops `EpisodeRecorder`,
    - writes `.hbr` directory to local disk,
    - prepares to receive VR operator data via REST.

***

## 11. REST API on Robot: /upload_dataset (port 9191)

Simple HTTP endpoint to receive VR operator data for a finished recording.

### 11.1 Request

- Method: `POST`
- URL: `http://<robot-ip>:9191/upload_dataset`
- Content-Type: `application/json`

Body example:

```json
{
  "source": "unity_quest_dataset",
  "generatedUtcIso": "2026-03-17T14:12:33.1234567Z",
  "datasetId": "a4e6a0b6-3d1a-4b0f-9d5a-2f4b32b89c01",
  "records": [
    {
      "label": "Approach to object",
      "taskName": "Approach point A",
      "data": {
        "startedLocalUnixTimeNs": 1710000000000000000,
        "endedLocalUnixTimeNs": 1710000005000000000,
        "startedEstimatedRosUnixTimeNs": 1710000000100000000,
        "endedEstimatedRosUnixTimeNs": 1710000005100000000,
        "rosTimeWasSynchronizedAtStart": true,
        "rosTimeWasSynchronizedAtEnd": true,
        "sourceWsUrl": "ws://192.168.1.100:9090",
        "sourceSendHz": 10.0,
        "frames": [
          {
            "localUnixTimeNs": 1710000000123456789,
            "localMonotonicSec": 123.45,
            "estimatedRosUnixTimeNs": 1710000000223456789,
            "rosClockOffsetSec": 0.100,
            "syncRttSec": 0.012,
            "rosTimeSynchronized": true,
            "inputMode": "controllers",
            "head": { "...": "..." },
            "left": { "...": "..." },
            "right": { "...": "..." },
            "joints": [
              { "name": "L_grip", "value": 0.7 },
              { "name": "L_index", "value": 0.2 }
            ]
          }
        ]
      }
    }
  ]
}
```

Key fields:

- `datasetId`: must match the `.hbr` dataset already created by the robot-side recorder.
- `frames`: full operator timeline; on robot, this is encoded into `operator_state.bin` and `events.jsonl` using the layouts above.


### 11.2 Response

- `200 OK` on success:

```json
{
  "status": "ok",
  "datasetId": "a4e6a0b6-3d1a-4b0f-9d5a-2f4b32b89c01",
  "message": "Operator data stored for dataset."
}
```

- `404` if `datasetId` not found on robot.
- `400` if payload invalid.

***

## 12. DATA_NODE Service

DATA_NODE is a separate service in the robot local network responsible for:

- Storing `.hbr` directories in S3-compatible storage.
- Providing REST APIs for indexing and retrieval.
- Merging multiple `.hbr` datasets into LeRobot datasets.


### 12.1 Storage

- S3-compatible storage (e.g. MinIO).
- Bucket: `hbr-datasets/`.
- Object key pattern:

```text
hbr-datasets/<datasetId>.hbr/<files...>
```


### 12.2 DATA_NODE REST API (high-level)

Minimal endpoints:

- `POST /sessions`
    - Registers a new `.hbr` dataset in storage.
    - Body contains `datasetId`, `taskName`, `label`, optional metadata.
- `GET /sessions/{datasetId}`
    - Returns metadata.json contents + storage status.
- `GET /sessions/{datasetId}/download`
    - Returns archive (e.g. `.tar.gz`) of the `.hbr` directory.
- `POST /lerobot/export`
    - Body: list of `datasetIds` and target dataset name.
    - DATA_NODE merges multiple `.hbr` datasets into one LeRobot dataset and optionally uploads to external storage / Hugging Face.

Exact schemas and code skeletons should be implemented in a separate file (e.g. `data_node_api.md` or OpenAPI).

***

## 13. LeRobot / GR00T Usage

From LeRobot / GR00T point of view, DATA_NODE must:

- Expose a path or S3 location where merged LeRobot datasets live.
- Ensure that the schema coming from `.hbr` is consistent with LeRobot conventions:
    - `observation.images.*` for cameras.
    - `observation.state` for robot state.
    - `observation.operator` for VR operator state.
    - `action` (optional, depending on how you interpret teleop commands).
    - `episode_index`, `frame_index`, `timestamp`, `next.done`.

The `lerobot_manifest/mapping.json` is the single source of truth for how to decode `.hbr` binaries into a LeRobot-compatible table.

