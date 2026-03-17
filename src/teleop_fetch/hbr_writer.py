"""
HBR dataset writer for robot/operator session data.
"""

import json
import math
import os
import struct
from typing import Any, Dict, List, Optional

from teleop_fetch.record_types import CameraFrame, OperatorFrame, RobotFrame

MAX_JOINTS = 32
INPUT_MODE_MAP = {
    "controllers": 0,
    "hands": 1,
    "mixed": 2,
    "none": 2,
}


def _pad_float_array(values: List[float], length: int) -> List[float]:
    arr = list(values[:length])
    while len(arr) < length:
        arr.append(float("nan"))
    return arr


def _frame_header_bytes(frame_time: Dict[str, Any]) -> bytes:
    return struct.pack(
        "<q d q f f B 7x",
        int(frame_time["local_unix_time_ns"]),
        float(frame_time["local_monotonic_sec"]),
        int(frame_time["estimated_ros_unix_time_ns"]),
        float(frame_time["ros_clock_offset_sec"]),
        float(frame_time["sync_rtt_sec"]),
        1 if bool(frame_time["ros_time_synchronized"]) else 0,
    )


def _safe_imu_vec(imu: Optional[Dict[str, Any]], key: str) -> List[float]:
    if not imu:
        return [float("nan"), float("nan"), float("nan")]
    vec = imu.get(key, {})
    return [float(vec.get("x", float("nan"))), float(vec.get("y", float("nan"))), float(vec.get("z", float("nan")))]


def write_robot_state_bin(path: str, robot_frames: List[RobotFrame]) -> int:
    count = 0
    with open(path, "wb") as f:
        for frame in robot_frames:
            header = _frame_header_bytes(
                {
                    "local_unix_time_ns": frame.local_unix_time_ns,
                    "local_monotonic_sec": frame.local_monotonic_sec,
                    "estimated_ros_unix_time_ns": frame.estimated_ros_unix_time_ns,
                    "ros_clock_offset_sec": frame.ros_clock_offset_sec,
                    "sync_rtt_sec": frame.sync_rtt_sec,
                    "ros_time_synchronized": frame.ros_time_synchronized,
                }
            )
            imu = frame.imu
            joints = frame.joints
            imu_orientation = [
                float(imu.orientation["x"]) if imu else float("nan"),
                float(imu.orientation["y"]) if imu else float("nan"),
                float(imu.orientation["z"]) if imu else float("nan"),
                float(imu.orientation["w"]) if imu else float("nan"),
            ]
            imu_ang = _safe_imu_vec(imu.__dict__ if imu else None, "angular_velocity")
            imu_lin = _safe_imu_vec(imu.__dict__ if imu else None, "linear_acceleration")

            joint_positions = _pad_float_array(joints.positions if joints else [], MAX_JOINTS)
            joint_velocities = _pad_float_array(joints.velocities if joints else [], MAX_JOINTS)
            joint_efforts = _pad_float_array(joints.efforts if joints else [], MAX_JOINTS)
            joint_temp = _pad_float_array([], MAX_JOINTS)
            joint_current = _pad_float_array([], MAX_JOINTS)

            payload = struct.pack(
                "<4B"  # mode/estop/battery/reserved
                "7f"   # base pose position + quaternion
                "10f"  # imu orientation + ang vel + lin acc
                + ("32f" * 5),  # joints, vel, effort, temp, current
                1,  # controlMode teleop
                0,  # estop
                100,  # battery%
                0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                imu_orientation[0], imu_orientation[1], imu_orientation[2], imu_orientation[3],
                imu_ang[0], imu_ang[1], imu_ang[2],
                imu_lin[0], imu_lin[1], imu_lin[2],
                *(joint_positions + joint_velocities + joint_efforts + joint_temp + joint_current),
            )
            f.write(header)
            f.write(payload)
            count += 1
    return count


def write_operator_state_bin(path: str, operator_frames: List[OperatorFrame]) -> int:
    count = 0
    with open(path, "wb") as f:
        for frame in operator_frames:
            header = _frame_header_bytes(
                {
                    "local_unix_time_ns": frame.local_unix_time_ns,
                    "local_monotonic_sec": frame.local_monotonic_sec,
                    "estimated_ros_unix_time_ns": frame.estimated_ros_unix_time_ns,
                    "ros_clock_offset_sec": frame.ros_clock_offset_sec,
                    "sync_rtt_sec": frame.sync_rtt_sec,
                    "ros_time_synchronized": frame.ros_time_synchronized,
                }
            )
            left_mask = 0
            right_mask = 0
            joints = _pad_float_array([float(j.value) for j in frame.joints], MAX_JOINTS)
            payload = struct.pack(
                "<B 3x"
                "3f 4f"  # head
                "3f 4f f f I"  # left controller
                "3f 4f f f I"  # right controller
                "32f",
                INPUT_MODE_MAP.get(frame.input_mode, 2),
                float(frame.head.position["x"]), float(frame.head.position["y"]), float(frame.head.position["z"]),
                float(frame.head.orientation["x"]), float(frame.head.orientation["y"]), float(frame.head.orientation["z"]), float(frame.head.orientation["w"]),
                float(frame.left.position["x"]), float(frame.left.position["y"]), float(frame.left.position["z"]),
                float(frame.left.orientation["x"]), float(frame.left.orientation["y"]), float(frame.left.orientation["z"]), float(frame.left.orientation["w"]),
                0.0, 0.0, left_mask,
                float(frame.right.position["x"]), float(frame.right.position["y"]), float(frame.right.position["z"]),
                float(frame.right.orientation["x"]), float(frame.right.orientation["y"]), float(frame.right.orientation["z"]), float(frame.right.orientation["w"]),
                0.0, 0.0, right_mask,
                *joints,
            )
            f.write(header)
            f.write(payload)
            count += 1
    return count


def write_camera_frames_jsonl(path: str, camera_frames: List[CameraFrame]) -> int:
    count = 0
    with open(path, "w", encoding="utf-8") as f:
        for frame in camera_frames:
            f.write(
                json.dumps(
                    {
                        "localUnixTimeNs": frame.local_unix_time_ns,
                        "width": frame.width,
                        "height": frame.height,
                        "encoding": frame.encoding,
                        "step": frame.step,
                        "frameId": frame.frame_id,
                        "dataBase64": frame.data_b64,
                    }
                )
                + "\n"
            )
            count += 1
    return count


def write_events_jsonl(path: str, events: List[Dict[str, Any]]) -> int:
    with open(path, "w", encoding="utf-8") as f:
        for event in events:
            f.write(json.dumps(event) + "\n")
    return len(events)


def write_metadata(path: str, metadata: Dict[str, Any]) -> None:
    with open(path, "w", encoding="utf-8") as f:
        json.dump(metadata, f, indent=2, ensure_ascii=True)
        f.write("\n")


def write_lerobot_manifest(base_dir: str, metadata: Dict[str, Any], num_robot_frames: int, num_operator_frames: int) -> None:
    manifest_dir = os.path.join(base_dir, "lerobot_manifest")
    os.makedirs(manifest_dir, exist_ok=True)
    fps = float(metadata.get("sourceSendHz", 0.0)) if metadata.get("sourceSendHz") else 0.0
    info = {
        "version": "2.1",
        "datasetId": metadata.get("datasetId"),
        "source": metadata.get("source", "unity_quest_dataset"),
        "taskName": metadata.get("taskName", ""),
        "label": metadata.get("label", ""),
        "robotType": metadata.get("robotType", "ainex_v1"),
        "numEpisodes": 1,
        "numFrames": int(max(num_robot_frames, num_operator_frames)),
        "fps": fps,
        "features": {
            "observation.images.cam_main": {"type": "raw_frames_jsonl", "path": "video/cam_main_frames.jsonl"},
            "observation.state": {"type": "binary", "path": "robot/robot_state.bin"},
            "observation.operator": {"type": "binary", "path": "operator/operator_state.bin"},
        },
    }
    episodes = {
        "episodeIndex": 0,
        "datasetId": metadata.get("datasetId"),
        "taskName": metadata.get("taskName", ""),
        "label": metadata.get("label", ""),
        "numFrames": int(max(num_robot_frames, num_operator_frames)),
        "startUnixTimeNs": int(metadata.get("startedLocalUnixTimeNs", 0)),
        "endUnixTimeNs": int(metadata.get("endedLocalUnixTimeNs", 0)),
    }
    mapping = {
        "version": "1.0.0",
        "robotType": metadata.get("robotType", "ainex_v1"),
        "robotStateLayout": "RobotStatePayloadV1",
        "operatorStateLayout": "OperatorStatePayloadV1",
        "features": info["features"],
    }
    write_metadata(os.path.join(manifest_dir, "info.json"), info)
    with open(os.path.join(manifest_dir, "episodes.jsonl"), "w", encoding="utf-8") as f:
        f.write(json.dumps(episodes) + "\n")
    with open(os.path.join(manifest_dir, "tasks.jsonl"), "w", encoding="utf-8") as f:
        f.write(
            json.dumps(
                {
                    "taskName": metadata.get("taskName", ""),
                    "numDatasets": 1,
                    "totalEpisodes": 1,
                    "totalFrames": int(max(num_robot_frames, num_operator_frames)),
                }
            )
            + "\n"
        )
    write_metadata(os.path.join(manifest_dir, "mapping.json"), mapping)


def default_metadata(dataset_id: str, task_name: str, label: str, source_send_hz: float, source_ws_url: str) -> Dict[str, Any]:
    return {
        "hbrVersion": "1.0.0",
        "datasetId": dataset_id,
        "source": "unity_quest_dataset",
        "taskName": task_name,
        "label": label,
        "robotType": "ainex_v1",
        "sourceWsUrl": source_ws_url,
        "sourceSendHz": source_send_hz,
        "startedLocalUnixTimeNs": 0,
        "endedLocalUnixTimeNs": 0,
        "startedEstimatedRosUnixTimeNs": 0,
        "endedEstimatedRosUnixTimeNs": 0,
        "rosTimeWasSynchronizedAtStart": False,
        "rosTimeWasSynchronizedAtEnd": False,
        "durationSec": 0.0,
        "video": {
            "mainCameraName": "cam_main",
            "frameRateHz": source_send_hz if source_send_hz > 0 else 0.0,
            "encoding": "raw_ros_image",
            "hasAudio": False,
        },
        "schema": {
            "robotStateFrameLayout": "v1",
            "operatorStateFrameLayout": "v1",
        },
    }
