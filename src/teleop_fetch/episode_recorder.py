"""
Dataset session lifecycle and .hbr finalization.
"""

import json
import os
import threading
import time
from typing import Any, Dict, List, Optional

import rospy

from teleop_fetch.hbr_writer import (
    default_metadata,
    write_camera_frames_jsonl,
    write_events_jsonl,
    write_lerobot_manifest,
    write_metadata,
    write_operator_state_bin,
    write_robot_state_bin,
)
from teleop_fetch.record_types import JointValue, OperatorFrame, PoseData, RobotFrame
from teleop_fetch.sensors.base_camera import BaseCamera
from teleop_fetch.sensors.base_imu import BaseIMU
from teleop_fetch.sensors.base_joint_sensor import BaseJointSensor


def _now_ros_unix_ns() -> int:
    rt = rospy.Time.now()
    if rt.to_nsec() > 0:
        return int(rt.to_nsec())
    return int(time.time_ns())


class EpisodeRecorder:
    def __init__(self, dataset_id: str, task_name: str, label: str, config: Dict[str, Any]):
        self.dataset_id = dataset_id
        self.task_name = task_name or ""
        self.label = label or ""
        self.config = config
        self.output_root = os.path.expanduser(config["output_root"])
        self.cache_root = os.path.expanduser(config["cache_root"])
        self._sample_hz = float(config.get("robot_frame_sample_hz", 100.0))
        self._active = False
        self._thread: Optional[threading.Thread] = None
        self._robot_frames: List[RobotFrame] = []
        self._operator_frames: List[OperatorFrame] = []
        self._events: List[Dict[str, Any]] = []
        self._lock = threading.Lock()
        self._started_local_ns = 0
        self._ended_local_ns = 0
        self._started_ros_ns = 0
        self._ended_ros_ns = 0
        self._ros_sync_start = False
        self._ros_sync_end = False
        self._source_send_hz = 0.0
        self._source_ws_url = ""
        self._camera: Optional[BaseCamera] = None
        self._imu: Optional[BaseIMU] = None
        self._joint_sensor: Optional[BaseJointSensor] = None
        self._hbr_dir = os.path.join(self.output_root, "%s.hbr" % self.dataset_id)
        self._session_cache_dir = os.path.join(self.cache_root, self.dataset_id)
        self._camera_frames_cache = []

    @property
    def hbr_dir(self) -> str:
        return self._hbr_dir

    def attach_camera(self, camera: BaseCamera) -> None:
        self._camera = camera

    def attach_imu(self, imu: BaseIMU) -> None:
        self._imu = imu

    def attach_joint_sensor(self, js: BaseJointSensor) -> None:
        self._joint_sensor = js

    def start(self, source_send_hz: float, source_ws_url: str, ros_time_synchronized: bool) -> None:
        with self._lock:
            if self._active:
                raise RuntimeError("Recorder already active")
            self._active = True
            self._robot_frames = []
            self._camera_frames_cache = []
            self._source_send_hz = float(source_send_hz)
            self._source_ws_url = source_ws_url
            self._started_local_ns = int(time.time_ns())
            self._started_ros_ns = _now_ros_unix_ns()
            self._ros_sync_start = bool(ros_time_synchronized)

        os.makedirs(self.output_root, exist_ok=True)
        os.makedirs(self.cache_root, exist_ok=True)
        os.makedirs(self._session_cache_dir, exist_ok=True)

        if self._camera:
            self._camera.start_recording(self.dataset_id)
        if self._imu:
            self._imu.start_recording(self.dataset_id)
        if self._joint_sensor:
            self._joint_sensor.start_recording(self.dataset_id)

        self._thread = threading.Thread(target=self._sampling_loop, name="episode_recorder_%s" % self.dataset_id, daemon=True)
        self._thread.start()

    def stop(self, ros_time_synchronized: bool) -> None:
        with self._lock:
            if not self._active:
                return
            self._active = False
            self._ended_local_ns = int(time.time_ns())
            self._ended_ros_ns = _now_ros_unix_ns()
            self._ros_sync_end = bool(ros_time_synchronized)

        if self._thread:
            self._thread.join(timeout=2.0)

        if self._camera:
            self._camera.stop_recording()
            self._camera_frames_cache = self._camera.drain_frames()
        if self._imu:
            self._imu.stop_recording()
        if self._joint_sensor:
            self._joint_sensor.stop_recording()

        self._write_robot_side()

    def _sampling_loop(self) -> None:
        period = 1.0 / self._sample_hz if self._sample_hz > 0 else 0.01
        while not rospy.is_shutdown():
            with self._lock:
                if not self._active:
                    break
            now_ns = int(time.time_ns())
            now_ros = _now_ros_unix_ns()
            imu_latest = self._imu.latest() if self._imu else None
            joints_latest = self._joint_sensor.latest() if self._joint_sensor else None
            frame = RobotFrame(
                local_unix_time_ns=now_ns,
                local_monotonic_sec=float(time.monotonic()),
                estimated_ros_unix_time_ns=now_ros,
                ros_clock_offset_sec=float((now_ros - now_ns) / 1e9),
                sync_rtt_sec=0.0,
                ros_time_synchronized=True,
                imu=imu_latest,
                joints=joints_latest,
                camera_frame_index=-1,
            )
            with self._lock:
                self._robot_frames.append(frame)
            time.sleep(period)

    def _write_robot_side(self) -> None:
        os.makedirs(self._hbr_dir, exist_ok=True)
        os.makedirs(os.path.join(self._hbr_dir, "robot"), exist_ok=True)
        os.makedirs(os.path.join(self._hbr_dir, "operator"), exist_ok=True)
        os.makedirs(os.path.join(self._hbr_dir, "video"), exist_ok=True)

        robot_path = os.path.join(self._hbr_dir, "robot", "robot_state.bin")
        cam_path = os.path.join(self._hbr_dir, "video", "cam_main_frames.jsonl")
        write_robot_state_bin(robot_path, self._robot_frames)
        write_camera_frames_jsonl(cam_path, self._camera_frames_cache)
        # Keep expected file present even when external transcoding is not configured.
        open(os.path.join(self._hbr_dir, "video", "cam_main.mp4"), "ab").close()

        metadata = default_metadata(
            dataset_id=self.dataset_id,
            task_name=self.task_name,
            label=self.label,
            source_send_hz=self._source_send_hz,
            source_ws_url=self._source_ws_url,
        )
        metadata["startedLocalUnixTimeNs"] = self._started_local_ns
        metadata["endedLocalUnixTimeNs"] = self._ended_local_ns
        metadata["startedEstimatedRosUnixTimeNs"] = self._started_ros_ns
        metadata["endedEstimatedRosUnixTimeNs"] = self._ended_ros_ns
        metadata["rosTimeWasSynchronizedAtStart"] = self._ros_sync_start
        metadata["rosTimeWasSynchronizedAtEnd"] = self._ros_sync_end
        metadata["durationSec"] = max(0.0, (self._ended_local_ns - self._started_local_ns) / 1e9)
        write_metadata(os.path.join(self._hbr_dir, "metadata.json"), metadata)
        write_events_jsonl(os.path.join(self._hbr_dir, "operator", "events.jsonl"), self._events)
        write_lerobot_manifest(self._hbr_dir, metadata, len(self._robot_frames), len(self._operator_frames))

    def attach_upload_record(
        self,
        record: Dict[str, Any],
        generated_utc_iso: str,
        source: str,
        accepted_at_utc_iso: str = "",
        teleop_control: Optional[Dict[str, Any]] = None,
    ) -> None:
        os.makedirs(self._hbr_dir, exist_ok=True)
        os.makedirs(os.path.join(self._hbr_dir, "robot"), exist_ok=True)
        os.makedirs(os.path.join(self._hbr_dir, "operator"), exist_ok=True)
        os.makedirs(os.path.join(self._hbr_dir, "video"), exist_ok=True)

        data = record["data"]
        frames = data.get("frames", [])
        operator_frames: List[OperatorFrame] = []
        events: List[Dict[str, Any]] = []
        for frame in frames:
            operator_frames.append(
                OperatorFrame(
                    local_unix_time_ns=int(frame.get("localUnixTimeNs", 0)),
                    local_monotonic_sec=float(frame.get("localMonotonicSec", 0.0)),
                    estimated_ros_unix_time_ns=int(frame.get("estimatedRosUnixTimeNs", 0)),
                    ros_clock_offset_sec=float(frame.get("rosClockOffsetSec", 0.0)),
                    sync_rtt_sec=float(frame.get("syncRttSec", 0.0)),
                    ros_time_synchronized=bool(frame.get("rosTimeSynchronized", False)),
                    input_mode=str(frame.get("inputMode", "none")),
                    head=PoseData(
                        position=dict(frame.get("head", {}).get("position", {"x": 0.0, "y": 0.0, "z": 0.0})),
                        orientation=dict(frame.get("head", {}).get("orientation", {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})),
                    ),
                    left=PoseData(
                        position=dict(frame.get("left", {}).get("position", {"x": 0.0, "y": 0.0, "z": 0.0})),
                        orientation=dict(frame.get("left", {}).get("orientation", {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})),
                    ),
                    right=PoseData(
                        position=dict(frame.get("right", {}).get("position", {"x": 0.0, "y": 0.0, "z": 0.0})),
                        orientation=dict(frame.get("right", {}).get("orientation", {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})),
                    ),
                    joints=[JointValue(name=str(j.get("name", "")), value=float(j.get("value", 0.0))) for j in frame.get("joints", [])],
                )
            )
        upload_ev: Dict[str, Any] = {
            "type": "upload_received",
            "timeUnixNs": int(time.time_ns()),
            "generatedUtcIso": generated_utc_iso,
            "source": source,
        }
        if str(accepted_at_utc_iso or "").strip():
            upload_ev["acceptedAtUtcIso"] = str(accepted_at_utc_iso).strip()
        if teleop_control is not None:
            upload_ev["teleopControl"] = teleop_control
        events.append(upload_ev)
        self._operator_frames = operator_frames
        self._events.extend(events)
        self.task_name = str(record.get("taskName", self.task_name))
        self.label = str(record.get("label", self.label))

        os.makedirs(os.path.join(self._hbr_dir, "operator"), exist_ok=True)
        operator_path = os.path.join(self._hbr_dir, "operator", "operator_state.bin")
        events_path = os.path.join(self._hbr_dir, "operator", "events.jsonl")
        write_operator_state_bin(operator_path, operator_frames)
        write_events_jsonl(events_path, self._events)

        metadata_path = os.path.join(self._hbr_dir, "metadata.json")
        if os.path.exists(metadata_path):
            with open(metadata_path, "r", encoding="utf-8") as f:
                metadata = json.load(f)
        else:
            metadata = default_metadata(self.dataset_id, self.task_name, self.label, self._source_send_hz, self._source_ws_url)
        metadata["taskName"] = self.task_name
        metadata["label"] = self.label
        metadata["source"] = source
        metadata["generatedUtcIso"] = generated_utc_iso
        if str(accepted_at_utc_iso or "").strip():
            metadata["acceptedAtUtcIso"] = str(accepted_at_utc_iso).strip()
        else:
            metadata.pop("acceptedAtUtcIso", None)
        if teleop_control is not None:
            metadata["teleopControl"] = teleop_control
        else:
            metadata.pop("teleopControl", None)
        metadata["sourceWsUrl"] = data.get("sourceWsUrl", metadata.get("sourceWsUrl", ""))
        metadata["sourceSendHz"] = float(data.get("sourceSendHz", metadata.get("sourceSendHz", 0.0)))
        metadata["startedLocalUnixTimeNs"] = int(data.get("startedLocalUnixTimeNs", metadata.get("startedLocalUnixTimeNs", 0)))
        metadata["endedLocalUnixTimeNs"] = int(data.get("endedLocalUnixTimeNs", metadata.get("endedLocalUnixTimeNs", 0)))
        metadata["startedEstimatedRosUnixTimeNs"] = int(
            data.get("startedEstimatedRosUnixTimeNs", metadata.get("startedEstimatedRosUnixTimeNs", 0))
        )
        metadata["endedEstimatedRosUnixTimeNs"] = int(
            data.get("endedEstimatedRosUnixTimeNs", metadata.get("endedEstimatedRosUnixTimeNs", 0))
        )
        metadata["rosTimeWasSynchronizedAtStart"] = bool(
            data.get("rosTimeWasSynchronizedAtStart", metadata.get("rosTimeWasSynchronizedAtStart", False))
        )
        metadata["rosTimeWasSynchronizedAtEnd"] = bool(
            data.get("rosTimeWasSynchronizedAtEnd", metadata.get("rosTimeWasSynchronizedAtEnd", False))
        )
        duration = max(0.0, (metadata["endedLocalUnixTimeNs"] - metadata["startedLocalUnixTimeNs"]) / 1e9)
        metadata["durationSec"] = duration
        write_metadata(metadata_path, metadata)
        write_lerobot_manifest(self._hbr_dir, metadata, len(self._robot_frames), len(operator_frames))


class DatasetSessionManager:
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self._active: Optional[EpisodeRecorder] = None
        self._sessions: Dict[str, EpisodeRecorder] = {}
        self._lock = threading.Lock()

    def start_session(
        self,
        dataset_id: str,
        task_name: str,
        label: str,
        source_send_hz: float,
        source_ws_url: str,
        ros_time_synchronized: bool,
        camera: Optional[BaseCamera],
        imu: Optional[BaseIMU],
        joint_sensor: Optional[BaseJointSensor],
    ) -> None:
        with self._lock:
            if self._active is not None:
                raise RuntimeError("another session is currently active: %s" % self._active.dataset_id)
            recorder = EpisodeRecorder(dataset_id, task_name, label, self.config)
            if camera:
                recorder.attach_camera(camera)
            if imu:
                recorder.attach_imu(imu)
            if joint_sensor:
                recorder.attach_joint_sensor(joint_sensor)
            self._sessions[dataset_id] = recorder
            self._active = recorder
        recorder.start(source_send_hz=source_send_hz, source_ws_url=source_ws_url, ros_time_synchronized=ros_time_synchronized)

    def stop_session(self, dataset_id: str, ros_time_synchronized: bool) -> None:
        with self._lock:
            if self._active is None:
                raise RuntimeError("no active session")
            if self._active.dataset_id != dataset_id:
                raise RuntimeError("active session id mismatch: active=%s requested=%s" % (self._active.dataset_id, dataset_id))
            recorder = self._active
            self._active = None
        recorder.stop(ros_time_synchronized=ros_time_synchronized)

    def attach_upload_payload(self, payload: Dict[str, Any]) -> List[str]:
        source = str(payload.get("source", "unity_quest_dataset"))
        generated_utc_iso = str(payload.get("generatedUtcIso", ""))
        accepted_at_utc_iso = str(payload.get("acceptedAtUtcIso", ""))
        teleop_control = payload.get("teleopControl")
        if teleop_control is not None and not isinstance(teleop_control, dict):
            teleop_control = None
        updated = []
        for rec in payload.get("records", []):
            record_id = str(rec.get("recordId", ""))
            if not record_id:
                continue
            with self._lock:
                recorder = self._sessions.get(record_id)
            if recorder is None:
                recorder = EpisodeRecorder(record_id, "", "", self.config)
                with self._lock:
                    self._sessions[record_id] = recorder
            recorder.attach_upload_record(
                rec,
                generated_utc_iso=generated_utc_iso,
                source=source,
                accepted_at_utc_iso=accepted_at_utc_iso,
                teleop_control=teleop_control,
            )
            updated.append(record_id)
        return updated

    def has_session(self, dataset_id: str) -> bool:
        with self._lock:
            return dataset_id in self._sessions

    def status_snapshot(self) -> Dict[str, Any]:
        with self._lock:
            active_id = self._active.dataset_id if self._active else None
            known = sorted(list(self._sessions.keys()))
        return {
            "activeDatasetId": active_id,
            "knownDatasetIds": known,
            "knownCount": len(known),
        }
