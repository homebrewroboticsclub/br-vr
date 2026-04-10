#!/usr/bin/env python3
"""
Robot-side dataset recorder node.

Responsibilities:
- Receive start/stop events from /record_sessions.
- Capture robot sensor streams at runtime.
- Finalize .hbr structure for each dataset.
- Consume upload payloads dropped into inbox by dataset_upload_server.
"""

import json
import os
import time
from urllib import request as urlrequest
from typing import Any, Dict

import rospy
from std_msgs.msg import String

try:
    from teleop_fetch.srv import SetPeaqDatasetClaim
except ImportError:
    SetPeaqDatasetClaim = None  # type: ignore

from teleop_fetch.episode_recorder import DatasetSessionManager
from teleop_fetch.sensors.ros_camera import ROSCamera
from teleop_fetch.sensors.ros_imu import ROSIMU
from teleop_fetch.sensors.ros_joint_sensor import ROSJointSensor
from teleop_fetch.upload_models import RecordSessionEvent

_ACTIVE_DATASET_CORR_PARAM = "/dataset_recorder/active_dataset_id"


def _set_active_dataset_correlation_param(dataset_id: str) -> None:
    rospy.set_param(_ACTIVE_DATASET_CORR_PARAM, dataset_id)


def _clear_active_dataset_correlation_param() -> None:
    try:
        rospy.delete_param(_ACTIVE_DATASET_CORR_PARAM)
    except KeyError:
        pass


def _load_recorder_config() -> Dict[str, Any]:
    def p(name, default):
        return rospy.get_param("~" + name, default)

    output_root = os.path.expanduser(p("output_root", "~/.teleop_fetch_datasets/hbr"))
    cache_root = os.path.expanduser(p("cache_root", "~/.teleop_fetch_datasets/cache"))
    upload_inbox = os.path.expanduser(p("upload_inbox_dir", "~/.teleop_fetch_datasets/upload_inbox"))
    cfg = {
        "record_sessions_topic": p("record_sessions_topic", "/record_sessions"),
        "robot_frame_sample_hz": float(p("robot_frame_sample_hz", 120.0)),
        "output_root": output_root,
        "cache_root": cache_root,
        "upload_inbox_dir": upload_inbox,
        "source_ws_url": p("source_ws_url", "ws://localhost:9090"),
        "logs_dir": os.path.expanduser(p("logs_dir", "~/.teleop_fetch_datasets/logs")),
        "camera": {
            "enabled": bool(p("camera/enabled", True)),
            "name": p("camera/name", "cam_main"),
            "topic": p("camera/topic", "/camera/image_raw"),
            "frame_rate_hz": float(p("camera/frame_rate_hz", 30.0)),
            "queue_size": int(p("camera/queue_size", 200)),
        },
        "imu": {
            "enabled": bool(p("imu/enabled", True)),
            "name": p("imu/name", "imu_main"),
            "topic": p("imu/topic", "/imu"),
            "queue_size": int(p("imu/queue_size", 500)),
        },
        "joints": {
            "enabled": bool(p("joints/enabled", True)),
            "topic": p("joints/topic", "/joint_states"),
            "queue_size": int(p("joints/queue_size", 500)),
        },
        "auto_push": {
            "enabled": bool(p("auto_push/enabled", True)),
            "data_node_url": p("auto_push/data_node_url", "http://127.0.0.1:8088"),
            "upload_path": p("auto_push/upload_path", "/sessions/upload"),
            "retries": int(p("auto_push/retries", 3)),
            "retry_delay_sec": float(p("auto_push/retry_delay_sec", 3.0)),
        },
    }
    os.makedirs(cfg["output_root"], exist_ok=True)
    os.makedirs(cfg["cache_root"], exist_ok=True)
    os.makedirs(cfg["upload_inbox_dir"], exist_ok=True)
    os.makedirs(cfg["logs_dir"], exist_ok=True)
    return cfg


class DatasetRecorderNode:
    def __init__(self):
        rospy.init_node("dataset_recorder", anonymous=False)
        self.config = _load_recorder_config()
        self.manager = DatasetSessionManager(self.config)
        self._log_file = os.path.join(self.config["logs_dir"], "dataset_recorder.log")
        self._state_file = os.path.join(self.config["cache_root"], "session_state.json")

        self.camera = None
        self.imu = None
        self.joint_sensor = None
        if self.config["camera"]["enabled"]:
            self.camera = ROSCamera(
                name=self.config["camera"]["name"],
                topic=self.config["camera"]["topic"],
                frame_rate_hz=self.config["camera"]["frame_rate_hz"],
                queue_size=self.config["camera"]["queue_size"],
            )
        if self.config["imu"]["enabled"]:
            self.imu = ROSIMU(
                name=self.config["imu"]["name"],
                topic=self.config["imu"]["topic"],
                queue_size=self.config["imu"]["queue_size"],
            )
        if self.config["joints"]["enabled"]:
            self.joint_sensor = ROSJointSensor(
                topic=self.config["joints"]["topic"],
                queue_size=self.config["joints"]["queue_size"],
            )

        self._sub = rospy.Subscriber(
            self.config["record_sessions_topic"],
            String,
            self._record_sessions_cb,
            queue_size=100,
        )
        self._poll_timer = rospy.Timer(rospy.Duration(1.0), self._poll_upload_inbox)
        if SetPeaqDatasetClaim is not None:
            rospy.Service(
                "/teleop_fetch/set_peaq_dataset_claim",
                SetPeaqDatasetClaim,
                self._handle_set_peaq_dataset_claim,
            )
        else:
            rospy.logwarn(
                "SetPeaqDatasetClaim srv missing (rebuild teleop_fetch); "
                "peaq dataset merge disabled."
            )
        rospy.loginfo("dataset_recorder ready: topic=%s", self.config["record_sessions_topic"])
        self._append_log("node_started", {"topic": self.config["record_sessions_topic"]})
        _clear_active_dataset_correlation_param()
        self._write_state_file()

    def _append_log(self, event: str, payload: Dict[str, Any]) -> None:
        row = {
            "timeUnixNs": int(time.time_ns()),
            "event": event,
            "payload": payload,
        }
        try:
            with open(self._log_file, "a", encoding="utf-8") as f:
                f.write(json.dumps(row, ensure_ascii=True) + "\n")
        except OSError:
            pass

    def _write_state_file(self) -> None:
        state = self.manager.status_snapshot()
        state["timeUnixNs"] = int(time.time_ns())
        try:
            tmp = self._state_file + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(state, f, ensure_ascii=True)
            os.replace(tmp, self._state_file)
        except OSError:
            pass

    def _active_dataset_id_from_state_file(self) -> str:
        if not os.path.exists(self._state_file):
            return ""
        try:
            with open(self._state_file, "r", encoding="utf-8") as f:
                state = json.load(f)
            active = str(state.get("activeDatasetId") or "").strip()
            if active:
                return active
            known = state.get("knownDatasetIds") or []
            if isinstance(known, list) and len(known) == 1:
                return str(known[0])
        except (OSError, json.JSONDecodeError, TypeError):
            return ""
        return ""

    def _handle_set_peaq_dataset_claim(self, req):
        """Merge peaq claim into metadata.json under key peaqClaim (DOC/DATA_NODE_PEAQ_CLAIM_SPEC)."""
        dataset_id = (req.dataset_id or "").strip()
        if not dataset_id:
            dataset_id = self._active_dataset_id_from_state_file()
        if not dataset_id:
            return {
                "success": False,
                "message": "dataset_id empty and no active dataset in session_state",
            }
        raw = (req.claim_json or "").strip()
        if not raw:
            return {"success": False, "message": "claim_json is empty"}
        try:
            claim_obj = json.loads(raw)
            if not isinstance(claim_obj, dict):
                return {
                    "success": False,
                    "message": "claim_json must be a JSON object",
                }
        except json.JSONDecodeError as e:
            return {"success": False, "message": "invalid JSON: %s" % e}

        root = os.path.join(self.config["output_root"], "%s.hbr" % dataset_id)
        meta_path = os.path.join(root, "metadata.json")
        if not os.path.isdir(root):
            return {
                "success": False,
                "message": "dataset directory not found: %s" % dataset_id,
            }
        meta = {}
        if os.path.isfile(meta_path):
            try:
                with open(meta_path, "r", encoding="utf-8") as f:
                    meta = json.load(f)
            except (OSError, json.JSONDecodeError):
                meta = {}
        meta["peaqClaim"] = claim_obj
        try:
            tmp = meta_path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(meta, f, indent=2, ensure_ascii=True)
                f.write("\n")
            os.replace(tmp, meta_path)
        except OSError as e:
            return {"success": False, "message": str(e)}
        self._append_log("peaq_claim_merged", {"datasetId": dataset_id})
        return {"success": True, "message": "ok"}

    def _auto_push_dataset(self, dataset_id: str) -> None:
        cfg = self.config.get("auto_push", {})
        if not bool(cfg.get("enabled", False)):
            return
        payload = {
            "datasetId": dataset_id,
            "dataNodeUrl": str(cfg.get("data_node_url", "http://127.0.0.1:8088")),
            "uploadPath": str(cfg.get("upload_path", "/sessions/upload")),
        }
        retries = max(1, int(cfg.get("retries", 3)))
        delay = max(0.1, float(cfg.get("retry_delay_sec", 3.0)))
        url = "http://127.0.0.1:9191/dataset_push"
        last_error = None
        for attempt in range(1, retries + 1):
            try:
                req = urlrequest.Request(
                    url=url,
                    method="POST",
                    data=json.dumps(payload).encode("utf-8"),
                    headers={"Content-Type": "application/json"},
                )
                with urlrequest.urlopen(req, timeout=120) as resp:
                    body = resp.read().decode("utf-8", errors="replace")
                    code = int(resp.getcode())
                self._append_log("auto_push_ok", {"datasetId": dataset_id, "attempt": attempt, "statusCode": code, "response": body[:500]})
                return
            except Exception as exc:
                last_error = str(exc)
                self._append_log("auto_push_retry", {"datasetId": dataset_id, "attempt": attempt, "error": last_error})
                if attempt < retries:
                    time.sleep(delay)
        self._append_log("auto_push_failed", {"datasetId": dataset_id, "error": last_error or "unknown"})

    def _record_sessions_cb(self, msg: String) -> None:
        try:
            event = RecordSessionEvent.from_std_string(msg.data)
        except Exception as exc:
            rospy.logerr("Invalid /record_sessions payload: %s", exc)
            self._append_log("record_event_invalid", {"error": str(exc)})
            return

        try:
            if event.event_type == "start":
                self.manager.start_session(
                    dataset_id=event.record_id,
                    task_name="",
                    label="",
                    source_send_hz=event.send_hz,
                    source_ws_url=self.config["source_ws_url"],
                    ros_time_synchronized=event.ros_time_synchronized,
                    camera=self.camera,
                    imu=self.imu,
                    joint_sensor=self.joint_sensor,
                )
                _set_active_dataset_correlation_param(event.record_id)
                rospy.loginfo("Dataset recording started: %s", event.record_id)
                self._append_log("record_start", {"recordId": event.record_id, "sendHz": event.send_hz})
            elif event.event_type == "stop":
                self.manager.stop_session(
                    dataset_id=event.record_id,
                    ros_time_synchronized=event.ros_time_synchronized,
                )
                _clear_active_dataset_correlation_param()
                rospy.loginfo("Dataset recording stopped: %s", event.record_id)
                self._append_log("record_stop", {"recordId": event.record_id})
            self._write_state_file()
        except Exception as exc:
            rospy.logerr("Dataset session event failed for %s: %s", event.record_id, exc)
            self._append_log("record_event_failed", {"recordId": event.record_id, "error": str(exc)})

    def _poll_upload_inbox(self, _evt) -> None:
        try:
            files = [f for f in os.listdir(self.config["upload_inbox_dir"]) if f.endswith(".json")]
        except OSError:
            return
        for name in files:
            full = os.path.join(self.config["upload_inbox_dir"], name)
            try:
                with open(full, "r", encoding="utf-8") as f:
                    payload = json.load(f)
                updated = self.manager.attach_upload_payload(payload)
                rospy.loginfo("Upload payload applied from %s, sessions=%s", name, ",".join(updated) if updated else "none")
                self._append_log("upload_applied", {"file": name, "updatedRecordIds": updated})
                self._write_state_file()
                for dataset_id in updated:
                    self._auto_push_dataset(dataset_id)
            except Exception as exc:
                rospy.logerr("Failed applying upload payload %s: %s", name, exc)
                self._append_log("upload_apply_failed", {"file": name, "error": str(exc)})
            finally:
                try:
                    os.remove(full)
                except OSError:
                    pass

    def run(self):
        rospy.spin()


def main():
    try:
        DatasetRecorderNode().run()
    except rospy.ROSInterruptException:
        rospy.loginfo("dataset_recorder stopped")


if __name__ == "__main__":
    main()
