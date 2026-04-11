"""
Validation helpers for record control and upload payloads.
"""

import json
from dataclasses import dataclass
from typing import Any, Dict, List, Optional


@dataclass
class RecordSessionEvent:
    record_id: str
    event_type: str
    app_session_id: str
    timestamp_unix_ns: int
    timestamp_ros_unix_ns: int
    ntp_time_synchronized: bool
    ros_time_synchronized: bool
    pose_topic: str
    joint_topic: str
    send_hz: float

    @classmethod
    def from_std_string(cls, raw: str) -> "RecordSessionEvent":
        payload = json.loads(raw)
        required = [
            "record_id",
            "event_type",
            "app_session_id",
            "timestamp_unix_ns",
            "timestamp_ros_unix_ns",
            "ntp_time_synchronized",
            "ros_time_synchronized",
            "pose_topic",
            "joint_topic",
            "send_hz",
        ]
        for key in required:
            if key not in payload:
                raise ValueError("Missing field in record_sessions payload: %s" % key)
        event_type = str(payload["event_type"]).strip().lower()
        if event_type not in ("start", "stop"):
            raise ValueError("event_type must be start|stop")
        return cls(
            record_id=str(payload["record_id"]),
            event_type=event_type,
            app_session_id=str(payload["app_session_id"]),
            timestamp_unix_ns=int(payload["timestamp_unix_ns"]),
            timestamp_ros_unix_ns=int(payload["timestamp_ros_unix_ns"]),
            ntp_time_synchronized=bool(payload["ntp_time_synchronized"]),
            ros_time_synchronized=bool(payload["ros_time_synchronized"]),
            pose_topic=str(payload["pose_topic"]),
            joint_topic=str(payload["joint_topic"]),
            send_hz=float(payload["send_hz"]),
        )


class UploadPayload:
    """
    Lightweight validator for upload_dataset payload.
    """

    def __init__(self, payload: Dict[str, Any]):
        self.payload = payload
        self.records = payload.get("records", [])
        self.source = payload.get("source", "")
        self.generated_utc_iso = payload.get("generatedUtcIso", "")
        self.accepted_at_utc_iso = payload.get("acceptedAtUtcIso", "")
        self.teleop_control: Optional[Dict[str, Any]] = None

    @classmethod
    def from_json_bytes(cls, raw: bytes, default_record_id: str = "") -> "UploadPayload":
        decoded = json.loads(raw.decode("utf-8"))
        model = cls(decoded)
        model.normalize(default_record_id=default_record_id)
        model.validate()
        return model

    @staticmethod
    def _first_present(obj: Dict[str, Any], keys: List[str], default=None):
        for key in keys:
            if key in obj and obj[key] is not None:
                return obj[key]
        return default

    def normalize(self, default_record_id: str = "") -> None:
        if not isinstance(self.payload, dict):
            return
        self.payload["source"] = str(self._first_present(self.payload, ["source"], "unity_quest_dataset"))
        self.payload["generatedUtcIso"] = str(
            self._first_present(self.payload, ["generatedUtcIso", "generated_utc_iso"], "")
        )
        self.payload["acceptedAtUtcIso"] = str(
            self._first_present(self.payload, ["acceptedAtUtcIso", "accepted_at_utc_iso"], "")
        )
        self.payload.pop("accepted_at_utc_iso", None)
        self._normalize_teleop_control_field()
        records = self.payload.get("records")
        if not isinstance(records, list):
            self.records = []
            self.payload["records"] = self.records
            return

        normalized_records: List[Dict[str, Any]] = []
        top_dataset_id = self._first_present(
            self.payload,
            ["recordId", "record_id", "datasetId", "dataset_id", "id"],
            "",
        )
        if not top_dataset_id and default_record_id:
            top_dataset_id = default_record_id
        for rec in records:
            if not isinstance(rec, dict):
                continue
            data = rec.get("data")
            if not isinstance(data, dict):
                # Some clients may send session fields directly in record item.
                data = dict(rec)
            record_id = self._first_present(
                rec,
                ["recordId", "record_id", "datasetId", "dataset_id", "id"],
                default=None,
            )
            if record_id is None:
                record_id = self._first_present(data, ["recordId", "record_id", "datasetId", "dataset_id", "id"], default=None)
            if record_id is None and top_dataset_id:
                record_id = top_dataset_id
            label = self._first_present(rec, ["label", "recordLabel", "record_label"], default="")
            task_name = self._first_present(rec, ["taskName", "task_name"], default="")
            data["recordId"] = str(
                self._first_present(data, ["recordId", "record_id", "datasetId", "dataset_id"], default=record_id or "")
            )
            normalized_records.append(
                {
                    "recordId": str(record_id) if record_id is not None else "",
                    "label": str(label),
                    "taskName": str(task_name),
                    "data": data,
                }
            )
        self.records = normalized_records
        self.payload["records"] = normalized_records
        self.source = self.payload["source"]
        self.generated_utc_iso = self.payload["generatedUtcIso"]
        self.accepted_at_utc_iso = self.payload.get("acceptedAtUtcIso", "")
        self.teleop_control = self.payload.get("teleopControl")

    def _normalize_teleop_control_field(self) -> None:
        raw = self._first_present(self.payload, ["teleopControl", "teleop_control"], None)
        self.payload.pop("teleop_control", None)
        if raw is None:
            self.payload.pop("teleopControl", None)
            return
        if not isinstance(raw, dict):
            self.payload["teleopControl"] = raw
            return
        out = dict(raw)
        events_raw = out.get("events")
        if events_raw is None:
            out["events"] = []
        elif isinstance(events_raw, list):
            norm_events: List[Dict[str, str]] = []
            for ev in events_raw:
                if not isinstance(ev, dict):
                    norm_events.append({"eventType": "", "timestampUtcIso": ""})
                    continue
                norm_events.append(
                    {
                        "eventType": str(
                            self._first_present(ev, ["eventType", "event_type"], "")
                        ),
                        "timestampUtcIso": str(
                            self._first_present(ev, ["timestampUtcIso", "timestamp_utc_iso"], "")
                        ),
                    }
                )
            out["events"] = norm_events
        self.payload["teleopControl"] = out

    def validate(self) -> None:
        if not isinstance(self.payload, dict):
            raise ValueError("payload must be a JSON object")
        if "records" not in self.payload or not isinstance(self.records, list):
            raise ValueError("records must be present and must be an array")
        if not self.records:
            raise ValueError("records array cannot be empty")
        for idx, rec in enumerate(self.records):
            if not isinstance(rec, dict):
                raise ValueError("records[%d] must be an object" % idx)
            for key in ("recordId", "label", "taskName", "data"):
                if key not in rec:
                    raise ValueError("records[%d].%s is required" % (idx, key))
            if not str(rec.get("recordId", "")).strip():
                raise ValueError("records[%d].recordId is required" % idx)
            data = rec["data"]
            if not isinstance(data, dict):
                raise ValueError("records[%d].data must be an object" % idx)
            if "frames" not in data or not isinstance(data["frames"], list):
                raise ValueError("records[%d].data.frames must be an array" % idx)
        if "teleopControl" in self.payload:
            tc = self.payload["teleopControl"]
            if not isinstance(tc, dict):
                raise ValueError("teleopControl must be an object")
            evs = tc.get("events")
            if evs is not None and not isinstance(evs, list):
                raise ValueError("teleopControl.events must be an array")
            if isinstance(evs, list):
                for i, ev in enumerate(evs):
                    if not isinstance(ev, dict):
                        raise ValueError("teleopControl.events[%d] must be an object" % i)
                    et = str(ev.get("eventType", "")).strip()
                    ts = str(ev.get("timestampUtcIso", "")).strip()
                    if not et:
                        raise ValueError("teleopControl.events[%d].eventType is required" % i)
                    if not ts:
                        raise ValueError("teleopControl.events[%d].timestampUtcIso is required" % i)

    def record_ids(self) -> List[str]:
        ids = []
        for rec in self.records:
            ids.append(str(rec["recordId"]))
        return ids

    def get_record(self, record_id: str) -> Optional[Dict[str, Any]]:
        for rec in self.records:
            if str(rec.get("recordId")) == record_id:
                return rec
        return None
