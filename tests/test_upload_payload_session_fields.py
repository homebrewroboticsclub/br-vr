# UploadPayload: acceptedAtUtcIso and teleopControl (upload_dataset).

import json

from teleop_fetch.upload_models import UploadPayload


def _minimal_record(record_id="rec1"):
    return {
        "recordId": record_id,
        "label": "lbl",
        "taskName": "task",
        "data": {"frames": []},
    }


def test_upload_payload_accepts_session_extension():
    raw = json.dumps(
        {
            "source": "unity_quest_dataset",
            "generatedUtcIso": "2026-03-29T18:10:00.000Z",
            "acceptedAtUtcIso": "2026-03-29T18:00:00.000Z",
            "teleopControl": {
                "events": [
                    {"eventType": "get_control", "timestampUtcIso": "2026-03-29T18:00:10.000Z"},
                    {"eventType": "lost_control", "timestampUtcIso": "2026-03-29T18:00:25.000Z"},
                ]
            },
            "records": [_minimal_record()],
        }
    ).encode("utf-8")
    model = UploadPayload.from_json_bytes(raw)
    assert model.payload["acceptedAtUtcIso"] == "2026-03-29T18:00:00.000Z"
    assert model.payload["teleopControl"]["events"][0]["eventType"] == "get_control"
    assert model.payload["teleopControl"]["events"][1]["eventType"] == "lost_control"


def test_upload_payload_snake_case_aliases_normalized():
    raw = json.dumps(
        {
            "source": "unity_quest_dataset",
            "generatedUtcIso": "2026-03-29T12:00:00.000Z",
            "accepted_at_utc_iso": "2026-03-29T11:00:00.000Z",
            "teleop_control": {
                "events": [
                    {"event_type": "get_control", "timestamp_utc_iso": "2026-03-29T11:05:00.000Z"},
                ]
            },
            "records": [_minimal_record("rid2")],
        }
    ).encode("utf-8")
    model = UploadPayload.from_json_bytes(raw)
    assert model.payload["acceptedAtUtcIso"] == "2026-03-29T11:00:00.000Z"
    assert "accepted_at_utc_iso" not in model.payload
    assert "teleop_control" not in model.payload
    ev = model.payload["teleopControl"]["events"][0]
    assert ev["eventType"] == "get_control"
    assert ev["timestampUtcIso"] == "2026-03-29T11:05:00.000Z"


def test_teleop_control_must_be_object():
    raw = json.dumps(
        {
            "source": "unity_quest_dataset",
            "generatedUtcIso": "2026-03-29T12:00:00.000Z",
            "teleopControl": "not-an-object",
            "records": [_minimal_record()],
        }
    ).encode("utf-8")
    try:
        UploadPayload.from_json_bytes(raw)
    except ValueError as exc:
        assert "teleopControl" in str(exc).lower() or "object" in str(exc).lower()
    else:
        raise AssertionError("expected ValueError")


def test_teleop_control_events_must_be_array():
    raw = json.dumps(
        {
            "source": "unity_quest_dataset",
            "generatedUtcIso": "2026-03-29T12:00:00.000Z",
            "teleopControl": {"events": "nope"},
            "records": [_minimal_record()],
        }
    ).encode("utf-8")
    try:
        UploadPayload.from_json_bytes(raw)
    except ValueError as exc:
        assert "events" in str(exc)
    else:
        raise AssertionError("expected ValueError")


def test_teleop_control_event_requires_fields():
    raw = json.dumps(
        {
            "source": "unity_quest_dataset",
            "generatedUtcIso": "2026-03-29T12:00:00.000Z",
            "teleopControl": {"events": [{"eventType": "", "timestampUtcIso": "2026-03-29T12:00:00.000Z"}]},
            "records": [_minimal_record()],
        }
    ).encode("utf-8")
    try:
        UploadPayload.from_json_bytes(raw)
    except ValueError as exc:
        assert "eventType" in str(exc)
    else:
        raise AssertionError("expected ValueError")
