# TELEOP_DATAS Implementation Status

Implemented in `teleop_fetch`:

- ROS recorder control topic: `/record_sessions` (`dataset_recorder_node.py`)
- Upload API endpoint: `POST /upload_dataset` on port `9191` (`dataset_upload_server.py`). **Remote operators (RAID):** use `https://<raid>/api/teleop/robots/<robotId>/dataset/upload_dataset` — see [RAID_APP_DATASET_PROXY_SPEC.md](RAID_APP_DATASET_PROXY_SPEC.md).
- Session binding logic: upload payload is attached to existing `recordId` session and persisted into `.hbr/operator/*`

---

## 1. ROS recording events (`/record_sessions`)

This is the message you send when starting and stopping a recording.

### `std_msgs/String` over rosbridge

Over rosbridge it is published as:

```json
{  
  "op": "publish",  
  "topic": "/record_sessions",  
  "msg": {  
    "data": "{\"record_id\":\"7d7d3d7c4f1b4f2c8d5c2b0e0a123456\",\"event_type\":\"start\",\"app_session_id\":\"f1d2d2f924e986ac86fdf7b36c94bcdf\",\"timestamp_unix_ns\":1760700000123456789,\"timestamp_ros_unix_ns\":1760700000223456789,\"ntp_time_synchronized\":true,\"ros_time_synchronized\":true,\"pose_topic\":\"/quest/poses\",\"joint_topic\":\"/quest/joints\",\"send_hz\":10.0}"  
  }  
}
```

The JSON payload inside `msg.data` looks like:

```json
{  
  "record_id": "7d7d3d7c4f1b4f2c8d5c2b0e0a123456",  
  "event_type": "start",  
  "app_session_id": "f1d2d2f924e986ac86fdf7b36c94bcdf",  
  "timestamp_unix_ns": 1760700000123456789,  
  "timestamp_ros_unix_ns": 1760700000223456789,  
  "ntp_time_synchronized": true,  
  "ros_time_synchronized": true,  
  "pose_topic": "/quest/poses",  
  "joint_topic": "/quest/joints",  
  "send_hz": 10.0  
}
```

To stop recording the same structure is used, but with:

```json
{  
  "event_type": "stop"  
}
```

---

## 2. Top-level JSON for REST `upload_dataset`

This is the final payload assembled by the headset `DatasetManager`.

### Top-level structure

```json
{  
  "source": "unity_quest_dataset",  
  "generatedUtcIso": "2026-03-17T18:45:12.3456789Z",  
  "acceptedAtUtcIso": "2026-03-17T18:40:00.000Z",  
  "teleopControl": {
    "events": [
      { "eventType": "get_control", "timestampUtcIso": "2026-03-17T18:40:05.000Z" },
      { "eventType": "lost_control", "timestampUtcIso": "2026-03-17T18:45:00.000Z" }
    ]
  },
  "records": []  
}
```

Optional top-level fields:

- `acceptedAtUtcIso` — ISO-8601 UTC when the VR app accepted the teleop session.
- `teleopControl` — object with `events[]`; each event has `eventType` and `timestampUtcIso` (strings). Typical `eventType` values: `get_control`, `lost_control`.

DATA_NODE ingest and multipart mirror: [DATA_NODE_OPERATOR_SESSION_SPEC.md](DATA_NODE_OPERATOR_SESSION_SPEC.md).

---

## 3. Structure of one `records[]` element

```json
{  
  "recordId": "7d7d3d7c4f1b4f2c8d5c2b0e0a123456",  
  "label": "Approach to point A",  
  "taskName": "Drive to marker near wall",  
  "data": {}  
}
```

Where:

- `recordId` — unique id of the recording
- `label` — short label from `RecordData.TextField`
- `taskName` — human-readable task name
- `data` — recorded telemetry session

---

## 4. Structure of `data` (`RecordedSession`)

```json
{  
  "recordId": "7d7d3d7c4f1b4f2c8d5c2b0e0a123456",  

  "startedLocalUnixTimeNs": 1760700000000000000,  
  "endedLocalUnixTimeNs": 1760700005000000000,  

  "startedEstimatedExternalUnixTimeNs": 1760700000100000000,  
  "endedEstimatedExternalUnixTimeNs": 1760700005100000000,  

  "startedEstimatedRosUnixTimeNs": 1760700000200000000,  
  "endedEstimatedRosUnixTimeNs": 1760700005200000000,  

  "rosTimeWasSynchronizedAtStart": true,  
  "rosTimeWasSynchronizedAtEnd": true,  

  "ntpTimeWasSynchronizedAtStart": true,  
  "ntpTimeWasSynchronizedAtEnd": true,  

  "sourceWsUrl": "ws://192.168.1.100:9090",  
  "sourceSendHz": 10.0,  

  "frames": []  
}
```

### Field meanings

- `startedLocalUnixTimeNs`, `endedLocalUnixTimeNs`  
  Local device time in ns.
- `startedEstimatedExternalUnixTimeNs`, `endedEstimatedExternalUnixTimeNs`  
  Time corrected using NTP.
- `startedEstimatedRosUnixTimeNs`, `endedEstimatedRosUnixTimeNs`  
  Time additionally corrected to ROS clock.
- `rosTimeWasSynchronizedAtStart/End`  
  Whether ROS time sync was active.
- `ntpTimeWasSynchronizedAtStart/End`  
  Whether NTP time sync was active.
- `sourceWsUrl`  
  rosbridge WebSocket URL.
- `sourceSendHz`  
  Data send frequency.
- `frames`  
  Array of telemetry frames.

---

## 5. Structure of one `frame` (`RecordedFrame`)

```json
{  
  "localUnixTimeNs": 1760700000123456789,  
  "localMonotonicSec": 123.456789,  

  "estimatedExternalUnixTimeNs": 1760700000223456789,  
  "estimatedRosUnixTimeNs": 1760700000323456789,  

  "ntpTimeSynchronized": true,  
  "ntpClockOffsetSec": 0.102314,  
  "ntpSyncRttSec": 0.0184,  

  "rosClockOffsetSec": 0.054211,  
  "syncRttSec": 0.0128,  
  "rosTimeSynchronized": true,  

  "inputMode": "controllers",  

  "head": {},  
  "left": {},  
  "right": {},  

  "joints": []  
}
```

---

## 6. Pose structure (`RecordedPose`)

```json
{  
  "position": {  
    "x": 0.12,  
    "y": 1.43,  
    "z": -0.55  
  },  
  "orientation": {  
    "x": 0.0,  
    "y": 0.707,  
    "z": 0.0,  
    "w": 0.707  
  }  
}
```

This structure is reused for:

- `head`
- `left`
- `right`

---

## 7. Joints array structure (`joints`)

```json
[  
  { "name": "L_grip", "value": 0.72 },  
  { "name": "L_index", "value": 0.15 },  
  { "name": "R_grip", "value": 0.01 },  
  { "name": "R_index", "value": 0.84 },  
  { "name": "L_X", "value": 1.0 },  
  { "name": "L_Y", "value": 0.0 },  
  { "name": "R_A", "value": 0.0 },  
  { "name": "R_B", "value": 1.0 },  
  { "name": "L_stick_x", "value": -0.23 },  
  { "name": "L_stick_y", "value": 0.91 },  
  { "name": "L_stick_click", "value": 0.0 },  
  { "name": "L_stick_touch", "value": 1.0 }  
]
```

If the input mode is `hands`, for example:

```json
[  
  { "name": "L_grip", "value": 0.31 },  
  { "name": "L_index", "value": 0.85 },  
  { "name": "L_pinch_index", "value": 0.92 },  
  { "name": "L_pinch_middle", "value": 0.11 },  
  { "name": "L_pinch_ring", "value": 0.04 },  
  { "name": "L_pinch_little", "value": 0.01 }  
]
```

---

## 8. Full example of the final payload

```json
{  
  "source": "unity_quest_dataset",  
  "generatedUtcIso": "2026-03-17T18:45:12.3456789Z",  
  "acceptedAtUtcIso": "2026-03-17T18:40:00.000Z",  
  "teleopControl": {
    "events": [
      { "eventType": "get_control", "timestampUtcIso": "2026-03-17T18:40:05.000Z" }
    ]
  },
  "records": [  
    {  
      "recordId": "7d7d3d7c4f1b4f2c8d5c2b0e0a123456",  
      "label": "Approach to point A",  
      "taskName": "Drive to marker near wall",  
      "data": {  
        "recordId": "7d7d3d7c4f1b4f2c8d5c2b0e0a123456",  
        "startedLocalUnixTimeNs": 1760700000000000000,  
        "endedLocalUnixTimeNs": 1760700005000000000,  
        "startedEstimatedExternalUnixTimeNs": 1760700000100000000,  
        "endedEstimatedExternalUnixTimeNs": 1760700005100000000,  
        "startedEstimatedRosUnixTimeNs": 1760700000200000000,  
        "endedEstimatedRosUnixTimeNs": 1760700005200000000,  
        "rosTimeWasSynchronizedAtStart": true,  
        "rosTimeWasSynchronizedAtEnd": true,  
        "ntpTimeWasSynchronizedAtStart": true,  
        "ntpTimeWasSynchronizedAtEnd": true,  
        "sourceWsUrl": "ws://192.168.1.100:9090",  
        "sourceSendHz": 10.0,  
        "frames": [  
          {  
            "localUnixTimeNs": 1760700000123456789,  
            "localMonotonicSec": 123.456789,  
            "estimatedExternalUnixTimeNs": 1760700000223456789,  
            "estimatedRosUnixTimeNs": 1760700000323456789,  
            "ntpTimeSynchronized": true,  
            "ntpClockOffsetSec": 0.102314,  
            "ntpSyncRttSec": 0.0184,  
            "rosClockOffsetSec": 0.054211,  
            "syncRttSec": 0.0128,  
            "rosTimeSynchronized": true,  
            "inputMode": "controllers",  
            "head": {  
              "position": { "x": 0.12, "y": 1.43, "z": -0.55 },  
              "orientation": { "x": 0.0, "y": 0.707, "z": 0.0, "w": 0.707 }  
            },  
            "left": {  
              "position": { "x": -0.23, "y": -0.18, "z": 0.44 },  
              "orientation": { "x": 0.01, "y": 0.12, "z": -0.02, "w": 0.99 }  
            },  
            "right": {  
              "position": { "x": 0.28, "y": -0.16, "z": 0.41 },  
              "orientation": { "x": -0.03, "y": -0.10, "z": 0.04, "w": 0.99 }  
            },  
            "joints": [  
              { "name": "L_grip", "value": 0.72 },  
              { "name": "L_index", "value": 0.15 },  
              { "name": "R_grip", "value": 0.01 },  
              { "name": "R_index", "value": 0.84 },  
              { "name": "L_X", "value": 1.0 },  
              { "name": "L_Y", "value": 0.0 },  
              { "name": "R_A", "value": 0.0 },  
              { "name": "R_B", "value": 1.0 },  
              { "name": "L_stick_x", "value": -0.23 },  
              { "name": "L_stick_y", "value": 0.91 },  
              { "name": "L_stick_click", "value": 0.0 },  
              { "name": "L_stick_touch", "value": 1.0 }  
            ]  
          }  
        ]  
      }  
    }  
  ]  
}
```

---

## 9. Formal JSON schema

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://example.com/schemas/unity-quest-dataset.schema.json",
  "title": "Unity Quest Dataset Upload",
  "type": "object",
  "additionalProperties": false,
  "required": [
    "source",
    "generatedUtcIso",
    "records"
  ],
  "properties": {
    "source": {
      "type": "string",
      "minLength": 1
    },
    "generatedUtcIso": {
      "type": "string",
      "format": "date-time"
    },
    "acceptedAtUtcIso": {
      "type": "string",
      "format": "date-time"
    },
    "teleopControl": {
      "$ref": "#/$defs/TeleopControl"
    },
    "records": {
      "type": "array",
      "items": {
        "$ref": "#/$defs/DatasetUploadRecord"
      }
    }
  },
  "$defs": {
    "TeleopControl": {
      "type": "object",
      "additionalProperties": true,
      "properties": {
        "events": {
          "type": "array",
          "items": {
            "$ref": "#/$defs/TeleopControlEvent"
          }
        }
      }
    },
    "TeleopControlEvent": {
      "type": "object",
      "additionalProperties": false,
      "required": ["eventType", "timestampUtcIso"],
      "properties": {
        "eventType": {
          "type": "string",
          "minLength": 1
        },
        "timestampUtcIso": {
          "type": "string",
          "format": "date-time"
        }
      }
    },
    "UnixTimeNs": {
      "type": "integer",
      "description": "Unix timestamp in nanoseconds"
    },
    "JsonVec3": {
      "type": "object",
      "additionalProperties": false,
      "required": ["x", "y", "z"],
      "properties": {
        "x": { "type": "number" },
        "y": { "type": "number" },
        "z": { "type": "number" }
      }
    },
    "JsonQuat": {
      "type": "object",
      "additionalProperties": false,
      "required": ["x", "y", "z", "w"],
      "properties": {
        "x": { "type": "number" },
        "y": { "type": "number" },
        "z": { "type": "number" },
        "w": { "type": "number" }
      }
    },
    "RecordedPose": {
      "type": "object",
      "additionalProperties": false,
      "required": ["position", "orientation"],
      "properties": {
        "position": { "$ref": "#/$defs/JsonVec3" },
        "orientation": { "$ref": "#/$defs/JsonQuat" }
      }
    },
    "RecordedJointValue": {
      "type": "object",
      "additionalProperties": false,
      "required": ["name", "value"],
      "properties": {
        "name": {
          "type": "string",
          "minLength": 1
        },
        "value": {
          "type": "number"
        }
      }
    },
    "RecordedFrame": {
      "type": "object",
      "additionalProperties": false,
      "required": [
        "localUnixTimeNs",
        "localMonotonicSec",
        "estimatedExternalUnixTimeNs",
        "estimatedRosUnixTimeNs",
        "ntpTimeSynchronized",
        "ntpClockOffsetSec",
        "ntpSyncRttSec",
        "rosClockOffsetSec",
        "syncRttSec",
        "rosTimeSynchronized",
        "inputMode",
        "head",
        "left",
        "right",
        "joints"
      ],
      "properties": {
        "localUnixTimeNs": {
          "$ref": "#/$defs/UnixTimeNs"
        },
        "localMonotonicSec": {
          "type": "number"
        },
        "estimatedExternalUnixTimeNs": {
          "$ref": "#/$defs/UnixTimeNs"
        },
        "estimatedRosUnixTimeNs": {
          "$ref": "#/$defs/UnixTimeNs"
        },
        "ntpTimeSynchronized": {
          "type": "boolean"
        },
        "ntpClockOffsetSec": {
          "type": "number"
        },
        "ntpSyncRttSec": {
          "type": "number",
          "minimum": 0
        },
        "rosClockOffsetSec": {
          "type": "number"
        },
        "syncRttSec": {
          "type": "number",
          "minimum": 0
        },
        "rosTimeSynchronized": {
          "type": "boolean"
        },
        "inputMode": {
          "type": "string",
          "enum": ["controllers", "hands", "none"]
        },
        "head": {
          "$ref": "#/$defs/RecordedPose"
        },
        "left": {
          "$ref": "#/$defs/RecordedPose"
        },
        "right": {
          "$ref": "#/$defs/RecordedPose"
        },
        "joints": {
          "type": "array",
          "items": {
            "$ref": "#/$defs/RecordedJointValue"
          }
        }
      }
    },
    "RecordedSession": {
      "type": "object",
      "additionalProperties": false,
      "required": [
        "recordId",
        "startedLocalUnixTimeNs",
        "endedLocalUnixTimeNs",
        "startedEstimatedExternalUnixTimeNs",
        "endedEstimatedExternalUnixTimeNs",
        "startedEstimatedRosUnixTimeNs",
        "endedEstimatedRosUnixTimeNs",
        "rosTimeWasSynchronizedAtStart",
        "rosTimeWasSynchronizedAtEnd",
        "ntpTimeWasSynchronizedAtStart",
        "ntpTimeWasSynchronizedAtEnd",
        "sourceWsUrl",
        "sourceSendHz",
        "frames"
      ],
      "properties": {
        "recordId": {
          "type": "string",
          "minLength": 1
        },
        "startedLocalUnixTimeNs": {
          "$ref": "#/$defs/UnixTimeNs"
        },
        "endedLocalUnixTimeNs": {
          "$ref": "#/$defs/UnixTimeNs"
        },
        "startedEstimatedExternalUnixTimeNs": {
          "$ref": "#/$defs/UnixTimeNs"
        },
        "endedEstimatedExternalUnixTimeNs": {
          "$ref": "#/$defs/UnixTimeNs"
        },
        "startedEstimatedRosUnixTimeNs": {
          "$ref": "#/$defs/UnixTimeNs"
        },
        "endedEstimatedRosUnixTimeNs": {
          "$ref": "#/$defs/UnixTimeNs"
        },
        "rosTimeWasSynchronizedAtStart": {
          "type": "boolean"
        },
        "rosTimeWasSynchronizedAtEnd": {
          "type": "boolean"
        },
        "ntpTimeWasSynchronizedAtStart": {
          "type": "boolean"
        },
        "ntpTimeWasSynchronizedAtEnd": {
          "type": "boolean"
        },
        "sourceWsUrl": {
          "type": "string",
          "minLength": 1
        },
        "sourceSendHz": {
          "type": "number",
          "exclusiveMinimum": 0
        },
        "frames": {
          "type": "array",
          "items": {
            "$ref": "#/$defs/RecordedFrame"
          }
        }
      }
    },
    "DatasetUploadRecord": {
      "type": "object",
      "additionalProperties": false,
      "required": [
        "recordId",
        "label",
        "taskName",
        "data"
      ],
      "properties": {
        "recordId": {
          "type": "string",
          "minLength": 1
        },
        "label": {
          "type": "string"
        },
        "taskName": {
          "type": "string"
        },
        "data": {
          "$ref": "#/$defs/RecordedSession"
        }
      }
    }
  }
}
```

