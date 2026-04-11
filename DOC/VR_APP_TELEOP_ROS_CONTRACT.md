# VR App — ROS / rosbridge contract (vNext)

**Audience:** Meta Quest teleop application developers.  
**Robot stack:** `br-vr-dev-sinc` (`teleop_fetch`, `vr_remapper`, `pose_source`, `fast_ik`), `br-kyr` (`kyr_proxy`), optional `rospy_x402`.  
**Version:** vNext — **breaking** relative to undocumented legacy behavior; coordinate release with robot image that implements [`TELEOP_SESSION_LIFECYCLE_AND_FAILURES.md](TELEOP_SESSION_LIFECYCLE_AND_FAILURES.md)`.

## 1. Purpose

Define how the VR app interacts with ROS over **rosbridge** so the robot can:

- Detect **operator presence** without a separate heartbeat topic (reuse pose/joint streams).
- Distinguish **intentional pause / headset off** from **network failure** where possible.
- Align **dataset upload** and **teleopControl** metadata with DATA_NODE ingest.

## 2. Transport

- **WebSocket:** same host/port as today (typically robot `9090`, launched from `br_bringup` / `ecosystem.launch`).
- **Authentication / headers:** follow RAID proxy conventions if used; see [`br-kyr/DOC/ROSBRIDGE_AND_RAID.md`](../../br-kyr/DOC/ROSBRIDGE_AND_RAID.md).

## 3. Presence and liveness (no dedicated heartbeat topic)

The robot treats **combined liveness** of these streams as the operator heartbeat:

| Topic | Message type | Rule |
|-------|----------------|------|
| `/quest/poses` | `geometry_msgs/PoseArray` | Must publish while the operator is expected to be in session and tracking is active. |
| `/quest/joints` | `sensor_msgs/JointState` | Must publish on the same schedule while in session (button edges for L_X / L_Y / R_A depend on this stream). |

**Watchdog (robot-side):** if **neither** stream receives a message for **`operator_presence_timeout_sec`** (robot param, default documented in `teleop.yaml`), the robot may treat the operator as lost, **disarm**, and drive KYR **`close_session`** with a documented `closure_reason` (e.g. `robot_watchdog_timeout`).

**vNext requirement:** while KYR teleop session is **ACTIVE** on the robot, the app **SHOULD** publish at least one of `/quest/poses` or `/quest/joints` at **≥ 1 Hz** averaged over any 2 s window, unless entering a documented idle state (§4).

## 4. Lifecycle and pause (avoid false watchdog)

If the app **stops** sending poses/joints when the headset is removed or the app goes to background, the robot cannot distinguish that from a network drop unless you signal intent.

**vNext — choose one or both:**

1. **Dedicated topic (recommended):** `/quest/teleop_lifecycle` — `std_msgs/String`, UTF-8 JSON per message, e.g.  
   `{"event":"pause","reason":"headset_removed","ts_unix_ms":…}`  
   Events (string values are normative for the robot mapping table in [`TELEOP_SESSION_LIFECYCLE_AND_FAILURES.md`](TELEOP_SESSION_LIFECYCLE_AND_FAILURES.md)):  
   - `session_active` — optional ack after grant (if implemented).  
   - `pause` — operator intentionally idle; robot may **extend** or **suspend** watchdog per policy.  
   - `resume` — tracking resumed.  
   - `disconnect` — clean shutdown; include `reason`: `user_exit` | `app_background` | `ping_exceeded` | `low_battery` | `network` | `unknown`.

2. **Metadata only:** embed the same semantics in `teleopControl.events` and `POST /upload_dataset` (see [`DATA_NODE_OPERATOR_SESSION_SPEC.md`](DATA_NODE_OPERATOR_SESSION_SPEC.md)) — **does not** prevent watchdog firing before upload; prefer the topic for real-time behavior.

## 5. Control semantics (unchanged unless product changes)

- **L_X:** arm **get_control** (rising edge) when `arm_stream_requires_lx` is true on the robot.
- **L_Y:** first press — disarm stream; second press (if `end_session_on_second_ly`) — end KYR session and billing path on robot.
- **R_A:** calibration in `vr_remapper` (see [`ARCHITECTURE.md`](ARCHITECTURE.md)).

## 6. Dataset upload (`POST /upload_dataset`)

- **Port:** robot `9191` or RAID reverse proxy (see [`RAID_APP_DATASET_PROXY_SPEC.md`](RAID_APP_DATASET_PROXY_SPEC.md)).
- **Root field (vNext):** `contractVersion` — integer, start at **2** for this document’s semantics.
- **Existing fields:** `source`, `generatedUtcIso`, `records[]`, optional `acceptedAtUtcIso`, `teleopControl` — see [`TELEOP_DATAS.md`](TELEOP_DATAS.md) and [`DATA_NODE_OPERATOR_SESSION_SPEC.md`](DATA_NODE_OPERATOR_SESSION_SPEC.md).

## 7. References

- Robot failure and session table: [`TELEOP_SESSION_LIFECYCLE_AND_FAILURES.md`](TELEOP_SESSION_LIFECYCLE_AND_FAILURES.md)
- DATA_NODE optional training metadata: [`DATA_NODE_TELEOP_RECOVERY_INGEST_SPEC.md`](DATA_NODE_TELEOP_RECOVERY_INGEST_SPEC.md)
- RAID / payment (consumer): [`../../rospy_x402/DOC/RAID_TELEOP_SESSION_FAILURE_AND_PAYMENT_SPEC.md`](../../rospy_x402/DOC/RAID_TELEOP_SESSION_FAILURE_AND_PAYMENT_SPEC.md)
