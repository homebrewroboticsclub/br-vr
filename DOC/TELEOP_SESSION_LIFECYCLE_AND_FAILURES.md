# Teleop session lifecycle, failures, and recovery (robot-centric)

**Audience:** robot integrators and agents working across `br-vr-dev-sinc`, `br-kyr`, `rospy_x402`.  
**Language:** this file is English per repository policy.

## 1. Normal flow (summary)

1. `rospy_x402` obtains signed SessionGrant from RAID (immediate or poll) → forwards to `/teleop_fetch/receive_grant`.
2. `teleop_fetch` calls `/kyr/open_session` → KYR **ACTIVE**, dashboard `session_open` event.
3. VR streams `/quest/poses`, `/quest/joints`; arms forward targets through `/kyr/bus_servo_in` when armed and policy allows.
4. End: `/teleop_fetch/end_session` or second L_Y → `/kyr/close_session` → `/x402/complete_teleop_payment`.

Details: [`ARCHITECTURE.md`](ARCHITECTURE.md), [`../../rospy_x402/DOC/RAID_APP_TELEOP_HELP_FULL_CYCLE_X402_SPEC.md`](../../rospy_x402/DOC/RAID_APP_TELEOP_HELP_FULL_CYCLE_X402_SPEC.md).

## 2. Failure modes and robot response

| Scenario | Typical detection | Robot action | Suggested `closure_reason` |
|----------|-------------------|--------------|----------------------------|
| VR ping policy kicked operator | VR may send `teleop_lifecycle` or upload-only hint | Watchdog if streams stop | `operator_disconnect_ping` or `robot_watchdog_timeout` |
| Operator closed app | Streams stop | Watchdog + optional lifecycle JSON | `operator_disconnect_app_exit` |
| Network loss | Streams stop | Watchdog | `operator_disconnect_network` or `robot_watchdog_timeout` |
| Headset battery / power off | Streams stop | Watchdog | `operator_disconnect_power` or `robot_watchdog_timeout` |
| Intentional pause / headset off | VR sends `pause` on `/quest/teleop_lifecycle` | Robot may suspend watchdog (policy) | N/A until resume or timeout |

**Safety:** on teardown path, **disarm** arm stream to `/kyr/bus_servo_in` before or with `close_session`.

**Idempotency:** watchdog must call teardown **once** per KYR session id.

## 3. Dataset recovery tiers

Robot may set optional `metadata.json` fields described in [`DATA_NODE_TELEOP_RECOVERY_INGEST_SPEC.md`](DATA_NODE_TELEOP_RECOVERY_INGEST_SPEC.md): `datasetTrainingTier`, `operatorDataCompleteness`, `sessionOutcome`.

## 4. Payment multiplier

Abnormal `closure_reason` values trigger **fractional** operator payment on the robot (default **0.5** of computed amount). See [`../../rospy_x402/DOC/RAID_TELEOP_SESSION_FAILURE_AND_PAYMENT_SPEC.md`](../../rospy_x402/DOC/RAID_TELEOP_SESSION_FAILURE_AND_PAYMENT_SPEC.md).

## 5. Consumer specs (external teams)

| Document | Audience |
|----------|----------|
| [`VR_APP_TELEOP_ROS_CONTRACT.md`](VR_APP_TELEOP_ROS_CONTRACT.md) | Quest developers (vNext) |
| [`DATA_NODE_TELEOP_RECOVERY_INGEST_SPEC.md`](DATA_NODE_TELEOP_RECOVERY_INGEST_SPEC.md) | DATA_NODE / ML |
| [`../../rospy_x402/DOC/RAID_TELEOP_SESSION_FAILURE_AND_PAYMENT_SPEC.md`](../../rospy_x402/DOC/RAID_TELEOP_SESSION_FAILURE_AND_PAYMENT_SPEC.md) | RAID / product |
