"""
Optional KYR/RAID correlation fields for DATA_NODE (HBR metadata.json).

See DOC/DATA_NODE_INGEST_AND_EVENTS_SPEC.md (dataset correlation).
See DOC/DATA_NODE_TELEOP_RECOVERY_INGEST_SPEC.md (training tier / session outcome).
"""

from typing import Any, Dict


def merge_data_node_correlation_metadata(metadata: Dict[str, Any], _dataset_id: str) -> None:
    """
    Populate optional keys kyrRobotId, kyrSessionId, raidRobotUuid when ROS params exist.

    _dataset_id is reserved for future scoping; currently all active correlation params apply.
    """
    import rospy

    if not rospy.core.is_initialized():
        return
    try:
        rid = rospy.get_param("/kyr_proxy/robot_id", "")
        if rid:
            metadata["kyrRobotId"] = str(rid)
        ks = rospy.get_param("/teleop_fetch/current_kyr_session_id", "")
        if ks:
            metadata["kyrSessionId"] = str(ks)
        raid = rospy.get_param("/x402_server/raid_robot_id", "")
        if raid:
            metadata["raidRobotUuid"] = str(raid)
    except Exception:
        pass


def _closure_suggests_recovery(closure: str) -> bool:
    c = (closure or "").strip().lower()
    if not c:
        return False
    return c.startswith("robot_watchdog") or c.startswith("operator_disconnect")


def merge_recovery_training_metadata(
    metadata: Dict[str, Any],
    operator_frame_count: int,
    robot_frame_count: int,
) -> None:
    """
    Optional datasetTrainingTier, operatorDataCompleteness, sessionOutcome for DATA_NODE / GR00T filters.

    Reads /teleop_fetch/last_* params set when KYR close_session succeeds (see teleop_node.py).
    """
    import rospy

    if not rospy.core.is_initialized():
        return
    try:
        closure = str(rospy.get_param("/teleop_fetch/last_session_closure_reason", "") or "")
        ended_by = str(rospy.get_param("/teleop_fetch/last_session_ended_by", "") or "")
        utc_iso = str(rospy.get_param("/teleop_fetch/last_session_outcome_utc_iso", "") or "")
        last_sid = str(rospy.get_param("/teleop_fetch/last_closed_kyr_session_id", "") or "")
        if not last_sid:
            last_sid = str(rospy.get_param("/teleop_fetch/current_kyr_session_id", "") or "")
    except Exception:
        return

    op_n = max(0, int(operator_frame_count))
    rob_n = max(0, int(robot_frame_count))
    abnormal = _closure_suggests_recovery(closure)

    if op_n <= 0 and rob_n > 0:
        tier = "robot_only"
        completeness = "absent"
    elif abnormal:
        tier = "recovery_slice"
        completeness = "partial" if op_n > 0 else "absent"
    else:
        tier = "full"
        if op_n > 0:
            completeness = "complete"
        elif rob_n > 0:
            completeness = "partial"
        else:
            completeness = "absent"

    metadata["datasetTrainingTier"] = tier
    metadata["operatorDataCompleteness"] = completeness

    if last_sid or closure or ended_by or utc_iso:
        metadata["sessionOutcome"] = {
            "kyrSessionId": last_sid,
            "closureReason": closure,
            "endedBy": ended_by or "unknown",
            "utcIso": utc_iso,
        }
