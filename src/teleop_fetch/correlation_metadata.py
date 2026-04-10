"""
Optional KYR/RAID correlation fields for DATA_NODE (HBR metadata.json).

See DOC/DATA_NODE_INGEST_AND_EVENTS_SPEC.md (dataset correlation).
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
