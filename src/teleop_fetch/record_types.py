"""
Shared typed record models for dataset recording.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional


@dataclass
class PoseData:
    position: Dict[str, float]
    orientation: Dict[str, float]


@dataclass
class JointValue:
    name: str
    value: float


@dataclass
class CameraFrame:
    local_unix_time_ns: int
    width: int
    height: int
    encoding: str
    step: int
    frame_id: str
    data_b64: str


@dataclass
class ImuSample:
    local_unix_time_ns: int
    orientation: Dict[str, float]
    angular_velocity: Dict[str, float]
    linear_acceleration: Dict[str, float]


@dataclass
class JointSample:
    local_unix_time_ns: int
    names: List[str]
    positions: List[float] = field(default_factory=list)
    velocities: List[float] = field(default_factory=list)
    efforts: List[float] = field(default_factory=list)


@dataclass
class RobotFrame:
    local_unix_time_ns: int
    local_monotonic_sec: float
    estimated_ros_unix_time_ns: int
    ros_clock_offset_sec: float
    sync_rtt_sec: float
    ros_time_synchronized: bool
    imu: Optional[ImuSample]
    joints: Optional[JointSample]
    camera_frame_index: int


@dataclass
class OperatorFrame:
    local_unix_time_ns: int
    local_monotonic_sec: float
    estimated_ros_unix_time_ns: int
    ros_clock_offset_sec: float
    sync_rtt_sec: float
    ros_time_synchronized: bool
    input_mode: str
    head: PoseData
    left: PoseData
    right: PoseData
    joints: List[JointValue]
