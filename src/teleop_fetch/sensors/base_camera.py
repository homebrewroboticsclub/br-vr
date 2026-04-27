from abc import ABC, abstractmethod
from typing import List

from teleop_fetch.record_types import CameraFrame


class BaseCamera(ABC):
    def __init__(self, name: str, topic: str, frame_rate_hz: float):
        self.name = name
        self.topic = topic
        self.frame_rate_hz = frame_rate_hz

    @abstractmethod
    def start_recording(self, dataset_id: str) -> None:
        pass

    @abstractmethod
    def stop_recording(self) -> None:
        pass

    @abstractmethod
    def drain_frames(self) -> List[CameraFrame]:
        pass
