from abc import ABC, abstractmethod
from typing import List

from teleop_fetch.record_types import JointSample


class BaseJointSensor(ABC):
    def __init__(self, joint_names: List[str]):
        self.joint_names = joint_names

    @abstractmethod
    def start_recording(self, dataset_id: str) -> None:
        pass

    @abstractmethod
    def stop_recording(self) -> None:
        pass

    @abstractmethod
    def drain_samples(self) -> List[JointSample]:
        pass

    @abstractmethod
    def latest(self) -> JointSample:
        pass
