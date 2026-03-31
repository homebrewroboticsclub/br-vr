from abc import ABC, abstractmethod
from typing import List

from teleop_fetch.record_types import ImuSample


class BaseIMU(ABC):
    def __init__(self, name: str, topic: str):
        self.name = name
        self.topic = topic

    @abstractmethod
    def start_recording(self, dataset_id: str) -> None:
        pass

    @abstractmethod
    def stop_recording(self) -> None:
        pass

    @abstractmethod
    def drain_samples(self) -> List[ImuSample]:
        pass

    @abstractmethod
    def latest(self) -> ImuSample:
        pass
