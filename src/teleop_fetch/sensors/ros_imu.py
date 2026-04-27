import threading
import time
from typing import List, Optional

import rospy
from sensor_msgs.msg import Imu

from teleop_fetch.record_types import ImuSample
from teleop_fetch.sensors.base_imu import BaseIMU


class ROSIMU(BaseIMU):
    def __init__(self, name: str, topic: str, queue_size: int = 500):
        super().__init__(name=name, topic=topic)
        self._queue_size = int(queue_size)
        self._samples: List[ImuSample] = []
        self._latest: Optional[ImuSample] = None
        self._lock = threading.Lock()
        self._active = False
        self._sub = rospy.Subscriber(topic, Imu, self._callback, queue_size=queue_size)

    def start_recording(self, dataset_id: str) -> None:
        del dataset_id
        with self._lock:
            self._samples = []
            self._latest = None
            self._active = True

    def stop_recording(self) -> None:
        with self._lock:
            self._active = False

    def drain_samples(self) -> List[ImuSample]:
        with self._lock:
            out = self._samples
            self._samples = []
        return out

    def latest(self) -> Optional[ImuSample]:
        with self._lock:
            return self._latest

    def _callback(self, msg: Imu) -> None:
        with self._lock:
            if not self._active:
                return
            if len(self._samples) >= self._queue_size:
                self._samples.pop(0)
            ts = int(time.time_ns())
            if msg.header.stamp.to_nsec() > 0:
                ts = int(msg.header.stamp.to_nsec())
            sample = ImuSample(
                local_unix_time_ns=ts,
                orientation={
                    "x": float(msg.orientation.x),
                    "y": float(msg.orientation.y),
                    "z": float(msg.orientation.z),
                    "w": float(msg.orientation.w),
                },
                angular_velocity={
                    "x": float(msg.angular_velocity.x),
                    "y": float(msg.angular_velocity.y),
                    "z": float(msg.angular_velocity.z),
                },
                linear_acceleration={
                    "x": float(msg.linear_acceleration.x),
                    "y": float(msg.linear_acceleration.y),
                    "z": float(msg.linear_acceleration.z),
                },
            )
            self._samples.append(sample)
            self._latest = sample
