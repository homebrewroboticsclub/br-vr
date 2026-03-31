import base64
import threading
import time
from typing import List

import rospy
from sensor_msgs.msg import Image

from teleop_fetch.record_types import CameraFrame
from teleop_fetch.sensors.base_camera import BaseCamera


class ROSCamera(BaseCamera):
    def __init__(self, name: str, topic: str, frame_rate_hz: float, queue_size: int = 200):
        super().__init__(name=name, topic=topic, frame_rate_hz=frame_rate_hz)
        self._queue_size = int(queue_size)
        self._frames: List[CameraFrame] = []
        self._lock = threading.Lock()
        self._active = False
        self._sub = rospy.Subscriber(topic, Image, self._callback, queue_size=queue_size)

    def start_recording(self, dataset_id: str) -> None:
        del dataset_id
        with self._lock:
            self._frames = []
            self._active = True

    def stop_recording(self) -> None:
        with self._lock:
            self._active = False

    def drain_frames(self) -> List[CameraFrame]:
        with self._lock:
            out = self._frames
            self._frames = []
        return out

    def _callback(self, msg: Image) -> None:
        with self._lock:
            if not self._active:
                return
            if len(self._frames) >= self._queue_size:
                self._frames.pop(0)
            ts = int(time.time_ns())
            if msg.header.stamp.to_nsec() > 0:
                ts = int(msg.header.stamp.to_nsec())
            self._frames.append(
                CameraFrame(
                    local_unix_time_ns=ts,
                    width=int(msg.width),
                    height=int(msg.height),
                    encoding=str(msg.encoding),
                    step=int(msg.step),
                    frame_id=str(msg.header.frame_id),
                    data_b64=base64.b64encode(msg.data).decode("ascii"),
                )
            )
