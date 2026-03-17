import threading
import time
from typing import List, Optional

import rospy
from sensor_msgs.msg import JointState

from teleop_fetch.record_types import JointSample
from teleop_fetch.sensors.base_joint_sensor import BaseJointSensor


class ROSJointSensor(BaseJointSensor):
    def __init__(self, topic: str, queue_size: int = 500):
        super().__init__(joint_names=[])
        self.topic = topic
        self._queue_size = int(queue_size)
        self._samples: List[JointSample] = []
        self._latest: Optional[JointSample] = None
        self._lock = threading.Lock()
        self._active = False
        self._sub = rospy.Subscriber(topic, JointState, self._callback, queue_size=queue_size)

    def start_recording(self, dataset_id: str) -> None:
        del dataset_id
        with self._lock:
            self._samples = []
            self._latest = None
            self._active = True

    def stop_recording(self) -> None:
        with self._lock:
            self._active = False

    def drain_samples(self) -> List[JointSample]:
        with self._lock:
            out = self._samples
            self._samples = []
        return out

    def latest(self) -> Optional[JointSample]:
        with self._lock:
            return self._latest

    def _callback(self, msg: JointState) -> None:
        with self._lock:
            if not self._active:
                return
            if len(self._samples) >= self._queue_size:
                self._samples.pop(0)
            ts = int(time.time_ns())
            if msg.header.stamp.to_nsec() > 0:
                ts = int(msg.header.stamp.to_nsec())
            sample = JointSample(
                local_unix_time_ns=ts,
                names=list(msg.name),
                positions=[float(v) for v in msg.position],
                velocities=[float(v) for v in msg.velocity],
                efforts=[float(v) for v in msg.effort],
            )
            self._samples.append(sample)
            self._latest = sample
            self.joint_names = sample.names
