"""Keeps the latest turtle pose.

The pose arrives at ~60 Hz. A MultiThreadedExecutor (needed by the action servers so a goal can wait
for services) burns a lot of CPU when a fast topic goes through it and makes the control loop jittery,
so the subscription lives in its own tiny node, spun by its own single-threaded executor in a thread.
"""
import threading
import time
from typing import Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor

from turtle_control import constants as C
from turtle_control.motion import Pose
from turtle_control.turtlesim_compat import Pose as TurtlePose


class PoseTracker:
    def __init__(self, name: str):
        self._pose: Optional[Pose] = None
        self._stamp = 0.0
        # use_global_arguments=False: otherwise `ros2 launch` renames this node like its parent (__node:=...)
        self._node = rclpy.create_node(name, start_parameter_services=False,
                                       use_global_arguments=False)
        self._node.create_subscription(TurtlePose, C.POSE_TOPIC, self._on_pose, 10)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()

    def _on_pose(self, msg):
        self._pose = Pose(msg.x, msg.y, msg.theta)
        self._stamp = time.monotonic()

    @property
    def pose(self) -> Optional[Pose]:
        return self._pose

    @property
    def age(self) -> float:
        """Seconds since the last pose message (inf if none yet)."""
        return time.monotonic() - self._stamp if self._pose else float('inf')

    def shutdown(self):
        self._executor.shutdown()
        self._node.destroy_node()
