#!/usr/bin/python3
"""Print one fresh Piper gripper opening feedback value in metres."""

from __future__ import annotations

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


class GripperReader(Node):
    def __init__(self) -> None:
        super().__init__("piper_grasp_gripper_reader")
        self.value = None
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            JointState, "/piper_x/feedback/joint_states", self._callback, qos
        )

    def _callback(self, message: JointState) -> None:
        positions = dict(zip(message.name, message.position))
        if "gripper" in positions and math.isfinite(float(positions["gripper"])):
            self.value = float(positions["gripper"])


def main() -> int:
    rclpy.init()
    node = GripperReader()
    try:
        deadline = time.monotonic() + 5.0
        while node.value is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.value is None:
            print("No fresh gripper feedback within 5 seconds", file=sys.stderr)
            return 1
        print(f"{node.value:.10f}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
