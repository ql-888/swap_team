#!/usr/bin/python3
"""Print one fresh Piper ROS joint sample in degrees, one value per line."""

from __future__ import annotations

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 7))


class JointReader(Node):
    def __init__(self) -> None:
        super().__init__("piper_grasp_axis_joint_reader")
        self.q = None
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            JointState, "/piper_x/feedback/joint_states", self._callback, qos
        )

    def _callback(self, message: JointState) -> None:
        positions = dict(zip(message.name, message.position))
        if all(name in positions for name in JOINT_NAMES):
            self.q = [float(positions[name]) for name in JOINT_NAMES]


def main() -> int:
    rclpy.init()
    node = JointReader()
    try:
        deadline = time.monotonic() + 5.0
        while node.q is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.q is None:
            print("No fresh /piper_x/feedback/joint_states message within 5 seconds", file=sys.stderr)
            return 1
        for value in node.q:
            print(f"{math.degrees(value):.10f}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
