#!/usr/bin/env python3
"""Display the D435i color stream without recording or commanding the arm."""

from __future__ import annotations

import argparse
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


WINDOW_NAME = "D435i live view - Q to close"


class D435iViewer(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("d435i_live_view")
        self.bridge = CvBridge()
        self.frame = None
        self.create_subscription(Image, topic, self.image_callback, qos_profile_sensor_data)

    def image_callback(self, message: Image) -> None:
        try:
            self.frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warning(f"D435i image conversion failed: {exc}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/sensors/d435i/color/image_raw")
    parser.add_argument("--timeout-s", type=float, default=10.0)
    args = parser.parse_args()

    rclpy.init()
    node = D435iViewer(args.topic)
    deadline = time.monotonic() + args.timeout_s
    try:
        while rclpy.ok() and node.frame is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.frame is None:
            raise RuntimeError(f"No D435i image received from {args.topic}")

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, 1280, 720)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            cv2.imshow(WINDOW_NAME, node.frame)
            if cv2.waitKey(1) & 0xFF in (ord("q"), ord("Q"), 27):
                break
        return 0
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"ERROR: {exc}", file=__import__("sys").stderr)
        raise SystemExit(1)
