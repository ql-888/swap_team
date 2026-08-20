#!/usr/bin/python3
"""Show Gemini images with live detections from two AprilTag families."""

from __future__ import annotations

from dataclasses import dataclass
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from apriltag_msgs.msg import AprilTagDetectionArray


WINDOW_NAME = "Gemini 336L - Dual AprilTag Monitor"
IMAGE_TOPIC = "/gemini336l/color/image_raw"


@dataclass
class FamilyState:
    label: str
    color: tuple[int, int, int]
    detections: tuple = ()
    updated_at: float = 0.0


class DualTagViewer(Node):
    def __init__(self) -> None:
        super().__init__("gemini336l_dual_tag_viewer")
        self.bridge = CvBridge()
        self.states = {
            "36h11": FamilyState("tag36h11", (40, 220, 40)),
            "Standard52h13": FamilyState("tagStandard52h13", (0, 180, 255)),
        }
        self.last_image_at = 0.0
        self.image_count = 0
        self.fps_started_at = time.monotonic()
        self.fps = 0.0

        self.create_subscription(
            Image, IMAGE_TOPIC, self.image_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            AprilTagDetectionArray,
            "/perception/gemini336l_live/tag36h11/detections",
            lambda message: self.detection_callback("36h11", message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            AprilTagDetectionArray,
            "/perception/gemini336l_live/standard52h13/detections",
            lambda message: self.detection_callback("Standard52h13", message),
            qos_profile_sensor_data,
        )

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, 1280, 720)
        self.get_logger().info(f"Waiting for images on {IMAGE_TOPIC}")

    def detection_callback(self, family: str, message: AprilTagDetectionArray) -> None:
        state = self.states[family]
        state.detections = tuple(message.detections)
        state.updated_at = time.monotonic()

    def image_callback(self, message: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        frame = np.ascontiguousarray(frame)
        now = time.monotonic()
        self.last_image_at = now
        self._update_fps(now)

        for state in self.states.values():
            active = now - state.updated_at <= 0.75
            detections = state.detections if active else ()
            for detection in detections:
                self._draw_detection(frame, detection, state)

        self._draw_status(frame, now)
        cv2.imshow(WINDOW_NAME, frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            rclpy.shutdown()

    def _update_fps(self, now: float) -> None:
        self.image_count += 1
        elapsed = now - self.fps_started_at
        if elapsed >= 1.0:
            self.fps = self.image_count / elapsed
            self.image_count = 0
            self.fps_started_at = now

    @staticmethod
    def _draw_detection(frame, detection, state: FamilyState) -> None:
        corners = np.asarray(
            [[point.x, point.y] for point in detection.corners], dtype=np.int32
        ).reshape((-1, 1, 2))
        cv2.polylines(frame, [corners], True, state.color, 3, cv2.LINE_AA)
        centre = (int(round(detection.centre.x)), int(round(detection.centre.y)))
        cv2.circle(frame, centre, 5, state.color, -1, cv2.LINE_AA)
        anchor = tuple(corners.reshape((-1, 2)).min(axis=0))
        text = (
            f"{state.label}  ID {detection.id}  "
            f"H {detection.hamming}  M {detection.decision_margin:.1f}"
        )
        DualTagViewer._outlined_text(
            frame, text, (int(anchor[0]), max(28, int(anchor[1]) - 10)), 0.72, state.color
        )

    def _draw_status(self, frame, now: float) -> None:
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (frame.shape[1], 106), (18, 18, 18), -1)
        cv2.addWeighted(overlay, 0.78, frame, 0.22, 0.0, frame)
        self._outlined_text(frame, f"Gemini 336L   {self.fps:.1f} FPS", (18, 30), 0.75, (255, 255, 255))

        x = 18
        for state in self.states.values():
            active = now - state.updated_at <= 0.75
            detections = state.detections if active else ()
            if detections:
                ids = ",".join(str(item.id) for item in detections)
                text = f"{state.label}: DETECTED  ID {ids}"
                color = state.color
            else:
                text = f"{state.label}: SEARCHING"
                color = (60, 80, 255)
            self._outlined_text(frame, text, (x, 76), 0.72, color)
            x += max(390, cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.72, 2)[0][0] + 45)

    @staticmethod
    def _outlined_text(frame, text, origin, scale, color) -> None:
        cv2.putText(
            frame, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 5, cv2.LINE_AA
        )
        cv2.putText(
            frame, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 2, cv2.LINE_AA
        )

    def destroy_node(self) -> bool:
        cv2.destroyAllWindows()
        return super().destroy_node()


def main() -> int:
    rclpy.init()
    node = DualTagViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
