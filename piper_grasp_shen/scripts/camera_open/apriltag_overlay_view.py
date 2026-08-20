#!/usr/bin/env python3
import argparse

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from apriltag_msgs.msg import AprilTagDetectionArray
from sensor_msgs.msg import Image


class AprilTagOverlay(Node):
    def __init__(self, image_topic: str, detections_topic: str, result_topic: str):
        super().__init__("apriltag_overlay_view")
        self.bridge = CvBridge()
        self.detections = ()
        self.image_pub = self.create_publisher(Image, result_topic, 10)
        self.create_subscription(
            AprilTagDetectionArray, detections_topic, self.detections_callback, qos_profile_sensor_data
        )
        self.create_subscription(Image, image_topic, self.image_callback, qos_profile_sensor_data)

    def detections_callback(self, message):
        self.detections = tuple(message.detections)

    def image_callback(self, message):
        image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        for detection in self.detections:
            points = [(int(corner.x), int(corner.y)) for corner in detection.corners]
            if len(points) == 4:
                cv2.polylines(image, [np.array(points)], True, (0, 255, 0), 2)
            cv2.putText(
                image,
                f"id={detection.id}",
                (int(detection.centre.x), int(detection.centre.y)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, encoding="bgr8"))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image-topic", required=True)
    parser.add_argument("--detections-topic", required=True)
    parser.add_argument("--result-topic", required=True)
    args = parser.parse_args()
    rclpy.init()
    node = AprilTagOverlay(args.image_topic, args.detections_topic, args.result_topic)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
