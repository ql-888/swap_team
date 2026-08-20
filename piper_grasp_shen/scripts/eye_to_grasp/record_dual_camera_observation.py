#!/usr/bin/env python3
"""Record a manually taught D435i observation pose without commanding the arm."""

from __future__ import annotations

import argparse
from pathlib import Path
import threading
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from sensor_msgs.msg import JointState
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2


JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))


def transform_dict(value):
    t = value.transform.translation
    q = value.transform.rotation
    return {
        "parent_frame": value.header.frame_id,
        "child_frame": value.child_frame_id,
        "translation_m": [float(t.x), float(t.y), float(t.z)],
        "quaternion_xyzw": [float(q.x), float(q.y), float(q.z), float(q.w)],
        "stamp_ns": int(value.header.stamp.sec * 1_000_000_000 + value.header.stamp.nanosec),
    }


class Recorder(Node):
    def __init__(self, args):
        super().__init__("record_dual_camera_observation")
        self.args = args
        self.tf = Buffer(cache_time=Duration(seconds=10.0), node=self)
        self.listener = TransformListener(self.tf, self)
        self.joints = None
        self.joint_stamp_ns = None
        self.bridge = CvBridge()
        self.global_image = None
        self.local_image = None
        self.create_subscription(Image, args.global_image_topic, self.global_image_callback, 10)
        self.create_subscription(Image, args.local_image_topic, self.local_image_callback, 10)
        self.record_requested = threading.Event()
        self.stop_requested = threading.Event()
        self.create_subscription(JointState, args.joint_topic, self.joint_callback, 10)

    def joint_callback(self, message):
        positions = dict(zip(message.name, message.position))
        if all(name in positions for name in JOINT_NAMES):
            self.joints = [float(positions[name]) for name in JOINT_NAMES]
            self.joint_stamp_ns = int(
                message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            )

    def global_image_callback(self, message):
        try:
            self.global_image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warning(f"Gemini image conversion failed: {exc}")

    def local_image_callback(self, message):
        try:
            self.local_image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warning(f"D435i image conversion failed: {exc}")

    def lookup(self, parent, child):
        try:
            return self.tf.lookup_transform(parent, child, Time())
        except TransformException:
            return None

    def sample(self, require_local_tag=True):
        global_tag = self.lookup(self.args.base_frame, self.args.global_tag_frame)
        flange = self.lookup(self.args.base_frame, self.args.flange_frame)
        local_tag = None
        if require_local_tag:
            local_tag = self.lookup(self.args.local_camera_frame, self.args.local_tag_frame)
        if not (global_tag and flange and self.joints):
            return None
        if require_local_tag and local_tag is None:
            return None
        sample = {
            "timestamp_s": time.time(),
            "joints_rad": list(self.joints),
            "joint_stamp_ns": self.joint_stamp_ns,
            "base_from_global_tag": transform_dict(global_tag),
            "base_from_flange": transform_dict(flange),
        }
        if local_tag is not None:
            sample["local_camera_from_tag"] = transform_dict(local_tag)
        return sample


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--samples", type=int, default=10)
    parser.add_argument("--base-frame", default="piper_x/base_link")
    parser.add_argument("--flange-frame", default="piper_x/flange_link")
    parser.add_argument("--global-tag-frame", default="gemini_apriltag_0")
    parser.add_argument("--local-camera-frame", default="d435i_color_optical_frame")
    parser.add_argument("--local-tag-frame", default="d435i_apriltag_0")
    parser.add_argument("--joint-topic", default="/piper_x/feedback/joint_states")
    parser.add_argument("--global-image-topic", default="/gemini336l/color/image_raw")
    parser.add_argument("--local-image-topic", default="/sensors/d435i/color/image_raw")
    args = parser.parse_args()
    if args.samples < 3:
        raise ValueError("--samples must be at least 3")

    rclpy.init()
    node = Recorder(args)

    def wait_for_enter():
        input("Move the arm manually until D435i sees the tag, then press ENTER to record.\n")
        node.record_requested.set()

    threading.Thread(target=wait_for_enter, daemon=True).start()
    samples = []
    try:
        while rclpy.ok() and not node.stop_requested.is_set():
            rclpy.spin_once(node, timeout_sec=0.05)
            if not node.record_requested.is_set():
                continue
            sample = node.sample()
            if sample is not None:
                if node.global_image is None or node.local_image is None:
                    print("waiting for both camera images", flush=True)
                    time.sleep(0.05)
                    continue
                image_dir = args.output.parent / (args.output.stem + "_images")
                image_dir.mkdir(parents=True, exist_ok=True)
                index = len(samples)
                global_path = image_dir / f"{index:03d}_gemini336l.png"
                local_path = image_dir / f"{index:03d}_d435i.png"
                cv2.imwrite(str(global_path), node.global_image)
                cv2.imwrite(str(local_path), node.local_image)
                sample["global_image"] = str(global_path)
                sample["local_image"] = str(local_path)
                samples.append(sample)
                print(f"recorded {len(samples)}/{args.samples}", flush=True)
            if len(samples) >= args.samples:
                output = {
                    "schema_version": 1,
                    "camera_global": "Orbbec Gemini 336L",
                    "camera_local": "Intel RealSense D435i",
                    "tag_family": "tag36h11",
                    "tag_id": 0,
                    "samples": samples,
                    "notes": "Manually taught local-camera observation pose; no motion commands issued.",
                }
                args.output.parent.mkdir(parents=True, exist_ok=True)
                with args.output.open("w", encoding="utf-8") as stream:
                    yaml.safe_dump(output, stream, sort_keys=False)
                print(f"saved: {args.output}")
                return 0
            time.sleep(0.05)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
