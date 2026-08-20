#!/usr/bin/python3
"""Average base->AprilTag TF samples and save a guarded planning input."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time

PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR / "src"))

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from piper_pink.live_pose import average_pose_samples, quaternion_xyzw_to_matrix


JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 7))


class PoseCollector(Node):
    def __init__(self, joint_topic: str) -> None:
        super().__init__("capture_d435i_apriltag_pose")
        self.buffer = Buffer(cache_time=Duration(seconds=5.0), node=self)
        self.listener = TransformListener(self.buffer, self)
        self.joint_position = None
        self.create_subscription(
            JointState, joint_topic, self._joint_callback, qos_profile_sensor_data
        )

    def _joint_callback(self, message: JointState) -> None:
        positions = dict(zip(message.name, message.position))
        if all(name in positions for name in JOINT_NAMES):
            values = np.asarray([positions[name] for name in JOINT_NAMES], dtype=float)
            if np.all(np.isfinite(values)):
                self.joint_position = values


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-frame", default="piper_x/base_link")
    parser.add_argument("--tag-frame", default="apriltag_0")
    parser.add_argument("--joint-topic", default="/piper_x/feedback/joint_states")
    parser.add_argument("--samples", type=int, default=30)
    parser.add_argument("--timeout-s", type=float, default=12.0)
    parser.add_argument("--maximum-age-s", type=float, default=0.5)
    parser.add_argument("--maximum-translation-rms-mm", type=float, default=10.0)
    parser.add_argument("--maximum-rotation-rms-deg", type=float, default=5.0)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.samples < 3:
        raise ValueError("--samples must be at least 3")
    rclpy.init()
    node = PoseCollector(args.joint_topic)
    translations = []
    quaternions = []
    seen_stamps = set()
    raw_unique_samples = 0
    rejected_planar_branches = 0
    deadline = time.monotonic() + args.timeout_s
    try:
        while len(translations) < args.samples and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            try:
                value = node.buffer.lookup_transform(args.base_frame, args.tag_frame, Time())
            except TransformException:
                continue
            stamp_ns = value.header.stamp.sec * 1_000_000_000 + value.header.stamp.nanosec
            if stamp_ns in seen_stamps:
                continue
            age_s = (node.get_clock().now().nanoseconds - stamp_ns) / 1.0e9
            if age_s < -0.2 or age_s > args.maximum_age_s:
                continue
            seen_stamps.add(stamp_ns)
            t = value.transform.translation
            q = value.transform.rotation
            raw_unique_samples += 1
            quaternion = [q.x, q.y, q.z, q.w]
            rotation = quaternion_xyzw_to_matrix(quaternion)
            normal_z = float(np.clip(rotation[2, 2], -1.0, 1.0))
            tilt_deg = math.degrees(math.acos(normal_z))
            if tilt_deg > 45.0:
                rejected_planar_branches += 1
                continue
            translations.append([t.x, t.y, t.z])
            quaternions.append(quaternion)

        if len(translations) < args.samples:
            raise RuntimeError(
                f"Received only {len(translations)}/{args.samples} valid top-surface TF samples "
                f"from {raw_unique_samples} raw samples. "
                "Keep arm feedback, the selected camera, its hand-eye TF, and its "
                "AprilTag node running, and hold the tag fully visible."
            )
        if node.joint_position is None:
            raise RuntimeError(f"No six-joint feedback received from {args.joint_topic}")

        averaged = average_pose_samples(
            translations,
            quaternions,
            rotation_cluster_threshold_deg=5.0,
            minimum_inlier_fraction=0.5,
        )
        if averaged.translation_rms_m * 1000.0 > args.maximum_translation_rms_mm:
            raise RuntimeError(
                f"AprilTag translation is unstable: {averaged.translation_rms_m*1000:.3f} mm RMS"
            )
        if averaged.rotation_rms_deg > args.maximum_rotation_rms_deg:
            raise RuntimeError(
                f"AprilTag rotation is unstable: {averaged.rotation_rms_deg:.3f} deg RMS"
            )

        output = {
            "frame_id": "base_link",
            "object_id": "apriltag_object",
            "confidence": float(
                max(
                    0.5,
                    1.0
                    - averaged.translation_rms_m / 0.02
                    - averaged.rotation_rms_deg / 20.0,
                )
            ),
            "timestamp_s": time.time(),
            "frame_from_object": averaged.transform.tolist(),
            "current_q_deg": np.rad2deg(node.joint_position).tolist(),
            "capture": {
                "source_frame": args.base_frame,
                "tag_frame": args.tag_frame,
                "sample_count": len(translations),
                "raw_unique_sample_count": raw_unique_samples,
                "rejected_planar_branch_count": rejected_planar_branches,
                "top_surface_sample_count": len(translations),
                "orientation_inlier_count": averaged.orientation_inlier_count,
                "translation_rms_mm": averaged.translation_rms_m * 1000.0,
                "translation_max_mm": averaged.translation_max_m * 1000.0,
                "rotation_rms_deg": averaged.rotation_rms_deg,
                "rotation_max_deg": averaged.rotation_max_deg,
            },
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(output, stream, sort_keys=False)
        xyz_mm = averaged.transform[:3, 3] * 1000.0
        print(f"saved: {args.output}")
        print(f"tag_xyz_mm: {xyz_mm[0]:.3f} {xyz_mm[1]:.3f} {xyz_mm[2]:.3f}")
        print(f"translation_rms_mm: {averaged.translation_rms_m*1000.0:.3f}")
        print(f"rotation_rms_deg: {averaged.rotation_rms_deg:.3f}")
        print(
            "orientation_samples: "
            f"raw={raw_unique_samples} rejected_planar={rejected_planar_branches} "
            f"top_surface={len(translations)} "
            f"main_cluster={averaged.orientation_inlier_count}"
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
