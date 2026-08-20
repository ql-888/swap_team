#!/usr/bin/python3
"""Average the full base-frame pose of Gemini Standard52h13 ID 0."""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR / "src"))

from piper_pink.live_pose import average_pose_samples, quaternion_xyzw_to_matrix


class Collector(Node):
    def __init__(self) -> None:
        super().__init__("capture_gemini_standard52_target")
        self.buffer = Buffer(cache_time=Duration(seconds=5.0), node=self)
        self.listener = TransformListener(self.buffer, self)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-frame", default="piper_x/base_link")
    parser.add_argument("--tag-frame", default="gemini_standard52h13_0")
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--timeout-s", type=float, default=12.0)
    parser.add_argument("--maximum-age-s", type=float, default=0.5)
    parser.add_argument("--maximum-rms-mm", type=float, default=10.0)
    parser.add_argument("--maximum-rotation-rms-deg", type=float, default=5.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.samples < 5:
        raise ValueError("--samples must be at least 5")

    rclpy.init()
    node = Collector()
    translations = []
    quaternions = []
    rejected_planar_branches = 0
    stamps = set()
    deadline = time.monotonic() + args.timeout_s
    try:
        while len(translations) < args.samples and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            try:
                value = node.buffer.lookup_transform(args.base_frame, args.tag_frame, Time())
            except TransformException:
                continue
            stamp_ns = value.header.stamp.sec * 1_000_000_000 + value.header.stamp.nanosec
            if stamp_ns in stamps:
                continue
            age = (node.get_clock().now().nanoseconds - stamp_ns) / 1.0e9
            if age < -0.2 or age > args.maximum_age_s:
                continue
            stamps.add(stamp_ns)
            t = value.transform.translation
            q = value.transform.rotation
            quaternion = [q.x, q.y, q.z, q.w]
            normal_z = float(np.clip(quaternion_xyzw_to_matrix(quaternion)[2, 2], -1.0, 1.0))
            if math.degrees(math.acos(normal_z)) > 45.0:
                rejected_planar_branches += 1
                continue
            translations.append([t.x, t.y, t.z])
            quaternions.append(quaternion)
        if len(translations) != args.samples:
            raise RuntimeError(
                f"Received only {len(translations)}/{args.samples} valid target samples from "
                f"{args.tag_frame}"
            )
        averaged = average_pose_samples(
            translations,
            quaternions,
            rotation_cluster_threshold_deg=5.0,
            minimum_inlier_fraction=0.7,
        )
        rms_mm = averaged.translation_rms_m * 1000.0
        if rms_mm > args.maximum_rms_mm:
            raise RuntimeError(f"Target translation is unstable: {rms_mm:.3f} mm RMS")
        if averaged.rotation_rms_deg > args.maximum_rotation_rms_deg:
            raise RuntimeError(
                f"Target rotation is unstable: {averaged.rotation_rms_deg:.3f} deg RMS"
            )
        output = {
            "frame_id": "base_link",
            "target_family": "Standard52h13",
            "target_id": 0,
            "tag_size_m": 0.0219,
            "measured_outer_edge_m": 0.0365,
            "size_conversion": "36.5 mm outer edge * width_at_border 6 / total_width 10",
            "timestamp_s": time.time(),
            "base_from_tag": averaged.transform.tolist(),
            "tag_xyz_m": averaged.transform[:3, 3].tolist(),
            "sample_count": args.samples,
            "orientation_inlier_count": averaged.orientation_inlier_count,
            "rejected_planar_branch_count": rejected_planar_branches,
            "translation_rms_mm": rms_mm,
            "translation_max_mm": averaged.translation_max_m * 1000.0,
            "rotation_rms_deg": averaged.rotation_rms_deg,
            "rotation_max_deg": averaged.rotation_max_deg,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(output, stream, sort_keys=False)
        print(f"saved: {args.output}")
        print(
            "target_tag_xyz_mm: "
            + " ".join(f"{x*1000.0:.3f}" for x in averaged.transform[:3, 3])
        )
        print(f"translation_rms_mm: {rms_mm:.3f}")
        print(f"rotation_rms_deg: {averaged.rotation_rms_deg:.3f}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError) as exc:
        print(f"ERROR: {exc}", file=__import__("sys").stderr)
        raise SystemExit(1)
