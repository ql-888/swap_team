#!/usr/bin/env python3
"""Build an eye-in-grasp-compatible recipe whose TCP is above the tag."""

import argparse
from pathlib import Path

import numpy as np
import yaml


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pose", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--height-m", type=float, default=0.15)
    args = parser.parse_args()
    with args.pose.open(encoding="utf-8") as stream:
        pose = yaml.safe_load(stream)
    base_from_object = np.asarray(pose["frame_from_object"], dtype=float)
    if base_from_object.shape != (4, 4):
        raise ValueError("frame_from_object must be a 4x4 transform")
    local_translation = base_from_object[:3, :3].T @ np.array([0.0, 0.0, args.height_m])
    recipe = {
        "object_id": "apriltag_object",
        "object_from_tcp": [
            [1.0, 0.0, 0.0, float(local_translation[0])],
            [0.0, -0.8660254037844386, 0.5, float(local_translation[1])],
            [0.0, -0.5, -0.8660254037844386, float(local_translation[2])],
            [0.0, 0.0, 0.0, 1.0],
        ],
        "axial_angles_deg": [0.0],
        "pregrasp_distance_m": 0.001,
        "retreat_distance_m": 0.001,
        "gripper_opening_m": 0.085,
        "gripper_closed_m": 0.058,
        "execution_ready": False,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(recipe, stream, sort_keys=False)
    print(f"saved: {args.output}")
    print("target_offset_world_z_m: %.3f" % args.height_m)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
