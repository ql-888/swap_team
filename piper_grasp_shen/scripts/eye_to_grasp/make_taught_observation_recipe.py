#!/usr/bin/env python3
"""Convert a taught base/flange observation into a reusable grasp recipe."""
import argparse
from pathlib import Path
import math
import numpy as np
import yaml


def qmat(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]], dtype=float)


def tf(item):
    out = np.eye(4)
    out[:3, :3] = qmat(item["quaternion_xyzw"])
    out[:3, 3] = item["translation_m"]
    return out


def inv(a):
    out = np.eye(4)
    out[:3, :3] = a[:3, :3].T
    out[:3, 3] = -out[:3, :3] @ a[:3, 3]
    return out


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--global-pose", type=Path, required=True)
    p.add_argument("--record", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    pose = yaml.safe_load(a.global_pose.read_text())
    record = yaml.safe_load(a.record.read_text())["samples"][0]
    base_tag = np.asarray(pose["frame_from_object"], dtype=float)
    taught_base_tag = tf(record["base_from_global_tag"])
    taught_base_flange = tf(record["base_from_flange"])
    # Measured Piper TCP used by the eye-in-grasp controller.
    flange_tcp = np.eye(4)
    yaw = math.radians(48.2)
    flange_tcp[:3, :3] = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                                   [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    flange_tcp[2, 3] = 0.198
    taught_tag_flange = inv(taught_base_tag) @ taught_base_flange
    desired_base_flange = base_tag @ taught_tag_flange
    desired_base_tcp = desired_base_flange @ flange_tcp
    object_from_tcp = inv(base_tag) @ desired_base_tcp
    recipe = {
        "object_id": "apriltag_object",
        "object_from_tcp": object_from_tcp.tolist(),
        "axial_angles_deg": [0.0],
        "pregrasp_distance_m": 0.001,
        "retreat_distance_m": 0.001,
        "gripper_opening_m": 0.085,
        "gripper_closed_m": 0.058,
        "execution_ready": False,
        "observation_offset_source": str(a.record),
    }
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(yaml.safe_dump(recipe, sort_keys=False))
    print(f"saved: {a.output}")
    print("taught_tag_to_flange_translation_m:", taught_tag_flange[:3, 3])


if __name__ == "__main__":
    main()
