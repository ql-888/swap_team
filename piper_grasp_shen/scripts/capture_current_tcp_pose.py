#!/usr/bin/env python3
"""Capture the current ROS joint state and TCP pose for transport planning."""

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import yaml

PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR / "src"))

from piper_pink.config import DEFAULT_TCP, find_default_urdf
from piper_pink.model import load_piper_model


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    reader = PROJECT_DIR / "scripts/read_ros_joint_degrees.py"
    values = subprocess.check_output([str(reader)], text=True).split()
    if len(values) != 6:
        raise RuntimeError("Failed to read six joint angles")
    q_deg = np.asarray([float(value) for value in values], dtype=float)
    model = load_piper_model(find_default_urdf(), DEFAULT_TCP)
    q = np.deg2rad(q_deg)
    pose = model.forward_tcp(q)
    output = {
        "schema_version": 1,
        "timestamp_s": time.time(),
        "frame_id": "base_link",
        "tcp_frame": DEFAULT_TCP.frame_name,
        "current_q_deg": q_deg.tolist(),
        "base_from_tcp": pose.homogeneous.tolist(),
        "tcp_xyz_m": pose.translation.tolist(),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(output, stream, sort_keys=False)
    print(f"saved: {args.output}")
    print("tcp_xyz_mm: " + " ".join(f"{x * 1000.0:.3f}" for x in pose.translation))
    print("current_q_deg: " + " ".join(f"{x:.3f}" for x in q_deg))
    print("Read-only capture; no motion command was sent.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError, OSError, subprocess.SubprocessError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
