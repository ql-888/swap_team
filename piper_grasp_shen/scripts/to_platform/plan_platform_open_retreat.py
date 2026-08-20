#!/usr/bin/env python3
"""Plan a vertical TCP retreat after the drone has been partially released."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time

import numpy as np
import yaml

PROJECT_DIR = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_DIR / "src"))

from piper_pink.collision import PiperSelfCollisionChecker, WorkspaceLimits
from piper_pink.config import DEFAULT_TCP, find_default_urdf
from piper_pink.execution import interpolate_joint_path
from piper_pink.ik import PinkIkSolver
from piper_pink.model import load_piper_model


def build_open_retreat_target(
    current_tcp: np.ndarray, retreat_m: float
) -> np.ndarray:
    target = np.asarray(current_tcp, dtype=float).copy()
    target[2, 3] += retreat_m
    return target


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--current-q-deg", nargs=6, type=float, required=True)
    parser.add_argument("--retreat-m", type=float, default=0.120)
    parser.add_argument("--open-width-m", type=float, required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    if len(args.current_q_deg) != 6:
        raise ValueError("--current-q-deg must contain six values")
    if not 0.010 <= args.retreat_m <= 0.180:
        raise ValueError("--retreat-m must be between 0.010 and 0.180 m")
    if not 0.065 <= args.open_width_m <= 0.085:
        raise ValueError("--open-width-m must be between 0.065 and 0.085 m")

    current_q = np.deg2rad(np.asarray(args.current_q_deg, dtype=float))
    urdf = find_default_urdf()
    model = load_piper_model(urdf, DEFAULT_TCP)
    current_tcp = model.forward_tcp(current_q).homogeneous
    target_tcp = build_open_retreat_target(current_tcp, args.retreat_m)
    target_pose = model.pose_from_matrix(target_tcp)
    WorkspaceLimits(0.65, 0.02).validate_pose(target_pose)
    solver = PinkIkSolver(model)
    result = solver.solve(
        target_pose, current_q, extra_seeds=(model.neutral()[:6],)
    )
    if not result.converged:
        raise RuntimeError(
            f"No IK solution for the {args.retreat_m * 1000.0:.0f} mm open retreat"
        )
    margin_deg = math.degrees(
        float(
            np.min(
                np.minimum(
                    result.q - model.lower_limits,
                    model.upper_limits - result.q,
                )
            )
        )
    )
    if margin_deg < 2.0:
        raise RuntimeError(
            f"Open retreat target is only {margin_deg:.3f} deg from a joint limit"
        )
    path = interpolate_joint_path(current_q, result.q, math.radians(0.25))
    collision = PiperSelfCollisionChecker(
        urdf,
        PROJECT_DIR / "assets/piper_x_moveit/piper_x.srdf",
        PROJECT_DIR / "config/scene.yaml",
    )
    collision.validate_path(path)
    report = {
        "schema_version": 1,
        "created_at_unix_s": time.time(),
        "mode": "platform_open_retreat_debug",
        "inputs": {
            "current_q_deg": [float(value) for value in args.current_q_deg],
            "retreat_m": args.retreat_m,
            "open_width_m": args.open_width_m,
            "orientation_policy": "preserve current TCP XY and rotation; move along base +Z",
        },
        "stages": {
            "retreat_open": {
                "target": target_tcp.tolist(),
                "q_deg": np.rad2deg(result.q).tolist(),
                "position_error_mm": result.position_error_m * 1000.0,
                "orientation_error_deg": math.degrees(result.orientation_error_rad),
                "minimum_joint_limit_margin_deg": margin_deg,
                "path_waypoints": len(path),
            }
        },
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    with args.report.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(report, stream, sort_keys=False)
    print(
        "target_tcp_xyz_mm: "
        + " ".join(f"{value * 1000.0:.3f}" for value in target_tcp[:3, 3])
    )
    print("target_q_deg: " + " ".join(f"{x:.4f}" for x in np.rad2deg(result.q)))
    print(f"path_waypoints: {len(path)}")
    print(f"minimum_joint_limit_margin_deg: {margin_deg:.3f}")
    print(f"open_width_m: {args.open_width_m:.4f}")
    print(f"report: {args.report}")
    print("Offline open-retreat validation only. No command was published.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError, OSError, KeyError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
