#!/usr/bin/env python3
"""Plan a single vertical descent while preserving the held-drone pose."""

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
from piper_pink.recipe import GraspRecipe


def build_descent_target(
    current_tcp: np.ndarray,
    object_from_tcp: np.ndarray,
    descent_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Lower the held object along base -Z without changing XY or rotation."""

    current_object = np.asarray(current_tcp, dtype=float) @ np.linalg.inv(
        np.asarray(object_from_tcp, dtype=float)
    )
    target_object = current_object.copy()
    target_object[2, 3] -= descent_m
    return target_object @ object_from_tcp, target_object


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--current-q-deg", nargs=6, type=float, required=True)
    parser.add_argument("--descent-m", type=float, default=0.100)
    parser.add_argument(
        "--stage-name", choices=("descend", "descend10"), default="descend"
    )
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    if len(args.current_q_deg) != 6:
        raise ValueError("--current-q-deg must contain six values")
    if not 0.005 <= args.descent_m <= 0.150:
        raise ValueError("--descent-m must be between 0.005 and 0.150 m")

    current_q = np.deg2rad(np.asarray(args.current_q_deg, dtype=float))
    urdf = find_default_urdf()
    model = load_piper_model(urdf, DEFAULT_TCP)
    recipe = GraspRecipe.load(
        PROJECT_DIR / "config/grasp_recipe_drone_apriltag_top.yaml"
    )
    current_tcp = model.forward_tcp(current_q).homogeneous
    target_tcp, target_object = build_descent_target(
        current_tcp, recipe.object_from_tcp, args.descent_m
    )
    target_pose = model.pose_from_matrix(target_tcp)
    WorkspaceLimits(0.65, 0.02).validate_pose(target_pose)

    solver = PinkIkSolver(model)
    result = solver.solve(
        target_pose, current_q, extra_seeds=(model.neutral()[:6],)
    )
    if not result.converged:
        raise RuntimeError(
            f"No IK solution for the {args.descent_m * 1000.0:.0f} mm vertical descent"
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
            f"Descent target is only {margin_deg:.3f} deg from a joint limit"
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
        "mode": "platform_descent_debug",
        "inputs": {
            "current_q_deg": [float(value) for value in args.current_q_deg],
            "descent_m": args.descent_m,
            "orientation_policy": "preserve current object XY, rotation, and gripper hold",
            "payload_platform_contact_model": (
                "not modeled; operator visually confirmed 100 mm landing clearance"
            ),
        },
        "stages": {
            args.stage_name: {
                "target": target_tcp.tolist(),
                "q_deg": np.rad2deg(result.q).tolist(),
                "position_error_mm": result.position_error_m * 1000.0,
                "orientation_error_deg": math.degrees(result.orientation_error_rad),
                "minimum_joint_limit_margin_deg": margin_deg,
                "path_waypoints": len(path),
            }
        },
        "object_target": target_object.tolist(),
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    with args.report.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(report, stream, sort_keys=False)
    print(
        "target_object_xyz_mm: "
        + " ".join(f"{value * 1000.0:.3f}" for value in target_object[:3, 3])
    )
    print("target_q_deg: " + " ".join(f"{x:.4f}" for x in np.rad2deg(result.q)))
    print(f"path_waypoints: {len(path)}")
    print(f"minimum_joint_limit_margin_deg: {margin_deg:.3f}")
    print(f"report: {args.report}")
    print("Offline descent validation only. No robot command was published.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError, OSError, KeyError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
