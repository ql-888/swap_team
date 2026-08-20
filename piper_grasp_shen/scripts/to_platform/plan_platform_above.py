#!/usr/bin/env python3
"""Plan a held-drone pose above a Standard52h13 platform tag.

The tag translation is used only for XY. Target Z is fixed from the measured
platform height and requested object clearance. The drone/object frame follows
the tag X axis in base XY, then the calibrated grasp transform determines TCP.
"""

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


def orthonormalize(rotation: np.ndarray) -> np.ndarray:
    u, _, vh = np.linalg.svd(rotation)
    value = u @ vh
    if np.linalg.det(value) < 0.0:
        u[:, -1] *= -1.0
        value = u @ vh
    return value


def tag_aligned_horizontal_rotation(tag_rotation: np.ndarray, tcp_rotation: np.ndarray):
    """Return a horizontal object frame and the tag X heading in base XY."""
    tag_x = np.asarray(tag_rotation, dtype=float)[:, 0].copy()
    tag_x[2] = 0.0
    norm = float(np.linalg.norm(tag_x))
    if norm < 1e-8:
        raise ValueError("Platform tag X axis cannot be projected into base XY")
    tag_x /= norm
    z_sign = 1.0 if float(np.asarray(tcp_rotation, dtype=float)[2, 2]) >= 0.0 else -1.0
    z_axis = np.array([0.0, 0.0, z_sign])
    y_axis = np.cross(z_axis, tag_x)
    y_axis /= np.linalg.norm(y_axis)
    rotation = orthonormalize(np.column_stack((tag_x, y_axis, z_axis)))
    yaw_deg = math.degrees(math.atan2(float(tag_x[1]), float(tag_x[0])))
    return rotation, yaw_deg


def rotation_about_x(angle_deg: float) -> np.ndarray:
    angle = math.radians(angle_deg)
    cosine, sine = math.cos(angle), math.sin(angle)
    return np.array(
        [[1.0, 0.0, 0.0], [0.0, cosine, -sine], [0.0, sine, cosine]]
    )


def build_preplacement_targets(
    base_from_tag: np.ndarray,
    current_tcp: np.ndarray,
    object_from_tcp: np.ndarray,
    platform_height_m: float,
    object_height_above_tag_m: float,
    lift_waypoint_m: float,
    transport_tilt_deg: float,
    alignment_tilt_deg: float,
) -> dict[str, np.ndarray]:
    """Build TCP targets for lift, position transport, and final alignment."""

    target_z = platform_height_m + object_height_above_tag_m
    lift_tcp = current_tcp.copy()
    lift_tcp[2, 3] += lift_waypoint_m

    horizontal_rotation, _ = tag_aligned_horizontal_rotation(
        base_from_tag[:3, :3], np.eye(3)
    )
    transport_object = np.eye(4)
    transport_object[:3, :3] = horizontal_rotation @ rotation_about_x(
        transport_tilt_deg
    )
    transport_object[:2, 3] = base_from_tag[:2, 3]
    transport_object[2, 3] = target_z

    align_object = np.eye(4)
    align_object[:3, :3] = horizontal_rotation @ rotation_about_x(
        alignment_tilt_deg
    )
    align_object[:2, 3] = base_from_tag[:2, 3]
    align_object[2, 3] = target_z

    return {
        "lift": lift_tcp,
        "transport": transport_object @ object_from_tcp,
        "align": align_object @ object_from_tcp,
    }


def solve_target(solver, model, target, seeds, label):
    for seed in seeds:
        result = solver.solve(target, seed)
        if result.converged:
            return result
    raise RuntimeError(f"No IK solution for platform stage: {label}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--target-pose", type=Path, required=True)
    parser.add_argument("--current-q-deg", nargs=6, type=float, required=True)
    parser.add_argument("--platform-height-m", type=float, default=0.170)
    parser.add_argument("--object-height-above-tag-m", type=float, default=0.200)
    parser.add_argument("--max-tag-tilt-deg", type=float, default=10.0)
    parser.add_argument("--lift-waypoint-m", type=float, default=0.100)
    parser.add_argument("--transport-tilt-deg", type=float, default=30.0)
    parser.add_argument("--alignment-tilt-deg", type=float, default=20.0)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    if len(args.current_q_deg) != 6:
        raise ValueError("--current-q-deg must contain six values")
    if not 0.10 <= args.platform_height_m <= 0.40:
        raise ValueError("--platform-height-m must be between 0.10 and 0.40 m")
    if not 0.05 <= args.object_height_above_tag_m <= 0.40:
        raise ValueError("--object-height-above-tag-m must be between 0.05 and 0.40 m")
    if not 0.0 <= args.max_tag_tilt_deg <= 45.0:
        raise ValueError("--max-tag-tilt-deg must be between 0 and 45 degrees")
    for name, value in (
        ("--transport-tilt-deg", args.transport_tilt_deg),
        ("--alignment-tilt-deg", args.alignment_tilt_deg),
    ):
        if not 0.0 <= value <= 60.0:
            raise ValueError(f"{name} must be between 0 and 60 degrees")

    with args.target_pose.open(encoding="utf-8") as stream:
        captured = yaml.safe_load(stream)
    if captured.get("target_family") != "Standard52h13" or captured.get("target_id") != 0:
        raise ValueError("Captured target must be Standard52h13 ID 0")
    if abs(float(captured.get("tag_size_m", 0.0)) - 0.0219) > 1e-9:
        raise ValueError("Captured target does not use the converted 21.9 mm pose size")
    if time.time() - float(captured["timestamp_s"]) > 30.0:
        raise ValueError("Captured platform target is stale; capture it again")

    base_from_tag = np.asarray(captured["base_from_tag"], dtype=float)
    if base_from_tag.shape != (4, 4):
        raise ValueError("Captured target must contain a 4x4 base_from_tag matrix")
    tag_normal_z = float(np.clip(base_from_tag[2, 2], -1.0, 1.0))
    tag_tilt_deg = math.degrees(math.acos(abs(tag_normal_z)))
    if tag_tilt_deg > args.max_tag_tilt_deg:
        raise ValueError(f"Platform tag is tilted {tag_tilt_deg:.2f} deg; expected horizontal")

    current_q = np.deg2rad(np.asarray(args.current_q_deg, dtype=float))
    urdf = find_default_urdf()
    model = load_piper_model(urdf, DEFAULT_TCP)
    current_tcp = model.forward_tcp(current_q)
    recipe = GraspRecipe.load(PROJECT_DIR / "config/grasp_recipe_drone_apriltag_top.yaml")
    _, target_yaw_deg = tag_aligned_horizontal_rotation(
        base_from_tag[:3, :3], np.eye(3)
    )
    targets = build_preplacement_targets(
        base_from_tag,
        current_tcp.homogeneous,
        recipe.object_from_tcp,
        args.platform_height_m,
        args.object_height_above_tag_m,
        args.lift_waypoint_m,
        args.transport_tilt_deg,
        args.alignment_tilt_deg,
    )
    target_poses = {
        name: model.pose_from_matrix(matrix) for name, matrix in targets.items()
    }
    workspace = WorkspaceLimits(0.65, 0.02)
    for pose in target_poses.values():
        workspace.validate_pose(pose)

    solver = PinkIkSolver(model)
    lift_result = solve_target(
        solver, model, target_poses["lift"], (current_q, model.neutral()[:6]), "lift"
    )
    transport_result = solve_target(
        solver,
        model,
        target_poses["transport"],
        (lift_result.q, current_q, model.neutral()[:6]),
        "transport",
    )
    align_result = solve_target(
        solver,
        model,
        target_poses["align"],
        (transport_result.q, lift_result.q, current_q, model.neutral()[:6]),
        "tilted_align",
    )
    paths = {
        "lift": interpolate_joint_path(current_q, lift_result.q, math.radians(0.5)),
        "transport": interpolate_joint_path(
            lift_result.q, transport_result.q, math.radians(0.5)
        ),
        "align": interpolate_joint_path(
            transport_result.q, align_result.q, math.radians(0.5)
        ),
    }

    collision = PiperSelfCollisionChecker(
        urdf,
        PROJECT_DIR / "assets/piper_x_moveit/piper_x.srdf",
        PROJECT_DIR / "config/scene.yaml",
    )
    for path in paths.values():
        collision.validate_path(path)
    fixed_object_z_m = args.platform_height_m + args.object_height_above_tag_m
    results = {
        "lift": lift_result,
        "transport": transport_result,
        "align": align_result,
    }
    joint_margins_deg = {
        name: math.degrees(
            float(
                np.min(
                    np.minimum(
                        result.q - model.lower_limits,
                        model.upper_limits - result.q,
                    )
                )
            )
        )
        for name, result in results.items()
    }
    for name, margin_deg in joint_margins_deg.items():
        if margin_deg < 2.0:
            raise RuntimeError(
                f"Platform stage {name} is only {margin_deg:.3f} deg from a joint limit"
            )
    report = {
        "schema_version": 1,
        "created_at_unix_s": time.time(),
        "mode": "platform_preplacement_debug",
        "inputs": {
            "target_pose": str(args.target_pose),
            "current_q_deg": [float(value) for value in args.current_q_deg],
            "platform_height_m": args.platform_height_m,
            "object_height_above_tag_m": args.object_height_above_tag_m,
            "fixed_object_z_m": fixed_object_z_m,
            "measured_tag_z_ignored": True,
            "tag_tilt_deg": tag_tilt_deg,
            "target_yaw_deg": target_yaw_deg,
            "transport_tilt_deg": args.transport_tilt_deg,
            "alignment_tilt_deg": args.alignment_tilt_deg,
            "orientation_policy": (
                f"transport is tag-aligned with a {args.transport_tilt_deg:.1f} "
                "degree X tilt; align reduces to the selected "
                f"{args.alignment_tilt_deg:.1f} degree X tilt"
            ),
            "payload_collision_model": "not_modeled; operator must clear held-drone path",
        },
        "stages": {
            name: {
                "target": targets[name].tolist(),
                "q_deg": np.rad2deg(results[name].q).tolist(),
                "position_error_mm": results[name].position_error_m * 1000.0,
                "orientation_error_deg": math.degrees(
                    results[name].orientation_error_rad
                ),
                "minimum_joint_limit_margin_deg": joint_margins_deg[name],
                "path_waypoints": len(paths[name]),
            }
            for name in ("lift", "transport", "align")
        },
        "object_target": (
            targets["align"] @ np.linalg.inv(recipe.object_from_tcp)
        ).tolist(),
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    with args.report.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(report, stream, sort_keys=False)
    print(f"fixed_object_z_mm: {fixed_object_z_m * 1000.0:.3f}")
    print(f"platform_tag_tilt_deg: {tag_tilt_deg:.3f}")
    print(f"platform_target_yaw_deg: {target_yaw_deg:.3f}")
    for name in ("lift", "transport", "align"):
        print(
            f"{name}_q_deg: "
            + " ".join(f"{x:.4f}" for x in np.rad2deg(results[name].q))
        )
        print(f"{name}_path_waypoints: {len(paths[name])}")
        print(
            f"{name}_minimum_joint_limit_margin_deg: "
            f"{joint_margins_deg[name]:.3f}"
        )
    print(f"report: {args.report}")
    print("Offline validation only. No robot command was published.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError, OSError, KeyError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
