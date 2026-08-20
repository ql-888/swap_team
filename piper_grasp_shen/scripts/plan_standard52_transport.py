#!/usr/bin/env python3
"""Plan a collision-checked move above a captured Standard52h13 target."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time

import numpy as np
import yaml

PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR / "src"))

from piper_pink.collision import PiperSelfCollisionChecker, WorkspaceLimits
from piper_pink.config import DEFAULT_TCP, find_default_urdf
from piper_pink.execution import interpolate_joint_path
from piper_pink.ik import PinkIkSolver
from piper_pink.model import load_piper_model
from piper_pink.recipe import GraspRecipe


def pose_matrix(pose):
    value = np.eye(4)
    value[:3, :3] = pose.rotation
    value[:3, 3] = pose.translation
    return value.tolist()


def orthonormalize_matrix(matrix):
    value = np.asarray(matrix, dtype=float).copy()
    u, _, vh = np.linalg.svd(value[:3, :3])
    rotation = u @ vh
    if np.linalg.det(rotation) < 0.0:
        u[:, -1] *= -1.0
        rotation = u @ vh
    value[:3, :3] = rotation
    value[3] = [0.0, 0.0, 0.0, 1.0]
    return value


def horizontalize_rotation(rotation):
    """Keep retreat yaw while making the TCP XY plane parallel to base XY."""
    x_axis = np.asarray(rotation, dtype=float)[:, 0]
    x_axis[2] = 0.0
    norm = np.linalg.norm(x_axis)
    if norm < 1e-8:
        raise ValueError("Retreat TCP X axis cannot define a horizontal heading")
    x_axis /= norm
    z_sign = 1.0 if float(np.asarray(rotation, dtype=float)[:, 2][2]) >= 0.0 else -1.0
    z_axis = np.array([0.0, 0.0, z_sign])
    y_axis = np.cross(z_axis, x_axis)
    y_axis /= np.linalg.norm(y_axis)
    value = np.column_stack((x_axis, y_axis, z_axis))
    u, _, vh = np.linalg.svd(value)
    value = u @ vh
    if np.linalg.det(value) < 0.0:
        u[:, -1] *= -1.0
        value = u @ vh
    return value


def target_matrix_from_tag(base_from_tag, tcp_rotation, height_m):
    base_from_tag = np.asarray(base_from_tag, dtype=float)
    tcp_rotation = np.asarray(tcp_rotation, dtype=float)
    if tcp_rotation.shape == (4, 4):
        tcp_rotation = base_from_tag[:3, :3] @ tcp_rotation[:3, :3]
    elif tcp_rotation.shape != (3, 3):
        raise ValueError("TCP rotation must be a 3x3 rotation or 4x4 transform")
    target = np.eye(4)
    # Preserve retreat yaw, but make the held drone horizontal in base XY.
    target[:3, :3] = horizontalize_rotation(tcp_rotation)
    target[:3, 3] = base_from_tag[:3, 3] + [0.0, 0.0, height_m]
    return target


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--target-pose", type=Path, required=True)
    parser.add_argument("--current-q-deg", nargs=6, type=float, required=True)
    parser.add_argument("--height-m", type=float, default=0.150)
    parser.add_argument("--via-pose", type=Path, default=None,
                        help="Saved TCP pose to use as an intermediate waypoint")
    parser.add_argument("--phase", choices=("lift", "target", "rotate"), default="target")
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    if not 0.10 <= args.height_m <= 0.30:
        raise ValueError("Transport height must be between 0.10 and 0.30 m")
    with args.target_pose.open(encoding="utf-8") as stream:
        captured = yaml.safe_load(stream)
    if captured.get("target_family") != "Standard52h13" or captured.get("target_id") != 0:
        raise ValueError("Captured target must be Standard52h13 ID 0")
    if abs(float(captured.get("tag_size_m", 0.0)) - 0.0222) > 1e-9:
        raise ValueError("Captured target does not use the converted 22.2 mm pose size")
    if args.phase == "lift" and time.time() - float(captured["timestamp_s"]) > 5.0:
        raise ValueError("Captured target pose is stale")

    current_q = np.deg2rad(np.asarray(args.current_q_deg, dtype=float))
    urdf = find_default_urdf()
    model = load_piper_model(urdf, DEFAULT_TCP)
    recipe = GraspRecipe.load(PROJECT_DIR / "config/grasp_recipe_drone_apriltag_top.yaml")
    retreat_tcp = model.forward_tcp(current_q)
    if args.phase == "rotate":
        tag_rotation = np.asarray(captured["base_from_tag"], dtype=float)[:3, :3]
        # Keep the held drone horizontal while aligning TCP +X with the
        # target tag's left-right (+X) direction.
        z_axis = np.array([0.0, 0.0, 1.0 if retreat_tcp.rotation[2, 2] >= 0.0 else -1.0])
        x_axis = tag_rotation[:, 0].copy()
        x_axis[2] = 0.0
        norm = np.linalg.norm(x_axis)
        if norm < 1e-8:
            raise ValueError("Target tag X axis is parallel to TCP Z axis")
        x_axis /= norm
        y_axis = np.cross(z_axis, x_axis)
        target_matrix = np.eye(4)
        target_matrix[:3, :3] = orthonormalize_matrix(np.block([
            [np.column_stack((x_axis, y_axis, z_axis)), np.zeros((3, 1))],
            [np.zeros((1, 3)), np.ones((1, 1))],
        ]))[:3, :3]
        target_matrix[:3, 3] = retreat_tcp.translation
    elif args.phase == "target":
        # Stage 8 diagnostic mode: solve position only by preserving the
        # current lifted TCP orientation. Orientation can be constrained later
        # after reachability is confirmed.
        target_matrix = np.eye(4)
        target_matrix[:3, :3] = retreat_tcp.rotation
        target_matrix[:3, 3] = np.asarray(captured["base_from_tag"], dtype=float)[:3, 3]
        target_matrix[:3, 3] += [0.0, 0.0, args.height_m]
        target_matrix = orthonormalize_matrix(target_matrix)
    else:
        target_matrix = orthonormalize_matrix(target_matrix_from_tag(
            captured["base_from_tag"], retreat_tcp.rotation, args.height_m
        ))
    target_xyz = target_matrix[:3, 3]
    target = model.pose_from_matrix(target_matrix)
    WorkspaceLimits(0.65, 0.15).validate_pose(target)
    # Solve the operator-selected intermediate waypoint first. If none is
    # supplied, use a conservative 250 mm vertical lift waypoint.
    solver = PinkIkSolver(model)
    via_label = "250 mm lift"
    if args.via_pose is not None:
        with args.via_pose.open(encoding="utf-8") as stream:
            via_capture = yaml.safe_load(stream)
        via_matrix = orthonormalize_matrix(via_capture["base_from_tcp"])
        via_label = "saved TCP waypoint"
    else:
        via_matrix = orthonormalize_matrix(retreat_tcp.homogeneous)
        via_matrix[:3, 3] += np.array([0.0, 0.0, 0.250])
    via = model.pose_from_matrix(via_matrix)
    if args.phase == "lift":
        via_result = solver.solve(via, current_q)
        if not via_result.converged:
            raise RuntimeError(f"No IK solution for the {via_label}")
    else:
        via_result = None
    if args.phase == "lift":
        path = interpolate_joint_path(current_q, via_result.q, math.radians(0.5))
        final_result = via_result
        final_pose = via
    else:
        # In target phase, the current pose is already the completed lift;
        # do not require solving the lift waypoint again.
        # Final target is solved only after the lift phase has completed.
        result = None
        for seed in (current_q, model.neutral()[:6]):
            candidate = solver.solve(target, seed)
            if candidate.converged:
                result = candidate
                break
        if result is None:
            raise RuntimeError(f"No IK solution for the {args.height_m * 1000:.0f} mm target-above pose")
        path = interpolate_joint_path(current_q, result.q, math.radians(0.5))
        final_result = result
        final_pose = target
    collision = PiperSelfCollisionChecker(
        urdf,
        PROJECT_DIR / "assets/piper_x_moveit/piper_x.srdf",
        PROJECT_DIR / "config/scene.yaml",
    )
    collision.validate_path(path)
    report = {
        "schema_version": 1,
        "created_at_unix_s": time.time(),
        "mode": "standard52_transport_hold",
        "inputs": {
            "target_pose": str(args.target_pose),
            "current_q_deg": args.current_q_deg,
            "height_m": args.height_m,
            "target_family": "Standard52h13",
            "target_id": 0,
            "tag_size_m": 0.0222,
            "measured_outer_edge_m": 0.037,
            "orientation_policy": "horizontal held drone; preserve retreat yaw",
            "orientation_source": "current_q_deg forward TCP pose, horizontalized",
            "path_policy": via_label if args.phase == "lift" else "direct from completed lift",
            "phase": args.phase,
        },
        "stages": {
            "transport": {
                "target": pose_matrix(final_pose),
                "q_deg": np.rad2deg(final_result.q).tolist(),
                "position_error_mm": final_result.position_error_m * 1000.0,
                "orientation_error_deg": math.degrees(final_result.orientation_error_rad),
            }
        },
        "path_waypoints": len(path),
        "payload_collision_model": "not_modeled; operator must clear transport path",
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    with args.report.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(report, stream, sort_keys=False)
    print("target_tcp_xyz_mm: " + " ".join(f"{x*1000.0:.3f}" for x in target_xyz))
    closing_alignment = float(target_matrix[:3, 0] @ np.asarray(captured["base_from_tag"])[:3, 0])
    print(f"closing_axis_tag_x_alignment: {closing_alignment:.6f}")
    print("target_q_deg: " + " ".join(f"{x:.4f}" for x in np.rad2deg(final_result.q)))
    print(f"path_waypoints: {len(path)}")
    print(f"report: {args.report}")
    print("Offline validation only. The held drone geometry is not modeled.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError, OSError, KeyError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
