"""Command-line entry point for Piper model checks, IK, and guarded motion."""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import replace
from pathlib import Path

import numpy as np

from .bridge import PiperSdkBridge
from .calibration import (
    HandEyeCalibration,
    handeye_consistency,
    solve_eye_in_hand,
    solve_eye_to_hand,
    solve_tcp_pivot,
)
from .collision import PiperSelfCollisionChecker, WorkspaceLimits
from .config import (
    DEFAULT_HARDWARE_SAFETY,
    DEFAULT_IK_SETTINGS,
    DEFAULT_SEED_RAD,
    DEFAULT_TCP,
    PiperTcp,
    find_default_urdf,
)
from .execution import execute_joint_path, interpolate_joint_path, wait_for_joint_target
from .grasp import axial_candidates, solve_grasp_candidates
from .ik import PinkIkSolver
from .model import load_piper_model
from .planning import plan_grasp
from .pose_io import load_object_pose
from .recipe import GraspRecipe
from .reporting import save_plan_report
from .transforms import compose
from .transforms import load_yaml, save_yaml
from .tool_axis import axis_plane_angle_rad, closing_axis_base, horizontal_j6_candidates
from .workflow import GraspExecutor
from .vision_aruco import (
    detect_aruco_pose,
    load_camera_intrinsics,
    save_object_pose,
)


MOTION_CONFIRMATION = "I_HAVE_CLEARED_THE_WORKSPACE"
PREGRASP_CONFIRMATION = "I_CONFIRM_PREGRASP_ONLY"


def _degrees(values):
    return [math.degrees(float(value)) for value in values]


def _radians(values):
    return [math.radians(float(value)) for value in values]


def _model_from_args(args):
    urdf = args.urdf.expanduser().resolve() if args.urdf else find_default_urdf()
    tcp = PiperTcp(
        parent_frame=args.tcp_parent,
        xyz_m=tuple(args.tcp_xyz_m),
        rpy_rad=tuple(_radians(args.tcp_rpy_deg)),
        frame_name="piper_tcp",
    )
    return urdf, load_piper_model(urdf, tcp)


def _target_from_args(model, args):
    return model.make_pose(args.xyz_m, _radians(args.rpy_deg))


def _se3_matrix(pose):
    matrix = np.eye(4)
    matrix[:3, :3] = pose.rotation
    matrix[:3, 3] = pose.translation
    return matrix


def _solver_from_args(model, args):
    settings = replace(
        DEFAULT_IK_SETTINGS,
        position_cost=args.position_cost,
        orientation_cost=args.orientation_cost,
        random_seed_count=args.random_seeds,
    )
    return PinkIkSolver(model, settings=settings, qp_solver=args.qp_solver)


def _print_result(result) -> None:
    q_deg = _degrees(result.q)
    print(f"converged: {result.converged}")
    print(f"iterations: {result.iterations}")
    print(f"seed_index: {result.seed_index}")
    print(f"position_error_mm: {result.position_error_m * 1000.0:.3f}")
    print(f"orientation_error_deg: {math.degrees(result.orientation_error_rad):.3f}")
    print("joints_deg: " + " ".join(f"{value:.4f}" for value in q_deg))


def command_info(args) -> int:
    urdf, model = _model_from_args(args)
    print(f"URDF: {urdf}")
    print(f"model: {model.model.name}")
    print(f"nq/nv: {model.model.nq}/{model.model.nv}")
    print(f"joints: {', '.join(model.joint_names)}")
    print(f"TCP frame: {model.tcp_frame}")
    print(f"TCP parent: {args.tcp_parent}")
    print(f"TCP xyz_m: {list(args.tcp_xyz_m)}")
    print(f"TCP rpy_deg: {list(args.tcp_rpy_deg)}")
    print("limits_deg:")
    for name, lower, upper in zip(model.joint_names, model.lower_limits, model.upper_limits):
        print(f"  {name}: [{math.degrees(lower):.3f}, {math.degrees(upper):.3f}]")
    return 0


def command_solve(args) -> int:
    _, model = _model_from_args(args)
    target = _target_from_args(model, args)
    solver = _solver_from_args(model, args)
    result = solver.solve(target, _radians(args.current_q_deg))
    _print_result(result)
    return 0 if result.converged else 1


def command_candidates(args) -> int:
    _, model = _model_from_args(args)
    base_target = _target_from_args(model, args)
    solver = _solver_from_args(model, args)
    candidates = axial_candidates(base_target, _radians(args.axial_angles_deg))
    solved = solve_grasp_candidates(solver, candidates, _radians(args.current_q_deg))
    if not solved:
        print("No grasp candidate converged.")
        return 1
    for rank, item in enumerate(solved, 1):
        print(
            f"#{rank} axial_deg={math.degrees(item.candidate.axial_angle_rad):.1f} "
            f"score={item.result.score:.4f}"
        )
        _print_result(item.result)
    return 0


def command_read_arm(args) -> int:
    bridge = PiperSdkBridge(args.can)
    try:
        bridge.connect()
        feedback = bridge.wait_for_feedback()
        print(f"feedback_hz: {feedback.hz:.1f}")
        print("joints_deg: " + " ".join(f"{value:.4f}" for value in _degrees(feedback.q_rad)))
    finally:
        bridge.close()
    return 0


def command_gripper_axis(args) -> int:
    """Compare measured J6 with model solutions for a horizontal closing line."""

    _, model = _model_from_args(args)
    q = np.asarray(_radians(args.current_q_deg), dtype=float)
    axis = closing_axis_base(model, q)
    angle_deg = math.degrees(axis_plane_angle_rad(axis))
    candidates = horizontal_j6_candidates(model, q)

    print("check_mode: READ_ONLY_OFFLINE_KINEMATICS")
    print("closing_line_model: piper_tcp +/-X")
    print("base_plane: base_link XY")
    print("current_q_deg: " + " ".join(f"{value:.4f}" for value in args.current_q_deg))
    outside = np.flatnonzero(
        (q < model.lower_limits) | (q > model.upper_limits)
    )
    if outside.size:
        details = ", ".join(
            f"{model.joint_names[index]}={args.current_q_deg[index]:.4f}deg"
            for index in outside
        )
        print(f"planning_limit_warning: {details}")
        print("planning_limit_warning_note: accepted for read-only FK; not accepted for motion planning")
    print(f"current_j6_deg: {args.current_q_deg[5]:.4f}")
    print("closing_axis_base: " + " ".join(f"{value:.8f}" for value in axis))
    print(f"signed_plane_angle_deg: {angle_deg:.4f}")
    print(f"horizontal_error_deg: {abs(angle_deg):.4f}")
    print(f"model_horizontal_within_1deg: {str(abs(angle_deg) <= 1.0).lower()}")
    if not candidates:
        print("horizontal_j6_candidates_deg: none within joint limits")
        return 1
    candidates_deg = np.rad2deg(candidates)
    nearest_index = int(np.argmin(np.abs(candidates_deg - args.current_q_deg[5])))
    nearest = float(candidates_deg[nearest_index])
    print(
        "horizontal_j6_candidates_deg: "
        + " ".join(f"{value:.4f}" for value in candidates_deg)
    )
    print(f"nearest_horizontal_j6_deg: {nearest:.4f}")
    print(f"current_minus_nearest_deg: {args.current_q_deg[5] - nearest:.4f}")
    print("J1-J5 were held fixed while solving J6.")
    print("No CAN connection, enable service, or motion command was opened.")
    return 0


def command_move(args) -> int:
    if args.execute and args.confirmation != MOTION_CONFIRMATION:
        print(
            "Real motion requires --confirmation " + MOTION_CONFIRMATION,
            file=sys.stderr,
        )
        return 2
    urdf, model = _model_from_args(args)
    target = _target_from_args(model, args)
    solver = _solver_from_args(model, args)
    safety = replace(
        DEFAULT_HARDWARE_SAFETY,
        speed_percent=args.speed,
        maximum_joint_step_rad=math.radians(args.max_step_deg),
    )
    bridge = PiperSdkBridge(args.can, safety=safety)
    try:
        bridge.connect()
        feedback = bridge.wait_for_feedback()
        print(f"feedback_hz: {feedback.hz:.1f}")
        result = solver.solve(target, feedback.q_rad)
        _print_result(result)
        if not result.converged:
            print("Refusing motion because IK did not converge.", file=sys.stderr)
            return 1
        path = interpolate_joint_path(
            feedback.q_rad, result.q, safety.maximum_joint_step_rad
        )
        collision = PiperSelfCollisionChecker(urdf, args.srdf, args.scene)
        collision.validate_path(path)
        print(f"planned_joint_waypoints: {len(path)}")
        if not args.execute:
            print("Dry plan only. No enable or motion command was sent.")
            return 0
        bridge.enable()
        execute_joint_path(bridge, model, path)
        final = bridge.wait_for_feedback()
        position_error, orientation_error = model.pose_error(final.q_rad, target)
        print(f"final_position_error_mm: {position_error * 1000.0:.3f}")
        print(f"final_orientation_error_deg: {math.degrees(orientation_error):.3f}")
        return 0
    finally:
        bridge.close()


def _load_test_points(path: Path):
    data = load_yaml(path)
    points = data.get("points")
    if not isinstance(points, list) or not points:
        raise ValueError("Test-points YAML must contain a non-empty 'points' list")
    normalized = []
    for index, item in enumerate(points, 1):
        if not isinstance(item, dict):
            raise ValueError(f"Point {index} must be a mapping")
        name = str(item.get("name", f"point_{index}")).strip()
        xyz_m = item.get("xyz_m")
        rpy_deg = item.get("rpy_deg")
        if not name or not isinstance(xyz_m, list) or not isinstance(rpy_deg, list):
            raise ValueError(
                f"Point {index} needs name, xyz_m=[x,y,z], and rpy_deg=[r,p,y]"
            )
        if len(xyz_m) != 3 or len(rpy_deg) != 3:
            raise ValueError(f"Point {index} xyz_m and rpy_deg must contain three values")
        try:
            xyz = [float(value) for value in xyz_m]
            rpy = [float(value) for value in rpy_deg]
        except (TypeError, ValueError) as exc:
            raise ValueError(f"Point {index} contains a non-numeric pose value") from exc
        if not np.all(np.isfinite(xyz + rpy)):
            raise ValueError(f"Point {index} contains NaN or infinity")
        normalized.append((name, xyz, rpy))
    return normalized


def command_test_points(args) -> int:
    if args.execute and args.confirmation != MOTION_CONFIRMATION:
        print(
            "Real motion requires --confirmation " + MOTION_CONFIRMATION,
            file=sys.stderr,
        )
        return 2

    urdf, model = _model_from_args(args)
    points = _load_test_points(args.points)
    solver = _solver_from_args(model, args)
    safety = replace(
        DEFAULT_HARDWARE_SAFETY,
        speed_percent=args.speed,
        maximum_joint_step_rad=math.radians(args.max_step_deg),
    )
    collision = PiperSelfCollisionChecker(urdf, args.srdf, args.scene)
    bridge = PiperSdkBridge(args.can, safety=safety) if args.execute else None
    try:
        if bridge is not None:
            bridge.connect()
            feedback = bridge.wait_for_feedback()
            current_q = feedback.q_rad
            print(f"feedback_hz: {feedback.hz:.1f}")
        else:
            current_q = tuple(_radians(args.current_q_deg))

        plans = []
        for index, (name, xyz_m, rpy_deg) in enumerate(points, 1):
            target = model.make_pose(xyz_m, _radians(rpy_deg))
            result = solver.solve(target, current_q)
            if not result.converged:
                raise RuntimeError(f"IK did not converge for point {index} ({name})")
            path = interpolate_joint_path(
                current_q, result.q, safety.maximum_joint_step_rad
            )
            collision.validate_path(path)
            plans.append((name, target, result, path))
            print(
                f"{index}. {name}: waypoints={len(path)} "
                f"position_error_mm={result.position_error_m * 1000.0:.3f} "
                f"orientation_error_deg={math.degrees(result.orientation_error_rad):.3f}"
            )
            print("   joints_deg: " + " ".join(f"{value:.3f}" for value in _degrees(result.q)))
            current_q = result.q

        if bridge is None:
            print("Dry plan only. No CAN connection, enable, or motion command was sent.")
            return 0

        bridge.enable()
        for index, (name, target, result, path) in enumerate(plans, 1):
            print(f"Executing {index}/{len(plans)}: {name}")
            execute_joint_path(bridge, model, path)
            final = bridge.wait_for_feedback()
            position_error, orientation_error = model.pose_error(final.q_rad, target)
            print(
                f"   final_position_error_mm={position_error * 1000.0:.3f} "
                f"final_orientation_error_deg={math.degrees(orientation_error):.3f}"
            )
            if position_error > DEFAULT_IK_SETTINGS.position_tolerance_m:
                raise RuntimeError(f"Point {name} did not reach the position tolerance")
        print("All test points reached.")
        return 0
    finally:
        if bridge is not None:
            bridge.close()


def _base_grasp_pose(model, object_pose, recipe, handeye_path, current_q):
    if object_pose.object_id != recipe.object_id:
        raise ValueError(
            f"Object pose is for '{object_pose.object_id}', recipe is for '{recipe.object_id}'"
        )
    if object_pose.confidence < 0.5:
        raise ValueError(f"Object-pose confidence is too low: {object_pose.confidence:.3f}")
    if object_pose.frame_id == "base_link":
        base_from_object = object_pose.frame_from_object
    else:
        if handeye_path is None:
            raise ValueError("A camera-frame object pose requires --handeye")
        calibration = HandEyeCalibration.load(handeye_path)
        if object_pose.frame_id != calibration.camera_frame:
            raise ValueError(
                f"Pose frame '{object_pose.frame_id}' does not match calibrated camera "
                f"frame '{calibration.camera_frame}'"
            )
        base_from_gripper = None
        if calibration.camera_mount == "wrist":
            # The calibration transform is defined from its recorded parent frame.
            # Looking that frame up explicitly prevents silently applying a result
            # collected from controller /end_pose as if it were gripper_base.
            base_from_gripper = _se3_matrix(
                model.forward_frame(current_q, calibration.parent_frame)
            )
        base_from_object = calibration.base_from_object(
            object_pose.frame_from_object, base_from_gripper
        )
    base_from_tcp = compose(base_from_object, recipe.object_from_tcp)
    return model.pose_from_matrix(base_from_tcp)


def _grasp_plan(model, solver, args, current_q, collision_checker):
    object_pose = load_object_pose(args.object_pose)
    recipe = GraspRecipe.load(args.recipe)
    base_grasp = _base_grasp_pose(model, object_pose, recipe, args.handeye, current_q)
    workspace = WorkspaceLimits(args.workspace_radius_m, args.minimum_tcp_z_m)
    plan = plan_grasp(
        solver,
        base_grasp,
        current_q,
        _radians(recipe.axial_angles_deg),
        recipe.pregrasp_distance_m,
        recipe.retreat_distance_m,
        workspace,
        collision_checker,
        math.radians(args.collision_step_deg),
    )
    return plan, recipe, object_pose


def _validate_pose_freshness(object_pose, maximum_age_s: float) -> None:
    if object_pose.timestamp_s is None:
        raise ValueError("Real grasp requires object_pose.timestamp_s from fresh perception")
    age = time.time() - object_pose.timestamp_s
    if age < -1.0:
        raise ValueError("Object-pose timestamp is in the future; check system clocks")
    if age > maximum_age_s:
        raise ValueError(
            f"Object pose is stale ({age:.3f} s old, limit {maximum_age_s:.3f} s)"
        )


def _print_grasp_plan(plan) -> None:
    print(f"selected_axial_deg: {math.degrees(plan.candidate.axial_angle_rad):.3f}")
    print(f"plan_score: {plan.score:.5f}")
    for name, result in (
        ("pregrasp", plan.pregrasp),
        ("grasp", plan.grasp),
        ("retreat", plan.retreat),
    ):
        values = " ".join(f"{value:.4f}" for value in _degrees(result.q))
        print(
            f"{name}: q_deg=[{values}] position_error_mm="
            f"{result.position_error_m * 1000.0:.3f} orientation_error_deg="
            f"{math.degrees(result.orientation_error_rad):.3f}"
        )


def command_plan_grasp(args) -> int:
    urdf, model = _model_from_args(args)
    solver = _solver_from_args(model, args)
    current_q = _radians(args.current_q_deg)
    collision = PiperSelfCollisionChecker(urdf, args.srdf, args.scene)
    plan, _, _ = _grasp_plan(model, solver, args, current_q, collision)
    _print_grasp_plan(plan)
    if args.report is not None:
        save_plan_report(
            args.report, plan, current_q, args.object_pose, args.recipe, "offline"
        )
        print(f"report: {args.report}")
    print("Offline validation only. No hardware connection was opened.")
    return 0


def command_grasp(args) -> int:
    if args.execute and args.confirmation != MOTION_CONFIRMATION:
        print(
            "Real grasp requires --confirmation " + MOTION_CONFIRMATION,
            file=sys.stderr,
        )
        return 2
    urdf, model = _model_from_args(args)
    solver = _solver_from_args(model, args)
    safety = replace(
        DEFAULT_HARDWARE_SAFETY,
        speed_percent=args.speed,
        maximum_joint_step_rad=math.radians(args.max_step_deg),
    )
    bridge = PiperSdkBridge(args.can, safety=safety)
    try:
        bridge.connect()
        feedback = bridge.wait_for_feedback()
        collision = PiperSelfCollisionChecker(urdf, args.srdf, args.scene)
        plan, recipe, object_pose = _grasp_plan(
            model, solver, args, feedback.q_rad, collision
        )
        _print_grasp_plan(plan)
        if args.report is not None:
            save_plan_report(
                args.report, plan, feedback.q_rad, args.object_pose, args.recipe, "hardware"
            )
            print(f"report: {args.report}")
        if not args.execute:
            print("Plan validated. No enable, gripper, or motion command was sent.")
            return 0
        if not recipe.execution_ready:
            raise RuntimeError(
                "This grasp recipe has execution_ready=false. Measure the object width, "
                "verify the configured TCP depth and workcell scene, then explicitly approve "
                "the recipe before real motion."
            )
        _validate_pose_freshness(object_pose, args.maximum_pose_age_s)
        executor = GraspExecutor(bridge, model, collision)
        report = executor.execute(
            plan,
            gripper_opening_m=recipe.gripper_opening_m,
            gripper_closed_m=recipe.gripper_closed_m,
        )
        print(f"grasp_state: {report.state.name}")
        print(report.message)
        return 0
    finally:
        bridge.close()


def command_pregrasp(args) -> int:
    """Open the gripper and execute only the current-to-pregrasp segment."""

    if args.execute and args.confirmation != PREGRASP_CONFIRMATION:
        print(
            "Pregrasp-only motion requires --confirmation " + PREGRASP_CONFIRMATION,
            file=sys.stderr,
        )
        return 2
    urdf, model = _model_from_args(args)
    solver = _solver_from_args(model, args)
    safety = replace(
        DEFAULT_HARDWARE_SAFETY,
        speed_percent=args.speed,
        maximum_joint_step_rad=math.radians(args.max_step_deg),
    )
    bridge = PiperSdkBridge(args.can, safety=safety)
    try:
        bridge.connect()
        feedback = bridge.wait_for_feedback()
        collision = PiperSelfCollisionChecker(urdf, args.srdf, args.scene)
        plan, recipe, object_pose = _grasp_plan(
            model, solver, args, feedback.q_rad, collision
        )
        _print_grasp_plan(plan)
        path = interpolate_joint_path(
            feedback.q_rad, plan.pregrasp.q, safety.maximum_joint_step_rad
        )
        collision.validate_path(path)
        print(f"pregrasp_joint_waypoints: {len(path)}")
        if args.report is not None:
            save_plan_report(
                args.report,
                plan,
                feedback.q_rad,
                args.object_pose,
                args.recipe,
                "pregrasp-only",
            )
            print(f"report: {args.report}")
        if not args.execute:
            print("Pregrasp plan validated. No enable, gripper, or motion command was sent.")
            return 0

        _validate_pose_freshness(object_pose, args.maximum_pose_age_s)
        bridge.enable()
        bridge.command_gripper(recipe.gripper_opening_m)
        execute_joint_path(bridge, model, path)
        final = wait_for_joint_target(
            bridge,
            model,
            plan.pregrasp.q,
            collision_checker=collision,
            tolerance_rad=math.radians(args.joint_goal_tolerance_deg),
            timeout_s=args.joint_goal_timeout_s,
        )
        position_error, orientation_error = model.pose_error(
            final.q_rad, plan.pregrasp_pose
        )
        print(f"final_position_error_mm: {position_error * 1000.0:.3f}")
        print(f"final_orientation_error_deg: {math.degrees(orientation_error):.3f}")
        if position_error > args.maximum_final_position_error_m:
            raise RuntimeError(
                f"Pregrasp TCP position error {position_error*1000.0:.3f} mm exceeds "
                f"{args.maximum_final_position_error_m*1000.0:.3f} mm"
            )
        if orientation_error > math.radians(args.maximum_final_orientation_error_deg):
            raise RuntimeError(
                f"Pregrasp TCP orientation error {math.degrees(orientation_error):.3f} deg "
                f"exceeds {args.maximum_final_orientation_error_deg:.3f} deg"
            )
        print("PREGRASP_ONLY_COMPLETE")
        print("Gripper remains open. No descent, closing, grasp, or retreat was commanded.")
        return 0
    finally:
        bridge.close()


def command_calibrate_tcp(args) -> int:
    data = load_yaml(args.samples)
    samples = [np.asarray(item, dtype=float) for item in data["base_from_flange"]]
    result = solve_tcp_pivot(samples)
    output = {
        "parent_frame": "flange_link",
        "tcp_translation_m": result.flange_from_tcp_translation_m.tolist(),
        "base_pivot_m": result.base_pivot_m.tolist(),
        "rms_residual_m": result.rms_residual_m,
    }
    save_yaml(args.output, output)
    print(f"tcp_translation_m: {result.flange_from_tcp_translation_m.tolist()}")
    print(f"rms_residual_mm: {result.rms_residual_m * 1000.0:.3f}")
    print(f"saved: {args.output}")
    return 0 if result.rms_residual_m <= args.maximum_rms_mm / 1000.0 else 1


def command_calibrate_handeye(args) -> int:
    data = load_yaml(args.samples)
    robot = [np.asarray(item, dtype=float) for item in data["base_from_gripper"]]
    observations = [
        np.asarray(item, dtype=float) for item in data["camera_from_target"]
    ]
    if args.mount == "wrist":
        calibration = solve_eye_in_hand(robot, observations)
    else:
        calibration = solve_eye_to_hand(robot, observations)
    calibration = replace(
        calibration,
        camera_frame=str(data.get("camera_frame", "camera_color_optical_frame")),
    )
    translation_rms, rotation_rms = handeye_consistency(
        calibration, robot, observations
    )
    calibration.save(args.output)
    print(f"saved: {args.output}")
    print(f"{calibration.parent_frame}_from_camera:")
    print(calibration.parent_from_camera)
    print(f"translation_rms_mm: {translation_rms * 1000.0:.3f}")
    print(f"rotation_rms_deg: {math.degrees(rotation_rms):.3f}")
    passed = (
        translation_rms <= args.maximum_translation_rms_mm / 1000.0
        and rotation_rms <= math.radians(args.maximum_rotation_rms_deg)
    )
    return 0 if passed else 1


def _append_matrix_sample(path: Path, key: str, matrix, metadata=None) -> None:
    data = load_yaml(path) if path.exists() else {}
    values = data.setdefault(key, [])
    if not isinstance(values, list):
        raise ValueError(f"Sample field '{key}' must be a list")
    values.append(np.asarray(matrix, dtype=float).tolist())
    if metadata:
        data.update(metadata)
    save_yaml(path, data)


def command_record_tcp_sample(args) -> int:
    _, model = _model_from_args(args)
    bridge = PiperSdkBridge(args.can)
    try:
        bridge.connect()
        feedback = bridge.wait_for_feedback()
        pose = _se3_matrix(model.forward_frame(feedback.q_rad, args.flange_frame))
        _append_matrix_sample(args.samples, "base_from_flange", pose)
        count = len(load_yaml(args.samples)["base_from_flange"])
        print(f"saved sample {count}: {args.samples}")
        print("Read-only capture complete. No enable or motion command was sent.")
        return 0
    finally:
        bridge.close()


def command_record_handeye_sample(args) -> int:
    camera_frame, camera_matrix, distortion = load_camera_intrinsics(args.intrinsics)
    camera_from_target = detect_aruco_pose(
        args.image,
        args.marker_id,
        args.marker_size_m,
        camera_matrix,
        distortion,
        args.dictionary,
    )
    _, model = _model_from_args(args)
    bridge = PiperSdkBridge(args.can)
    try:
        bridge.connect()
        feedback = bridge.wait_for_feedback()
        base_from_gripper = _se3_matrix(
            model.forward_frame(feedback.q_rad, args.gripper_frame)
        )
    finally:
        bridge.close()
    data = load_yaml(args.samples) if args.samples.exists() else {}
    robot = data.setdefault("base_from_gripper", [])
    observations = data.setdefault("camera_from_target", [])
    if not isinstance(robot, list) or not isinstance(observations, list):
        raise ValueError("Hand-eye sample fields must be lists")
    if len(robot) != len(observations):
        raise ValueError("Existing hand-eye sample lists have different lengths")
    robot.append(base_from_gripper.tolist())
    observations.append(camera_from_target.tolist())
    data["camera_frame"] = camera_frame
    save_yaml(args.samples, data)
    print(f"saved paired sample {len(robot)}: {args.samples}")
    print("Read-only capture complete. No enable or motion command was sent.")
    return 0


def command_validate_handeye(args) -> int:
    calibration = HandEyeCalibration.load(args.calibration)
    print(f"camera_mount: {calibration.camera_mount}")
    print(f"parent_frame: {calibration.parent_frame}")
    print(f"camera_frame: {calibration.camera_frame}")
    print(calibration.parent_from_camera)
    return 0


def command_doctor(args) -> int:
    import cv2
    import pinocchio
    import pink
    import piper_sdk
    import qpsolvers

    urdf, model = _model_from_args(args)
    collision = PiperSelfCollisionChecker(urdf, args.srdf, args.scene)
    seed = np.asarray(DEFAULT_SEED_RAD, dtype=float)
    model.validate_q(seed)
    collision.validate_q(seed)
    print(f"Pinocchio: {pinocchio.__version__}")
    print(f"Pink: {pink.__version__}")
    print(f"OpenCV: {cv2.__version__}")
    print(f"QP solvers: {', '.join(qpsolvers.available_solvers)}")
    print(f"Piper SDK: {Path(piper_sdk.__file__).resolve()}")
    print(f"URDF: {urdf}")
    print(f"SRDF: {args.srdf.resolve()}")
    print(f"Scene: {args.scene.resolve()}")
    print("Model and collision smoke test: PASS")

    can_path = Path("/sys/class/net") / args.can
    if can_path.exists():
        print(f"CAN interface {args.can}: present")
    else:
        message = f"CAN interface {args.can}: missing (offline work is still available)"
        print(message)
        if args.require_can:
            return 1
    if args.handeye is not None:
        calibration = HandEyeCalibration.load(args.handeye)
        if np.allclose(calibration.parent_from_camera, np.eye(4)):
            print("Hand-eye: WARNING identity matrix looks like an uncalibrated placeholder")
        elif calibration.camera_mount == "wrist":
            try:
                model.forward_frame(seed, calibration.parent_frame)
            except ValueError as exc:
                print(f"Hand-eye: ERROR {exc}")
                return 1
            print(
                "Hand-eye transform: valid; wrist parent frame "
                f"{calibration.parent_frame} is present"
            )
        else:
            print("Hand-eye transform: valid")
    print(
        f"TCP: parent={args.tcp_parent} "
        f"xyz_m={list(args.tcp_xyz_m)} "
        f"rpy_deg={list(args.tcp_rpy_deg)}"
    )
    return 0


def command_detect_aruco(args) -> int:
    camera_frame, camera_matrix, distortion = load_camera_intrinsics(args.intrinsics)
    pose = detect_aruco_pose(
        args.image,
        args.marker_id,
        args.marker_size_m,
        camera_matrix,
        distortion,
        args.dictionary,
    )
    object_id = args.object_id or f"aruco_{args.marker_id}"
    save_object_pose(args.output, camera_frame, object_id, pose)
    print(f"saved: {args.output}")
    print(pose)
    return 0


def _add_model_options(parser) -> None:
    parser.add_argument(
        "--urdf", type=Path, help="override piper_x_with_gripper_description.urdf"
    )
    parser.add_argument("--tcp-parent", default=DEFAULT_TCP.parent_frame)
    parser.add_argument("--tcp-xyz-m", nargs=3, type=float, default=DEFAULT_TCP.xyz_m)
    parser.add_argument(
        "--tcp-rpy-deg",
        nargs=3,
        type=float,
        default=tuple(_degrees(DEFAULT_TCP.rpy_rad)),
    )


def _add_target_options(parser) -> None:
    parser.add_argument("--xyz-m", nargs=3, type=float, required=True)
    parser.add_argument("--rpy-deg", nargs=3, type=float, required=True)
    parser.add_argument(
        "--current-q-deg", nargs=6, type=float, default=tuple(_degrees(DEFAULT_SEED_RAD))
    )
    parser.add_argument("--position-cost", type=float, default=1.0)
    parser.add_argument("--orientation-cost", type=float, default=0.2)
    parser.add_argument("--random-seeds", type=int, default=12)
    parser.add_argument("--qp-solver")


def _add_solver_options(parser, include_current: bool = True) -> None:
    if include_current:
        parser.add_argument(
            "--current-q-deg",
            nargs=6,
            type=float,
            default=tuple(_degrees(DEFAULT_SEED_RAD)),
        )
    parser.add_argument("--position-cost", type=float, default=1.0)
    parser.add_argument("--orientation-cost", type=float, default=0.2)
    parser.add_argument("--random-seeds", type=int, default=12)
    parser.add_argument("--qp-solver")


def _add_grasp_options(parser, include_current: bool) -> None:
    parser.add_argument("--object-pose", type=Path, required=True)
    parser.add_argument("--recipe", type=Path, required=True)
    parser.add_argument("--handeye", type=Path)
    parser.add_argument(
        "--srdf",
        type=Path,
        default=Path(__file__).resolve().parents[2]
        / "assets/piper_x_moveit/piper_x.srdf",
    )
    parser.add_argument(
        "--scene",
        type=Path,
        default=Path(__file__).resolve().parents[2] / "config/scene.yaml",
    )
    parser.add_argument("--workspace-radius-m", type=float, default=0.65)
    parser.add_argument("--minimum-tcp-z-m", type=float, default=0.02)
    parser.add_argument("--collision-step-deg", type=float, default=0.5)
    parser.add_argument("--report", type=Path)
    _add_solver_options(parser, include_current=include_current)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Pinocchio/Pink integration for Piper")
    commands = parser.add_subparsers(dest="command", required=True)

    info = commands.add_parser("info", help="validate and display the reduced model")
    _add_model_options(info)
    info.set_defaults(handler=command_info)

    solve = commands.add_parser("solve", help="solve one Cartesian target offline")
    _add_model_options(solve)
    _add_target_options(solve)
    solve.set_defaults(handler=command_solve)

    candidates = commands.add_parser("candidates", help="rank axial grasp rotations")
    _add_model_options(candidates)
    _add_target_options(candidates)
    candidates.add_argument(
        "--axial-angles-deg",
        nargs="+",
        type=float,
        default=(-90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0),
    )
    candidates.set_defaults(handler=command_candidates)

    read_arm = commands.add_parser("read-arm", help="read real joints without motion")
    read_arm.add_argument("--can", default="can0")
    read_arm.set_defaults(handler=command_read_arm)

    gripper_axis = commands.add_parser(
        "gripper-axis", help="check whether the modeled finger closing line is horizontal"
    )
    _add_model_options(gripper_axis)
    gripper_axis.add_argument("--current-q-deg", nargs=6, type=float, required=True)
    gripper_axis.set_defaults(handler=command_gripper_axis)

    move = commands.add_parser("move", help="plan, then optionally execute one target")
    _add_model_options(move)
    _add_target_options(move)
    move.add_argument("--can", default="can0")
    move.add_argument("--speed", type=int, choices=range(1, 31), default=10)
    move.add_argument("--max-step-deg", type=float, choices=None, default=0.5)
    move.add_argument("--execute", action="store_true")
    move.add_argument("--confirmation", default="")
    move.add_argument(
        "--srdf",
        type=Path,
        default=Path(__file__).resolve().parents[2]
        / "assets/piper_x_moveit/piper_x.srdf",
    )
    move.add_argument(
        "--scene",
        type=Path,
        default=Path(__file__).resolve().parents[2] / "config/scene.yaml",
    )
    move.set_defaults(handler=command_move)

    test_points = commands.add_parser(
        "test-points", help="IK-plan several points, then optionally execute them on the arm"
    )
    _add_model_options(test_points)
    test_points.add_argument("--points", type=Path, required=True)
    test_points.add_argument("--can", default="can0")
    test_points.add_argument("--speed", type=int, choices=range(1, 31), default=10)
    test_points.add_argument("--max-step-deg", type=float, default=0.25)
    test_points.add_argument("--execute", action="store_true")
    test_points.add_argument("--confirmation", default="")
    test_points.add_argument(
        "--current-q-deg", nargs=6, type=float, default=tuple(_degrees(DEFAULT_SEED_RAD))
    )
    test_points.add_argument("--position-cost", type=float, default=1.0)
    test_points.add_argument("--orientation-cost", type=float, default=0.2)
    test_points.add_argument("--random-seeds", type=int, default=12)
    test_points.add_argument("--qp-solver")
    test_points.add_argument(
        "--srdf",
        type=Path,
        default=Path(__file__).resolve().parents[2]
        / "assets/piper_x_moveit/piper_x.srdf",
    )
    test_points.add_argument(
        "--scene",
        type=Path,
        default=Path(__file__).resolve().parents[2] / "config/scene.yaml",
    )
    test_points.set_defaults(handler=command_test_points)

    plan_grasp_parser = commands.add_parser(
        "plan-grasp", help="plan and collision-check a complete grasp offline"
    )
    _add_model_options(plan_grasp_parser)
    _add_grasp_options(plan_grasp_parser, include_current=True)
    plan_grasp_parser.set_defaults(handler=command_plan_grasp)

    grasp_parser = commands.add_parser(
        "grasp", help="plan a grasp, then optionally execute on the real arm"
    )
    _add_model_options(grasp_parser)
    _add_grasp_options(grasp_parser, include_current=False)
    grasp_parser.add_argument("--can", default="can0")
    grasp_parser.add_argument("--speed", type=int, choices=range(1, 31), default=10)
    grasp_parser.add_argument("--max-step-deg", type=float, default=0.5)
    grasp_parser.add_argument("--execute", action="store_true")
    grasp_parser.add_argument("--confirmation", default="")
    grasp_parser.add_argument("--maximum-pose-age-s", type=float, default=2.0)
    grasp_parser.set_defaults(handler=command_grasp)

    pregrasp_parser = commands.add_parser(
        "pregrasp", help="open the gripper and optionally move only to pregrasp"
    )
    _add_model_options(pregrasp_parser)
    _add_grasp_options(pregrasp_parser, include_current=False)
    pregrasp_parser.add_argument("--can", default="can0")
    pregrasp_parser.add_argument("--speed", type=int, choices=range(1, 11), default=5)
    pregrasp_parser.add_argument("--max-step-deg", type=float, default=0.25)
    pregrasp_parser.add_argument("--execute", action="store_true")
    pregrasp_parser.add_argument("--confirmation", default="")
    pregrasp_parser.add_argument("--maximum-pose-age-s", type=float, default=2.0)
    pregrasp_parser.add_argument("--joint-goal-tolerance-deg", type=float, default=0.5)
    pregrasp_parser.add_argument("--joint-goal-timeout-s", type=float, default=25.0)
    pregrasp_parser.add_argument("--maximum-final-position-error-m", type=float, default=0.002)
    pregrasp_parser.add_argument("--maximum-final-orientation-error-deg", type=float, default=1.0)
    pregrasp_parser.set_defaults(handler=command_pregrasp)

    tcp_parser = commands.add_parser(
        "calibrate-tcp", help="solve a pivot TCP calibration from recorded flange poses"
    )
    tcp_parser.add_argument("--samples", type=Path, required=True)
    tcp_parser.add_argument("--output", type=Path, required=True)
    tcp_parser.add_argument("--maximum-rms-mm", type=float, default=1.0)
    tcp_parser.set_defaults(handler=command_calibrate_tcp)

    handeye_parser = commands.add_parser(
        "calibrate-handeye", help="solve wrist or fixed-camera hand-eye calibration"
    )
    handeye_parser.add_argument("--samples", type=Path, required=True)
    handeye_parser.add_argument("--output", type=Path, required=True)
    handeye_parser.add_argument("--mount", choices=("wrist", "fixed"), required=True)
    handeye_parser.add_argument("--maximum-translation-rms-mm", type=float, default=2.0)
    handeye_parser.add_argument("--maximum-rotation-rms-deg", type=float, default=0.5)
    handeye_parser.set_defaults(handler=command_calibrate_handeye)

    tcp_sample = commands.add_parser(
        "record-tcp-sample", help="append one read-only real-arm flange pose"
    )
    _add_model_options(tcp_sample)
    tcp_sample.add_argument("--can", default="can0")
    tcp_sample.add_argument("--flange-frame", default="flange_link")
    tcp_sample.add_argument("--samples", type=Path, required=True)
    tcp_sample.set_defaults(handler=command_record_tcp_sample)

    handeye_sample = commands.add_parser(
        "record-handeye-sample", help="append one synchronized robot/ArUco pair"
    )
    _add_model_options(handeye_sample)
    handeye_sample.add_argument("--can", default="can0")
    handeye_sample.add_argument("--gripper-frame", default="gripper_base")
    handeye_sample.add_argument("--image", type=Path, required=True)
    handeye_sample.add_argument("--intrinsics", type=Path, required=True)
    handeye_sample.add_argument("--marker-id", type=int, required=True)
    handeye_sample.add_argument("--marker-size-m", type=float, required=True)
    handeye_sample.add_argument("--dictionary", default="DICT_4X4_50")
    handeye_sample.add_argument("--samples", type=Path, required=True)
    handeye_sample.set_defaults(handler=command_record_handeye_sample)

    validate_handeye = commands.add_parser(
        "validate-handeye", help="validate a saved fixed or wrist camera transform"
    )
    validate_handeye.add_argument("--calibration", type=Path, required=True)
    validate_handeye.set_defaults(handler=command_validate_handeye)

    doctor = commands.add_parser("doctor", help="check environment, model, collision, and CAN")
    _add_model_options(doctor)
    doctor.add_argument("--can", default="can0")
    doctor.add_argument("--require-can", action="store_true")
    doctor.add_argument("--handeye", type=Path)
    doctor.add_argument(
        "--srdf",
        type=Path,
        default=Path(__file__).resolve().parents[2]
        / "assets/piper_x_moveit/piper_x.srdf",
    )
    doctor.add_argument(
        "--scene",
        type=Path,
        default=Path(__file__).resolve().parents[2] / "config/scene.yaml",
    )
    doctor.set_defaults(handler=command_doctor)

    aruco = commands.add_parser(
        "detect-aruco", help="estimate an ArUco 6D pose and write object-pose YAML"
    )
    aruco.add_argument("--image", type=Path, required=True)
    aruco.add_argument("--intrinsics", type=Path, required=True)
    aruco.add_argument("--marker-id", type=int, required=True)
    aruco.add_argument("--marker-size-m", type=float, required=True)
    aruco.add_argument("--dictionary", default="DICT_4X4_50")
    aruco.add_argument("--object-id")
    aruco.add_argument("--output", type=Path, required=True)
    aruco.set_defaults(handler=command_detect_aruco)
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    if hasattr(args, "max_step_deg") and not 0.01 <= args.max_step_deg <= 0.5:
        print("--max-step-deg must be between 0.01 and 0.5", file=sys.stderr)
        return 2
    if hasattr(args, "collision_step_deg") and not 0.05 <= args.collision_step_deg <= 2.0:
        print("--collision-step-deg must be between 0.05 and 2.0", file=sys.stderr)
        return 2
    if hasattr(args, "maximum_pose_age_s") and args.maximum_pose_age_s <= 0.0:
        print("--maximum-pose-age-s must be positive", file=sys.stderr)
        return 2
    try:
        return args.handler(args)
    except KeyError as exc:
        print(f"Error: configuration is missing field {exc}", file=sys.stderr)
        return 1
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
