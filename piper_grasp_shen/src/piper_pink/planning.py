"""Plan pre-grasp, grasp, and retreat configurations from a 6D target."""

from __future__ import annotations

from dataclasses import dataclass

from .collision import WorkspaceLimits
from .execution import interpolate_joint_path
from .grasp import GraspCandidate, axial_candidates
from .ik import IkResult, PinkIkSolver
from .model import _robotics_stack


@dataclass(frozen=True)
class GraspPlan:
    candidate: GraspCandidate
    pregrasp_pose: object
    grasp_pose: object
    retreat_pose: object
    pregrasp: IkResult
    grasp: IkResult
    retreat: IkResult
    score: float


def offset_along_local_z(pose, distance_m: float):
    np, pin = _robotics_stack()
    return pose * pin.SE3(np.eye(3), np.asarray([0.0, 0.0, distance_m]))


def _plan_candidate(
    solver: PinkIkSolver,
    candidate: GraspCandidate,
    current_q,
    pregrasp_distance_m: float,
    retreat_distance_m: float,
    workspace: WorkspaceLimits,
    collision_checker=None,
    maximum_path_step_rad: float = 0.01,
) -> GraspPlan | None:
    grasp_pose = candidate.target
    pregrasp_pose = offset_along_local_z(grasp_pose, -abs(pregrasp_distance_m))
    retreat_pose = offset_along_local_z(grasp_pose, -abs(retreat_distance_m))
    for pose in (pregrasp_pose, grasp_pose, retreat_pose):
        workspace.validate_pose(pose)

    pregrasp = solver.solve(pregrasp_pose, current_q)
    if not pregrasp.converged:
        return None
    grasp = solver.solve_seed(grasp_pose, pregrasp.q, posture_q=pregrasp.q)
    if not grasp.converged:
        return None
    retreat = solver.solve_seed(retreat_pose, grasp.q, posture_q=grasp.q)
    if not retreat.converged:
        return None
    if collision_checker is not None:
        segments = (
            (current_q, pregrasp.q),
            (pregrasp.q, grasp.q),
            (grasp.q, retreat.q),
        )
        try:
            for start_q, goal_q in segments:
                collision_checker.validate_path(
                    interpolate_joint_path(start_q, goal_q, maximum_path_step_rad)
                )
        except ValueError:
            return None
    score = pregrasp.score + grasp.score + retreat.score
    return GraspPlan(
        candidate,
        pregrasp_pose,
        grasp_pose,
        retreat_pose,
        pregrasp,
        grasp,
        retreat,
        score,
    )


def plan_grasp(
    solver: PinkIkSolver,
    base_grasp_pose,
    current_q,
    axial_angles_rad,
    pregrasp_distance_m: float = 0.10,
    retreat_distance_m: float = 0.10,
    workspace: WorkspaceLimits = WorkspaceLimits(),
    collision_checker=None,
    maximum_path_step_rad: float = 0.01,
) -> GraspPlan:
    plans = []
    for candidate in axial_candidates(base_grasp_pose, axial_angles_rad):
        plan = _plan_candidate(
            solver,
            candidate,
            current_q,
            pregrasp_distance_m,
            retreat_distance_m,
            workspace,
            collision_checker,
            maximum_path_step_rad,
        )
        if plan is not None:
            plans.append(plan)
    if not plans:
        raise RuntimeError(
            "No grasp candidate passed IK, workspace, and full-path collision checks"
        )
    return min(plans, key=lambda item: item.score)
