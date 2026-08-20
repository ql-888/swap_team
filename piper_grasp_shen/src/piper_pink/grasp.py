"""Generate and rank simple axial grasp candidates."""

from __future__ import annotations

from dataclasses import dataclass

from .ik import IkResult, PinkIkSolver
from .model import _robotics_stack


@dataclass(frozen=True)
class GraspCandidate:
    axial_angle_rad: float
    target: object


@dataclass(frozen=True)
class SolvedGrasp:
    candidate: GraspCandidate
    result: IkResult


def axial_candidates(target, angles_rad) -> list[GraspCandidate]:
    """Rotate a target about its local Z axis without changing translation."""

    np, pin = _robotics_stack()
    candidates = []
    axis = np.asarray([0.0, 0.0, 1.0])
    for angle in angles_rad:
        axial_rotation = pin.AngleAxis(float(angle), axis).matrix()
        pose = pin.SE3(target.rotation @ axial_rotation, target.translation.copy())
        candidates.append(GraspCandidate(float(angle), pose))
    return candidates


def solve_grasp_candidates(
    solver: PinkIkSolver, candidates: list[GraspCandidate], current_q
) -> list[SolvedGrasp]:
    """Solve all candidates and return converged solutions ordered by score."""

    solved = [SolvedGrasp(candidate, solver.solve(candidate.target, current_q)) for candidate in candidates]
    return sorted(
        (item for item in solved if item.result.converged),
        key=lambda item: item.result.score,
    )
