"""Pink-based inverse kinematics for a reduced six-axis Piper model."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable, Sequence

from .config import DEFAULT_IK_SETTINGS, IkSettings
from .model import PiperModel, _robotics_stack


def _pink_stack() -> tuple[Any, Any, Any, Any]:
    try:
        import pink
        from pink.tasks import FrameTask, PostureTask
        from qpsolvers import available_solvers
    except ImportError as exc:
        raise RuntimeError(
            "Pink dependencies are missing. Activate piper_pink and run: "
            "python -m pip install pin pin-pink qpsolvers quadprog"
        ) from exc
    return pink, FrameTask, PostureTask, available_solvers


@dataclass(frozen=True)
class IkResult:
    q: Any
    converged: bool
    iterations: int
    position_error_m: float
    orientation_error_rad: float
    score: float
    seed_index: int


class PinkIkSolver:
    """Solve full-pose IK with multiple seeds and deterministic scoring."""

    def __init__(
        self,
        piper_model: PiperModel,
        settings: IkSettings = DEFAULT_IK_SETTINGS,
        qp_solver: str | None = None,
    ) -> None:
        self.piper = piper_model
        self.settings = settings
        np, _ = _robotics_stack()
        numeric_settings = np.asarray(
            [
                settings.dt_s,
                settings.position_cost,
                settings.orientation_cost,
                settings.posture_cost,
                settings.lm_damping,
                settings.position_tolerance_m,
                settings.orientation_tolerance_rad,
                settings.joint_limit_margin_rad,
            ],
            dtype=float,
        )
        if not np.all(np.isfinite(numeric_settings)) or np.any(numeric_settings < 0.0):
            raise ValueError("IK settings must be finite and non-negative")
        if settings.dt_s <= 0.0 or settings.max_iterations <= 0:
            raise ValueError("IK time step and iteration count must be positive")
        if settings.random_seed_count < 0:
            raise ValueError("IK random seed count must not be negative")
        _, _, _, available_solvers = _pink_stack()
        installed = list(available_solvers)
        if qp_solver is not None and qp_solver not in installed:
            raise RuntimeError(
                f"QP solver '{qp_solver}' is unavailable; installed solvers: {installed}"
            )
        if qp_solver is None:
            for candidate in ("quadprog", "proxqp", "osqp", "clarabel"):
                if candidate in installed:
                    qp_solver = candidate
                    break
        if qp_solver is None:
            raise RuntimeError(
                "No supported QP solver is installed. Run: python -m pip install quadprog"
            )
        self.qp_solver = qp_solver

    def _tasks(self, configuration, target, posture_q):
        _, FrameTask, PostureTask, _ = _pink_stack()
        frame_task = FrameTask(
            self.piper.tcp_frame,
            position_cost=self.settings.position_cost,
            orientation_cost=self.settings.orientation_cost,
            lm_damping=self.settings.lm_damping,
        )
        frame_task.set_target(target)
        posture_task = PostureTask(cost=self.settings.posture_cost)
        posture_task.set_target(posture_q)
        return [frame_task, posture_task]

    def step(self, q, target, posture_q=None):
        """Run one feedback IK step and return a new joint configuration."""

        np, _ = _robotics_stack()
        pink, _, _, _ = _pink_stack()
        q = np.asarray(q, dtype=float).copy()
        self.piper.validate_q(q)
        if posture_q is None:
            posture_q = q
        configuration = pink.Configuration(self.piper.model, self.piper.data, q)
        tasks = self._tasks(configuration, target, np.asarray(posture_q, dtype=float))
        velocity = pink.solve_ik(
            configuration,
            tasks,
            self.settings.dt_s,
            solver=self.qp_solver,
        )
        configuration.integrate_inplace(velocity, self.settings.dt_s)
        q_next = np.asarray(configuration.q, dtype=float).copy()
        q_next = np.minimum(np.maximum(q_next, self.piper.lower_limits), self.piper.upper_limits)
        self.piper.validate_q(q_next)
        return q_next

    def solve_seed(self, target, seed_q, posture_q=None, seed_index: int = 0) -> IkResult:
        np, _ = _robotics_stack()
        q = np.asarray(seed_q, dtype=float).copy()
        self.piper.validate_q(q)
        posture = q.copy() if posture_q is None else np.asarray(posture_q, dtype=float)
        iterations = 0
        for iterations in range(1, self.settings.max_iterations + 1):
            position_error, orientation_error = self.piper.pose_error(q, target)
            if (
                position_error <= self.settings.position_tolerance_m
                and orientation_error <= self.settings.orientation_tolerance_rad
            ):
                break
            q = self.step(q, target, posture)

        position_error, orientation_error = self.piper.pose_error(q, target)
        converged = (
            position_error <= self.settings.position_tolerance_m
            and orientation_error <= self.settings.orientation_tolerance_rad
        )
        joint_distance = float(np.linalg.norm(q - posture))
        lower_clearance = q - self.piper.lower_limits
        upper_clearance = self.piper.upper_limits - q
        minimum_clearance = float(np.min(np.minimum(lower_clearance, upper_clearance)))
        limit_penalty = 1.0 / max(minimum_clearance, 1.0e-4)
        score = (
            1000.0 * position_error
            + 2.0 * orientation_error
            + 0.05 * joint_distance
            + 0.001 * limit_penalty
            + (0.0 if converged else 1000.0)
        )
        return IkResult(
            q=q,
            converged=converged,
            iterations=iterations,
            position_error_m=position_error,
            orientation_error_rad=orientation_error,
            score=score,
            seed_index=seed_index,
        )

    def generate_seeds(self, current_q, count: int | None = None, rng_seed: int = 7):
        np, _ = _robotics_stack()
        current = np.asarray(current_q, dtype=float)
        self.piper.validate_q(current)
        count = self.settings.random_seed_count if count is None else max(0, count)
        yield current.copy()
        neutral = self.piper.neutral()
        if not np.allclose(current, neutral):
            yield neutral
        rng = np.random.default_rng(rng_seed)
        margin = self.settings.joint_limit_margin_rad
        lower = self.piper.lower_limits + margin
        upper = self.piper.upper_limits - margin
        for _ in range(count):
            yield rng.uniform(lower, upper)

    def solve(self, target, current_q, extra_seeds: Iterable[Sequence[float]] = ()) -> IkResult:
        np, _ = _robotics_stack()
        current = np.asarray(current_q, dtype=float)
        seeds = list(self.generate_seeds(current))
        seeds.extend(np.asarray(seed, dtype=float) for seed in extra_seeds)
        results = [
            self.solve_seed(target, seed, posture_q=current, seed_index=index)
            for index, seed in enumerate(seeds)
        ]
        return min(results, key=lambda result: result.score)
