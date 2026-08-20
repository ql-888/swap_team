"""Guarded grasp state machine built on planned joint configurations."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto

from .execution import execute_joint_path, interpolate_joint_path


class GraspState(Enum):
    CREATED = auto()
    VALIDATED = auto()
    ENABLED = auto()
    PREGRASP_REACHED = auto()
    GRASP_REACHED = auto()
    GRIPPER_CLOSED = auto()
    RETREATED = auto()
    COMPLETE = auto()
    FAILED = auto()


@dataclass
class GraspExecutionReport:
    state: GraspState = GraspState.CREATED
    message: str = ""


class GraspExecutor:
    def __init__(self, bridge, model, collision_checker) -> None:
        self.bridge = bridge
        self.model = model
        self.collision_checker = collision_checker

    def _segment(self, start_q, goal_q):
        path = interpolate_joint_path(
            start_q, goal_q, self.bridge.safety.maximum_joint_step_rad
        )
        self.collision_checker.validate_path(path)
        execute_joint_path(self.bridge, self.model, path)
        return path[-1]

    def execute(
        self,
        plan,
        gripper_opening_m: float = 0.07,
        gripper_closed_m: float = 0.0,
    ) -> GraspExecutionReport:
        report = GraspExecutionReport()
        failed = False
        try:
            feedback = self.bridge.wait_for_feedback()
            for q in (plan.pregrasp.q, plan.grasp.q, plan.retreat.q):
                self.model.validate_q(q)
                self.collision_checker.validate_q(q)
            report.state = GraspState.VALIDATED

            self.bridge.enable()
            report.state = GraspState.ENABLED
            self.bridge.command_gripper(gripper_opening_m)

            current_q = self._segment(feedback.q_rad, plan.pregrasp.q)
            report.state = GraspState.PREGRASP_REACHED
            current_q = self._segment(current_q, plan.grasp.q)
            report.state = GraspState.GRASP_REACHED

            self.bridge.command_gripper(gripper_closed_m)
            report.state = GraspState.GRIPPER_CLOSED
            current_q = self._segment(current_q, plan.retreat.q)
            report.state = GraspState.RETREATED
            del current_q

            report.state = GraspState.COMPLETE
            report.message = "Grasp sequence completed"
            return report
        except Exception as exc:
            failed = True
            report.state = GraspState.FAILED
            report.message = str(exc)
            raise
        finally:
            try:
                self.bridge.hold_current_position()
            except Exception:
                if not failed:
                    raise
