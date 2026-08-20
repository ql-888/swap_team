"""Serializable run reports for offline plans and guarded hardware attempts."""

from __future__ import annotations

import math
import time

from .transforms import save_yaml


def _pose_matrix(pose):
    import numpy as np

    value = np.eye(4)
    value[:3, :3] = pose.rotation
    value[:3, 3] = pose.translation
    return value.tolist()


def plan_report(plan, current_q, object_pose_path, recipe_path, mode="offline"):
    return {
        "schema_version": 1,
        "created_at_unix_s": time.time(),
        "mode": mode,
        "inputs": {
            "object_pose": str(object_pose_path),
            "recipe": str(recipe_path),
            "current_q_deg": [math.degrees(float(value)) for value in current_q],
        },
        "selected_axial_deg": math.degrees(plan.candidate.axial_angle_rad),
        "score": float(plan.score),
        "stages": {
            name: {
                "target": _pose_matrix(pose),
                "q_deg": [math.degrees(float(value)) for value in result.q],
                "position_error_mm": result.position_error_m * 1000.0,
                "orientation_error_deg": math.degrees(result.orientation_error_rad),
            }
            for name, pose, result in (
                ("pregrasp", plan.pregrasp_pose, plan.pregrasp),
                ("grasp", plan.grasp_pose, plan.grasp),
                ("retreat", plan.retreat_pose, plan.retreat),
            )
        },
    }


def save_plan_report(path, plan, current_q, object_pose_path, recipe_path, mode="offline"):
    save_yaml(path, plan_report(plan, current_q, object_pose_path, recipe_path, mode))
