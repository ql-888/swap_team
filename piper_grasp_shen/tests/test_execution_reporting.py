from pathlib import Path

import numpy as np
import pytest

from piper_pink.execution import interpolate_joint_path, wait_for_joint_target
from piper_pink.reporting import save_plan_report
from piper_pink.transforms import load_yaml


def test_interpolation_rejects_nonfinite_endpoint():
    with pytest.raises(ValueError, match="NaN"):
        interpolate_joint_path(np.zeros(6), [0, 0, 0, 0, 0, np.nan], 0.01)


def test_wait_for_joint_target_uses_measured_feedback():
    class Model:
        def validate_q(self, q):
            assert np.asarray(q).shape == (6,)
    class Safety:
        execution_timeout_s = 0.1
        control_period_s = 0.0
    class Feedback:
        def __init__(self, q): self.q_rad = tuple(q)
    class Bridge:
        safety = Safety()
        def __init__(self): self.calls = 0
        def wait_for_feedback(self):
            self.calls += 1
            return Feedback(np.full(6, min(1.0, self.calls / 3.0)))
        def command(self, q, model): self.last_command = np.asarray(q)
    bridge = Bridge()
    result = wait_for_joint_target(
        bridge, Model(), np.ones(6), tolerance_rad=1e-6, timeout_s=0.2
    )
    assert bridge.calls == 3
    assert np.allclose(result.q_rad, 1.0)


def test_plan_report_is_serializable(tmp_path, monkeypatch):
    class Pose:
        rotation = np.eye(3)
        translation = np.array([0.3, 0.0, 0.2])

    class Result:
        q = np.zeros(6)
        position_error_m = 0.0002
        orientation_error_rad = 0.001

    class Candidate:
        axial_angle_rad = 0.0

    class Plan:
        candidate = Candidate()
        score = 1.0
        pregrasp_pose = Pose()
        grasp_pose = Pose()
        retreat_pose = Pose()
        pregrasp = Result()
        grasp = Result()
        retreat = Result()

    output = tmp_path / "report.yaml"
    save_plan_report(output, Plan(), np.zeros(6), Path("pose.yaml"), Path("recipe.yaml"))
    data = load_yaml(output)
    assert data["schema_version"] == 1
    assert set(data["stages"]) == {"pregrasp", "grasp", "retreat"}
