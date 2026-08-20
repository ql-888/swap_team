from pathlib import Path
import re

import numpy as np

from piper_pink.config import DEFAULT_TCP, JOINT_LIMITS_RAD, find_default_urdf
from piper_pink.ik import PinkIkSolver
from piper_pink.model import load_piper_model
from piper_pink.tool_axis import axis_plane_angle_rad, closing_axis_base, horizontal_j6_candidates


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def test_piper_x_urdf_meshes_are_project_relative_and_present():
    urdf = PROJECT_ROOT / "assets/piper_x_description/urdf/piper_x_with_gripper_description.urdf"
    text = urdf.read_text(encoding="utf-8")
    assert "/home/" not in text
    assert ".dae" not in text.lower()
    mesh_uris = re.findall(r'<mesh filename="([^"]+)"', text)
    assert mesh_uris
    for uri in mesh_uris:
        prefix = "package://"
        assert uri.startswith(prefix)
        assert (PROJECT_ROOT / "assets" / uri[len(prefix) :]).is_file(), uri


def test_default_tcp_matches_measured_physical_assembly():
    assert DEFAULT_TCP.parent_frame == "flange_link"
    assert np.allclose(DEFAULT_TCP.xyz_m, [0.0, 0.0, 0.198])
    assert np.allclose(DEFAULT_TCP.rpy_rad, np.deg2rad([0.0, 0.0, 48.2]))
    assert DEFAULT_TCP.frame_name == "piper_tcp"


def test_forward_tcp_accepts_feedback_tuple():
    piper = load_piper_model(find_default_urdf(), DEFAULT_TCP)
    pose = piper.forward_tcp(tuple(piper.neutral()))
    assert pose.translation.shape == (3,)


def test_read_only_forward_tcp_accepts_feedback_just_outside_planning_limits():
    piper = load_piper_model(find_default_urdf(), DEFAULT_TCP)
    q = np.array([0.0, -0.04, 0.05, 0.0, 0.0, 0.0])
    pose = piper.forward_tcp(q, validate_limits=False)
    assert pose.translation.shape == (3,)


def test_reduced_model_has_six_arm_joints():
    piper = load_piper_model(find_default_urdf(), DEFAULT_TCP)
    assert piper.model.name == "piper_x"
    assert piper.model.nq == 6
    assert piper.model.nv == 6
    assert piper.joint_names == tuple(f"joint{index}" for index in range(1, 7))
    expected = np.asarray(JOINT_LIMITS_RAD)
    assert np.allclose(piper.lower_limits, expected[:, 0])
    assert np.allclose(piper.upper_limits, expected[:, 1])


def test_horizontal_j6_candidates_level_the_modeled_closing_axis():
    piper = load_piper_model(find_default_urdf(), DEFAULT_TCP)
    q = np.array([0.30, 1.10, -1.20, 0.35, 0.20, -0.40])
    candidates = horizontal_j6_candidates(piper, q)
    assert candidates
    for candidate in candidates:
        trial = q.copy()
        trial[5] = candidate
        assert abs(axis_plane_angle_rad(closing_axis_base(piper, trial))) < 1.0e-8


def test_pink_recovers_a_reachable_pose():
    piper = load_piper_model(find_default_urdf(), DEFAULT_TCP)
    q_goal = np.array([0.25, 1.25, -1.35, 0.35, 0.25, -0.30])
    target = piper.forward_tcp(q_goal)
    solver = PinkIkSolver(piper)
    seed = np.array([0.0, 1.57, -1.57, 0.0, 0.0, 0.0])
    result = solver.solve(target, seed)
    assert result.converged
    assert result.position_error_m < 0.001
    assert result.orientation_error_rad < np.deg2rad(1.0)
