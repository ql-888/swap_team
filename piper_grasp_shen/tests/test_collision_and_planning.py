from pathlib import Path

import numpy as np
import yaml

from piper_pink.collision import PiperSelfCollisionChecker
from piper_pink.config import DEFAULT_TCP, find_default_urdf
from piper_pink.ik import PinkIkSolver
from piper_pink.model import load_piper_model
from piper_pink.planning import plan_grasp


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def test_known_pose_has_no_self_collision():
    checker = PiperSelfCollisionChecker(
        find_default_urdf(),
        PROJECT_ROOT / "assets/piper_x_moveit/piper_x.srdf",
        PROJECT_ROOT / "config/scene.yaml",
    )
    q = np.array([0.0, 1.57, -1.57, 0.0, 0.0, 0.0])
    assert checker.collision_pairs(q) == []


def test_scene_obstacle_collision_is_detected(tmp_path):
    scene = tmp_path / "blocked_scene.yaml"
    scene.write_text(
        yaml.safe_dump(
            {
                "boxes": [
                    {
                        "name": "blocked_volume",
                        "size_m": [1.0, 1.0, 1.0],
                        "xyz_m": [0.0, 0.0, 0.35],
                        "rpy_deg": [0.0, 0.0, 0.0],
                        "ignore_links": ["base_link"],
                    }
                ]
            }
        ),
        encoding="utf-8",
    )
    checker = PiperSelfCollisionChecker(
        find_default_urdf(),
        PROJECT_ROOT / "assets/piper_x_moveit/piper_x.srdf",
        scene,
    )
    q = np.array([0.0, 1.57, -1.57, 0.0, 0.0, 0.0])
    assert checker.collision_pairs(q)


def test_complete_grasp_plan_converges():
    piper = load_piper_model(find_default_urdf(), DEFAULT_TCP)
    solver = PinkIkSolver(piper)
    known_reachable = np.array([0.0, 1.0, -1.0, 0.0, 0.0, 0.0])
    base_grasp = piper.forward_tcp(known_reachable)
    current = np.array([0.0, 1.57, -1.57, 0.0, 0.0, 0.0])
    plan = plan_grasp(solver, base_grasp, current, np.deg2rad([0]))
    assert plan.pregrasp.converged
    assert plan.grasp.converged
    assert plan.retreat.converged


def test_grasp_planner_checks_all_joint_path_segments():
    class CountingChecker:
        def __init__(self):
            self.paths = []

        def validate_path(self, path):
            self.paths.append(path)

    piper = load_piper_model(find_default_urdf(), DEFAULT_TCP)
    solver = PinkIkSolver(piper)
    known_reachable = np.array([0.0, 1.0, -1.0, 0.0, 0.0, 0.0])
    base_grasp = piper.forward_tcp(known_reachable)
    current = np.array([0.0, 1.57, -1.57, 0.0, 0.0, 0.0])
    checker = CountingChecker()
    plan_grasp(
        solver,
        base_grasp,
        current,
        np.deg2rad([0]),
        collision_checker=checker,
    )
    assert len(checker.paths) == 3
    assert all(path for path in checker.paths)
