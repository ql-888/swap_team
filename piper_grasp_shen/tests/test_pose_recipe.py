from pathlib import Path

import numpy as np
import pytest
import yaml

from piper_pink.pose_io import load_object_pose
from piper_pink.recipe import GraspRecipe
from scripts.plan_standard52_transport import target_matrix_from_tag


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def test_example_pose_and_recipe_match():
    pose = load_object_pose(PROJECT_ROOT / "config/object_pose.example.yaml")
    recipe = GraspRecipe.load(PROJECT_ROOT / "config/grasp_recipe.example.yaml")
    assert pose.object_id == recipe.object_id
    assert pose.frame_id == "base_link"
    assert recipe.pregrasp_distance_m > 0.0
    assert recipe.execution_ready is False


def test_recipe_rejects_empty_axial_candidates(tmp_path):
    path = tmp_path / "recipe.yaml"
    path.write_text(
        yaml.safe_dump(
            {
                "object_id": "part",
                "object_from_tcp": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
                "axial_angles_deg": [],
                "pregrasp_distance_m": 0.1,
                "retreat_distance_m": 0.1,
                "gripper_opening_m": 0.05,
                "gripper_closed_m": 0.01,
                "execution_ready": False,
            }
        ),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="axial"):
        GraspRecipe.load(path)


def test_drone_recipe_accepts_85mm_gripper_opening():
    recipe_names = [
        "grasp_recipe_drone_apriltag_top.yaml",
        "grasp_recipe_drone_apriltag_top_approach50.yaml",
        "grasp_recipe_drone_apriltag_top_approach25.yaml",
    ]
    for recipe_name in recipe_names:
        recipe = GraspRecipe.load(PROJECT_ROOT / "config" / recipe_name)
        assert recipe.gripper_opening_m == pytest.approx(0.085)
        assert recipe.gripper_closed_m == pytest.approx(0.058)
        assert recipe.object_from_tcp[2, 3] == pytest.approx(-0.045)


def test_standard52_transport_aligns_closing_axis_with_tag_x():
    recipe = GraspRecipe.load(
        PROJECT_ROOT / "config/grasp_recipe_drone_apriltag_top.yaml"
    )
    base_from_tag = np.eye(4)
    base_from_tag[:3, :3] = np.asarray(
        [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
    )
    base_from_tag[:3, 3] = [0.2, 0.3, 0.05]
    target = target_matrix_from_tag(base_from_tag, recipe.object_from_tcp, 0.15)
    assert np.allclose(target[:3, 0], base_from_tag[:3, 0])
    assert np.allclose(target[:3, 3], [0.2, 0.3, 0.2])
