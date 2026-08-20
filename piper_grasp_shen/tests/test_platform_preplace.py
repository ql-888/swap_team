from pathlib import Path
import importlib.util
import subprocess
import sys

import numpy as np
import pytest
import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def load_platform_planner():
    path = PROJECT_ROOT / "scripts/to_platform/plan_platform_above.py"
    spec = importlib.util.spec_from_file_location("plan_platform_above", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def load_platform_executor():
    path = PROJECT_ROOT / "scripts/to_platform/execute_platform_stage.py"
    spec = importlib.util.spec_from_file_location("execute_platform_stage", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def load_platform_descent_planner():
    path = PROJECT_ROOT / "scripts/to_platform/plan_platform_descent.py"
    spec = importlib.util.spec_from_file_location("plan_platform_descent", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def load_platform_release_planner():
    path = PROJECT_ROOT / "scripts/to_platform/plan_platform_release.py"
    spec = importlib.util.spec_from_file_location("plan_platform_release", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def load_platform_retreat_planner():
    path = PROJECT_ROOT / "scripts/to_platform/plan_platform_open_retreat.py"
    spec = importlib.util.spec_from_file_location("plan_platform_open_retreat", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_platform_detector_uses_21_9mm_pose_size_for_36_5mm_outer_edge():
    config = yaml.safe_load(
        (
            PROJECT_ROOT
            / "config/apriltag_gemini336l_live_standard52h13.yaml"
        ).read_text(encoding="utf-8")
    )["/**"]["ros__parameters"]

    assert config["family"] == "Standard52h13"
    assert config["size"] == pytest.approx(0.0219)
    assert config["tag"]["ids"] == [0]
    assert config["tag"]["sizes"] == pytest.approx([0.0219])


def test_preplacement_targets_use_safe_transport_tilt_then_selected_tilt_at_fixed_xyz():
    planner = load_platform_planner()
    base_from_tag = np.eye(4)
    base_from_tag[:3, :3] = np.array(
        [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
    )
    base_from_tag[:3, 3] = [0.31, -0.12, 0.99]
    current_tcp = np.eye(4)
    current_tcp[:3, :3] = np.array(
        [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]]
    )
    current_tcp[:3, 3] = [0.20, -0.20, 0.25]

    targets = planner.build_preplacement_targets(
        base_from_tag,
        current_tcp,
        np.eye(4),
        platform_height_m=0.170,
        object_height_above_tag_m=0.200,
        lift_waypoint_m=0.100,
        transport_tilt_deg=30.0,
        alignment_tilt_deg=20.0,
    )

    assert set(targets) == {"lift", "transport", "align"}
    assert np.allclose(targets["lift"][:3, 3], [0.20, -0.20, 0.35])
    assert np.allclose(targets["lift"][:3, :3], current_tcp[:3, :3])
    assert np.allclose(targets["transport"][:3, 3], [0.31, -0.12, 0.370])
    assert np.allclose(targets["transport"][:3, 0], [0.0, 1.0, 0.0])
    assert np.dot(targets["transport"][:3, 2], [0.0, 0.0, 1.0]) == pytest.approx(
        np.cos(np.deg2rad(30.0))
    )
    assert np.allclose(targets["align"][:3, 3], [0.31, -0.12, 0.370])
    assert np.allclose(targets["align"][:3, 0], [0.0, 1.0, 0.0])
    assert np.dot(targets["align"][:3, 2], [0.0, 0.0, 1.0]) == pytest.approx(
        np.cos(np.deg2rad(20.0))
    )


def test_preplacement_targets_ignore_measured_tag_z():
    planner = load_platform_planner()
    first = np.eye(4)
    second = np.eye(4)
    first[:3, 3] = [0.2, 0.3, 0.05]
    second[:3, 3] = [0.2, 0.3, 0.80]

    first_targets = planner.build_preplacement_targets(
        first, np.eye(4), np.eye(4), 0.170, 0.200, 0.100, 30.0, 20.0
    )
    second_targets = planner.build_preplacement_targets(
        second, np.eye(4), np.eye(4), 0.170, 0.200, 0.100, 30.0, 20.0
    )

    assert np.allclose(first_targets["transport"], second_targets["transport"])
    assert np.allclose(first_targets["align"], second_targets["align"])


def test_platform_stage_rejects_wrong_confirmation_before_ros(tmp_path):
    report = tmp_path / "report.yaml"
    report.write_text(
        yaml.safe_dump(
            {
                "mode": "platform_preplacement_debug",
                "stages": {
                    "lift": {"q_deg": [1, 2, 3, 4, 5, 6]},
                    "transport": {"q_deg": [2, 3, 4, 5, 6, 7]},
                    "align": {"q_deg": [3, 4, 5, 6, 7, 8]},
                },
            }
        ),
        encoding="utf-8",
    )

    result = subprocess.run(
        [
            sys.executable,
            str(PROJECT_ROOT / "scripts/to_platform/execute_platform_stage.py"),
            "--report",
            str(report),
            "--stage",
            "lift",
            "--confirmation",
            "WRONG",
        ],
        text=True,
        capture_output=True,
    )

    assert result.returncode != 0
    assert "I_CONFIRM_PLATFORM_SAFE_LIFT_ONLY" in result.stderr
    assert "rclpy" not in result.stderr


def test_platform_stage_delegates_to_hold_mode_with_58mm_close_command():
    executor = load_platform_executor()

    command = executor.build_executor_command(Path("/tmp/selected-stage.yaml"))

    assert command[0] == sys.executable
    assert command[1] == str(PROJECT_ROOT / "scripts/execute_ros_pregrasp.py")
    assert command[command.index("--mode") + 1] == "transport_hold"
    assert command[command.index("--gripper-closed-m") + 1] == "0.058"
    assert command[command.index("--confirmation") + 1] == (
        "I_CONFIRM_TRANSPORT_WITH_GRIP_TO_STANDARD52"
    )
    assert "grasp_close" not in command


def test_debug_launcher_without_action_prints_only_supported_stages():
    result = subprocess.run(
        [str(PROJECT_ROOT / "scripts/to_platform/run_platform_preplace_debug.sh")],
        text=True,
        capture_output=True,
    )

    assert result.returncode == 2
    assert (
        "plan|lift|transport|align|plan-descend|descend|plan-descend-10mm|descend-10mm"
        in result.stderr
    )
    assert "release" not in result.stderr.lower()


def test_debug_launcher_rejects_wrong_stage_confirmation_before_ros():
    result = subprocess.run(
        [
            str(PROJECT_ROOT / "scripts/to_platform/run_platform_preplace_debug.sh"),
            "lift",
            "WRONG",
        ],
        text=True,
        capture_output=True,
    )

    assert result.returncode == 2
    assert "I_CONFIRM_PLATFORM_SAFE_LIFT_ONLY" in result.stderr
    assert "gripper" not in result.stderr.lower()
    assert "Traceback" not in result.stderr


def test_debug_launcher_names_tilted_alignment_confirmation():
    result = subprocess.run(
        [
            str(PROJECT_ROOT / "scripts/to_platform/run_platform_preplace_debug.sh"),
            "align",
            "WRONG",
        ],
        text=True,
        capture_output=True,
    )

    assert result.returncode == 2
    assert "I_CONFIRM_PLATFORM_TILTED_ALIGN_ONLY" in result.stderr
    assert "HORIZONTAL_ALIGN" not in result.stderr


def test_descent_target_moves_current_object_down_100mm_without_rotation_change():
    planner = load_platform_descent_planner()
    current_tcp = np.eye(4)
    current_tcp[:3, 3] = [0.42, 0.20, 0.33]
    object_from_tcp = np.eye(4)
    object_from_tcp[2, 3] = -0.045

    target_tcp, target_object = planner.build_descent_target(
        current_tcp, object_from_tcp, descent_m=0.100
    )

    assert np.allclose(target_object[:3, 3], [0.42, 0.20, 0.275])
    assert np.allclose(target_tcp[:3, 3], [0.42, 0.20, 0.230])
    assert np.allclose(target_tcp[:3, :3], current_tcp[:3, :3])


def test_debug_launcher_rejects_wrong_descent_confirmation_before_ros():
    result = subprocess.run(
        [
            str(PROJECT_ROOT / "scripts/to_platform/run_platform_preplace_debug.sh"),
            "descend",
            "WRONG",
        ],
        text=True,
        capture_output=True,
    )

    assert result.returncode == 2
    assert "I_CONFIRM_PLATFORM_DESCEND_100MM_ONLY" in result.stderr
    assert "gripper" not in result.stderr.lower()


def test_debug_launcher_rejects_wrong_10mm_descent_confirmation_before_ros():
    result = subprocess.run(
        [
            str(PROJECT_ROOT / "scripts/to_platform/run_platform_preplace_debug.sh"),
            "descend-10mm",
            "WRONG",
        ],
        text=True,
        capture_output=True,
    )

    assert result.returncode == 2
    assert "I_CONFIRM_PLATFORM_DESCEND_10MM_ONLY" in result.stderr
    assert "100MM" not in result.stderr


def test_partial_release_adds_exactly_10mm_to_feedback_width():
    planner = load_platform_release_planner()

    assert planner.partial_open_target(0.0621, 0.010) == pytest.approx(0.0721)


def test_partial_release_rejects_target_above_debug_open_limit():
    planner = load_platform_release_planner()

    with pytest.raises(ValueError, match="opening limit"):
        planner.partial_open_target(0.080, 0.010)


def test_open_retreat_moves_tcp_up_200mm_without_rotation_or_xy_change():
    planner = load_platform_retreat_planner()
    current_tcp = np.eye(4)
    current_tcp[:3, :3] = np.array(
        [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
    )
    current_tcp[:3, 3] = [0.44, 0.21, 0.21]

    target = planner.build_open_retreat_target(current_tcp, retreat_m=0.200)

    assert np.allclose(target[:3, 3], [0.44, 0.21, 0.41])
    assert np.allclose(target[:3, :3], current_tcp[:3, :3])
