import os
from pathlib import Path
import subprocess

from piper_pink.__main__ import main


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def test_execute_confirmation_is_checked_before_can_access(capsys):
    result = main(
        [
            "grasp",
            "--object-pose",
            str(PROJECT_ROOT / "config/object_pose.example.yaml"),
            "--recipe",
            str(PROJECT_ROOT / "config/grasp_recipe.example.yaml"),
            "--execute",
        ]
    )
    captured = capsys.readouterr()
    assert result == 2
    assert "I_HAVE_CLEARED_THE_WORKSPACE" in captured.err
    assert "can0" not in captured.err


def test_missing_can_is_reported_without_sdk_traceback(capsys):
    result = main(["read-arm", "--can", "definitely_missing_can"])
    captured = capsys.readouterr()
    assert result == 1
    assert "does not exist" in captured.err


def test_test_points_confirmation_is_checked_before_can_access(capsys):
    result = main(
        [
            "test-points",
            "--points",
            str(PROJECT_ROOT / "config/test_points.example.yaml"),
            "--execute",
            "--can",
            "definitely_missing_can",
        ]
    )
    captured = capsys.readouterr()
    assert result == 2
    assert "I_HAVE_CLEARED_THE_WORKSPACE" in captured.err
    assert "definitely_missing_can" not in captured.err


def test_pregrasp_confirmation_is_checked_before_can_access(capsys):
    result = main(
        [
            "pregrasp",
            "--object-pose",
            str(PROJECT_ROOT / "config/object_pose.example.yaml"),
            "--recipe",
            str(PROJECT_ROOT / "config/grasp_recipe_apriltag_top.yaml"),
            "--execute",
            "--can",
            "definitely_missing_can",
        ]
    )
    captured = capsys.readouterr()
    assert result == 2
    assert "I_CONFIRM_PREGRASP_ONLY" in captured.err
    assert "definitely_missing_can" not in captured.err


def test_transport_mode_requires_specific_confirmation(tmp_path):
    import subprocess
    import yaml

    report = tmp_path / "transport.yaml"
    report.write_text(
        yaml.safe_dump({"stages": {"transport": {"q_deg": [0] * 6}}}),
        encoding="utf-8",
    )
    environment = os.environ.copy()
    environment["PYTHONPATH"] = ":".join(
        (
            "/opt/ros/humble/lib/python3.10/site-packages",
            "/opt/ros/humble/local/lib/python3.10/dist-packages",
        )
    )
    result = subprocess.run(
        [
            str(PROJECT_ROOT / "scripts/execute_ros_pregrasp.py"),
            "--report", str(report),
            "--confirmation", "WRONG",
            "--mode", "transport_hold",
        ],
        text=True,
        capture_output=True,
        env=environment,
    )
    assert result.returncode != 0
    assert "Incorrect ROS pregrasp confirmation" in result.stderr


def test_setup_env_reuses_project_environment_without_dependency_resolution():
    script = (PROJECT_ROOT / "scripts/setup_env.sh").read_text(encoding="utf-8")
    assert 'ENV_DIR="${PROJECT_DIR}/.conda"' in script
    assert "pip install --no-deps -e" in script.replace("\\\n", " ")
    assert "pin-pink==" not in script


def test_ros_python_entrypoints_use_system_python():
    names = (
        "capture_d435i_apriltag_pose.py",
        "capture_gemini_standard52_target.py",
        "execute_ros_pregrasp.py",
        "live_gemini336l_dual_tag_viewer.py",
        "read_ros_gripper_m.py",
        "read_ros_joint_degrees.py",
    )
    for name in names:
        first_line = (PROJECT_ROOT / "scripts" / name).read_text(encoding="utf-8").splitlines()[0]
        assert first_line == "#!/usr/bin/python3", name


def test_offline_verification_does_not_write_python_caches_into_project():
    script = (PROJECT_ROOT / "scripts/verify_offline.sh").read_text(encoding="utf-8")
    assert "PYTHONDONTWRITEBYTECODE=1" in script
    assert "-p no:cacheprovider" in script
    assert "py_compile src/piper_pink/*.py" not in script


def test_ros_shell_entrypoints_use_current_workspace_loader():
    helper = PROJECT_ROOT / "scripts/source_ros_env.sh"
    text = helper.read_text(encoding="utf-8")
    assert "PIPER_ROS_SETUP" in text
    assert "gy_ws/handeye/install/setup.bash" in text
    assert "AGX_ARM_ROS_SETUP" in text
    assert "agx_arm_ws/install/setup.bash" in text
    for script in (PROJECT_ROOT / "scripts").glob("*.sh"):
        content = script.read_text(encoding="utf-8")
        if script.name != "source_ros_env.sh":
            assert "agx_arm_ws/install/setup.bash" not in content, script.name


def test_ros_environment_loader_is_safe_when_nounset_is_already_enabled():
    helper = PROJECT_ROOT / "scripts/source_ros_env.sh"
    result = subprocess.run(
        [
            "bash",
            "-uc",
            'source "$1"; printf "ROS_ENV_READY\\n"',
            "source-check",
            str(helper),
        ],
        text=True,
        capture_output=True,
    )

    assert result.returncode == 0, result.stderr
    assert result.stdout == "ROS_ENV_READY\n"


def test_eye_to_grasp_boot_configures_can_before_starting_robot():
    script = (
        PROJECT_ROOT / "scripts/eye_to_grasp/boot_and_grasp.sh"
    ).read_text(encoding="utf-8")

    assert 'setup_can.sh" can0 1000000' in script
    assert script.index('setup_can.sh" can0 1000000') < script.index(
        'start_robot_stack.sh'
    )


def test_piper_driver_launches_forward_the_control_gate():
    control = (PROJECT_ROOT / "scripts/start_piper_x_ros_control_5pct.sh").read_text(
        encoding="utf-8"
    )
    readonly = (PROJECT_ROOT / "scripts/start_piper_x_ros_readonly.sh").read_text(
        encoding="utf-8"
    )
    for content in (control, readonly):
        assert "start_single_agx_arm.launch.py" in content
        assert "start_single_agx_arm_rviz.launch.py" not in content
        assert "custom_model:=" not in content
    assert "control_enabled:=true" in control
    assert "speed_percent:=5" in control
    assert "control_enabled:=false" in readonly


def test_drone_grasp_only_preflight_reports_the_physical_contract():
    result = subprocess.run(
        [
            str(PROJECT_ROOT / "scripts/run_full_d435i_drone_grasp_retreat.sh"),
            "--preflight",
        ],
        text=True,
        capture_output=True,
    )

    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines() == [
        "mode: preflight_only",
        "tag_size_m: 0.029",
        "tcp_below_tag_m: 0.045",
        "gripper_open_m: 0.085",
        "gripper_closed_m: 0.058",
        (
            "stages: capture_plan,pregrasp_100mm,approach_50mm,"
            "approach_25mm,grasp_close,retreat_hold"
        ),
    ]


def test_grouped_eye_in_grasp_launchers_report_their_dry_run_components():
    launcher_dir = PROJECT_ROOT / "scripts" / "eye_in _grasp"
    expected = {
        "start_robot_stack.sh": [
            "mode: dry_run",
            "stack: robot",
            "component: piper_x_control_5pct",
            "component: piper_x_robot_state_publisher",
        ],
        "start_vision_stack.sh": [
            "mode: dry_run",
            "stack: vision",
            "component: d435i_camera",
            "component: d435i_handeye_tf",
            "component: d435i_drone_apriltag_29mm",
        ],
    }

    for name, expected_lines in expected.items():
        result = subprocess.run(
            ["bash", str(launcher_dir / name), "--dry-run"],
            text=True,
            capture_output=True,
        )
        assert result.returncode == 0, result.stderr
        assert result.stdout.splitlines() == expected_lines
