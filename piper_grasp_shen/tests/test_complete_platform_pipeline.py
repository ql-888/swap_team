from pathlib import Path
import os
import shutil
import subprocess


PROJECT_ROOT = Path(__file__).resolve().parents[1]
PIPELINE = PROJECT_ROOT / "scripts/to_platform/run_grasp_place_complete.sh"


def write_executable(path: Path, body: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("#!/usr/bin/env bash\nset -eu\n" + body, encoding="utf-8")
    path.chmod(0o755)


def fake_project(tmp_path: Path) -> tuple[Path, Path, dict[str, str]]:
    assert PIPELINE.exists(), "complete grasp-to-platform entry point is missing"
    root = tmp_path / "project"
    script = root / "scripts/to_platform/run_grasp_place_complete.sh"
    script.parent.mkdir(parents=True)
    shutil.copy2(PIPELINE, script)
    log = tmp_path / "pipeline.log"

    recorder = 'printf "%s\\n" "$(basename "$0") $*" >>"${PIPELINE_TEST_LOG}"\n'
    for relative in (
        "scripts/eye_to_grasp/boot_and_grasp.sh",
        "scripts/to_platform/run_platform_preplace_debug.sh",
        "scripts/to_platform/execute_platform_partial_release.py",
        "scripts/to_platform/execute_platform_open_retreat.py",
    ):
        write_executable(root / relative, recorder)

    write_executable(
        root / "scripts/run_apriltag_gemini336l_standard52h13.sh",
        recorder + "while true; do sleep 1; done\n",
    )
    gripper_count = tmp_path / "gripper-count"
    env_count = str(gripper_count)
    write_executable(
        root / "scripts/read_ros_gripper_m.py",
        f'count=$(cat "{env_count}" 2>/dev/null || echo 0)\n'
        f'count=$((count + 1))\n'
        f'echo "$count" >"{env_count}"\n'
        'if [[ "$count" -le 2 ]]; then echo "0.0621"; else echo "0.0718"; fi\n',
    )
    write_executable(
        root / "scripts/read_ros_joint_degrees.py",
        'printf "14.640\\n87.244\\n-104.060\\n66.137\\n-22.796\\n-68.715\\n"\n',
    )
    write_executable(root / "scripts/source_ros_env.sh", ":\n")
    write_executable(root / ".conda/bin/python", recorder)
    write_executable(
        root / "fake-bin/ros2",
        'printf "Type: fake\\nPublisher count: 1\\nSubscription count: 1\\n"\n',
    )
    env = os.environ.copy()
    env["PIPELINE_TEST_LOG"] = str(log)
    env["PATH"] = f"{root / 'fake-bin'}:{env['PATH']}"
    return script, log, env


def launch(script: Path, env: dict[str, str], answers: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        [
            str(script),
            "I_CONFIRM_COMPLETE_PLATFORM_PLACEMENT",
            "I_HAVE_CLEARED_THE_WORKSPACE",
            "I_CONFIRM_DRONE_AND_TAG_FIXED",
            "I_HAVE_CLEARED_THE_TRANSPORT_PATH",
        ],
        input=answers,
        text=True,
        capture_output=True,
        env=env,
        timeout=10,
    )


def test_complete_pipeline_runs_verified_stages_in_order_after_three_checkpoints(tmp_path):
    script, log, env = fake_project(tmp_path)

    result = launch(
        script,
        env,
        "\n".join(
            [
                "I_CONFIRM_HELD_DRONE_TRANSPORT",
                "I_CONFIRM_PLATFORM_ABOVE_DESCENT",
                "I_CONFIRM_DRONE_SUPPORTED_RELEASE",
            ]
        )
        + "\n",
    )

    assert result.returncode == 0, result.stderr
    lines = log.read_text(encoding="utf-8").splitlines()
    assert lines[:12] == [
        "boot_and_grasp.sh I_CONFIRM_BOOT_AND_GRASP I_HAVE_CLEARED_THE_WORKSPACE I_CONFIRM_DRONE_AND_TAG_FIXED",
        "run_apriltag_gemini336l_standard52h13.sh ",
        "run_platform_preplace_debug.sh plan",
        "run_platform_preplace_debug.sh lift I_CONFIRM_PLATFORM_SAFE_LIFT_ONLY",
        "run_platform_preplace_debug.sh transport I_CONFIRM_PLATFORM_TRANSPORT_POSITION_ONLY",
        "run_platform_preplace_debug.sh align I_CONFIRM_PLATFORM_TILTED_ALIGN_ONLY",
        "run_platform_preplace_debug.sh plan-descend",
        "run_platform_preplace_debug.sh descend I_CONFIRM_PLATFORM_DESCEND_100MM_ONLY",
        "run_platform_preplace_debug.sh plan-descend-10mm",
        "run_platform_preplace_debug.sh descend-10mm I_CONFIRM_PLATFORM_DESCEND_10MM_ONLY",
        "run_platform_preplace_debug.sh plan-descend-10mm",
        "run_platform_preplace_debug.sh descend-10mm I_CONFIRM_PLATFORM_DESCEND_10MM_ONLY",
    ]
    assert "plan_platform_release.py --current-width-m 0.0621 --delta-m 0.010" in lines[12]
    assert lines[13].startswith("execute_platform_partial_release.py --report ")
    assert lines[13].endswith(
        "--confirmation I_CONFIRM_PLATFORM_OPEN_GRIPPER_10MM_ONLY"
    )
    assert (
        "plan_platform_open_retreat.py --current-q-deg 14.640 87.244 -104.060 "
        "66.137 -22.796 -68.715 --retreat-m 0.120 --open-width-m 0.0718"
        in lines[14]
    )
    assert lines[15].startswith("execute_platform_open_retreat.py --report ")
    assert lines[15].endswith(
        "--confirmation I_CONFIRM_PLATFORM_OPEN_RETREAT_120MM_ONLY"
    )
    assert "COMPLETE_PLATFORM_PLACEMENT_FINISHED" in result.stdout


def test_complete_pipeline_stops_if_first_checkpoint_is_not_confirmed(tmp_path):
    script, log, env = fake_project(tmp_path)

    result = launch(script, env, "NO\n")

    assert result.returncode == 5
    assert log.read_text(encoding="utf-8").splitlines() == [
        "boot_and_grasp.sh I_CONFIRM_BOOT_AND_GRASP I_HAVE_CLEARED_THE_WORKSPACE I_CONFIRM_DRONE_AND_TAG_FIXED"
    ]
    assert "I_CONFIRM_HELD_DRONE_TRANSPORT" in result.stderr


def test_complete_pipeline_rejects_open_gripper_feedback_after_grasp(tmp_path):
    script, log, env = fake_project(tmp_path)
    root = script.parents[2]
    write_executable(root / "scripts/read_ros_gripper_m.py", 'echo "0.0718"\n')

    result = launch(script, env, "I_CONFIRM_HELD_DRONE_TRANSPORT\n")

    assert result.returncode != 0
    assert log.read_text(encoding="utf-8").splitlines() == [
        "boot_and_grasp.sh I_CONFIRM_BOOT_AND_GRASP I_HAVE_CLEARED_THE_WORKSPACE I_CONFIRM_DRONE_AND_TAG_FIXED"
    ]
    assert "held-drone gripper feedback out of range" in result.stderr


def test_complete_pipeline_never_descends_without_above_platform_confirmation(tmp_path):
    script, log, env = fake_project(tmp_path)

    result = launch(script, env, "I_CONFIRM_HELD_DRONE_TRANSPORT\nNO\n")

    assert result.returncode == 5
    lines = log.read_text(encoding="utf-8").splitlines()
    assert lines[-1] == (
        "run_platform_preplace_debug.sh align "
        "I_CONFIRM_PLATFORM_TILTED_ALIGN_ONLY"
    )
    assert not any("descend" in line for line in lines)
    assert "I_CONFIRM_PLATFORM_ABOVE_DESCENT" in result.stderr


def test_complete_pipeline_never_releases_without_supported_drone_confirmation(tmp_path):
    script, log, env = fake_project(tmp_path)

    result = launch(
        script,
        env,
        "I_CONFIRM_HELD_DRONE_TRANSPORT\nI_CONFIRM_PLATFORM_ABOVE_DESCENT\nNO\n",
    )

    assert result.returncode == 5
    lines = log.read_text(encoding="utf-8").splitlines()
    assert lines[-1] == (
        "run_platform_preplace_debug.sh descend-10mm "
        "I_CONFIRM_PLATFORM_DESCEND_10MM_ONLY"
    )
    assert not any("release" in line or "retreat" in line for line in lines)
    assert "I_CONFIRM_DRONE_SUPPORTED_RELEASE" in result.stderr


def test_complete_pipeline_rejects_bad_start_confirmation_before_motion():
    assert PIPELINE.exists(), "complete grasp-to-platform entry point is missing"

    result = subprocess.run(
        [str(PIPELINE), "WRONG"], text=True, capture_output=True
    )

    assert result.returncode == 2
    assert "I_CONFIRM_COMPLETE_PLATFORM_PLACEMENT" in result.stderr
    assert "ROS" not in result.stderr
