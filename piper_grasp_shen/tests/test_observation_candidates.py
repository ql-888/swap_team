from pathlib import Path
import subprocess
import sys

import numpy as np
import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_DIR = PROJECT_ROOT / "scripts/eye_to_grasp"
sys.path.insert(0, str(SCRIPT_DIR))

from make_observation_candidate_recipes import build_candidate_recipes
from observation_candidate_controls import build_single_sample_candidate, terminal_action


def transform(translation):
    return {
        "translation_m": list(translation),
        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
    }


def sample(flange_x):
    return {
        "joints_rad": [0.0] * 6,
        "base_from_global_tag": transform([0.0, 0.0, 0.0]),
        "local_camera_from_tag": transform([0.0, 0.0, 0.3]),
        "base_from_flange": transform([flange_x, 0.0, 0.1]),
    }


def test_candidate_recipes_preserve_recorded_order_and_orientation():
    record = {
        "schema_version": 2,
        "candidates": [
            {"candidate_id": "candidate_01", "samples": [sample(0.1), sample(0.1)]},
            {"candidate_id": "candidate_02", "samples": [sample(0.2), sample(0.2)]},
        ],
    }

    recipes = build_candidate_recipes(record)

    assert [item["candidate_id"] for item in recipes] == ["candidate_01", "candidate_02"]
    assert all(item["axial_angles_deg"] == [0.0] for item in recipes)
    np.testing.assert_allclose(
        np.asarray(recipes[0]["object_from_tcp"])[:3, 3],
        [0.1, 0.0, 0.298],
        atol=1e-9,
    )
    np.testing.assert_allclose(
        np.asarray(recipes[1]["object_from_tcp"])[:3, 3],
        [0.2, 0.0, 0.298],
        atol=1e-9,
    )


def test_recorder_is_headless_and_terminal_commands_are_unambiguous():
    text = (SCRIPT_DIR / "record_observation_candidates.py").read_text(encoding="utf-8")

    assert "create_publisher" not in text
    assert "cv2.imshow" not in text
    assert terminal_action("\n") == "capture"
    assert terminal_action("q\n") == "save"
    assert terminal_action("Q\n") == "save"
    assert terminal_action("") == "eof"
    assert terminal_action("anything else\n") == "invalid"


def test_one_snapshot_without_d435i_tag_is_a_complete_candidate():
    snapshot = sample(0.15)
    snapshot.pop("local_camera_from_tag")

    candidate = build_single_sample_candidate(
        "candidate_01",
        snapshot,
        timestamp_s=123.0,
        global_image="gemini.png",
        local_image="d435i.png",
    )

    assert candidate == {
        "candidate_id": "candidate_01",
        "timestamp_s": 123.0,
        "global_image": "gemini.png",
        "local_image": "d435i.png",
        "samples": [snapshot],
    }
    recipe = build_candidate_recipes(
        {"schema_version": 2, "candidates": [candidate]}
    )[0]
    assert recipe["candidate_id"] == "candidate_01"


def test_recipe_cli_prepends_legacy_candidate_and_renumbers_new_candidates(tmp_path):
    legacy_record = tmp_path / "legacy.yaml"
    new_record = tmp_path / "new.yaml"
    output_dir = tmp_path / "recipes"
    legacy_record.write_text(
        yaml.safe_dump(
            {"schema_version": 1, "samples": [sample(0.05), sample(0.9)]}
        ),
        encoding="utf-8",
    )
    new_record.write_text(
        yaml.safe_dump(
            {
                "schema_version": 2,
                "candidates": [
                    {"candidate_id": "candidate_01", "samples": [sample(0.1)]},
                    {"candidate_id": "candidate_02", "samples": [sample(0.2)]},
                ],
            }
        ),
        encoding="utf-8",
    )

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_DIR / "make_observation_candidate_recipes.py"),
            "--record",
            str(new_record),
            "--legacy-record",
            str(legacy_record),
            "--output-dir",
            str(output_dir),
        ],
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    recipes = [
        yaml.safe_load(path.read_text(encoding="utf-8"))
        for path in sorted(output_dir.glob("candidate_*.yaml"))
    ]
    assert [item["candidate_id"] for item in recipes] == [
        "candidate_01",
        "candidate_02",
        "candidate_03",
    ]
    assert [item["object_from_tcp"][0][3] for item in recipes] == [0.05, 0.1, 0.2]
    assert recipes[0]["observation_offset_source"] == str(legacy_record)


def test_d435i_viewer_is_separate_and_read_only():
    text = (SCRIPT_DIR / "view_d435i.py").read_text(encoding="utf-8")

    assert "cv2.imshow" in text
    assert "create_publisher" not in text


def test_d435i_startup_does_not_treat_gemini_static_tf_as_duplicate():
    text = (
        PROJECT_ROOT / "scripts/eye_in _grasp/start_vision_stack.sh"
    ).read_text(encoding="utf-8")

    duplicate_check = text.split("Vision-stack nodes are already running", 1)[0]
    assert "static_transform_publisher_" not in duplicate_check


def test_runtime_tries_exact_candidates_before_translation_fallbacks():
    text = (SCRIPT_DIR / "run_relative_observation.sh").read_text(encoding="utf-8")

    assert "observation_candidates.yaml" in text
    assert "dual_camera_observation.yaml" in text
    assert "--legacy-record" in text
    assert text.index("EXACT TAUGHT CANDIDATES") < text.index("TRANSLATION FALLBACKS")
    assert "[0.0, 90.0, 180.0, 270.0]" not in text
    assert 'item["axial_angles_deg"] = [0.0]' in text
