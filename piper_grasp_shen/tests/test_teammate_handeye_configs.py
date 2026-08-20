import json
from pathlib import Path

import numpy as np
import yaml

from piper_pink.calibration import HandEyeCalibration


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _load_yaml(path):
    with path.open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _quaternion_xyzw_to_matrix(quaternion):
    x, y, z, w = np.asarray(quaternion, dtype=float)
    scale = 2.0 / float(x * x + y * y + z * z + w * w)
    return np.asarray(
        [
            [1.0 - scale * (y * y + z * z), scale * (x * y - z * w), scale * (x * z + y * w)],
            [scale * (x * y + z * w), 1.0 - scale * (x * x + z * z), scale * (y * z - x * w)],
            [scale * (x * z - y * w), scale * (y * z + x * w), 1.0 - scale * (x * x + y * y)],
        ]
    )


def test_orbbec_fixed_teammate_result_is_runtime_loadable():
    calibration = HandEyeCalibration.load(
        PROJECT_ROOT / "config" / "handeye_orbbec_fixed.yaml"
    )
    assert calibration.camera_mount == "fixed"
    assert calibration.parent_frame == "base_link"
    assert calibration.camera_frame == "camera_color_optical_frame"
    assert np.allclose(
        calibration.parent_from_camera[:3, 3],
        [0.6589229214393953, 0.02259948292410663, 0.7379360892425801],
    )


def test_d435i_result_uses_confirmed_flange_parent():
    calibration = HandEyeCalibration.load(
        PROJECT_ROOT / "config" / "handeye_d435i_wrist_end_pose.yaml"
    )
    assert calibration.camera_mount == "wrist"
    assert calibration.parent_frame == "flange_link"
    assert calibration.camera_frame == "d435i_color_optical_frame"
    assert np.isclose(np.linalg.det(calibration.parent_from_camera[:3, :3]), 1.0)


def test_d435i_source_is_project_local_and_matches_runtime_transform():
    config = _load_yaml(PROJECT_ROOT / "config" / "handeye_d435i_wrist_end_pose.yaml")
    assert config["source"]["file"] == "calibration_data/d435i_eye_in_hand.json"
    source_path = PROJECT_ROOT / config["source"]["file"]
    with source_path.open(encoding="utf-8") as stream:
        source = json.load(stream)
    expected = np.eye(4)
    expected[:3, :3] = _quaternion_xyzw_to_matrix(
        source["calibration_result"]["orientation"]
    )
    expected[:3, 3] = source["calibration_result"]["position"]
    assert np.allclose(config["parent_from_camera"], expected)


def test_gemini_source_is_project_local_and_matches_runtime_transform():
    config = _load_yaml(PROJECT_ROOT / "config" / "handeye_orbbec_fixed.yaml")
    assert config["source"]["file"] == "calibration_data/gemini336l_eye_to_hand.json"
    source_path = PROJECT_ROOT / config["source"]["file"]
    with source_path.open(encoding="utf-8") as stream:
        source = json.load(stream)
    assert source["quality_passed"] is True
    assert np.allclose(config["parent_from_camera"], source["matrix"])
