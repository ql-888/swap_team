import pytest
import numpy as np

from piper_pink.live_pose import average_pose_samples


def test_average_pose_samples_handles_quaternion_sign_and_reports_spread():
    translations = np.asarray([[1.0, 2.0, 3.0], [1.002, 2.0, 3.0], [0.998, 2.0, 3.0]])
    quaternions = np.asarray([[0.0, 0.0, 0.0, 1.0], [0.0, 0.0, 0.0, -1.0], [0.0, 0.0, 0.0, 1.0]])
    result = average_pose_samples(translations, quaternions)
    assert np.allclose(result.transform, np.asarray([[1, 0, 0, 1], [0, 1, 0, 2], [0, 0, 1, 3], [0, 0, 0, 1]]))
    assert result.translation_rms_m == pytest.approx(np.sqrt(8.0 / 3.0) / 1000.0)
    assert result.rotation_rms_deg == pytest.approx(0.0)


def test_average_pose_samples_rejects_too_few_samples():
    with pytest.raises(ValueError, match="At least three"):
        average_pose_samples([[0, 0, 0]], [[0, 0, 0, 1]])


def test_average_pose_samples_selects_main_planar_pose_branch():
    translations = np.zeros((30, 3))
    good = np.tile([0.0, 0.0, 0.0, 1.0], (20, 1))
    angle = np.deg2rad(74.0) / 2.0
    wrong = np.tile([np.sin(angle), 0.0, 0.0, np.cos(angle)], (10, 1))
    result = average_pose_samples(translations, np.vstack((good, wrong)))
    assert result.raw_sample_count == 30
    assert result.orientation_inlier_count == 20
    assert result.rotation_rms_deg == pytest.approx(0.0)


def test_average_pose_samples_rejects_translation_outlier_in_main_branch():
    translations = np.zeros((20, 3))
    translations[-1] = [0.025, 0.0, 0.0]
    quaternions = np.tile([0.0, 0.0, 0.0, 1.0], (20, 1))
    result = average_pose_samples(translations, quaternions)
    assert result.orientation_inlier_count == 19
    assert result.translation_rms_m == pytest.approx(0.0)
