import cv2
import numpy as np
import pytest

from piper_pink.vision_aruco import pose_from_marker_corners


def test_marker_pose_from_synthetic_projection():
    camera = np.array([[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]])
    distortion = np.zeros(5)
    size = 0.08
    half = size / 2.0
    object_points = np.array(
        [[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]],
        dtype=float,
    )
    rotation_vector = np.array([0.10, -0.15, 0.05])
    translation = np.array([0.02, -0.01, 0.55])
    corners, _ = cv2.projectPoints(
        object_points, rotation_vector, translation, camera, distortion
    )
    estimated = pose_from_marker_corners(corners, size, camera, distortion)
    expected_rotation, _ = cv2.Rodrigues(rotation_vector)
    assert np.allclose(estimated[:3, 3], translation, atol=1e-6)
    assert np.allclose(estimated[:3, :3], expected_rotation, atol=1e-6)


def test_marker_pose_rejects_invalid_size():
    with pytest.raises(ValueError, match="marker size"):
        pose_from_marker_corners(np.zeros((4, 2)), 0.0, np.eye(3), np.zeros(5))
