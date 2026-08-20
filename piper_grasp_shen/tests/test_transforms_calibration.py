import math

import numpy as np
import pytest

from piper_pink.calibration import (
    HandEyeCalibration,
    handeye_consistency,
    solve_eye_to_hand,
    solve_tcp_pivot,
)
from piper_pink.transforms import compose, invert_transform, validate_transform


def rotation_x(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def rotation_y(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def rotation_z(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def test_inverse_transform_round_trip():
    transform = np.eye(4)
    transform[:3, :3] = rotation_z(0.4) @ rotation_y(-0.2)
    transform[:3, 3] = [0.2, -0.1, 0.5]
    assert np.allclose(compose(transform, invert_transform(transform)), np.eye(4))


def test_transform_rejects_reflection():
    reflection = np.eye(4)
    reflection[0, 0] = -1.0
    with pytest.raises(ValueError, match="determinant"):
        validate_transform(reflection)


def test_tcp_pivot_calibration_recovers_tool_point():
    tool_point = np.array([0.012, -0.008, 0.132])
    pivot_point = np.array([0.35, 0.04, 0.22])
    rotations = [
        rotation_x(0.2),
        rotation_y(-0.35),
        rotation_z(0.5),
        rotation_x(-0.45) @ rotation_y(0.2),
        rotation_y(0.5) @ rotation_z(-0.25),
        rotation_z(0.7) @ rotation_x(0.3),
        rotation_x(-0.3) @ rotation_z(-0.6),
        rotation_y(-0.55) @ rotation_x(0.4),
    ]
    samples = []
    for rotation in rotations:
        transform = np.eye(4)
        transform[:3, :3] = rotation
        transform[:3, 3] = pivot_point - rotation @ tool_point
        samples.append(transform)
    result = solve_tcp_pivot(samples)
    assert np.allclose(result.flange_from_tcp_translation_m, tool_point, atol=1e-10)
    assert np.allclose(result.base_pivot_m, pivot_point, atol=1e-10)
    assert result.rms_residual_m < 1e-10


def test_fixed_handeye_composes_camera_pose():
    base_from_camera = np.eye(4)
    base_from_camera[:3, 3] = [0.2, 0.0, 0.4]
    camera_from_object = np.eye(4)
    camera_from_object[:3, 3] = [0.1, 0.2, 0.3]
    calibration = HandEyeCalibration(
        "fixed", base_from_camera, "base_link", "camera"
    )
    result = calibration.base_from_object(camera_from_object)
    assert np.allclose(result[:3, 3], [0.3, 0.2, 0.7])


def test_fixed_handeye_solver_recovers_synthetic_transform():
    base_from_camera = np.eye(4)
    base_from_camera[:3, :3] = rotation_z(0.25) @ rotation_x(-0.1)
    base_from_camera[:3, 3] = [0.25, -0.18, 0.55]
    gripper_from_target = np.eye(4)
    gripper_from_target[:3, :3] = rotation_y(0.15)
    gripper_from_target[:3, 3] = [0.02, 0.01, 0.12]
    robot = []
    observations = []
    for index in range(12):
        pose = np.eye(4)
        pose[:3, :3] = (
            rotation_z(-0.5 + 0.09 * index)
            @ rotation_y(-0.35 + 0.06 * index)
            @ rotation_x(0.25 * math.sin(index))
        )
        pose[:3, 3] = [
            0.25 + 0.025 * math.cos(index),
            0.04 * math.sin(index * 0.7),
            0.25 + 0.015 * index,
        ]
        robot.append(pose)
        observations.append(
            compose(invert_transform(base_from_camera), pose, gripper_from_target)
        )
    calibration = solve_eye_to_hand(robot, observations)
    translation_rms, rotation_rms = handeye_consistency(
        calibration, robot, observations
    )
    assert np.allclose(calibration.parent_from_camera, base_from_camera, atol=1e-7)
    assert translation_rms < 1e-8
    assert rotation_rms < 1e-8
