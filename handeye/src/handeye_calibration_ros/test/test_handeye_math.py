import math

import numpy as np
from scipy.spatial.transform import Rotation

from handeye_calibration_ros.handeye_session import (
    capture_quality, solve_handeye, transform_delta)


def random_transform(rng):
    transform = np.eye(4)
    transform[:3, :3] = Rotation.random(random_state=rng).as_matrix()
    transform[:3, 3] = rng.uniform(-0.4, 0.4, 3)
    return transform


def assert_transform_close(expected, actual):
    translation, rotation = transform_delta(expected, actual)
    assert translation < 1e-8
    assert math.degrees(rotation) < 1e-6


def test_eye_in_hand_recovers_gripper_to_camera():
    rng = np.random.default_rng(12)
    expected = random_transform(rng)
    base_to_target = random_transform(rng)
    robot = []
    target = []
    for _ in range(25):
        base_to_gripper = random_transform(rng)
        camera_to_target = (
            np.linalg.inv(expected) @ np.linalg.inv(base_to_gripper) @ base_to_target)
        robot.append(base_to_gripper)
        target.append(camera_to_target)
    assert_transform_close(expected, solve_handeye(robot, target, 'eye_in_hand')[0]['transform'])


def test_eye_to_hand_recovers_base_to_camera():
    rng = np.random.default_rng(21)
    expected = random_transform(rng)
    gripper_to_target = random_transform(rng)
    robot = []
    target = []
    for _ in range(25):
        base_to_gripper = random_transform(rng)
        camera_to_target = np.linalg.inv(expected) @ base_to_gripper @ gripper_to_target
        robot.append(base_to_gripper)
        target.append(camera_to_target)
    assert_transform_close(expected, solve_handeye(robot, target, 'eye_to_hand')[0]['transform'])


def perturb(transform, rng, translation_sigma, rotation_sigma_deg):
    noisy = transform.copy()
    noisy[:3, 3] += rng.normal(0.0, translation_sigma, 3)
    rotation_noise = Rotation.from_rotvec(
        rng.normal(0.0, np.radians(rotation_sigma_deg), 3)).as_matrix()
    noisy[:3, :3] = noisy[:3, :3] @ rotation_noise
    return noisy


def test_realistic_noise_stays_within_quality_thresholds():
    rng = np.random.default_rng(31)
    expected = random_transform(rng)
    base_to_target = random_transform(rng)
    robot = []
    target = []
    for _ in range(30):
        base_to_gripper = random_transform(rng)
        camera_to_target = (
            np.linalg.inv(expected) @ np.linalg.inv(base_to_gripper) @ base_to_target)
        robot.append(perturb(base_to_gripper, rng, 0.0005, 0.05))
        target.append(perturb(camera_to_target, rng, 0.001, 0.10))
    solutions = solve_handeye(robot, target, 'eye_in_hand')
    best = solutions[0]
    assert best['residual']['translation_rms_m'] < 0.02
    assert best['residual']['rotation_rms_deg'] < 3.0
    method_translation, method_rotation = transform_delta(
        solutions[0]['transform'], solutions[1]['transform'])
    assert method_translation < 0.03
    assert math.degrees(method_rotation) < 3.0


def test_bad_samples_exceed_quality_thresholds():
    rng = np.random.default_rng(41)
    expected = random_transform(rng)
    base_to_target = random_transform(rng)
    robot = []
    target = []
    for index in range(25):
        base_to_gripper = random_transform(rng)
        camera_to_target = (
            np.linalg.inv(expected) @ np.linalg.inv(base_to_gripper) @ base_to_target)
        if index % 4 == 0:
            camera_to_target = perturb(camera_to_target, rng, 0.08, 12.0)
        robot.append(base_to_gripper)
        target.append(camera_to_target)
    best = solve_handeye(robot, target, 'eye_in_hand')[0]
    assert (
        best['residual']['translation_rms_m'] > 0.02
        or best['residual']['rotation_rms_deg'] > 3.0
    )


def test_capture_quality_first_pose_gets_full_novelty_and_coverage():
    quality = capture_quality(True, 38.0, np.eye(4), [])
    assert quality['total'] == 98.0
    assert quality['novelty'] == 40.0
    assert quality['coverage'] == 20.0
    assert quality['verdict'] == 'GOOD'


def test_capture_quality_duplicate_pose_scores_low():
    quality = capture_quality(True, 38.0, np.eye(4), [np.eye(4)])
    assert quality['novelty'] == 0.0
    assert quality['coverage'] == 0.0
    assert quality['total'] == 38.0
    assert quality['verdict'] == 'LOW'


def test_capture_quality_distinct_pose_scores_above_duplicate():
    candidate = np.eye(4)
    candidate[:3, 3] = [0.10, 0.0, 0.0]
    candidate[:3, :3] = Rotation.from_euler('y', 30, degrees=True).as_matrix()
    distinct = capture_quality(True, 38.0, candidate, [np.eye(4)])
    duplicate = capture_quality(True, 38.0, np.eye(4), [np.eye(4)])
    assert distinct['novelty'] > 35.0
    assert distinct['coverage'] > 0.0
    assert distinct['total'] > duplicate['total']
