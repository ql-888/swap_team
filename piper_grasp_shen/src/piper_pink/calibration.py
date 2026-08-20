"""TCP pivot calibration and hand-eye transform management."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Literal, Sequence

import numpy as np

from .transforms import compose, invert_transform, load_yaml, save_yaml, validate_transform


@dataclass(frozen=True)
class TcpPivotResult:
    flange_from_tcp_translation_m: np.ndarray
    base_pivot_m: np.ndarray
    rms_residual_m: float


def solve_tcp_pivot(base_from_flange_samples: Sequence[np.ndarray]) -> TcpPivotResult:
    """Estimate a tool-tip translation while the tip is held on one fixed point."""

    if len(base_from_flange_samples) < 6:
        raise ValueError("TCP pivot calibration needs at least six diverse poses")
    rows = []
    rhs = []
    transforms = [
        validate_transform(sample, f"base_from_flange[{index}]")
        for index, sample in enumerate(base_from_flange_samples)
    ]
    for transform in transforms:
        rows.append(np.hstack((transform[:3, :3], -np.eye(3))))
        rhs.append(-transform[:3, 3])
    solution, _, rank, _ = np.linalg.lstsq(np.vstack(rows), np.concatenate(rhs), rcond=None)
    if rank < 6:
        raise ValueError("TCP poses are degenerate; use more varied wrist orientations")
    tool_point = solution[:3]
    pivot_point = solution[3:]
    residuals = [
        transform[:3, :3] @ tool_point + transform[:3, 3] - pivot_point
        for transform in transforms
    ]
    rms = float(np.sqrt(np.mean(np.square(np.vstack(residuals)))))
    return TcpPivotResult(tool_point, pivot_point, rms)


@dataclass(frozen=True)
class HandEyeCalibration:
    camera_mount: Literal["fixed", "wrist"]
    parent_from_camera: np.ndarray
    parent_frame: str
    camera_frame: str

    def __post_init__(self) -> None:
        validate_transform(self.parent_from_camera, "parent_from_camera")
        if self.camera_mount == "fixed" and self.parent_frame != "base_link":
            raise ValueError("A fixed camera calibration must use base_link as parent")

    def base_from_object(self, camera_from_object, base_from_gripper=None):
        camera_from_object = validate_transform(camera_from_object, "camera_from_object")
        if self.camera_mount == "fixed":
            return compose(self.parent_from_camera, camera_from_object)
        if base_from_gripper is None:
            raise ValueError("Wrist-camera conversion requires base_from_gripper")
        return compose(base_from_gripper, self.parent_from_camera, camera_from_object)

    def save(self, path: Path) -> None:
        save_yaml(
            path,
            {
                "camera_mount": self.camera_mount,
                "parent_frame": self.parent_frame,
                "camera_frame": self.camera_frame,
                "parent_from_camera": self.parent_from_camera.tolist(),
            },
        )

    @classmethod
    def load(cls, path: Path) -> "HandEyeCalibration":
        data = load_yaml(path)
        return cls(
            camera_mount=data["camera_mount"],
            parent_from_camera=np.asarray(data["parent_from_camera"], dtype=float),
            parent_frame=data["parent_frame"],
            camera_frame=data["camera_frame"],
        )


def solve_eye_in_hand(base_from_gripper, camera_from_target) -> HandEyeCalibration:
    """Solve gripper-from-camera using OpenCV's hand-eye implementation."""

    if len(base_from_gripper) != len(camera_from_target) or len(base_from_gripper) < 8:
        raise ValueError("Hand-eye calibration needs at least eight paired poses")
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError("OpenCV is required for hand-eye calibration") from exc
    robot = [validate_transform(item, "base_from_gripper") for item in base_from_gripper]
    observations = [
        validate_transform(item, "camera_from_target") for item in camera_from_target
    ]
    rotation, translation = cv2.calibrateHandEye(
        [item[:3, :3] for item in robot],
        [item[:3, 3] for item in robot],
        [item[:3, :3] for item in observations],
        [item[:3, 3] for item in observations],
        method=cv2.CALIB_HAND_EYE_PARK,
    )
    gripper_from_camera = np.eye(4)
    gripper_from_camera[:3, :3] = rotation
    gripper_from_camera[:3, 3] = np.asarray(translation).reshape(3)
    return HandEyeCalibration(
        camera_mount="wrist",
        parent_from_camera=gripper_from_camera,
        parent_frame="gripper_base",
        camera_frame="camera",
    )


def _vector_to_transform(vector: np.ndarray) -> np.ndarray:
    from scipy.spatial.transform import Rotation

    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_rotvec(vector[:3]).as_matrix()
    transform[:3, 3] = vector[3:]
    return transform


def _transform_to_vector(transform: np.ndarray) -> np.ndarray:
    from scipy.spatial.transform import Rotation

    value = validate_transform(transform)
    return np.concatenate((Rotation.from_matrix(value[:3, :3]).as_rotvec(), value[:3, 3]))


def solve_eye_to_hand(base_from_gripper, camera_from_target) -> HandEyeCalibration:
    """Solve base-from-camera for a fixed camera and gripper-mounted target."""

    if len(base_from_gripper) != len(camera_from_target) or len(base_from_gripper) < 8:
        raise ValueError("Hand-eye calibration needs at least eight paired poses")
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation

    robot = [validate_transform(item, "base_from_gripper") for item in base_from_gripper]
    observations = [
        validate_transform(item, "camera_from_target") for item in camera_from_target
    ]
    initial_base_from_camera = compose(robot[0], invert_transform(observations[0]))
    initial = np.concatenate(
        (_transform_to_vector(initial_base_from_camera), np.zeros(6, dtype=float))
    )

    def residual(vector):
        base_from_camera = _vector_to_transform(vector[:6])
        gripper_from_target = _vector_to_transform(vector[6:])
        values = []
        for base_from_gripper_i, camera_from_target_i in zip(robot, observations):
            left = compose(base_from_gripper_i, gripper_from_target)
            right = compose(base_from_camera, camera_from_target_i)
            values.extend((left[:3, 3] - right[:3, 3]) / 0.1)
            values.extend(
                Rotation.from_matrix(left[:3, :3].T @ right[:3, :3]).as_rotvec()
            )
        return np.asarray(values)

    solution = least_squares(
        residual,
        initial,
        method="trf",
        max_nfev=5000,
        ftol=1.0e-12,
        xtol=1.0e-12,
        gtol=1.0e-12,
    )
    if not solution.success or np.linalg.matrix_rank(solution.jac, tol=1.0e-7) < 12:
        raise ValueError("Fixed-camera samples are degenerate; vary all wrist axes")
    return HandEyeCalibration(
        camera_mount="fixed",
        parent_from_camera=_vector_to_transform(solution.x[:6]),
        parent_frame="base_link",
        camera_frame="camera",
    )


def handeye_consistency(calibration, base_from_gripper, camera_from_target):
    """Return RMS translation and rotation scatter for paired calibration samples."""

    from scipy.spatial.transform import Rotation

    if len(base_from_gripper) != len(camera_from_target) or not base_from_gripper:
        raise ValueError("Hand-eye validation requires paired samples")
    derived = []
    for robot, observation in zip(base_from_gripper, camera_from_target):
        robot = validate_transform(robot, "base_from_gripper")
        observation = validate_transform(observation, "camera_from_target")
        if calibration.camera_mount == "wrist":
            derived.append(compose(robot, calibration.parent_from_camera, observation))
        else:
            derived.append(
                compose(invert_transform(robot), calibration.parent_from_camera, observation)
            )
    translations = np.asarray([item[:3, 3] for item in derived])
    mean_translation = translations.mean(axis=0)
    translation_rms = float(
        np.sqrt(np.mean(np.sum(np.square(translations - mean_translation), axis=1)))
    )
    rotations = Rotation.from_matrix(np.asarray([item[:3, :3] for item in derived]))
    mean_rotation = rotations.mean()
    rotation_errors = (mean_rotation.inv() * rotations).magnitude()
    rotation_rms = float(np.sqrt(np.mean(np.square(rotation_errors))))
    return translation_rms, rotation_rms
