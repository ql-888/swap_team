"""Baseline ArUco perception provider that emits the common object-pose format."""

from __future__ import annotations

from pathlib import Path
import time

import numpy as np

from .transforms import load_yaml, save_yaml, validate_transform


def load_camera_intrinsics(path: Path):
    data = load_yaml(path)
    camera_matrix = np.asarray(data["camera_matrix"], dtype=float)
    distortion = np.asarray(data.get("distortion_coefficients", []), dtype=float)
    if camera_matrix.shape != (3, 3):
        raise ValueError("camera_matrix must be 3x3")
    if not np.all(np.isfinite(camera_matrix)) or not np.all(np.isfinite(distortion)):
        raise ValueError("Camera intrinsics contain NaN or infinity")
    if camera_matrix[0, 0] <= 0.0 or camera_matrix[1, 1] <= 0.0:
        raise ValueError("Camera focal lengths must be positive")
    return str(data["camera_frame"]), camera_matrix, distortion


def pose_from_marker_corners(corners_px, marker_size_m, camera_matrix, distortion):
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError("OpenCV is required for ArUco pose estimation") from exc
    marker_size_m = float(marker_size_m)
    if not np.isfinite(marker_size_m) or marker_size_m <= 0.0:
        raise ValueError("ArUco marker size must be finite and positive")
    corners_px = np.asarray(corners_px, dtype=np.float64).reshape(4, 2)
    if not np.all(np.isfinite(corners_px)):
        raise ValueError("ArUco corners contain NaN or infinity")
    half = marker_size_m / 2.0
    object_points = np.asarray(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float64,
    )
    success, rotation_vector, translation = cv2.solvePnP(
        object_points,
        corners_px,
        np.asarray(camera_matrix, dtype=np.float64),
        np.asarray(distortion, dtype=np.float64),
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    if not success:
        raise RuntimeError("OpenCV could not estimate the marker pose")
    rotation, _ = cv2.Rodrigues(rotation_vector)
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = np.asarray(translation).reshape(3)
    return validate_transform(transform, "camera_from_marker", atol=1.0e-4)


def detect_aruco_pose(
    image_path: Path,
    marker_id: int,
    marker_size_m: float,
    camera_matrix,
    distortion,
    dictionary_name: str = "DICT_4X4_50",
):
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError("OpenCV is required for ArUco detection") from exc
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise FileNotFoundError(f"Could not read image: {image_path}")
    dictionary_id = getattr(cv2.aruco, dictionary_name, None)
    if dictionary_id is None:
        raise ValueError(f"Unknown ArUco dictionary: {dictionary_name}")
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    corners, ids, _ = detector.detectMarkers(image)
    if ids is None:
        raise RuntimeError("No ArUco marker was detected")
    ids = ids.reshape(-1)
    matches = np.flatnonzero(ids == marker_id)
    if matches.size != 1:
        raise RuntimeError(f"Expected one marker id {marker_id}, found {matches.size}")
    return pose_from_marker_corners(
        corners[int(matches[0])], marker_size_m, camera_matrix, distortion
    )


def save_object_pose(path, camera_frame, object_id, camera_from_object) -> None:
    save_yaml(
        path,
        {
            "frame_id": camera_frame,
            "object_id": object_id,
            "confidence": 1.0,
            "timestamp_s": time.time(),
            "frame_from_object": camera_from_object.tolist(),
        },
    )
