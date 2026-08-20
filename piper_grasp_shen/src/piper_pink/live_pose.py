"""Robust averaging helpers for live TF object-pose samples."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .transforms import validate_transform


@dataclass(frozen=True)
class AveragedPose:
    transform: np.ndarray
    translation_rms_m: float
    translation_max_m: float
    rotation_rms_deg: float
    rotation_max_deg: float
    raw_sample_count: int
    orientation_inlier_count: int


def quaternion_xyzw_to_matrix(quaternion) -> np.ndarray:
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        raise ValueError("Quaternion must contain four finite XYZW values")
    norm = float(np.linalg.norm(q))
    if norm < 1.0e-12:
        raise ValueError("Quaternion norm is zero")
    x, y, z, w = q / norm
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def average_pose_samples(
    translations,
    quaternions_xyzw,
    rotation_cluster_threshold_deg: float = 5.0,
    translation_cluster_threshold_m: float = 0.003,
    minimum_inlier_fraction: float = 0.5,
) -> AveragedPose:
    translations = np.asarray(translations, dtype=float)
    quaternions = np.asarray(quaternions_xyzw, dtype=float)
    if translations.ndim != 2 or translations.shape[1:] != (3,):
        raise ValueError("Translations must have shape (N, 3)")
    if quaternions.shape != (translations.shape[0], 4):
        raise ValueError("Quaternions must have shape (N, 4)")
    if translations.shape[0] < 3:
        raise ValueError("At least three pose samples are required")
    if not np.all(np.isfinite(translations)) or not np.all(np.isfinite(quaternions)):
        raise ValueError("Pose samples contain NaN or infinity")

    norms = np.linalg.norm(quaternions, axis=1)
    if np.any(norms < 1.0e-12):
        raise ValueError("Pose samples contain a zero quaternion")
    quaternions = quaternions / norms[:, None]

    if not 0.0 < rotation_cluster_threshold_deg < 180.0:
        raise ValueError("Rotation cluster threshold must be between 0 and 180 degrees")
    if not np.isfinite(translation_cluster_threshold_m) or translation_cluster_threshold_m <= 0.0:
        raise ValueError("Translation cluster threshold must be finite and positive")
    if not 0.0 < minimum_inlier_fraction <= 1.0:
        raise ValueError("Minimum inlier fraction must be in (0, 1]")

    # A planar AprilTag can alternate between distinct pose branches even when
    # its image corners and translation are stable.  Find the densest
    # quaternion neighborhood first instead of averaging incompatible branches.
    similarities = np.clip(np.abs(quaternions @ quaternions.T), 0.0, 1.0)
    pairwise_deg = np.rad2deg(2.0 * np.arccos(similarities))
    neighbor_counts = np.sum(pairwise_deg <= rotation_cluster_threshold_deg, axis=1)
    seed_index = int(np.argmax(neighbor_counts))
    inliers = pairwise_deg[seed_index] <= rotation_cluster_threshold_deg
    minimum_count = max(3, int(np.ceil(translations.shape[0] * minimum_inlier_fraction)))
    if int(np.sum(inliers)) < minimum_count:
        raise ValueError(
            "No stable AprilTag orientation branch contains enough samples: "
            f"{int(np.sum(inliers))}/{translations.shape[0]}"
        )

    orientation_indices = np.flatnonzero(inliers)
    orientation_translations = translations[orientation_indices]
    pairwise_translation = np.linalg.norm(
        orientation_translations[:, None, :] - orientation_translations[None, :, :],
        axis=2,
    )
    translation_neighbor_counts = np.sum(
        pairwise_translation <= translation_cluster_threshold_m, axis=1
    )
    translation_seed = int(np.argmax(translation_neighbor_counts))
    translation_inliers = (
        pairwise_translation[translation_seed] <= translation_cluster_threshold_m
    )
    final_indices = orientation_indices[translation_inliers]
    if final_indices.size < minimum_count:
        raise ValueError(
            "No stable AprilTag pose branch contains enough translation inliers: "
            f"{final_indices.size}/{translations.shape[0]}"
        )

    inlier_quaternions = quaternions[final_indices]
    inlier_translations = translations[final_indices]

    # Markley's quaternion mean handles the q/-q representation ambiguity.
    accumulator = sum(np.outer(q, q) for q in inlier_quaternions)
    eigenvalues, eigenvectors = np.linalg.eigh(accumulator)
    mean_q = eigenvectors[:, int(np.argmax(eigenvalues))]
    if mean_q[3] < 0.0:
        mean_q = -mean_q

    mean_translation = inlier_translations.mean(axis=0)
    translation_errors = np.linalg.norm(inlier_translations - mean_translation, axis=1)
    dots = np.clip(np.abs(inlier_quaternions @ mean_q), 0.0, 1.0)
    rotation_errors_deg = np.rad2deg(2.0 * np.arccos(dots))

    transform = np.eye(4)
    transform[:3, :3] = quaternion_xyzw_to_matrix(mean_q)
    transform[:3, 3] = mean_translation
    validate_transform(transform, "averaged pose")
    return AveragedPose(
        transform=transform,
        translation_rms_m=float(np.sqrt(np.mean(translation_errors**2))),
        translation_max_m=float(np.max(translation_errors)),
        rotation_rms_deg=float(np.sqrt(np.mean(rotation_errors_deg**2))),
        rotation_max_deg=float(np.max(rotation_errors_deg)),
        raw_sample_count=int(translations.shape[0]),
        orientation_inlier_count=int(final_indices.size),
    )
