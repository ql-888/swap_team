"""Read-only checks for the Piper X physical gripper closing axis."""

from __future__ import annotations

import math

import numpy as np


def closing_axis_base(model, q) -> np.ndarray:
    """Return the gripper two-finger closing line expressed in base coordinates.

    The Piper X URDF used by this project models the finger prismatic joints along
    gripper +/-Y.  The gripper mount is rotated +90 degrees about flange Z, so the
    same unoriented physical line is piper_tcp +/-X.
    """

    return np.asarray(
        model.forward_tcp(q, validate_limits=False).rotation[:, 0], dtype=float
    )


def axis_plane_angle_rad(axis) -> float:
    """Return the signed angle between a unit axis and the base XY plane."""

    value = np.asarray(axis, dtype=float)
    if value.shape != (3,) or not np.all(np.isfinite(value)):
        raise ValueError("Axis must contain three finite values")
    norm = float(np.linalg.norm(value))
    if norm <= 0.0:
        raise ValueError("Axis must be non-zero")
    return math.asin(float(np.clip(value[2] / norm, -1.0, 1.0)))


def horizontal_j6_candidates(model, q, sample_count: int = 1441) -> list[float]:
    """Solve J6 values that put the closing line in the base XY plane.

    J1--J5 are held at their measured values.  The returned roots respect the
    model's J6 limits and are expressed in radians.
    """

    values = np.asarray(q, dtype=float).copy()
    if values.shape != (6,) or not np.all(np.isfinite(values)):
        raise ValueError("Expected six finite joint values")
    if values[5] < model.lower_limits[5] or values[5] > model.upper_limits[5]:
        raise ValueError("Measured J6 is outside the model limits")
    lower = float(model.lower_limits[5])
    upper = float(model.upper_limits[5])
    samples = np.linspace(lower, upper, sample_count)

    def height(j6: float) -> float:
        trial = values.copy()
        trial[5] = j6
        return float(closing_axis_base(model, trial)[2])

    heights = np.asarray([height(value) for value in samples])
    roots: list[float] = []
    tolerance = 1.0e-10
    for index in range(len(samples) - 1):
        left, right = float(samples[index]), float(samples[index + 1])
        f_left, f_right = float(heights[index]), float(heights[index + 1])
        if abs(f_left) <= tolerance:
            roots.append(left)
        if f_left * f_right < 0.0:
            for _ in range(60):
                middle = 0.5 * (left + right)
                f_middle = height(middle)
                if f_left * f_middle <= 0.0:
                    right, f_right = middle, f_middle
                else:
                    left, f_left = middle, f_middle
            roots.append(0.5 * (left + right))
    if abs(float(heights[-1])) <= tolerance:
        roots.append(float(samples[-1]))

    unique: list[float] = []
    for root in sorted(roots):
        if not unique or abs(root - unique[-1]) > 1.0e-6:
            unique.append(root)
    return unique
