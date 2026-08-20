"""Rigid-transform validation and serialization helpers."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml


def validate_transform(matrix, name: str = "transform", atol: float = 1.0e-5):
    value = np.asarray(matrix, dtype=float)
    if value.shape != (4, 4):
        raise ValueError(f"{name} must be a 4x4 matrix")
    if not np.all(np.isfinite(value)):
        raise ValueError(f"{name} contains NaN or infinity")
    if not np.allclose(value[3], [0.0, 0.0, 0.0, 1.0], atol=atol):
        raise ValueError(f"{name} has an invalid homogeneous bottom row")
    rotation = value[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=atol):
        raise ValueError(f"{name} rotation is not orthonormal")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=atol):
        raise ValueError(f"{name} rotation determinant is not +1")
    return value


def invert_transform(matrix):
    value = validate_transform(matrix)
    result = np.eye(4)
    result[:3, :3] = value[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ value[:3, 3]
    return result


def compose(*matrices):
    result = np.eye(4)
    for index, matrix in enumerate(matrices):
        result = result @ validate_transform(matrix, f"transform[{index}]")
    return validate_transform(result, "composed transform")


def load_yaml(path: Path):
    path = Path(path).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"YAML file does not exist: {path}")
    try:
        with path.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        raise ValueError(f"Invalid YAML in {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise ValueError(f"YAML root must be a mapping: {path}")
    return data


def save_yaml(path: Path, data) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(data, stream, sort_keys=False)
