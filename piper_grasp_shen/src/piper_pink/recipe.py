"""Object-specific grasp recipe loaded independently from perception."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .transforms import load_yaml, validate_transform


@dataclass(frozen=True)
class GraspRecipe:
    object_id: str
    object_from_tcp: np.ndarray
    axial_angles_deg: tuple[float, ...]
    pregrasp_distance_m: float
    retreat_distance_m: float
    gripper_opening_m: float
    gripper_closed_m: float
    execution_ready: bool

    def __post_init__(self) -> None:
        validate_transform(self.object_from_tcp, "object_from_tcp")
        if not self.object_id.strip():
            raise ValueError("Grasp recipe object_id must not be empty")
        if not self.axial_angles_deg or not np.all(np.isfinite(self.axial_angles_deg)):
            raise ValueError("At least one finite axial grasp angle is required")
        if self.pregrasp_distance_m <= 0.0 or self.retreat_distance_m <= 0.0:
            raise ValueError("Pre-grasp and retreat distances must be positive")
        if not np.all(
            np.isfinite([self.pregrasp_distance_m, self.retreat_distance_m])
        ):
            raise ValueError("Grasp distances must be finite")
        for value in (self.gripper_opening_m, self.gripper_closed_m):
            if not 0.0 <= value <= 0.10:
                raise ValueError("Gripper openings must be between 0.0 and 0.10 m")
        if self.gripper_closed_m > self.gripper_opening_m:
            raise ValueError("Closed gripper opening cannot exceed open gripper opening")
        if not isinstance(self.execution_ready, bool):
            raise ValueError("execution_ready must be true or false")

    @classmethod
    def load(cls, path: Path) -> "GraspRecipe":
        data = load_yaml(path)
        return cls(
            object_id=str(data.get("object_id", "object")),
            object_from_tcp=np.asarray(data["object_from_tcp"], dtype=float),
            axial_angles_deg=tuple(float(value) for value in data["axial_angles_deg"]),
            pregrasp_distance_m=float(data["pregrasp_distance_m"]),
            retreat_distance_m=float(data["retreat_distance_m"]),
            gripper_opening_m=float(data["gripper_opening_m"]),
            gripper_closed_m=float(data["gripper_closed_m"]),
            execution_ready=data.get("execution_ready", False),
        )
