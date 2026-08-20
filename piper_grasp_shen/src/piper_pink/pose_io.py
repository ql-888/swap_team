"""Stable file contract for perception systems that provide object poses."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .transforms import load_yaml, validate_transform


@dataclass(frozen=True)
class ObjectPose:
    frame_id: str
    object_id: str
    frame_from_object: np.ndarray
    confidence: float
    timestamp_s: float | None = None

    def __post_init__(self) -> None:
        validate_transform(self.frame_from_object, "frame_from_object")
        if not self.frame_id.strip() or not self.object_id.strip():
            raise ValueError("Object pose frame_id and object_id must not be empty")
        if not np.isfinite(self.confidence):
            raise ValueError("Object-pose confidence must be finite")
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("Object-pose confidence must be between 0 and 1")
        if self.timestamp_s is not None and not np.isfinite(self.timestamp_s):
            raise ValueError("Object-pose timestamp must be finite")


def load_object_pose(path: Path) -> ObjectPose:
    data = load_yaml(path)
    return ObjectPose(
        frame_id=str(data["frame_id"]),
        object_id=str(data.get("object_id", "object")),
        frame_from_object=np.asarray(data["frame_from_object"], dtype=float),
        confidence=float(data.get("confidence", 1.0)),
        timestamp_s=(
            None if data.get("timestamp_s") is None else float(data["timestamp_s"])
        ),
    )
