"""Configuration shared by the Piper X offline solver and hardware bridge."""

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from pathlib import Path


JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 7))
GRIPPER_JOINT_NAMES = ("gripper", "gripper_joint1", "gripper_joint2")

# These limits come from the official AgileX piper_x_description.urdf.
JOINT_LIMITS_RAD = (
    (-2.6179938, 2.6179938),
    (0.0, 3.1415926),
    (-2.9670597, 0.0),
    (-1.553343, 1.553343),
    (-1.553343, 1.553343),
    (-2.0943951, 2.0943951),
)

DEFAULT_SEED_RAD = (
    0.0,
    math.radians(90.0),
    math.radians(-90.0),
    0.0,
    0.0,
    0.0,
)


@dataclass(frozen=True)
class IkSettings:
    """Numerical settings for offline and closed-loop IK."""

    dt_s: float = 0.02
    max_iterations: int = 500
    position_cost: float = 1.0
    orientation_cost: float = 0.2
    posture_cost: float = 1.0e-3
    lm_damping: float = 1.0e-6
    position_tolerance_m: float = 0.001
    orientation_tolerance_rad: float = math.radians(1.0)
    random_seed_count: int = 12
    joint_limit_margin_rad: float = math.radians(2.0)


@dataclass(frozen=True)
class HardwareSafety:
    """Conservative defaults for the first physical-arm tests."""

    control_period_s: float = 0.02
    maximum_joint_step_rad: float = math.radians(0.5)
    minimum_feedback_hz: float = 5.0
    feedback_timeout_s: float = 2.0
    execution_timeout_s: float = 20.0
    speed_percent: int = 10


@dataclass(frozen=True)
class PiperTcp:
    """A fixed TCP transform expressed relative to a URDF frame."""

    parent_frame: str
    xyz_m: tuple[float, float, float]
    rpy_rad: tuple[float, float, float]
    frame_name: str = "piper_tcp"


# Measured on the physical Piper X assembly after the force sensor and gripper
# were installed.  Pivot calibration gives X/Y=0 mm and Z=198 mm.  Three
# physical horizontal-line observations after remounting the gripper give a
# fixed +48.2 degree yaw relative to the robot flange.
DEFAULT_TCP = PiperTcp(
    parent_frame="flange_link",
    xyz_m=(0.0, 0.0, 0.198),
    rpy_rad=(0.0, 0.0, math.radians(48.2)),
)

DEFAULT_IK_SETTINGS = IkSettings()
DEFAULT_HARDWARE_SAFETY = HardwareSafety()


def find_default_urdf() -> Path:
    """Find the vendored official Piper X-with-gripper URDF."""

    override = os.environ.get("PIPER_URDF")
    candidates = []
    if override:
        candidates.append(Path(override).expanduser())
    candidates.extend(
        [
            Path(__file__).resolve().parents[2]
            / "assets/piper_x_description/urdf/piper_x_with_gripper_description.urdf",
        ]
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    searched = "\n  - ".join(str(path) for path in candidates)
    raise FileNotFoundError(
        "Could not find the Piper X URDF. Set PIPER_URDF to its path. "
        f"Searched:\n  - {searched}"
    )
