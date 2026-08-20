"""Load and validate a six-axis Piper X model for Pinocchio and Pink."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .config import GRIPPER_JOINT_NAMES, JOINT_LIMITS_RAD, JOINT_NAMES, PiperTcp
from .transforms import validate_transform


def _robotics_stack() -> tuple[Any, Any]:
    try:
        import numpy as np
        import pinocchio as pin
    except ImportError as exc:
        raise RuntimeError(
            "Pinocchio dependencies are missing. Activate piper_pink and run: "
            "python -m pip install pin pin-pink qpsolvers quadprog"
        ) from exc
    return np, pin


@dataclass
class PiperModel:
    model: Any
    data: Any
    tcp_frame: str
    joint_names: tuple[str, ...]
    q_indices: tuple[int, ...]

    @property
    def lower_limits(self):
        return self.model.lowerPositionLimit.copy()

    @property
    def upper_limits(self):
        return self.model.upperPositionLimit.copy()

    def neutral(self):
        _, pin = _robotics_stack()
        return pin.neutral(self.model)

    def forward_tcp(self, q, *, validate_limits: bool = True):
        np, pin = _robotics_stack()
        values = np.asarray(q, dtype=float)
        if validate_limits:
            self.validate_q(values)
        elif values.shape != (6,) or not np.all(np.isfinite(values)):
            raise ValueError("Expected six finite joint values")
        pin.forwardKinematics(self.model, self.data, values)
        pin.updateFramePlacements(self.model, self.data)
        frame_id = self.model.getFrameId(self.tcp_frame)
        return self.data.oMf[frame_id].copy()

    def forward_frame(self, q, frame_name: str):
        np, pin = _robotics_stack()
        if not _frame_exists(self.model, frame_name):
            raise ValueError(f"Frame is absent from Piper model: {frame_name}")
        values = np.asarray(q, dtype=float)
        self.validate_q(values)
        pin.forwardKinematics(self.model, self.data, values)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.model.getFrameId(frame_name)].copy()

    def make_pose(self, xyz_m, rpy_rad):
        """Create a Pinocchio SE3 target from XYZ metres and RPY radians."""

        np, pin = _robotics_stack()
        xyz = np.asarray(xyz_m, dtype=float)
        rpy = np.asarray(rpy_rad, dtype=float)
        if xyz.shape != (3,) or rpy.shape != (3,):
            raise ValueError("Pose XYZ and RPY must each contain three values")
        if not np.all(np.isfinite(xyz)) or not np.all(np.isfinite(rpy)):
            raise ValueError("Pose XYZ or RPY contains NaN or infinity")
        rotation = pin.rpy.rpyToMatrix(rpy)
        return pin.SE3(rotation, xyz)

    def pose_from_matrix(self, matrix):
        np, pin = _robotics_stack()
        value = validate_transform(np.asarray(matrix, dtype=float), "pose matrix")
        return pin.SE3(value[:3, :3], value[:3, 3])

    def pose_error(self, q, target) -> tuple[float, float]:
        """Return translation error in metres and rotation error in radians."""

        np, pin = _robotics_stack()
        current = self.forward_tcp(q)
        position_error = float(np.linalg.norm(target.translation - current.translation))
        orientation_error = float(np.linalg.norm(pin.log3(current.rotation.T @ target.rotation)))
        return position_error, orientation_error

    def validate_q(self, q, margin_rad: float = 0.0) -> None:
        """Reject malformed or out-of-range six-axis configurations."""

        np, _ = _robotics_stack()
        values = np.asarray(q, dtype=float)
        if values.shape != (6,):
            raise ValueError(f"Expected six joint values, got shape {values.shape}")
        if not np.all(np.isfinite(values)):
            raise ValueError("Joint configuration contains NaN or infinity")
        lower = self.lower_limits + margin_rad
        upper = self.upper_limits - margin_rad
        outside = np.flatnonzero((values < lower) | (values > upper))
        if outside.size:
            names = ", ".join(self.joint_names[index] for index in outside)
            raise ValueError(f"Joint configuration is outside limits: {names}")


def _frame_exists(model: Any, frame_name: str) -> bool:
    return any(frame.name == frame_name for frame in model.frames)


def load_piper_model(urdf_path: Path, tcp: PiperTcp) -> PiperModel:
    """Load the Piper X URDF, lock gripper joints, and append a calibrated TCP."""

    np, pin = _robotics_stack()
    urdf_path = Path(urdf_path).expanduser().resolve()
    if not urdf_path.is_file():
        raise FileNotFoundError(f"URDF does not exist: {urdf_path}")

    model = pin.buildModelFromUrdf(str(urdf_path))
    q_reference = pin.neutral(model)
    locked_ids = [
        model.getJointId(name)
        for name in GRIPPER_JOINT_NAMES
        if model.existJointName(name)
    ]
    if locked_ids:
        model = pin.buildReducedModel(model, locked_ids, q_reference)

    missing = [name for name in JOINT_NAMES if not model.existJointName(name)]
    if missing:
        raise ValueError(f"URDF is missing Piper arm joints: {', '.join(missing)}")
    if model.nq != 6 or model.nv != 6:
        raise ValueError(
            f"Expected a reduced Piper X 6-DoF model, got nq={model.nq}, nv={model.nv}"
        )
    if not _frame_exists(model, tcp.parent_frame):
        raise ValueError(f"TCP parent frame is absent from URDF: {tcp.parent_frame}")

    parent_frame_id = model.getFrameId(tcp.parent_frame)
    parent_frame = model.frames[parent_frame_id]
    rotation = pin.rpy.rpyToMatrix(np.asarray(tcp.rpy_rad, dtype=float))
    relative_tcp = pin.SE3(rotation, np.asarray(tcp.xyz_m, dtype=float))
    tcp_placement = parent_frame.placement * relative_tcp
    tcp_frame = pin.Frame(
        tcp.frame_name,
        parent_frame.parentJoint,
        parent_frame_id,
        tcp_placement,
        pin.FrameType.OP_FRAME,
    )
    model.addFrame(tcp_frame, False)
    data = model.createData()

    q_indices = tuple(model.joints[model.getJointId(name)].idx_q for name in JOINT_NAMES)
    if q_indices != tuple(range(6)):
        raise ValueError(f"Unexpected Piper joint ordering in URDF: {q_indices}")
    piper = PiperModel(model, data, tcp.frame_name, JOINT_NAMES, q_indices)

    expected_lower = np.asarray([limit[0] for limit in JOINT_LIMITS_RAD])
    expected_upper = np.asarray([limit[1] for limit in JOINT_LIMITS_RAD])
    if not np.allclose(piper.lower_limits, expected_lower, atol=2.0e-3) or not np.allclose(
        piper.upper_limits, expected_upper, atol=2.0e-3
    ):
        raise ValueError(
            "URDF joint limits do not match the official Piper X model. Verify "
            "that the URDF and robot hardware are both Piper X."
        )
    return piper
