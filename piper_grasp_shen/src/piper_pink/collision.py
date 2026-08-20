"""Self-collision and coarse workspace checks for Piper trajectories."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from .config import GRIPPER_JOINT_NAMES
from .model import _robotics_stack
from .transforms import load_yaml


@dataclass(frozen=True)
class WorkspaceLimits:
    maximum_radius_m: float = 0.65
    minimum_tcp_z_m: float = 0.02

    def validate_pose(self, pose) -> None:
        np, _ = _robotics_stack()
        position = pose.translation
        if np.asarray(position).shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError("TCP pose translation must contain three finite values")
        radius = float(np.linalg.norm(position))
        if radius > self.maximum_radius_m:
            raise ValueError(
                f"TCP radius {radius:.3f} m exceeds {self.maximum_radius_m:.3f} m"
            )
        if float(position[2]) < self.minimum_tcp_z_m:
            raise ValueError(
                f"TCP z={position[2]:.3f} m is below {self.minimum_tcp_z_m:.3f} m"
            )


class PiperSelfCollisionChecker:
    """Use Pinocchio/HPP-FCL for self-collision and configured scene boxes."""

    def __init__(
        self,
        urdf_path: Path,
        srdf_path: Path | None = None,
        scene_path: Path | None = None,
    ) -> None:
        np, pin = _robotics_stack()
        urdf_path = Path(urdf_path).resolve()
        package_root = urdf_path.parents[2]
        full_model = pin.buildModelFromUrdf(str(urdf_path))
        geometry = pin.buildGeomFromUrdf(
            full_model,
            str(urdf_path),
            pin.GeometryType.COLLISION,
            package_dirs=[str(package_root)],
        )
        geometry.addAllCollisionPairs()
        if srdf_path is not None and Path(srdf_path).is_file():
            pin.removeCollisionPairs(full_model, geometry, str(srdf_path), False)
        q_reference = pin.neutral(full_model)
        locked_ids = [
            full_model.getJointId(name)
            for name in GRIPPER_JOINT_NAMES
            if full_model.existJointName(name)
        ]
        if locked_ids:
            full_model, geometries = pin.buildReducedModel(
                full_model, [geometry], locked_ids, q_reference
            )
            geometry = geometries[0]
        if scene_path is not None:
            try:
                import coal
            except ImportError as exc:
                raise RuntimeError("Pinocchio collision support requires coal/HPP-FCL") from exc
            scene = load_yaml(scene_path)
            boxes = scene.get("boxes", [])
            if not isinstance(boxes, list):
                raise ValueError("Scene 'boxes' must be a list")
            names = set()
            robot_geometry_count = len(geometry.geometryObjects)
            for item in boxes:
                if not isinstance(item, dict):
                    raise ValueError("Each scene obstacle must be a mapping")
                name = str(item.get("name", "")).strip()
                if not name or name in names:
                    raise ValueError("Scene obstacle names must be non-empty and unique")
                names.add(name)
                size = np.asarray(item.get("size_m"), dtype=float)
                xyz = np.asarray(item.get("xyz_m"), dtype=float)
                rpy = np.asarray(item.get("rpy_deg", [0, 0, 0]), dtype=float)
                if size.shape != (3,) or xyz.shape != (3,) or rpy.shape != (3,):
                    raise ValueError(f"Scene box '{name}' needs three size/xyz/rpy values")
                if not np.all(np.isfinite(np.concatenate((size, xyz, rpy)))):
                    raise ValueError(f"Scene box '{name}' contains NaN or infinity")
                if np.any(size <= 0.0):
                    raise ValueError(f"Scene box '{name}' sizes must be positive")
                rotation = pin.rpy.rpyToMatrix(
                    np.deg2rad(rpy)
                )
                placement = pin.SE3(rotation, xyz)
                box = coal.Box(*[float(value) for value in size])
                obstacle = pin.GeometryObject(name, 0, 0, placement, box)
                obstacle_id = geometry.addGeometryObject(obstacle)
                ignored = set(item.get("ignore_links", []))
                for robot_geometry_id in range(robot_geometry_count):
                    robot_object = geometry.geometryObjects[robot_geometry_id]
                    parent_frame = full_model.frames[robot_object.parentFrame].name
                    if parent_frame in ignored:
                        continue
                    geometry.addCollisionPair(
                        pin.CollisionPair(robot_geometry_id, obstacle_id)
                    )
        self.pin = pin
        self.model = full_model
        self.data = full_model.createData()
        self.geometry = geometry
        self.geometry_data = pin.GeometryData(geometry)

    def collision_pairs(self, q) -> list[tuple[str, str]]:
        np, _ = _robotics_stack()
        q = np.asarray(q, dtype=float)
        if q.shape != (self.model.nq,) or not np.all(np.isfinite(q)):
            raise ValueError("Collision query requires a finite six-joint configuration")
        if np.any(q < self.model.lowerPositionLimit) or np.any(
            q > self.model.upperPositionLimit
        ):
            raise ValueError("Collision query joint configuration is outside limits")
        collision = self.pin.computeCollisions(
            self.model,
            self.data,
            self.geometry,
            self.geometry_data,
            q,
            False,
        )
        if not collision:
            return []
        pairs = []
        for index, result in enumerate(self.geometry_data.collisionResults):
            if not result.isCollision():
                continue
            pair = self.geometry.collisionPairs[index]
            first = self.geometry.geometryObjects[pair.first].name
            second = self.geometry.geometryObjects[pair.second].name
            pairs.append((first, second))
        return pairs

    def validate_q(self, q) -> None:
        pairs = self.collision_pairs(q)
        if pairs:
            description = ", ".join(f"{first}<->{second}" for first, second in pairs)
            raise ValueError(f"Piper collision detected: {description}")

    def validate_path(self, path) -> None:
        for index, q in enumerate(path):
            try:
                self.validate_q(q)
            except ValueError as exc:
                raise ValueError(f"Collision at path waypoint {index}: {exc}") from exc
