"""Small URDF kinematics implementation used by planning and camera timing."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation


ARM_JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 7))


def _vector(text: str | None, default: tuple[float, float, float]) -> np.ndarray:
    if text is None:
        return np.asarray(default, dtype=float)
    return np.asarray([float(value) for value in text.split()], dtype=float)


def _origin_matrix(joint: ET.Element) -> np.ndarray:
    origin = joint.find("origin")
    transform = np.eye(4)
    if origin is None:
        return transform
    transform[:3, :3] = Rotation.from_euler(
        "xyz", _vector(origin.get("rpy"), (0.0, 0.0, 0.0))
    ).as_matrix()
    transform[:3, 3] = _vector(origin.get("xyz"), (0.0, 0.0, 0.0))
    return transform


def _axis_rotation(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_rotvec(axis * angle_rad).as_matrix()
    return transform


@dataclass(frozen=True)
class JointSpec:
    name: str
    kind: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray
    lower: float | None
    upper: float | None


class URDFKinematics:
    """Forward kinematics sourced directly from the received xArm6/G1 URDF."""

    def __init__(self, urdf_path: Path):
        self.urdf_path = Path(urdf_path)
        root = ET.parse(self.urdf_path).getroot()
        self.joints: dict[str, JointSpec] = {}
        self.joint_by_child: dict[str, JointSpec] = {}
        for element in root.findall("joint"):
            limit = element.find("limit")
            spec = JointSpec(
                name=str(element.get("name")),
                kind=str(element.get("type")),
                parent=str(element.find("parent").get("link")),
                child=str(element.find("child").get("link")),
                origin=_origin_matrix(element),
                axis=_vector(
                    None if element.find("axis") is None else element.find("axis").get("xyz"),
                    (0.0, 0.0, 1.0),
                ),
                lower=None if limit is None or limit.get("lower") is None else float(limit.get("lower")),
                upper=None if limit is None or limit.get("upper") is None else float(limit.get("upper")),
            )
            self.joints[spec.name] = spec
            self.joint_by_child[spec.child] = spec

    @property
    def arm_limits(self) -> tuple[np.ndarray, np.ndarray]:
        lower = np.asarray([self.joints[name].lower for name in ARM_JOINT_NAMES], dtype=float)
        upper = np.asarray([self.joints[name].upper for name in ARM_JOINT_NAMES], dtype=float)
        return lower, upper

    def _chain(self, target_link: str, base_link: str) -> list[JointSpec]:
        chain: list[JointSpec] = []
        link = target_link
        while link != base_link:
            joint = self.joint_by_child[link]
            chain.append(joint)
            link = joint.parent
        chain.reverse()
        return chain

    def forward(
        self,
        joint_rad: np.ndarray | tuple[float, ...],
        target_link: str = "link_tcp",
        base_link: str = "link_base",
    ) -> np.ndarray:
        values = dict(zip(ARM_JOINT_NAMES, np.asarray(joint_rad, dtype=float), strict=True))
        transform = np.eye(4)
        for joint in self._chain(target_link, base_link):
            transform = transform @ joint.origin
            if joint.kind in {"revolute", "continuous"}:
                transform = transform @ _axis_rotation(joint.axis, values.get(joint.name, 0.0))
        return transform

    def jacobian(
        self,
        joint_rad: np.ndarray | tuple[float, ...],
        target_link: str = "link_tcp",
        epsilon: float = 1.0e-5,
    ) -> np.ndarray:
        joint = np.asarray(joint_rad, dtype=float)
        base = self.forward(joint, target_link)
        jacobian = np.zeros((6, joint.size), dtype=float)
        for index in range(joint.size):
            moved_joint = joint.copy()
            moved_joint[index] += epsilon
            moved = self.forward(moved_joint, target_link)
            jacobian[:3, index] = (moved[:3, 3] - base[:3, 3]) / epsilon
            world_delta = moved[:3, :3] @ base[:3, :3].T
            jacobian[3:, index] = Rotation.from_matrix(world_delta).as_rotvec() / epsilon
        return jacobian

    def radial_metrics(
        self, joint_rad: np.ndarray | tuple[float, ...]
    ) -> dict[str, float | list[float]]:
        transform = self.forward(joint_rad)
        position = transform[:3, 3]
        tool_axis = transform[:3, 2]
        radial = position[:2]
        radius = float(np.linalg.norm(radial))
        outward_dot = float(np.dot(tool_axis[:2], radial))
        return {
            "tcp_position_m": position.tolist(),
            "tool_axis_world": tool_axis.tolist(),
            "tcp_horizontal_radius_m": radius,
            "outward_dot_m": outward_dot,
            "tool_axis_elevation_deg": math.degrees(math.asin(float(tool_axis[2]))),
        }
