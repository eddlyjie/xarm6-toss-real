"""Configuration loading for the real cube demo."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any


DEMO_ROOT = Path(__file__).resolve().parents[2]
PROJECT_ROOT = DEMO_ROOT.parent
DEFAULT_HARDWARE_PATH = DEMO_ROOT / "configs" / "hardware.json"
DEFAULT_PLAN_PATH = DEMO_ROOT / "configs" / "pick_place.json"
DEFAULT_HANDOFF_PATH = DEMO_ROOT / "configs" / "handoff_place.json"
DEFAULT_PROBE_PATH = DEMO_ROOT / "configs" / "probe.json"
POSE_NAMES = ("home", "pregrasp", "grasp", "lift", "preplace", "place")


@dataclass(frozen=True)
class CameraConfig:
    role: str
    model: str
    serial: str
    intrinsics_path: Path
    extrinsics_path: Path


@dataclass(frozen=True)
class HardwareConfig:
    ip: str
    motion_confirmed: bool
    joint_speed_rad_s: float
    joint_acceleration_rad_s2: float
    gripper_kind: str
    gripper_model_confirmed: bool
    gripper_open_position: float
    gripper_closed_position: float
    gripper_speed: float
    cameras: tuple[CameraConfig, ...]


@dataclass(frozen=True)
class PickPlacePlan:
    poses: dict[str, tuple[float, ...] | None]
    tcp_poses_at_recording: dict[str, tuple[float, ...]]

    @property
    def missing_poses(self) -> tuple[str, ...]:
        return tuple(name for name in POSE_NAMES if self.poses[name] is None)


@dataclass(frozen=True)
class HandoffPlacePlan:
    handoff_joint_rad: tuple[float, ...]
    preplace_joint_rad: tuple[float, ...]
    preplace_tcp: tuple[float, ...]
    place_tcp: tuple[float, ...]
    tcp_speed_mm_s: float
    tcp_acceleration_mm_s2: float


@dataclass(frozen=True)
class ProbeMotion:
    name: str
    joint_index: int
    amplitude_rad: float
    frequency_hz: float
    duration_s: float


@dataclass(frozen=True)
class ProbePlan:
    control_period_s: float
    settle_s: float
    recover_s: float
    center_joint_rad: tuple[float, ...]
    motions: tuple[ProbeMotion, ...]

    @property
    def duration_s(self) -> float:
        return self.settle_s + sum(m.duration_s for m in self.motions) + self.recover_s


def _read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _joint_pose(value: Any, name: str) -> tuple[float, ...] | None:
    if value is None:
        return None
    if not isinstance(value, list) or len(value) != 6:
        raise ValueError(f"pose {name!r} must contain six joint angles")
    return tuple(float(item) for item in value)


def load_hardware(path: Path = DEFAULT_HARDWARE_PATH) -> HardwareConfig:
    raw = _read_json(path)
    if raw.get("schema") != "real_cube_hardware_v1":
        raise ValueError("unsupported hardware configuration")
    robot = raw["robot"]
    gripper = raw["gripper"]
    cameras = tuple(
        CameraConfig(
            role=role,
            model=value["model"],
            serial=str(value["serial"]),
            intrinsics_path=PROJECT_ROOT / value["intrinsics"],
            extrinsics_path=PROJECT_ROOT / value["extrinsics"],
        )
        for role, value in raw["cameras"].items()
    )
    return HardwareConfig(
        ip=robot["ip"],
        motion_confirmed=bool(raw["motion_confirmed"]),
        joint_speed_rad_s=float(robot["joint_speed_rad_s"]),
        joint_acceleration_rad_s2=float(robot["joint_acceleration_rad_s2"]),
        gripper_kind=gripper["kind"],
        gripper_model_confirmed=bool(gripper["model_confirmed"]),
        gripper_open_position=float(gripper["open_position"]),
        gripper_closed_position=float(gripper["closed_position"]),
        gripper_speed=float(gripper["speed"]),
        cameras=cameras,
    )


def load_plan(path: Path = DEFAULT_PLAN_PATH) -> PickPlacePlan:
    raw = _read_json(path)
    if raw.get("schema") != "real_cube_pick_place_v1":
        raise ValueError("unsupported pick-and-place plan")
    poses = {name: _joint_pose(raw["poses"].get(name), name) for name in POSE_NAMES}
    tcp = {
        name: tuple(float(item) for item in value)
        for name, value in raw.get("tcp_poses_at_recording", {}).items()
    }
    return PickPlacePlan(poses=poses, tcp_poses_at_recording=tcp)


def load_handoff_plan(path: Path = DEFAULT_HANDOFF_PATH) -> HandoffPlacePlan:
    raw = _read_json(path)
    if raw.get("schema") != "real_cube_handoff_place_v1":
        raise ValueError("unsupported handoff-and-place plan")

    handoff = _joint_pose(raw["handoff_joint_rad"], "handoff")

    def tcp_pose(name: str) -> tuple[float, ...]:
        value = raw[name]
        if not isinstance(value, list) or len(value) != 6:
            raise ValueError(f"{name} must contain xyz and roll-pitch-yaw")
        return tuple(float(item) for item in value)

    return HandoffPlacePlan(
        handoff_joint_rad=handoff,
        preplace_joint_rad=_joint_pose(raw["preplace_joint_rad"], "preplace"),
        preplace_tcp=tcp_pose("preplace_tcp"),
        place_tcp=tcp_pose("place_tcp"),
        tcp_speed_mm_s=float(raw["tcp_speed_mm_s"]),
        tcp_acceleration_mm_s2=float(raw["tcp_acceleration_mm_s2"]),
    )


def load_probe_plan(path: Path = DEFAULT_PROBE_PATH) -> ProbePlan:
    raw = _read_json(path)
    if raw.get("schema") != "real_cube_joint_probe_v1":
        raise ValueError("unsupported joint-probe plan")
    motions = tuple(
        ProbeMotion(
            name=value["name"],
            joint_index=int(value["joint_index"]),
            amplitude_rad=float(value["amplitude_rad"]),
            frequency_hz=float(value["frequency_hz"]),
            duration_s=float(value["duration_s"]),
        )
        for value in raw["motions"]
    )
    return ProbePlan(
        control_period_s=float(raw["control_period_s"]),
        settle_s=float(raw["settle_s"]),
        recover_s=float(raw["recover_s"]),
        center_joint_rad=_joint_pose(raw["center_joint_rad"], "probe center"),
        motions=motions,
    )


def save_recorded_pose(
    name: str,
    joint_rad: list[float],
    tcp_pose: list[float],
    path: Path = DEFAULT_PLAN_PATH,
) -> None:
    if name not in POSE_NAMES:
        raise ValueError(f"pose name must be one of: {', '.join(POSE_NAMES)}")
    raw = _read_json(path)
    raw["poses"][name] = [round(float(value), 7) for value in joint_rad]
    raw.setdefault("tcp_poses_at_recording", {})[name] = [
        round(float(value), 4) for value in tcp_pose
    ]
    path.write_text(json.dumps(raw, indent=2) + "\n", encoding="utf-8")
