"""Small JSON configuration types for the xArm 6 starter kit."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any


def _mapping(value: Any, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be a JSON object")
    return value


def _finite_vector(value: Any, size: int, name: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != size:
        raise ValueError(f"{name} must contain exactly {size} values")
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{name} must contain finite numbers")
    return result


@dataclass(frozen=True)
class RobotConfig:
    ip: str
    dof: int
    control_period_s: float
    home_joint_rad: tuple[float, ...]
    hardware_confirmed: bool
    gripper_kind: str
    gripper_open_position: float | None
    gripper_closed_position: float | None
    gripper_speed: float | None

    @property
    def gripper_confirmed(self) -> bool:
        return (
            self.hardware_confirmed
            and self.gripper_open_position is not None
            and self.gripper_closed_position is not None
            and self.gripper_speed is not None
        )


@dataclass(frozen=True)
class ThrowPlan:
    name: str
    object_id: str
    control_period_s: float
    start_joint_rad: tuple[float, ...]
    release_joint_rad: tuple[float, ...]
    followthrough_joint_rad: tuple[float, ...]
    duration_to_release_s: float
    duration_followthrough_s: float
    status: str

    @property
    def dof(self) -> int:
        return len(self.start_joint_rad)


def _read_json(path: Path) -> dict[str, Any]:
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    return _mapping(value, str(path))


def load_robot_config(path: Path) -> RobotConfig:
    raw = _read_json(path)
    if raw.get("schema") != "xarm6_toss_robot_config_v1":
        raise ValueError("unsupported robot config schema")
    robot = _mapping(raw.get("robot"), "robot")
    gripper = _mapping(raw.get("gripper"), "gripper")
    dof = int(robot.get("dof", 0))
    if dof != 6:
        raise ValueError("this starter kit expects an xArm 6 with six joints")
    period = float(robot.get("control_period_s", 0.0))
    if not 0.005 <= period <= 0.1:
        raise ValueError("control_period_s must lie in [0.005, 0.1]")
    ip = str(robot.get("ip", "")).strip()
    if not ip:
        raise ValueError("robot.ip is required")

    def optional_number(name: str) -> float | None:
        value = gripper.get(name)
        if value is None:
            return None
        result = float(value)
        if not math.isfinite(result):
            raise ValueError(f"gripper.{name} must be finite")
        return result

    return RobotConfig(
        ip=ip,
        dof=dof,
        control_period_s=period,
        home_joint_rad=_finite_vector(
            robot.get("home_joint_rad"), dof, "robot.home_joint_rad"
        ),
        hardware_confirmed=bool(raw.get("hardware_confirmed", False)),
        gripper_kind=str(gripper.get("kind", "")).strip(),
        gripper_open_position=optional_number("open_position"),
        gripper_closed_position=optional_number("closed_position"),
        gripper_speed=optional_number("speed"),
    )


def load_throw_plan(path: Path) -> ThrowPlan:
    raw = _read_json(path)
    if raw.get("schema") != "xarm6_toss_throw_plan_v1":
        raise ValueError("unsupported throw plan schema")
    if raw.get("joint_units") != "radian":
        raise ValueError("throw plan joint_units must be radian")
    period = float(raw.get("control_period_s", 0.0))
    first_duration = float(raw.get("duration_to_release_s", 0.0))
    second_duration = float(raw.get("duration_followthrough_s", 0.0))
    if not 0.005 <= period <= 0.1:
        raise ValueError("control_period_s must lie in [0.005, 0.1]")
    if first_duration <= period or second_duration <= period:
        raise ValueError("both throw segments must exceed one control period")
    start = _finite_vector(raw.get("start_joint_rad"), 6, "start_joint_rad")
    release = _finite_vector(
        raw.get("release_joint_rad"), 6, "release_joint_rad"
    )
    follow = _finite_vector(
        raw.get("followthrough_joint_rad"), 6, "followthrough_joint_rad"
    )
    return ThrowPlan(
        name=str(raw.get("name", "")).strip(),
        object_id=str(raw.get("object_id", "")).strip(),
        control_period_s=period,
        start_joint_rad=start,
        release_joint_rad=release,
        followthrough_joint_rad=follow,
        duration_to_release_s=first_duration,
        duration_followthrough_s=second_duration,
        status=str(raw.get("status", "")).strip(),
    )
