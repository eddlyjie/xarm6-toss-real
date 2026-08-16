"""Thin official-SDK adapter. Importing this module never connects or moves."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

from .config import RobotConfig


def _sdk_type():
    try:
        from xarm.wrapper import XArmAPI
    except ImportError as exc:
        raise RuntimeError(
            "xarm-python-sdk is not installed; install it on the real-robot "
            "computer after confirming controller compatibility"
        ) from exc
    return XArmAPI


def _value(result: Any) -> Any:
    if isinstance(result, tuple) and len(result) == 2:
        code, value = result
        if int(code) != 0:
            raise RuntimeError(f"xArm SDK getter failed with code {code}")
        return value
    return result


def _require_ok(code: Any, operation: str) -> None:
    if code is None:
        return
    if int(code) != 0:
        raise RuntimeError(f"{operation} failed with xArm SDK code {code}")


@dataclass(frozen=True)
class RobotSnapshot:
    state: Any
    err_warn_code: Any
    joint_rad: Any
    tcp_pose: Any
    gripper_position: Any

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


class XArm6Client:
    """Explicit-lifetime xArm client used by the starter scripts."""

    def __init__(self, config: RobotConfig):
        self.config = config
        self.arm = None

    def connect(self) -> None:
        if self.arm is not None:
            raise RuntimeError("xArm client is already connected")
        api = _sdk_type()
        self.arm = api(self.config.ip, is_radian=True, do_not_open=True)
        _require_ok(self.arm.connect(), "connect")

    def disconnect(self) -> None:
        if self.arm is not None:
            self.arm.disconnect()
            self.arm = None

    def __enter__(self) -> "XArm6Client":
        self.connect()
        return self

    def __exit__(self, *_: Any) -> None:
        self.disconnect()

    def _connected(self):
        if self.arm is None:
            raise RuntimeError("xArm client is not connected")
        return self.arm

    def snapshot(self) -> RobotSnapshot:
        arm = self._connected()
        gripper_position = None
        if self.config.gripper_kind.startswith("xarm_gripper"):
            try:
                gripper_position = _value(arm.get_gripper_position())
            except Exception as exc:
                gripper_position = {"unavailable": str(exc)}
        return RobotSnapshot(
            state=_value(arm.get_state()),
            err_warn_code=_value(arm.get_err_warn_code()),
            joint_rad=_value(arm.get_servo_angle(is_radian=True)),
            tcp_pose=_value(arm.get_position(is_radian=True)),
            gripper_position=gripper_position,
        )

    def prepare_gripper_only(self) -> None:
        if not self.config.gripper_confirmed:
            raise RuntimeError(
                "gripper settings and hardware_confirmed must be completed"
            )
        arm = self._connected()
        _require_ok(arm.clean_gripper_error(), "clean_gripper_error")
        _require_ok(arm.set_gripper_enable(True), "set_gripper_enable")
        _require_ok(arm.set_gripper_mode(0), "set_gripper_mode")
        _require_ok(
            arm.set_gripper_speed(self.config.gripper_speed),
            "set_gripper_speed",
        )

    def set_gripper(self, position: float, *, wait: bool = True) -> None:
        arm = self._connected()
        _require_ok(
            arm.set_gripper_position(position, wait=wait),
            "set_gripper_position",
        )
