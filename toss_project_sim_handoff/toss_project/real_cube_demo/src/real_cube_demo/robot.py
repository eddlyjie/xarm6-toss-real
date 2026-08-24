"""Slow xArm 6 pick-and-place control built on the xarm_6 SDK adapter."""

from __future__ import annotations

from dataclasses import asdict
import sys
import time
from typing import Any

from .config import HardwareConfig, PROJECT_ROOT


XARM6_SRC = PROJECT_ROOT / "xarm_6" / "src"
if str(XARM6_SRC) not in sys.path:
    sys.path.insert(0, str(XARM6_SRC))

from xarm6_toss.config import RobotConfig  # noqa: E402
from xarm6_toss.xarm_adapter import XArm6Client  # noqa: E402


def _value(result: Any, operation: str) -> Any:
    if isinstance(result, tuple) and len(result) == 2:
        code, value = result
        if int(code) != 0:
            raise RuntimeError(f"{operation} failed with xArm SDK code {code}")
        return value
    return result


def _ok(code: Any, operation: str) -> None:
    if code is not None and int(code) != 0:
        raise RuntimeError(f"{operation} failed with xArm SDK code {code}")


class PickPlaceRobot:
    def __init__(self, hardware: HardwareConfig):
        self.hardware = hardware
        adapter_config = RobotConfig(
            ip=hardware.ip,
            dof=6,
            control_period_s=0.02,
            home_joint_rad=(0.0,) * 6,
            hardware_confirmed=hardware.motion_confirmed,
            gripper_kind=hardware.gripper_kind,
            gripper_open_position=hardware.gripper_open_position,
            gripper_closed_position=hardware.gripper_closed_position,
            gripper_speed=hardware.gripper_speed,
        )
        self.client = XArm6Client(adapter_config)

    def __enter__(self) -> "PickPlaceRobot":
        self.client.connect()
        return self

    def __exit__(self, *_: Any) -> None:
        self.client.disconnect()

    @property
    def arm(self):
        return self.client._connected()

    def inventory(self) -> dict[str, Any]:
        snapshot = asdict(self.client.snapshot())
        snapshot.update(
            connected=bool(self.arm.connected),
            mode=int(self.arm.mode),
            motor_enable_states=list(self.arm.motor_enable_states[:6]),
            motor_brake_states=list(self.arm.motor_brake_states[:6]),
            joint_temperature_c=list(self.arm.temperatures[:6]),
            servo_status_code=[list(value) for value in self.arm.servo_codes[:6]],
            joint_voltage_v=list(self.arm.voltages[:6]),
            motor_current_a=list(self.arm.currents[:6]),
            robot_version=_value(self.arm.get_version(), "get_version"),
            robot_sn=_value(self.arm.get_robot_sn(), "get_robot_sn"),
            gripper_version=_value(
                self.arm.get_gripper_version(), "get_gripper_version"
            ),
            gripper_status=_value(
                self.arm.get_gripper_status(), "get_gripper_status"
            ),
            gripper_error=_value(
                self.arm.get_gripper_err_code(), "get_gripper_err_code"
            ),
        )
        return snapshot

    def prepare_motion(self) -> None:
        if not self.hardware.motion_confirmed:
            raise RuntimeError(
                "set motion_confirmed=true in configs/hardware.json after checking the workspace"
            )
        _ok(self.arm.clean_warn(), "clean_warn")
        _ok(self.arm.clean_error(), "clean_error")
        _ok(self.arm.motion_enable(enable=True), "motion_enable")
        _ok(self.arm.set_mode(0), "set_mode")
        _ok(self.arm.set_state(0), "set_state")
        self.client.prepare_gripper_only()

    def move_joints(
        self,
        joint_rad: tuple[float, ...],
        name: str,
        *,
        speed_rad_s: float | None = None,
        acceleration_rad_s2: float | None = None,
    ) -> None:
        print(f"moving to {name}: {[round(value, 4) for value in joint_rad]}")
        _ok(
            self.arm.set_servo_angle(
                angle=list(joint_rad),
                speed=(
                    self.hardware.joint_speed_rad_s
                    if speed_rad_s is None
                    else speed_rad_s
                ),
                mvacc=(
                    self.hardware.joint_acceleration_rad_s2
                    if acceleration_rad_s2 is None
                    else acceleration_rad_s2
                ),
                is_radian=True,
                wait=True,
                radius=-1,
            ),
            f"move to {name}",
        )

    def move_tcp(
        self,
        pose: tuple[float, ...],
        name: str,
        speed_mm_s: float,
        acceleration_mm_s2: float,
    ) -> None:
        print(f"moving to {name}: {[round(value, 4) for value in pose]}")
        _ok(
            self.arm.set_position(
                *pose,
                speed=speed_mm_s,
                mvacc=acceleration_mm_s2,
                is_radian=True,
                wait=True,
                radius=-1,
            ),
            f"move to {name}",
        )

    def joint_signals(self) -> dict[str, Any]:
        joint_state = _value(
            self.arm.get_joint_states(is_radian=True, num=3), "get_joint_states"
        )
        position, velocity, effort = joint_state
        return {
            "joint_position_rad": list(position[:6]),
            "joint_velocity_rad_s": list(velocity[:6]),
            "joint_effort": list(effort[:6]),
            "motor_current": list(self.arm.currents[:6]),
            "gripper_position": _value(
                self.arm.get_gripper_position(), "get_gripper_position"
            ),
        }

    def reported_joint_signals(self) -> dict[str, list[float]]:
        position, velocity, effort = _value(
            self.arm.get_joint_states(is_radian=True, num=3),
            "get_joint_states",
        )
        return {
            "joint_position_rad": list(position[:6]),
            "joint_velocity_rad_s": list(velocity[:6]),
            "joint_effort": list(effort[:6]),
            "motor_current": list(self.arm.currents[:6]),
        }

    def controller_status(self) -> dict[str, Any]:
        return {
            "connected": bool(self.arm.connected),
            "mode": int(self.arm.mode),
            "state": _value(self.arm.get_state(), "get_state"),
            "err_warn_code": _value(
                self.arm.get_err_warn_code(), "get_err_warn_code"
            ),
        }

    def enter_servo_mode(self) -> None:
        _ok(self.arm.set_mode(1), "set_mode(servo)")
        _ok(self.arm.set_state(0), "set_state")
        time.sleep(0.2)

    def servo_j(self, joint_rad: tuple[float, ...]) -> None:
        _ok(
            self.arm.set_servo_angle_j(list(joint_rad), is_radian=True),
            "set_servo_angle_j",
        )

    def enter_position_mode(self) -> None:
        _ok(self.arm.set_mode(0), "set_mode(position)")
        _ok(self.arm.set_state(0), "set_state")

    def forward_kinematics(self, joint_rad: tuple[float, ...]) -> tuple[float, ...]:
        pose = _value(
            self.arm.get_forward_kinematics(
                list(joint_rad),
                input_is_radian=True,
                return_is_radian=True,
            ),
            "get_forward_kinematics",
        )
        return tuple(float(value) for value in pose[:6])

    def inverse_kinematics(
        self,
        tcp_pose: tuple[float, ...],
        ref_joint_rad: tuple[float, ...] | None = None,
    ) -> tuple[float, ...]:
        joint_rad = _value(
            self.arm.get_inverse_kinematics(
                list(tcp_pose),
                input_is_radian=True,
                return_is_radian=True,
                ref_angles=(
                    None if ref_joint_rad is None else list(ref_joint_rad)
                ),
            ),
            "get_inverse_kinematics",
        )
        return tuple(float(value) for value in joint_rad[:6])

    def linear_speed_limit_factor(self) -> float:
        return float(
            _value(
                self.arm.get_linear_spd_limit_factor(),
                "get_linear_spd_limit_factor",
            )
        )

    def set_linear_speed_limit_factor(self, factor: float) -> None:
        _ok(
            self.arm.set_linear_spd_limit_factor(float(factor)),
            "set_linear_spd_limit_factor",
        )

    def gripper_position(self, *, check_baud: bool = True) -> float:
        return float(
            _value(
                self.arm.get_gripper_position(check_baud=check_baud),
                "get_gripper_position",
            )
        )

    def set_gripper_position(self, position: float) -> None:
        print(f"moving G1 gripper to {position:g}")
        self.client.set_gripper(position, wait=True)

    def command_gripper_position(self, position: float) -> None:
        """Send a G1 target immediately, including while the arm is moving."""
        _ok(
            self.arm.set_gripper_position(
                position,
                wait=False,
                wait_motion=False,
                check_baud=False,
                check_err=False,
            ),
            "set_gripper_position(nonblocking)",
        )

    def open_gripper(self) -> None:
        print(f"opening G1 gripper to {self.hardware.gripper_open_position:g}")
        self.client.set_gripper(self.hardware.gripper_open_position, wait=True)

    def close_gripper(self) -> None:
        print(f"closing G1 gripper toward {self.hardware.gripper_closed_position:g}")
        self.client.set_gripper(self.hardware.gripper_closed_position, wait=True)

    def stop(self) -> None:
        _ok(self.arm.set_state(4), "set_state(stop)")
