"""Load real-robot limits and camera calibration as simulation inputs."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
SIM_CONFIG_PATH = REPO_ROOT / "configs" / "sim_to_real.json"


@dataclass(frozen=True)
class CameraCalibration:
    role: str
    serial: str
    width: int
    height: int
    fps: int
    intrinsic: np.ndarray
    mount_from_camera: np.ndarray

    def project(self, mount_point_m: np.ndarray) -> tuple[float, float, float] | None:
        camera_from_mount = np.linalg.inv(self.mount_from_camera)
        camera_point = camera_from_mount @ np.append(np.asarray(mount_point_m, dtype=float), 1.0)
        if camera_point[2] <= 0.0:
            return None
        pixel = self.intrinsic @ camera_point[:3]
        u = float(pixel[0] / pixel[2])
        v = float(pixel[1] / pixel[2])
        if not (0.0 <= u < self.width and 0.0 <= v < self.height):
            return None
        return u, v, float(camera_point[2])


@dataclass(frozen=True)
class RealSetup:
    repo_root: Path
    source_root: Path
    urdf_path: Path
    control_period_s: float
    max_joint_speed_rad_s: float
    max_joint_acceleration_rad_s2: float
    arm_tracking_delay_s: float
    detach_delay_range_s: tuple[float, float]
    held_gripper_position: float
    partial_open_gripper_position: float
    close_gripper_position: float
    cube_side_m: float
    cube_mass_range_kg: tuple[float, float]
    nominal_cube_mass_kg: float
    third_view: CameraCalibration
    wrist: CameraCalibration
    acceptance: dict[str, float | int]


class G1ApertureModel:
    """Map G1 encoder position to projected width using the received URDF."""

    def __init__(
        self,
        urdf_path: Path,
        anchor_position: float = 370.0,
        anchor_width_m: float = 0.038,
    ):
        root = ET.parse(urdf_path).getroot()
        joints = {str(joint.get("name")): joint for joint in root.findall("joint")}
        drive_limit = joints["drive_joint"].find("limit")
        self.closed_joint_rad = float(drive_limit.get("upper"))
        left_outer = joints["drive_joint"].find("origin")
        left_finger = joints["left_finger_joint"].find("origin")
        self.outer_y_m = float(left_outer.get("xyz").split()[1])
        finger_xyz = [float(value) for value in left_finger.get("xyz").split()]
        self.finger_y_m = finger_xyz[1]
        self.finger_z_m = finger_xyz[2]
        anchor_center_gap = self._finger_center_gap(anchor_position)
        self.contact_surface_offset_m = anchor_center_gap - anchor_width_m

    def _joint_angle(self, gripper_position: float) -> float:
        return self.closed_joint_rad - float(gripper_position) / 1000.0

    def _finger_center_gap(self, gripper_position: float) -> float:
        angle = self._joint_angle(gripper_position)
        half_gap = (
            self.outer_y_m
            + self.finger_y_m * np.cos(angle)
            - self.finger_z_m * np.sin(angle)
        )
        return 2.0 * float(half_gap)

    def projected_width_m(self, gripper_position: float) -> float:
        return self._finger_center_gap(gripper_position) - self.contact_surface_offset_m


def _camera(
    source_root: Path,
    hardware: dict,
    camera_cfg: dict,
    role: str,
) -> CameraCalibration:
    raw = hardware["cameras"][role]
    intrinsic_raw = yaml.safe_load((source_root / raw["intrinsics"]).read_text(encoding="utf-8"))
    extrinsic_raw = yaml.safe_load((source_root / raw["extrinsics"]).read_text(encoding="utf-8"))
    return CameraCalibration(
        role=role,
        serial=str(raw["serial"]),
        width=int(camera_cfg["width"]),
        height=int(camera_cfg["height"]),
        fps=int(camera_cfg["requested_fps"]),
        intrinsic=np.asarray(intrinsic_raw["K"], dtype=float),
        mount_from_camera=np.asarray(extrinsic_raw["X_CammountCam"], dtype=float),
    )


def load_real_setup(config_path: Path = SIM_CONFIG_PATH) -> RealSetup:
    config_path = Path(config_path)
    config = json.loads(config_path.read_text(encoding="utf-8"))
    repo_root = config_path.parent.parent
    source_root = repo_root / config["source_project"]
    hardware = json.loads((source_root / config["hardware_config"]).read_text(encoding="utf-8"))
    robot = hardware["robot"]
    gripper = config["gripper"]
    cube = config["cube"]
    return RealSetup(
        repo_root=repo_root,
        source_root=source_root,
        urdf_path=source_root / config["robot_urdf"],
        control_period_s=float(robot["control_period_s"]),
        max_joint_speed_rad_s=float(robot["joint_speed_rad_s"]),
        max_joint_acceleration_rad_s2=float(robot["joint_acceleration_rad_s2"]),
        arm_tracking_delay_s=float(config["arm_tracking_delay_s"]),
        detach_delay_range_s=tuple(float(value) for value in gripper["detach_delay_range_s"]),
        held_gripper_position=float(gripper["held_position"]),
        partial_open_gripper_position=float(gripper["partial_open_position"]),
        close_gripper_position=float(gripper["close_position"]),
        cube_side_m=float(cube["side_m"]),
        cube_mass_range_kg=tuple(float(value) for value in cube["mass_range_kg"]),
        nominal_cube_mass_kg=float(cube["nominal_mass_kg"]),
        third_view=_camera(source_root, hardware, config["camera"], "global"),
        wrist=_camera(source_root, hardware, config["camera"], "wrist"),
        acceptance=config["acceptance"],
    )
