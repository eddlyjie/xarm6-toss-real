"""Local xArm 6 URDF forward kinematics with the installed G1 TCP."""

from pathlib import Path

import numpy as np
from yourdfpy import URDF

from .config import PROJECT_ROOT
from .spin_toss import matrix_pose


URDF_PATH = (
    PROJECT_ROOT
    / "RobotCamCalib"
    / "RobotCamCalib"
    / "assets"
    / "robots"
    / "xarm6"
    / "xarm6_wo_ee.urdf"
)
G1_TCP_OFFSET_M = 0.172


class LocalXArmFK:
    """URDF FK matching the xArm controller's installed G1 TCP offset."""

    def __init__(self, urdf_path: Path = URDF_PATH):
        self.urdf = URDF.load(urdf_path, load_meshes=False)

    def forward_kinematics(self, joint_rad):
        self.urdf.update_cfg(np.asarray(joint_rad, dtype=float))
        transform = self.urdf.get_transform("link_eef", "link_base").copy()
        transform[:3, 3] += transform[:3, :3] @ np.asarray(
            [0.0, 0.0, G1_TCP_OFFSET_M]
        )
        return matrix_pose(transform)
