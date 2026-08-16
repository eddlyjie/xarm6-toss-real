from pathlib import Path
import sys
import unittest


DEMO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(DEMO_ROOT / "src"))

from real_cube_demo.config import (
    load_handoff_plan,
    load_hardware,
    load_plan,
    load_probe_plan,
)
from real_cube_demo.spin_toss import load_spin_toss_spec


class ConfigTest(unittest.TestCase):
    def test_hardware_roles_and_confirmed_g1(self):
        hardware = load_hardware()
        self.assertEqual(hardware.gripper_kind, "xarm_gripper_g1")
        self.assertTrue(hardware.gripper_model_confirmed)
        self.assertEqual(
            [camera.role for camera in hardware.cameras], ["global", "wrist"]
        )
        self.assertEqual(
            [camera.serial for camera in hardware.cameras],
            ["317222073552", "233622079809"],
        )

    def test_initial_plan_requires_teaching(self):
        plan = load_plan()
        self.assertEqual(
            plan.missing_poses,
            ("home", "pregrasp", "grasp", "lift", "preplace", "place"),
        )

    def test_hardcoded_handoff_plan(self):
        plan = load_handoff_plan()
        self.assertEqual(len(plan.handoff_joint_rad), 6)
        self.assertEqual(len(plan.preplace_joint_rad), 6)
        self.assertEqual(plan.preplace_tcp[:2], (590.3, -44.3))
        self.assertGreater(plan.preplace_tcp[2], plan.place_tcp[2])

    def test_probe_returns_to_center(self):
        plan = load_probe_plan()
        self.assertEqual(len(plan.center_joint_rad), 6)
        self.assertEqual([motion.joint_index for motion in plan.motions], [4, 5])
        self.assertEqual(plan.control_period_s, 0.02)

    def test_spin_toss_uses_observed_geometry_without_object_priors(self):
        plan = load_spin_toss_spec()
        self.assertEqual(plan.capture_mode, "observed_ballistic_intercept")
        self.assertEqual(plan.prethrow_path, "cartesian_vertical")
        self.assertEqual(len(plan.grasp_offset_tool_m), 3)
        self.assertEqual(
            plan.geometry_source, "global_camera_observed_grasp_offset"
        )
        self.assertIn("20260815_211845_cube", plan.grasp_offset_source)


if __name__ == "__main__":
    unittest.main()
