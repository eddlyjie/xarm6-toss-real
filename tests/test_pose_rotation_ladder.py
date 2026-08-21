from __future__ import annotations

import json
from pathlib import Path
import sys
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from xarm6_toss.control_reference import (  # noqa: E402
    QuinticJointSegment,
    generate_joint_reference,
)
from xarm6_toss.motion_limits import evaluate_reference_samples  # noqa: E402
from xarm6_toss_sim.kinematics import URDFKinematics  # noqa: E402


class PoseRotationLadderTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.kinematics = URDFKinematics(
            ROOT / "sim/assets/xarm6_g1/xarm6_g1.urdf"
        )

    def test_rotation_targets_are_monotonic_and_real_envelope_safe(self) -> None:
        hand_omegas = []
        for label, requested in (("r30", 30.0), ("r60", 60.0), ("r90", 90.0)):
            with self.subTest(label=label):
                config = json.loads(
                    (ROOT / f"sim/configs/pose_rotation_throwonly_{label}.json").read_text(
                        encoding="utf-8"
                    )
                )
                samples = generate_joint_reference(
                    tuple(
                        QuinticJointSegment(**segment)
                        for segment in config["reference_segments"]
                    ),
                    config["control_period_s"],
                )
                limits = evaluate_reference_samples(samples)
                self.assertTrue(limits["joint_mechanical_limits_pass"], limits)
                q = np.asarray([sample.joint_position_rad for sample in samples])
                dq = np.asarray([sample.joint_velocity_rad_s for sample in samples])
                np.testing.assert_allclose(q[:, 3], q[0, 3], atol=1.0e-12)
                np.testing.assert_allclose(q[:, 5], q[0, 5], atol=1.0e-12)
                np.testing.assert_allclose(dq[:, 3], 0.0, atol=1.0e-12)
                np.testing.assert_allclose(dq[:, 5], 0.0, atol=1.0e-12)
                self.assertAlmostEqual(
                    config["kinematic_design"]["requested_rotation_deg"], requested
                )
                hand_omegas.append(
                    config["kinematic_design"]["predicted_hand_axis_omega_rad_s"]
                )
                self.assertLessEqual(
                    config["kinematic_design"]["predicted_peak_tcp_speed_m_s"],
                    1.6 + 1.0e-6,
                )
                self.assertEqual(
                    config["lineage"]["goal"], "goal.md v5"
                )
        self.assertTrue(all(a < b for a, b in zip(hand_omegas, hand_omegas[1:])))

    def test_contact_aligned_brake_advances_only_the_brake_phase(self) -> None:
        baseline = json.loads(
            (ROOT / "sim/configs/pose_rotation_throwonly_r90.json").read_text(
                encoding="utf-8"
            )
        )
        aligned = json.loads(
            (ROOT / "sim/configs/pose_rotation_throwonly_r90_brake20.json").read_text(
                encoding="utf-8"
            )
        )
        baseline_segments = baseline["reference_segments"]
        aligned_segments = aligned["reference_segments"]
        self.assertEqual(
            aligned_segments[2]["duration_s"],
            baseline_segments[2]["duration_s"] - 0.020,
        )
        for index in (0, 1):
            self.assertEqual(aligned_segments[index], baseline_segments[index])
        self.assertAlmostEqual(
            aligned["kinematic_design"]["contact_phase_brake_advance_s"],
            0.020,
        )
        samples = generate_joint_reference(
            tuple(QuinticJointSegment(**segment) for segment in aligned_segments),
            aligned["control_period_s"],
        )
        self.assertTrue(evaluate_reference_samples(samples)["joint_mechanical_limits_pass"])

    def test_stock_g1_catchable_reference_reduces_outward_velocity(self) -> None:
        config = json.loads(
            (ROOT / "sim/configs/pose_rotation_throwonly_r10c.json").read_text(
                encoding="utf-8"
            )
        )
        design = config["kinematic_design"]
        cube_velocity = design["predicted_cube_com_velocity_m_s"]
        self.assertLess(abs(cube_velocity[0]), 0.03)
        self.assertGreater(cube_velocity[2], 1.0)
        self.assertGreater(design["predicted_hand_axis_omega_rad_s"], 3.0)
        samples = generate_joint_reference(
            tuple(
                QuinticJointSegment(**segment)
                for segment in config["reference_segments"]
            ),
            config["control_period_s"],
        )
        self.assertTrue(evaluate_reference_samples(samples)["joint_mechanical_limits_pass"])
        q = np.asarray([sample.joint_position_rad for sample in samples])
        np.testing.assert_allclose(q[:, 3], q[0, 3], atol=1.0e-12)
        np.testing.assert_allclose(q[:, 5], q[0, 5], atol=1.0e-12)

    def test_high_release_adds_height_without_changing_release_velocity(self) -> None:
        low = json.loads(
            (ROOT / "sim/configs/pose_rotation_throwonly_r10c.json").read_text(
                encoding="utf-8"
            )
        )
        high = json.loads(
            (ROOT / "sim/configs/pose_rotation_throwonly_r10ch.json").read_text(
                encoding="utf-8"
            )
        )
        low_design = low["kinematic_design"]
        high_design = high["kinematic_design"]
        self.assertEqual(
            high_design["release_joint_velocity_rad_s"],
            low_design["release_joint_velocity_rad_s"],
        )
        self.assertGreater(
            high_design["predicted_peak_tcp_position_m"][2]
            - low_design["predicted_peak_tcp_position_m"][2],
            0.10,
        )
        self.assertTrue(
            high_design["reference_limit_evidence"]["joint_mechanical_limits_pass"]
        )

    def test_residual_compensated_reference_biases_predicted_outward_velocity(self) -> None:
        config = json.loads(
            (ROOT / "sim/configs/pose_rotation_throwonly_r10cx.json").read_text(
                encoding="utf-8"
            )
        )
        design = config["kinematic_design"]
        cube_velocity = design["predicted_cube_com_velocity_m_s"]
        self.assertGreater(cube_velocity[0], 0.15)
        self.assertLess(cube_velocity[0], 0.25)
        self.assertGreater(design["predicted_hand_axis_omega_rad_s"], 2.3)
        self.assertTrue(
            design["reference_limit_evidence"]["joint_mechanical_limits_pass"]
        )
        velocity = np.asarray(design["release_joint_velocity_rad_s"])
        self.assertAlmostEqual(velocity[3], 0.0)
        self.assertAlmostEqual(velocity[5], 0.0)

    def test_tracking_compensated_reference_preserves_static_j4_j6(self) -> None:
        config = json.loads(
            (ROOT / "sim/configs/pose_rotation_throwonly_r10cy.json").read_text(
                encoding="utf-8"
            )
        )
        design = config["kinematic_design"]
        self.assertGreater(design["predicted_hand_axis_omega_rad_s"], 2.7)
        self.assertTrue(
            design["reference_limit_evidence"]["joint_mechanical_limits_pass"]
        )
        velocity = np.asarray(design["release_joint_velocity_rad_s"])
        self.assertAlmostEqual(velocity[3], 0.0)
        self.assertAlmostEqual(velocity[5], 0.0)


if __name__ == "__main__":
    unittest.main()
