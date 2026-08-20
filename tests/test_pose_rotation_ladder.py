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


if __name__ == "__main__":
    unittest.main()
