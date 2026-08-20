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


class ForwardRotationLadderTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.kinematics = URDFKinematics(
            ROOT / "sim/assets/xarm6_g1/xarm6_g1.urdf"
        )

    def test_ladder_uses_static_j4_j6_and_transfer_safe_j2_j3_j5(self) -> None:
        for label, target in (("0p8", 0.8), ("1p2", 1.2), ("1p6", 1.6), ("2p0", 2.0)):
            with self.subTest(label=label):
                config = json.loads(
                    (
                        ROOT
                        / f"sim/configs/j5_forward_rotation_throwonly_{label}.json"
                    ).read_text(encoding="utf-8")
                )
                samples = generate_joint_reference(
                    tuple(
                        QuinticJointSegment(**segment)
                        for segment in config["reference_segments"]
                    ),
                    config["control_period_s"],
                )
                evidence = evaluate_reference_samples(samples)
                self.assertTrue(evidence["joint_mechanical_limits_pass"], evidence)
                q = np.asarray([sample.joint_position_rad for sample in samples])
                dq = np.asarray([sample.joint_velocity_rad_s for sample in samples])
                np.testing.assert_allclose(q[:, 3], q[0, 3], atol=1.0e-12)
                np.testing.assert_allclose(q[:, 5], q[0, 5], atol=1.0e-12)
                np.testing.assert_allclose(dq[:, 3], 0.0, atol=1.0e-12)
                np.testing.assert_allclose(dq[:, 5], 0.0, atol=1.0e-12)
                self.assertLess(float(np.min(dq[:, 1])), 0.0)
                self.assertLess(float(np.min(dq[:, 2])), 0.0)
                self.assertGreater(float(np.max(dq[:, 4])), 0.0)
                self.assertAlmostEqual(q[0, 3], np.deg2rad(165.0), places=8)

                detach_time = config["kinematic_design"][
                    "predicted_physical_detach_reference_time_s"
                ]
                detach = min(samples, key=lambda sample: abs(sample.time_s - detach_time))
                detach_q = np.asarray(detach.joint_position_rad)
                detach_dq = np.asarray(detach.joint_velocity_rad_s)
                transform = self.kinematics.forward(detach_q)
                finger = transform[:3, 2]
                axis = np.cross(finger, [0.0, 0.0, 1.0])
                axis /= np.linalg.norm(axis)
                twist = self.kinematics.jacobian(detach_q) @ detach_dq
                projection = float(np.dot(twist[3:], axis))
                alignment = abs(projection) / float(np.linalg.norm(twist[3:]))
                self.assertAlmostEqual(projection, target, delta=0.03)
                self.assertGreaterEqual(alignment, 0.90)


if __name__ == "__main__":
    unittest.main()
