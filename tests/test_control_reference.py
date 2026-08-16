from __future__ import annotations

from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from xarm6_toss.control_reference import (
    GripperEvent,
    QuinticJointSegment,
    g1_drive_joint_rad_to_position,
    g1_position_to_drive_joint_rad,
    generate_joint_reference,
)


class ControlReferenceTests(unittest.TestCase):
    def test_g1_real_and_sim_positions_share_the_official_scale(self) -> None:
        for real_position, drive_rad in (
            (0.0, 0.0),
            (370.0, 0.37),
            (520.0, 0.52),
            (850.0, 0.85),
        ):
            self.assertAlmostEqual(
                g1_position_to_drive_joint_rad(real_position), drive_rad
            )
            self.assertAlmostEqual(
                g1_drive_joint_rad_to_position(drive_rad), real_position
            )

    def test_release_boundary_preserves_nonzero_q_and_dq(self) -> None:
        zeros = (0.0,) * 6
        release_q = (0.0, 0.30, -0.40, 0.0, 0.50, 0.0)
        release_dq = (0.0, 0.40, -1.30, 0.0, 1.40, 0.0)
        follow_q = tuple(
            q + 0.08 * dq for q, dq in zip(release_q, release_dq)
        )
        segments = (
            QuinticJointSegment(
                "throw",
                0.60,
                zeros,
                zeros,
                release_q,
                release_dq,
            ),
            QuinticJointSegment(
                "followthrough",
                0.16,
                release_q,
                release_dq,
                follow_q,
                zeros,
            ),
        )
        samples = generate_joint_reference(segments, 0.02)
        release = samples[30]
        self.assertEqual(release.phase, "throw")
        np.testing.assert_allclose(release.joint_position_rad, release_q)
        np.testing.assert_allclose(release.joint_velocity_rad_s, release_dq)
        self.assertTrue(
            all(a < b for a, b in zip(
                [sample.time_s for sample in samples],
                [sample.time_s for sample in samples][1:],
            ))
        )

    def test_gripper_event_uses_real_and_sim_units_together(self) -> None:
        event = GripperEvent(0.60, "release_open", 850.0)
        self.assertAlmostEqual(event.drive_joint_rad, 0.85)

    def test_segment_can_preserve_constant_nonzero_acceleration(self) -> None:
        q0 = (0.0,) * 6
        v0 = (0.1,) * 6
        acceleration = (0.2,) * 6
        duration = 0.2
        q1 = tuple(
            q + v * duration + 0.5 * a * duration**2
            for q, v, a in zip(q0, v0, acceleration)
        )
        v1 = tuple(v + a * duration for v, a in zip(v0, acceleration))
        samples = generate_joint_reference(
            (
                QuinticJointSegment(
                    "catch_approach",
                    duration,
                    q0,
                    v0,
                    q1,
                    v1,
                    acceleration,
                    acceleration,
                ),
            ),
            0.02,
        )
        np.testing.assert_allclose(samples[-1].joint_position_rad, q1)
        np.testing.assert_allclose(samples[-1].joint_velocity_rad_s, v1)
        np.testing.assert_allclose(
            [sample.joint_acceleration_rad_s2 for sample in samples],
            np.tile(acceleration, (len(samples), 1)),
            atol=1.0e-10,
        )


if __name__ == "__main__":
    unittest.main()
