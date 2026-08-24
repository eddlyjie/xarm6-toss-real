from __future__ import annotations

import json
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from xarm6_toss.control_reference import QuinticJointSegment, generate_joint_reference
from xarm6_toss.motion_limits import (
    FLOAT32_COMMAND_QUANTIZATION_RAD,
    TRANSFER_MAX_JOINT_ACCELERATION_RAD_S2,
    TRANSFER_MAX_JOINT_STEP_RAD,
    TRANSFER_MAX_JOINT_SPEED_RAD_S,
    evaluate_joint_trajectory,
    evaluate_reference_samples,
)


class MotionLimitTests(unittest.TestCase):
    def test_current_real_detach_reference_is_inside_transfer_envelope(self) -> None:
        config = json.loads(
            (ROOT / "sim/configs/outward_vertical_real_detach_v7.json").read_text(
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
        evidence = evaluate_reference_samples(samples)
        self.assertTrue(evidence["joint_mechanical_limits_pass"], evidence)
        self.assertGreaterEqual(evidence["minimum_joint_margin_rad"], 0.15)

    def test_camera_under_tumble_reference_is_inside_transfer_envelope(self) -> None:
        config = json.loads(
            (ROOT / "sim/configs/camera_under_tumble_v3.json").read_text(
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
        evidence = evaluate_reference_samples(samples)
        self.assertTrue(evidence["joint_mechanical_limits_pass"], evidence)
        self.assertGreaterEqual(evidence["minimum_joint_margin_rad"], 0.15)
        self.assertAlmostEqual(
            max(abs(sample.joint_velocity_rad_s[5]) for sample in samples), 0.0
        )

    def test_speed_violation_is_not_clipped(self) -> None:
        q = np.zeros((2, 6))
        dq = np.zeros((2, 6))
        dq[1, 2] = TRANSFER_MAX_JOINT_SPEED_RAD_S + 0.01
        evidence = evaluate_joint_trajectory(q, dq, np.zeros((2, 6)))
        self.assertFalse(evidence["joint_speed_pass"])
        self.assertFalse(evidence["joint_mechanical_limits_pass"])
        self.assertGreater(
            evidence["max_joint_speed_rad_s"], TRANSFER_MAX_JOINT_SPEED_RAD_S
        )

    def test_acceleration_and_qdot_step_are_both_checked(self) -> None:
        q = np.zeros((2, 6))
        dq = np.zeros((2, 6))
        dq[1, 0] = 0.30
        ddq = np.zeros((2, 6))
        ddq[1, 0] = TRANSFER_MAX_JOINT_ACCELERATION_RAD_S2 + 0.01
        evidence = evaluate_joint_trajectory(q, dq, ddq)
        self.assertFalse(evidence["joint_acceleration_pass"])
        self.assertFalse(evidence["qdot_change_pass"])

    def test_float32_command_step_at_limit_is_not_a_false_violation(self) -> None:
        q = np.zeros((2, 6))
        q[1, 2] = np.float32(0.03489673137664795)
        evidence = evaluate_joint_trajectory(
            q, np.zeros((2, 6)), np.zeros((2, 6))
        )
        self.assertTrue(evidence["joint_step_pass"], evidence)
        q[1, 2] = TRANSFER_MAX_JOINT_STEP_RAD + 10.0 * FLOAT32_COMMAND_QUANTIZATION_RAD
        evidence = evaluate_joint_trajectory(
            q, np.zeros((2, 6)), np.zeros((2, 6))
        )
        self.assertFalse(evidence["joint_step_pass"])

    def test_joint_margin_and_effort_are_hard_gates(self) -> None:
        q = np.zeros((2, 6))
        q[:, 4] = -1.60
        efforts = np.zeros((2, 6))
        efforts[1, 5] = 20.1
        evidence = evaluate_joint_trajectory(
            q, np.zeros((2, 6)), np.zeros((2, 6)), efforts_nm=efforts
        )
        self.assertTrue(evidence["joint_hard_bounds_pass"])
        self.assertFalse(evidence["handoff_joint_margin_pass"])
        self.assertFalse(evidence["effort_pass"])
        self.assertFalse(evidence["joint_mechanical_limits_pass"])


if __name__ == "__main__":
    unittest.main()
