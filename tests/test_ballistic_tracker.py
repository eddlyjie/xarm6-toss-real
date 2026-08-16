from pathlib import Path
import sys
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from xarm6_toss.ballistic_tracker import BallisticTracker


class BallisticTrackerTests(unittest.TestCase):
    def test_encoder_prior_propagates_through_blind_gap(self):
        tracker = BallisticTracker()
        tracker.set_encoder_prior(0.1, [0.2, 0.0, 0.4], [0.1, 0.0, 0.5])
        estimate = tracker.estimate(0.2)
        np.testing.assert_allclose(
            estimate.position_m,
            [0.21, 0.0, 0.4 + 0.05 - 0.5 * 9.81 * 0.1**2],
        )
        np.testing.assert_allclose(
            estimate.velocity_m_s,
            [0.1, 0.0, 0.5 - 9.81 * 0.1],
        )
        self.assertEqual(estimate.camera_sample_count, 0)

    def test_gravity_constrained_fit_recovers_release_state(self):
        tracker = BallisticTracker()
        release_position = np.asarray([0.18, 0.01, 0.37])
        release_velocity = np.asarray([0.28, 0.05, 0.45])
        gravity = np.asarray([0.0, 0.0, -9.81])
        tracker.set_encoder_prior(0.30, release_position, release_velocity)
        for time_s in [0.32, 0.34, 0.36, 0.38]:
            dt = time_s - 0.30
            position = (
                release_position
                + release_velocity * dt
                + 0.5 * gravity * dt**2
            )
            tracker.add_camera_position(time_s, position)
        estimate = tracker.estimate(0.41)
        dt = 0.11
        expected_position = (
            release_position
            + release_velocity * dt
            + 0.5 * gravity * dt**2
        )
        expected_velocity = release_velocity + gravity * dt
        np.testing.assert_allclose(estimate.position_m, expected_position, atol=1e-10)
        np.testing.assert_allclose(estimate.velocity_m_s, expected_velocity, atol=1e-10)
        self.assertLess(estimate.fit_rms_m, 1e-10)

    def test_constant_camera_bias_does_not_corrupt_fitted_velocity(self):
        tracker = BallisticTracker(max_camera_samples=3)
        tracker.set_encoder_prior(0.0, [0.0, 0.0, 0.0], [0.2, 0.0, 0.4])
        bias = np.asarray([-0.02, 0.004, 0.006])
        gravity = np.asarray([0.0, 0.0, -9.81])
        for time_s in [0.02, 0.04, 0.06, 0.08]:
            position = (
                np.asarray([0.0, 0.0, 0.0])
                + np.asarray([0.2, 0.0, 0.4]) * time_s
                + 0.5 * gravity * time_s**2
                + bias
            )
            tracker.add_camera_position(time_s, position)
        estimate = tracker.estimate(0.08)
        np.testing.assert_allclose(
            estimate.velocity_m_s,
            [0.2, 0.0, 0.4 - 9.81 * 0.08],
            atol=1e-10,
        )
        self.assertEqual(estimate.camera_sample_count, 3)


if __name__ == "__main__":
    unittest.main()
