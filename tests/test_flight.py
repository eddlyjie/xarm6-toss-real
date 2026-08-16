import math
from pathlib import Path
import sys
import unittest

import numpy as np
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from xarm6_toss.flight import (
    angular_velocity_for_target_rotation,
    flight_time_to_height,
    make_flight_state,
    nominal_object_twist,
    propagate_cube,
)


class FlightPhysicsTests(unittest.TestCase):
    def test_cube_returns_to_release_height(self):
        upward = 0.8
        flight_time = flight_time_to_height(0.25, upward, 0.25)
        state = make_flight_state(
            [0.2, 0.0, 0.25], np.eye(3), [0.0, 0.0, upward], [0.0, 0.0, 0.0]
        )
        returned = propagate_cube(state, flight_time)
        self.assertAlmostEqual(returned.position_m[2], 0.25, places=9)
        self.assertAlmostEqual(returned.linear_velocity_m_s[2], -upward, places=9)

    def test_target_rotation_is_reached_without_mass_or_size(self):
        duration = 0.25
        angular = angular_velocity_for_target_rotation(
            [0.0, 0.0, 1.0], math.pi / 4.0, duration
        )
        state = make_flight_state(
            [0.0, 0.0, 0.0], np.eye(3), [0.0, 0.0, 0.0], angular
        )
        final = propagate_cube(state, duration)
        angle = Rotation.from_matrix(final.rotation_world_object).magnitude()
        self.assertAlmostEqual(angle, math.pi / 4.0, places=9)

    def test_object_linear_velocity_contains_lever_arm_term(self):
        linear, angular = nominal_object_twist(
            [0.0, 0.0, 0.5], [0.0, 2.0, 0.0], [0.1, 0.0, 0.0]
        )
        self.assertEqual(angular, (0.0, 2.0, 0.0))
        np.testing.assert_allclose(linear, [0.0, 0.0, 0.3])


if __name__ == "__main__":
    unittest.main()
