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
    continuous_free_flight_evidence,
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


    def test_obvious_free_flight_requires_apex_and_descending_contact(self):
        baseline = [0.0, 0.0, 0.0]
        records = []
        for index in range(9):
            time_s = 0.02 * index
            z = 0.30 + 0.60 * time_s - 4.905 * time_s**2
            velocity_z = 0.60 - 9.81 * time_s
            angle = 3.0 * time_s
            contact = 1.0 if index == 8 else 0.0
            records.append(
                {
                    "time_s": time_s,
                    "phase": "catch" if index == 8 else "flight",
                    "cube_position_w_m": [0.0, 0.0, z],
                    "cube_linear_velocity_w_m_s": [
                        0.0,
                        0.0,
                        0.5 if index == 8 else velocity_z,
                    ],
                    "cube_angular_velocity_w_rad_s": [0.0, 3.0, 0.0],
                    "cube_quaternion_wxyz": [
                        math.cos(angle / 2.0),
                        0.0,
                        math.sin(angle / 2.0),
                        0.0,
                    ],
                    "cube_position_hand_m": [0.0, 0.0, 0.5 * time_s],
                    "left_finger_cube_contact_force_n": contact,
                    "right_finger_cube_contact_force_n": contact,
                }
            )
        evidence = continuous_free_flight_evidence(
            records, baseline, release_height_m=0.20
        )
        self.assertTrue(evidence["obvious_free_flight"])
        self.assertAlmostEqual(
            evidence["continuous_free_flight_duration_s"], 0.16
        )
        self.assertGreaterEqual(
            evidence["free_flight_rise_from_kinematic_release_m"], 0.04
        )
        self.assertTrue(evidence["free_flight_apex_is_internal"])
        self.assertLess(evidence["precontact_vertical_velocity_m_s"], 0.0)
        self.assertGreater(evidence["free_flight_rotation_rad"], 0.3)
        self.assertGreater(evidence["free_flight_rotation_deg"], 8.0)
        self.assertTrue(evidence["visible_spin"])

    def test_micro_release_is_not_obvious_toss(self):
        records = []
        for index in range(4):
            records.append(
                {
                    "time_s": 0.02 * index,
                    "phase": "catch" if index == 3 else "flight",
                    "cube_position_w_m": [0.0, 0.0, 0.3 + 0.01 * index],
                    "cube_linear_velocity_w_m_s": [0.0, 0.0, 0.2],
                    "cube_angular_velocity_w_rad_s": [0.0, 0.0, 0.0],
                    "cube_quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                    "cube_position_hand_m": [0.0, 0.0, 0.005 * index],
                    "left_finger_cube_contact_force_n": 1.0 if index == 3 else 0.0,
                    "right_finger_cube_contact_force_n": 1.0 if index == 3 else 0.0,
                }
            )
        evidence = continuous_free_flight_evidence(records, [0.0, 0.0, 0.0])
        self.assertFalse(evidence["obvious_free_flight"])
        self.assertFalse(evidence["free_flight_apex_is_internal"])
        self.assertFalse(evidence["visible_spin"])


if __name__ == "__main__":
    unittest.main()
