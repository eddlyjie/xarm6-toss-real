from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from xarm6_toss.release_insert import (  # noqa: E402
    d_roller_mesh,
    strike_return_command,
)


class DReleaseRollerTests(unittest.TestCase):
    def test_mesh_has_flat_positive_y_face_and_circular_back(self) -> None:
        radius = 0.006
        chord = 0.002
        length = 0.024
        vertices, faces = d_roller_mesh(radius, chord, length, arc_segments=12)
        self.assertEqual(len(vertices), 26)
        self.assertEqual(len(faces), 48)
        ys = [vertex[1] for vertex in vertices]
        self.assertAlmostEqual(max(ys), chord)
        self.assertAlmostEqual(min(vertex[0] for vertex in vertices), -length / 2)
        self.assertAlmostEqual(max(vertex[0] for vertex in vertices), length / 2)
        for _, y, z in vertices:
            self.assertLessEqual(y, chord + 1.0e-12)
            self.assertAlmostEqual(math.hypot(y, z), radius)

    def test_right_part_is_a_rigid_x_axis_half_turn(self) -> None:
        vertices, _ = d_roller_mesh(0.006, 0.002, 0.024)
        mirrored = [(x, -y, -z) for x, y, z in vertices]
        self.assertAlmostEqual(min(vertex[1] for vertex in mirrored), -0.002)
        self.assertGreater(max(vertex[1] for vertex in mirrored), 0.005)


class StrikeReturnTests(unittest.TestCase):
    def test_strike_then_return(self) -> None:
        self.assertEqual(strike_return_command(-0.001, 0.003, 0.005, 0.002), (0.0, 0.0))
        position, velocity = strike_return_command(0.0025, 0.003, 0.005, 0.002)
        self.assertAlmostEqual(position, -0.0015)
        self.assertAlmostEqual(velocity, -0.6)
        position, velocity = strike_return_command(0.006, 0.003, 0.005, 0.002)
        self.assertAlmostEqual(position, -0.0015)
        self.assertAlmostEqual(velocity, 1.5)
        self.assertEqual(strike_return_command(0.008, 0.003, 0.005, 0.002), (0.0, 0.0))

    def test_without_return_holds_full_stroke(self) -> None:
        self.assertEqual(strike_return_command(0.008, 0.003, 0.005, None), (-0.003, 0.0))

    def test_hold_delays_return(self) -> None:
        position, velocity = strike_return_command(
            0.006, 0.003, 0.005, 0.002, hold_s=0.002
        )
        self.assertEqual((position, velocity), (-0.003, 0.0))


if __name__ == "__main__":
    unittest.main()
