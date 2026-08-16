from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from xarm6_toss import generate_throw_samples, load_throw_plan
from xarm6_toss.config import load_robot_config


class StarterKitTests(unittest.TestCase):
    def test_example_plan_has_one_release_and_exact_endpoints(self) -> None:
        plan = load_throw_plan(ROOT / "configs" / "throw_only_cube.json")
        samples = generate_throw_samples(plan)
        releases = [sample for sample in samples if sample.release_gripper]
        self.assertEqual(len(releases), 1)
        self.assertEqual(samples[0].joint_rad, plan.start_joint_rad)
        self.assertEqual(releases[0].joint_rad, plan.release_joint_rad)
        self.assertEqual(samples[-1].joint_rad, plan.followthrough_joint_rad)
        self.assertAlmostEqual(
            releases[0].time_s, plan.duration_to_release_s
        )
        self.assertAlmostEqual(
            samples[-1].time_s,
            plan.duration_to_release_s + plan.duration_followthrough_s,
        )

    def test_example_robot_config_is_not_execution_ready(self) -> None:
        config = load_robot_config(ROOT / "configs" / "robot.example.json")
        self.assertEqual(config.dof, 6)
        self.assertFalse(config.hardware_confirmed)
        self.assertFalse(config.gripper_confirmed)

    def test_non_six_dof_plan_is_rejected(self) -> None:
        source = json.loads(
            (ROOT / "configs" / "throw_only_cube.json").read_text()
        )
        source["start_joint_rad"] = [0.0] * 7
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.json"
            path.write_text(json.dumps(source), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "exactly 6"):
                load_throw_plan(path)

    def test_samples_are_strictly_time_ordered(self) -> None:
        plan = load_throw_plan(ROOT / "configs" / "throw_only_cube.json")
        times = [sample.time_s for sample in generate_throw_samples(plan)]
        self.assertTrue(all(a < b for a, b in zip(times, times[1:])))


if __name__ == "__main__":
    unittest.main()
