from __future__ import annotations

import json
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from xarm6_toss.method import (
    CatchObjectiveTerms,
    CoordinatorCandidate,
    ProbeCandidate,
    catch_objective,
    select_coordinated,
    select_fixed_confidence,
    select_probe,
)


class MethodCoreTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.config = json.loads(
            (ROOT / "configs" / "method.example.json").read_text(
                encoding="utf-8"
            )
        )

    def target_candidates(self, target_id: str):
        values = []
        for skill in self.config["skills"]:
            values.append(
                CoordinatorCandidate(
                    skill_id=skill["skill_id"],
                    catch_probability=skill["catch_probability"],
                    detach_flight_uncertainty=skill[
                        "detach_flight_uncertainty"
                    ],
                    collision_contact_risk=skill[
                        "collision_contact_risk"
                    ],
                    **skill["targets"][target_id],
                )
            )
        return values

    def test_task_conditioned_probe_prefers_useful_safe_excitation(self):
        values = [
            ProbeCandidate(**item)
            for item in self.config["probe_candidates"]
        ]
        selected, ranking = select_probe(values, task_conditioned=True)
        self.assertEqual(selected.name, "short_pitch_chirp")
        self.assertEqual(ranking[0][0], selected.name)

    def test_catch_objective_rewards_probability(self):
        base = dict(
            task_grasp_error=0.1,
            relative_contact_velocity=0.1,
            impact_energy=0.1,
            slip_risk=0.1,
            arm_motion_cost=0.1,
            cvar_failure=0.1,
        )
        likely = catch_objective(
            CatchObjectiveTerms(catch_probability=0.9, **base)
        )
        unlikely = catch_objective(
            CatchObjectiveTerms(catch_probability=0.2, **base)
        )
        self.assertLess(likely, unlikely)

    def test_m2_is_target_independent_but_m3_changes_with_target(self):
        upright = self.target_candidates("upright_forward")
        rotated = self.target_candidates("quarter_turn_forward")
        self.assertEqual(select_fixed_confidence(upright).skill_id, "low_spin")
        self.assertEqual(select_fixed_confidence(rotated).skill_id, "low_spin")
        self.assertEqual(select_coordinated(upright)[0].skill_id, "low_spin")
        self.assertEqual(
            select_coordinated(rotated)[0].skill_id,
            "quarter_turn_regrasp",
        )

    def test_no_eligible_target_skill_stops(self):
        candidate = self.target_candidates("quarter_turn_forward")[0]
        with self.assertRaisesRegex(RuntimeError, "no target-coordinated"):
            select_coordinated([candidate])


if __name__ == "__main__":
    unittest.main()
