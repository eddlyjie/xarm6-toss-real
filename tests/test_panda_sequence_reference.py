from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import unittest


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "sim_reference"
    / "panda_sequence_reference.py"
)
SPEC = importlib.util.spec_from_file_location("panda_sequence_reference", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
REFERENCE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = REFERENCE
SPEC.loader.exec_module(REFERENCE)


class PandaSequenceReferenceTests(unittest.TestCase):
    def test_joint_reference_interpolates_q_and_dq_together(self) -> None:
        plan = REFERENCE.JointReference(
            time_s=(0.0, 1.0),
            joint_position_rad=((0.0, 1.0), (2.0, 3.0)),
            joint_velocity_rad_s=((0.0, 0.5), (1.0, 1.5)),
        )
        q, dq = plan.sample(0.5)
        self.assertEqual(q, (1.0, 2.0))
        self.assertEqual(dq, (0.5, 1.0))

    def test_gripper_release_and_catch_lead(self) -> None:
        schedule = REFERENCE.GripperSchedule(
            release_time_s=1.0,
            catch_time_s=1.5,
            close_lead_s=0.1,
            grasp_width_m=0.02,
            open_width_m=0.08,
            catch_width_m=0.03,
        )
        self.assertEqual(schedule.width_at(0.9), 0.02)
        self.assertEqual(schedule.width_at(1.0), 0.08)
        self.assertEqual(schedule.width_at(1.39), 0.08)
        self.assertEqual(schedule.width_at(1.4), 0.03)

    def test_target_rotation_changes_selected_skill(self) -> None:
        skills = (
            REFERENCE.PoseConditionedSkill("low", 0.0, 0.9, 0.2),
            REFERENCE.PoseConditionedSkill("quarter", 45.0, 0.9, 0.2),
        )
        upright, _ = REFERENCE.select_pose_conditioned_skill(0.0, skills)
        quarter, _ = REFERENCE.select_pose_conditioned_skill(45.0, skills)
        self.assertEqual(upright.skill_id, "low")
        self.assertEqual(quarter.skill_id, "quarter")

    def test_phase_timeline_contains_complete_pipeline(self) -> None:
        names = [phase.name for phase in REFERENCE.reference_phases()]
        self.assertEqual(
            names,
            [
                "table_pick_and_lift",
                "active_probe",
                "prethrow",
                "throw_and_release",
                "free_flight_and_catch",
                "stable_hold",
                "postcatch_multiview",
                "target_transport",
            ],
        )


if __name__ == "__main__":
    unittest.main()
