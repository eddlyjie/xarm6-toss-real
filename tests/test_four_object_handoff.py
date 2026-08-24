from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "four_object_handoff", ROOT / "scripts/27_check_four_object_handoff.py"
)
handoff = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(handoff)


def test_all_four_objects_have_offline_valid_baseline_and_next_pose():
    report = handoff.build_report(ROOT)
    assert report["robot_connection_attempted"] is False
    assert report["dynamic_joints_one_based"] == [2, 3, 5]
    assert report["fixed_joints_one_based"] == [1, 4, 6]
    assert [row["key"] for row in report["objects"]] == ["O0", "O1", "O2", "O3"]
    for row in report["objects"]:
        for profile in (row["baseline"], row["next_pose"]):
            assert profile["plan_only_verified"] is True
            assert profile["joint_envelope_pass"] is True
            assert profile["sample_count"] == 92


def test_only_existing_cube_is_currently_g1_calibrated():
    report = handoff.build_report(ROOT)
    states = {
        row["key"]: row["baseline"]["g1_calibration"]["complete"]
        for row in report["objects"]
    }
    assert states == {"O0": True, "O1": False, "O2": False, "O3": False}
    assert report["objects"][0]["onsite_next_action"] == "run staged O0 baseline"
    for row in report["objects"][1:]:
        assert row["onsite_next_action"].startswith("measure held")
