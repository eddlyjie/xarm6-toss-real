from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
spec = importlib.util.spec_from_file_location(
    "g1_calibration", ROOT / "scripts/26_calibrate_open_loop_profile.py"
)
calibration = importlib.util.module_from_spec(spec)
spec.loader.exec_module(calibration)

from xarm6_toss.open_loop_demo import prepare_deployment  # noqa: E402


def template_profile():
    return json.loads(
        (ROOT / "configs/open_loop_flip/cuboid30/pose_conditioned_5p5deg.json").read_text()
    )


def test_calibration_unlocks_only_requested_stage(tmp_path):
    profile, schedule = calibration.calibrate_profile(
        template_profile(),
        held_position=360,
        release_position=540,
        preclose_position=430,
        close_position=360,
        stage="empty_g1",
        schedule_path="real_handoff/test/g1_schedule.json",
    )
    assert profile["hardware_modes_allowed"] == ["empty_arm", "empty_g1"]
    assert profile["g1"]["held_position"] == 360
    assert [event["position"] for event in profile["g1"]["events"]] == [540, 430, 360]
    assert schedule["held_position"] == 360
    path = tmp_path / "profile.json"
    path.write_text(json.dumps(profile))
    plan, _ = prepare_deployment(ROOT, profile_path=path, mode="empty_g1")
    assert plan["g1_held_position"] == 360
    assert [event["position"] for event in plan["g1_events"]] == [540, 430, 360]
    assert plan["profile"] == str(path.resolve())


def test_object_stage_is_explicit_and_cumulative():
    profile, _ = calibration.calibrate_profile(
        template_profile(),
        held_position=370,
        release_position=520,
        preclose_position=441,
        close_position=370,
        stage="object",
        schedule_path="real_handoff/test/g1_schedule.json",
    )
    assert profile["hardware_modes_allowed"] == [
        "empty_arm", "empty_g1", "throw_only", "object"
    ]


@pytest.mark.parametrize(
    "values, message",
    [
        ((360, 350, 355, 360), "open farther"),
        ((360, 540, 550, 360), "between held and release"),
        ((-1, 540, 430, 360), "range"),
    ],
)
def test_invalid_measured_positions_are_rejected(values, message):
    with pytest.raises(ValueError, match=message):
        calibration.calibrate_profile(
            template_profile(),
            held_position=values[0],
            release_position=values[1],
            preclose_position=values[2],
            close_position=values[3],
            stage="empty_g1",
            schedule_path="real_handoff/test/g1_schedule.json",
        )
