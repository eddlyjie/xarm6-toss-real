from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "record_real_trials", ROOT / "scripts/31_record_real_trials.py"
)
trials = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(trials)


def args(**overrides):
    values = {
        "object": "O1",
        "trial_id": "o1_low_01",
        "profile": "configs/open_loop_flip/real_calibrated/cuboid30/day1/object.json",
        "desired_angle_deg": 3.0,
        "measured_angle_deg": 3.4,
        "rotation_axis": "forward_tumble",
        "held_position": 360,
        "release_position": 540,
        "preclose_position": 430,
        "close_position": 360,
        "detached": "yes",
        "caught": "yes",
        "hold_s": 0.8,
        "video": "videos/o1_low_01.mp4",
        "runner_summary": "outputs/o1_low_01/summary.json",
        "notes": "",
    }
    values.update(overrides)
    return argparse.Namespace(**values)


def test_success_requires_detach_catch_and_half_second_hold():
    successful = trials.build_trial(args())
    short_hold = trials.build_trial(args(trial_id="short", hold_s=0.49))
    missed = trials.build_trial(args(trial_id="miss", caught="no"))

    assert successful["complete_demo_success"] is True
    assert short_hold["complete_demo_success"] is False
    assert missed["complete_demo_success"] is False
    assert successful["robot_connection_attempted_by_this_tool"] is False


def test_trial_file_is_independent_and_never_overwritten(tmp_path):
    trial = trials.build_trial(args())
    output = tmp_path / "o1_low_01.trial.json"
    trials.write_trial(trial, output)
    assert json.loads(output.read_text(encoding="utf-8"))["trial_id"] == "o1_low_01"
    with pytest.raises(FileExistsError, match="refusing to overwrite"):
        trials.write_trial(trial, output)


def test_summary_groups_profile_trials_and_computes_rate(tmp_path):
    rows = [
        trials.build_trial(args(trial_id="run1", measured_angle_deg=3.0)),
        trials.build_trial(args(trial_id="run2", measured_angle_deg=4.0)),
        trials.build_trial(args(trial_id="run3", measured_angle_deg=5.0, caught="no")),
    ]
    for row in rows:
        trials.write_trial(row, tmp_path / f"{row['trial_id']}.trial.json")

    report = trials.summarize(trials.load_trials(tmp_path))
    group = report["groups"][0]
    assert report["trial_count"] == 3
    assert group["trials"] == 3
    assert group["successes"] == 2
    assert group["catch_rate"] == pytest.approx(2 / 3)
    assert group["measured_angle_mean_deg"] == pytest.approx(4.0)
    assert group["measured_angle_std_deg"] == pytest.approx((2 / 3) ** 0.5)


@pytest.mark.parametrize(
    "overrides, message",
    [
        ({"release_position": 350}, "open farther"),
        ({"preclose_position": 600}, "between held and release"),
        ({"held_position": -1}, "0..850"),
        ({"hold_s": -0.1}, "non-negative"),
        ({"trial_id": "../bad"}, "simple label"),
    ],
)
def test_invalid_manual_record_is_rejected(overrides, message):
    with pytest.raises(ValueError, match=message):
        trials.build_trial(args(**overrides))
