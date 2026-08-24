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


def runner_args(tmp_path: Path, summary_path: Path, **overrides):
    values = {
        "runner_summary": summary_path,
        "trial_id": "o2_low_01",
        "measured_angle_deg": 5.1,
        "angle_summary": None,
        "rotation_axis": "forward_tumble",
        "detached": "yes",
        "caught": "yes",
        "hold_s": 0.7,
        "video": "videos/o2_low_01.mp4",
        "notes": "",
        "output": tmp_path / "o2_low_01.trial.json",
        "write": False,
    }
    values.update(overrides)
    return argparse.Namespace(**values)


def write_runner_summary(path: Path, *, error=None, mode="object"):
    payload = {
        "plan": {
            "schema": "xarm6_open_loop_demo_plan_v1",
            "mode": mode,
            "profile_id": "cuboid33_low_real_day1_object",
            "profile": "configs/open_loop_flip/real_calibrated/cuboid33/day1/object.json",
            "desired_angle_deg": 5.0,
            "object_id": "cuboid_50p5x51x33p5mm_26p6g",
            "g1_held_position": 355,
            "g1_events": [
                {"name": "release", "time_s": 0.62, "position": 545},
                {"name": "preclose", "time_s": 0.80, "position": 430},
                {"name": "close", "time_s": 0.86, "position": 355},
            ],
        },
        "execution": {
            "error": error,
            "g1_position_samples": [
                {"label": "before_servo", "time_s": 0.0, "position": 355.0, "error": None},
                {"label": "after_servo", "time_s": 2.32, "position": 354.0, "error": None},
            ],
        },
    }
    path.write_text(json.dumps(payload), encoding="utf-8")


def test_runner_summary_auto_fills_profile_object_angle_and_g1(tmp_path):
    summary_path = tmp_path / "summary.json"
    write_runner_summary(summary_path)
    trial = trials.build_trial_from_runner(runner_args(tmp_path, summary_path))

    assert trial["object_key"] == "O2"
    assert trial["profile"].endswith("cuboid33/day1/object.json")
    assert trial["desired_angle_deg"] == 5.0
    assert trial["g1_positions"] == {
        "held": 355,
        "release": 545,
        "preclose": 430,
        "close": 355,
    }
    assert trial["runner_fields_auto_filled"] is True
    assert [row["position"] for row in trial["runner_g1_position_samples"]] == [
        355.0,
        354.0,
    ]
    assert trial["complete_demo_success"] is True


def test_runner_execution_error_prevents_success_label(tmp_path):
    summary_path = tmp_path / "summary.json"
    write_runner_summary(
        summary_path,
        error={"type": "RuntimeError", "message": "servo stopped"},
    )
    trial = trials.build_trial_from_runner(runner_args(tmp_path, summary_path))
    assert trial["complete_demo_success"] is False
    assert trial["runner_execution_error"]["type"] == "RuntimeError"


def test_angle_summary_auto_fills_measured_rotation(tmp_path):
    summary_path = tmp_path / "runner" / "summary.json"
    summary_path.parent.mkdir()
    write_runner_summary(summary_path)
    angle_path = tmp_path / "angle" / "summary.json"
    angle_path.parent.mkdir()
    angle_path.write_text(
        json.dumps(
            {
                "schema": "xarm6_offline_marker_rotation_v1",
                "measurement_scope": "2d_image_plane_rotation",
                "detected_frame_count": 18,
                "first_time_s": 1.20,
                "last_time_s": 1.35,
                "signed_rotation_first_to_last_deg": 5.42,
                "peak_absolute_rotation_deg": 5.57,
                "annotated_video": "angle/annotated.mp4",
            }
        ),
        encoding="utf-8",
    )
    trial = trials.build_trial_from_runner(
        runner_args(
            tmp_path,
            summary_path,
            measured_angle_deg=None,
            angle_summary=angle_path,
        )
    )
    assert trial["measured_angle_deg"] == 5.42
    assert trial["angle_measurement"]["detected_frame_count"] == 18
    assert trial["angle_measurement"]["peak_absolute_rotation_deg"] == 5.57


def test_record_from_runner_requires_full_recatch_mode(tmp_path):
    summary_path = tmp_path / "summary.json"
    write_runner_summary(summary_path, mode="throw_only")
    with pytest.raises(ValueError, match="recatch run"):
        trials.build_trial_from_runner(runner_args(tmp_path, summary_path))
