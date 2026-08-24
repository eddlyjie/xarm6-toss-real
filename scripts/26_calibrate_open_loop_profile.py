#!/usr/bin/env python3
"""Create a staged real-G1 profile from an uncalibrated Sim handoff."""

from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from xarm6_toss.open_loop_demo import prepare_deployment  # noqa: E402


STAGE_MODES = {
    "empty_g1": ["empty_arm", "empty_g1"],
    "throw_only": ["empty_arm", "empty_g1", "throw_only"],
    "object": ["empty_arm", "empty_g1", "throw_only", "object"],
}


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def g1_position(value: int, name: str) -> int:
    position = int(value)
    if position < 0 or position > 850:
        raise ValueError(f"{name} must be in the real G1 range [0, 850]")
    return position


def calibrate_profile(
    template: dict,
    *,
    held_position: int,
    release_position: int,
    preclose_position: int,
    close_position: int,
    stage: str,
    schedule_path: str,
) -> tuple[dict, dict]:
    if template.get("schema") != "xarm6_open_loop_flip_profile_v1":
        raise ValueError("unsupported profile schema")
    if stage not in STAGE_MODES:
        raise ValueError(f"unsupported commissioning stage: {stage}")
    positions = {
        "held": g1_position(held_position, "held_position"),
        "release": g1_position(release_position, "release_position"),
        "preclose": g1_position(preclose_position, "preclose_position"),
        "close": g1_position(close_position, "close_position"),
    }
    if positions["release"] <= positions["held"]:
        raise ValueError("release_position must open farther than held_position")
    if not positions["held"] <= positions["preclose"] <= positions["release"]:
        raise ValueError("preclose_position must lie between held and release")
    events = template["g1"]["events"]
    if [event["name"] for event in events] != ["release", "preclose", "close"]:
        raise ValueError("profile must contain release/preclose/close events")

    profile = copy.deepcopy(template)
    profile["profile_id"] = f"{template['profile_id']}_{stage}_calibrated"
    profile["g1_schedule"] = schedule_path
    profile["status"] = f"onsite_g1_calibrated_{stage}_real_unverified"
    profile["hardware_modes_allowed"] = STAGE_MODES[stage]
    profile["g1"]["held_position"] = positions["held"]
    for event in profile["g1"]["events"]:
        event["position"] = positions[event["name"]]
    profile.setdefault("evidence", {})["real_g1_calibration"] = {
        "source": "onsite_manual_measurement",
        "stage": stage,
        "held_position": positions["held"],
        "release_position": positions["release"],
        "preclose_position": positions["preclose"],
        "close_position": positions["close"],
        "real_object_trial_verified": False,
    }
    schedule = {
        "schema": "xarm6_g1_schedule_v1",
        "calibration_required": False,
        "object_id": template["evidence"].get("object_id", template.get("object_profile")),
        "held_position": positions["held"],
        "speed": int(template["g1"]["speed"]),
        "events": copy.deepcopy(profile["g1"]["events"]),
        "commissioning_stage": stage,
    }
    return profile, schedule


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--template-profile", type=Path, required=True)
    parser.add_argument("--output-profile", type=Path, required=True)
    parser.add_argument("--output-schedule", type=Path, required=True)
    parser.add_argument("--held-position", type=int, required=True)
    parser.add_argument("--release-position", type=int, required=True)
    parser.add_argument("--preclose-position", type=int, required=True)
    parser.add_argument("--close-position", type=int, required=True)
    parser.add_argument("--stage", choices=tuple(STAGE_MODES), default="empty_g1")
    args = parser.parse_args()
    for output in (args.output_profile, args.output_schedule):
        if output.exists():
            raise FileExistsError(f"refusing to overwrite existing handoff: {output}")
    schedule_relative = args.output_schedule.resolve().relative_to(ROOT.resolve())
    profile, schedule = calibrate_profile(
        load_json(args.template_profile),
        held_position=args.held_position,
        release_position=args.release_position,
        preclose_position=args.preclose_position,
        close_position=args.close_position,
        stage=args.stage,
        schedule_path=str(schedule_relative),
    )
    args.output_profile.parent.mkdir(parents=True, exist_ok=True)
    args.output_schedule.parent.mkdir(parents=True, exist_ok=True)
    args.output_schedule.write_text(json.dumps(schedule, indent=2) + "\n", encoding="utf-8")
    args.output_profile.write_text(json.dumps(profile, indent=2) + "\n", encoding="utf-8")
    plan, _ = prepare_deployment(
        ROOT, profile_path=args.output_profile.resolve(), mode=args.stage
    )
    print(json.dumps(plan, indent=2))
    print("profile and schedule created; no robot connection or command was sent")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
