#!/usr/bin/env python3
"""Validate all four real-robot handoffs without importing a robot SDK."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from xarm6_toss.open_loop_demo import prepare_deployment  # noqa: E402


DEFAULT_MANIFEST = ROOT / "configs/real_commissioning/four_object_demo.json"


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def calibration_state(profile: dict) -> dict:
    events = profile["g1"]["events"]
    held = profile["g1"].get("held_position")
    missing = [event["name"] for event in events if event.get("position") is None]
    return {
        "complete": held is not None and not missing,
        "held_position": held,
        "missing_event_positions": missing,
    }


def check_profile(root: Path, profile_relative: str) -> dict:
    profile_path = root / profile_relative
    profile = load_json(profile_path)
    plan, samples = prepare_deployment(root, profile_path=profile_path, mode="plan")
    active = profile.get("active_throw_joints_one_based")
    fixed = profile.get("fixed_throw_joints_one_based")
    if active != [2, 3, 5] or fixed != [1, 4, 6]:
        raise ValueError(f"{profile_relative} does not preserve the J2/J3/J5 contract")
    calibration = calibration_state(profile)
    return {
        "profile": profile_relative,
        "profile_id": plan["profile_id"],
        "object_id": plan["object_id"],
        "desired_angle_deg": plan["desired_angle_deg"],
        "sample_count": len(samples),
        "duration_s": plan["duration_s"],
        "joint_envelope_pass": bool(
            plan["joint_limits"]["joint_mechanical_limits_pass"]
        ),
        "hardware_modes_allowed": profile["hardware_modes_allowed"],
        "g1_calibration": calibration,
        "plan_only_verified": True,
    }


def build_report(root: Path, manifest_path: Path = DEFAULT_MANIFEST) -> dict:
    manifest = load_json(manifest_path)
    if manifest.get("schema") != "xarm6_four_object_commissioning_v1":
        raise ValueError("unsupported commissioning manifest")
    rows = []
    for item in manifest["objects"]:
        baseline = check_profile(root, item["baseline_profile"])
        next_pose = check_profile(root, item["next_pose_profile"])
        if baseline["object_id"] != item["object_id"]:
            raise ValueError(f"{item['key']} baseline object does not match manifest")
        if next_pose["object_id"] != item["object_id"]:
            raise ValueError(f"{item['key']} next pose object does not match manifest")
        rows.append(
            {
                "key": item["key"],
                "object_id": item["object_id"],
                "real_object_mode": item["real_object_mode"],
                "baseline": baseline,
                "next_pose": next_pose,
                "onsite_next_action": (
                    "run staged O0 baseline"
                    if baseline["g1_calibration"]["complete"]
                    else "measure held/release/preclose/close G1 positions"
                ),
            }
        )
    return {
        "schema": "xarm6_four_object_handoff_check_v1",
        "robot_connection_attempted": False,
        "dynamic_joints_one_based": [2, 3, 5],
        "fixed_joints_one_based": [1, 4, 6],
        "objects": rows,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = build_report(ROOT, args.manifest.resolve())
    rendered = json.dumps(report, indent=2) + "\n"
    print(rendered, end="")
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
        print(f"saved offline handoff check: {args.output}")
    print("offline check complete; no robot connection or command was sent")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
