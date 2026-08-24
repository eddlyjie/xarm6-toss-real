#!/usr/bin/env python3
"""Show four-object onsite progress and the next useful command.

This is an offline convenience tool. It reads commissioning bundles and real
trial records; it does not import the xArm SDK or connect to the robot.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
TRIAL_SCHEMA = "xarm6_real_open_loop_trial_v1"
OBJECTS = {
    "O0": {
        "name": "cube38",
        "baseline_profile": "configs/open_loop_flip/cube38/low_5deg.json",
        "pose_profiles": (
            "configs/open_loop_flip/cube38/medium_6p5deg.json",
            "configs/open_loop_flip/cube38/high_8deg.json",
        ),
        "required_success_profiles": 3,
    },
    "O1": {
        "name": "cuboid30",
        "baseline_profile": "configs/open_loop_flip/cuboid30/low_3deg.json",
        "required_success_profiles": 2,
    },
    "O2": {
        "name": "cuboid33",
        "baseline_profile": "configs/open_loop_flip/cuboid33/low_5deg.json",
        "required_success_profiles": 2,
    },
    "O3": {
        "name": "cuboid38",
        "baseline_profile": "configs/open_loop_flip/cuboid38/low_4p5deg.json",
        "required_success_profiles": 2,
    },
}


def load_trials(trial_root: Path) -> list[dict[str, Any]]:
    if not trial_root.exists():
        return []
    trials = []
    for path in sorted(trial_root.rglob("*.trial.json")):
        payload = json.loads(path.read_text(encoding="utf-8"))
        if payload.get("schema") != TRIAL_SCHEMA:
            continue
        trials.append({**payload, "record_path": str(path)})
    return trials


def _latest_commissioning_bundle(root: Path, object_key: str) -> dict | None:
    spec = OBJECTS[object_key]
    paths = sorted(
        (root / "real_handoff" / spec["name"] / "low").glob(
            "*/commissioning_bundle.json"
        )
    )
    valid = []
    for path in paths:
        payload = json.loads(path.read_text(encoding="utf-8"))
        if payload.get("schema") != "xarm6_object_commissioning_bundle_v1":
            continue
        if payload.get("object_key") != object_key:
            continue
        profiles = payload.get("profiles", {})
        if not all(
            isinstance(profiles.get(stage), str)
            and (root / profiles[stage]).is_file()
            for stage in ("empty_g1", "throw_only", "object")
        ):
            continue
        valid.append((path, payload))
    if not valid:
        return None
    path, payload = valid[-1]
    return {"path": str(path), **payload}


def _pose_ladder_path(root: Path, object_key: str, label: str) -> Path:
    return (
        root
        / "real_handoff"
        / OBJECTS[object_key]["name"]
        / "pose_ladder"
        / label
        / "pose_ladder_bundle.json"
    )


def _trial_stats(rows: list[dict]) -> dict:
    successes = [row for row in rows if row.get("complete_demo_success") is True]
    profiles = sorted({str(row["profile"]) for row in successes})
    profile_trials: dict[str, int] = {}
    for row in rows:
        profile = str(row["profile"])
        profile_trials[profile] = profile_trials.get(profile, 0) + 1
    return {
        "trials": len(rows),
        "successes": len(successes),
        "successful_profiles": profiles,
        "successful_profile_count": len(profiles),
        "trials_by_profile": profile_trials,
    }


def _object_row(root: Path, object_key: str, trials: list[dict]) -> dict:
    spec = OBJECTS[object_key]
    rows = [row for row in trials if row.get("object_key") == object_key]
    stats = _trial_stats(rows)
    bundle = None if object_key == "O0" else _latest_commissioning_bundle(root, object_key)
    calibration_ready = object_key == "O0" or bundle is not None
    if not calibration_ready:
        state = "awaiting_g1_calibration"
    elif stats["successful_profile_count"] == 0:
        state = "ready_for_first_staged_demo"
    elif stats["successful_profile_count"] < spec["required_success_profiles"]:
        state = "first_demo_success_pose_extension_pending"
    else:
        state = "pose_coverage_reached_repeat_trials_pending"
    return {
        "object_key": object_key,
        "object_name": spec["name"],
        "baseline_profile": spec["baseline_profile"],
        "g1_calibration_ready": calibration_ready,
        "commissioning_bundle": None if bundle is None else bundle["path"],
        "commissioning_label": None if bundle is None else bundle["label"],
        "required_success_profiles": spec["required_success_profiles"],
        **stats,
        "state": state,
    }


def _first_demo_recommendation(row: dict) -> dict:
    key = row["object_key"]
    if not row["g1_calibration_ready"]:
        return {
            "objective": f"measure {key} G1 held/release/preclose/close positions",
            "command": (
                "python scripts/30_measure_g1_position.py "
                f"--object {key} --purpose held --position <CANDIDATE>"
            ),
        }
    if key == "O0":
        return {
            "objective": "restore the O0 low staged micro-toss baseline",
            "command": (
                "python scripts/24_run_cube_open_loop_demo.py "
                f"--profile {row['baseline_profile']}"
            ),
        }
    bundle = json.loads(Path(row["commissioning_bundle"]).read_text(encoding="utf-8"))
    return {
        "objective": f"run the {key} low commissioning ladder",
        "command": bundle["execution_order"][0]["command"],
        "commissioning_bundle": row["commissioning_bundle"],
        "note": "continue through the bundle execution_order only after each stage passes",
    }


def _next_pose_recommendation(root: Path, row: dict) -> dict:
    key = row["object_key"]
    if key == "O0":
        index = min(row["successful_profile_count"] - 1, 1)
        profile = OBJECTS[key]["pose_profiles"][index]
        return {
            "objective": f"add the next distinguishable O0 pose ({profile})",
            "command": (
                "python scripts/24_run_cube_open_loop_demo.py "
                f"--profile {profile}"
            ),
        }
    label = row["commissioning_label"]
    ladder_path = _pose_ladder_path(root, key, label)
    if not ladder_path.is_file():
        return {
            "objective": f"generate staged next/high profiles for {key}",
            "command": (
                "python scripts/32_prepare_pose_ladder.py "
                f"--commissioning-bundle {row['commissioning_bundle']} --write"
            ),
        }
    ladder = json.loads(ladder_path.read_text(encoding="utf-8"))
    next_pose = ladder["poses"]["next"]
    return {
        "objective": f"run the next distinguishable pose for {key}",
        "command": next_pose["execution_order"][0]["command"],
        "pose_ladder_bundle": str(ladder_path),
        "note": "continue through the pose execution_order only after each stage passes",
    }


def build_status(root: Path = ROOT, trial_root: Path | None = None) -> dict:
    trial_root = root / "real_results" if trial_root is None else trial_root
    trials = load_trials(trial_root)
    rows = [_object_row(root, key, trials) for key in OBJECTS]

    missing_first_demo = next(
        (row for row in rows if row["successful_profile_count"] == 0), None
    )
    if missing_first_demo is not None:
        recommended = _first_demo_recommendation(missing_first_demo)
    else:
        missing_pose = next(
            (
                row
                for row in rows
                if row["successful_profile_count"] < row["required_success_profiles"]
            ),
            None,
        )
        if missing_pose is not None:
            recommended = _next_pose_recommendation(root, missing_pose)
        else:
            under_repeated = next(
                (
                    (row, profile, count)
                    for row in rows
                    for profile, count in row["trials_by_profile"].items()
                    if profile in row["successful_profiles"] and count < 5
                ),
                None,
            )
            if under_repeated is None:
                recommended = {
                    "objective": "four-object pose coverage and five-trial coverage reached",
                    "command": (
                        "python scripts/31_record_real_trials.py summarize "
                        f"--input-root {trial_root}"
                    ),
                }
            else:
                row, profile, count = under_repeated
                recommended = {
                    "objective": (
                        f"repeat {row['object_key']} profile {profile} "
                        f"({count}/5 recorded trials)"
                    ),
                    "command": "repeat the same staged object profile and record the trial",
                }

    return {
        "schema": "xarm6_four_object_onsite_progress_v1",
        "offline_only": True,
        "robot_connection_attempted": False,
        "trial_root": str(trial_root),
        "four_object_first_demo_coverage": sum(
            row["successful_profile_count"] > 0 for row in rows
        ),
        "objects": rows,
        "recommended_next": recommended,
    }


def render(status: dict) -> str:
    lines = [
        "xArm6 four-object onsite progress (offline)",
        "Object  G1    trials  success  poses  state",
    ]
    for row in status["objects"]:
        lines.append(
            f"{row['object_key']:<6}  "
            f"{'READY' if row['g1_calibration_ready'] else 'WAIT':<5} "
            f"{row['trials']:>6}  {row['successes']:>7}  "
            f"{row['successful_profile_count']:>5}/"
            f"{row['required_success_profiles']:<1}  {row['state']}"
        )
    next_step = status["recommended_next"]
    lines.extend(
        [
            "",
            f"NEXT: {next_step['objective']}",
            f"  {next_step['command']}",
            "No robot connection or command was attempted.",
        ]
    )
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trial-root", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    status = build_status(trial_root=args.trial_root)
    print(render(status), end="")
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(status, indent=2) + "\n", encoding="utf-8")
        print(f"saved onsite progress: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
