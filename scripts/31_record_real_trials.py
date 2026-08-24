#!/usr/bin/env python3
"""Record and summarize real open-loop toss/catch trials offline."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import re
from statistics import mean, pstdev


TRIAL_SCHEMA = "xarm6_real_open_loop_trial_v1"
OBJECTS = ("O0", "O1", "O2", "O3")


def _label(value: str, name: str) -> str:
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", value):
        raise ValueError(f"{name} must be a simple label")
    return value


def _yes(value: str) -> bool:
    return value == "yes"


def build_trial(args: argparse.Namespace) -> dict:
    positions = {
        "held": int(args.held_position),
        "release": int(args.release_position),
        "preclose": int(args.preclose_position),
        "close": int(args.close_position),
    }
    if any(not 0 <= value <= 850 for value in positions.values()):
        raise ValueError("all G1 positions must be within 0..850")
    if positions["release"] <= positions["held"]:
        raise ValueError("release position must open farther than held")
    if not positions["held"] <= positions["preclose"] <= positions["release"]:
        raise ValueError("preclose must lie between held and release")
    if args.hold_s < 0.0:
        raise ValueError("hold_s must be non-negative")

    detached = _yes(args.detached)
    caught = _yes(args.caught)
    success = detached and caught and args.hold_s >= 0.5
    return {
        "schema": TRIAL_SCHEMA,
        "recorded_utc": datetime.now(timezone.utc).isoformat(),
        "trial_id": _label(args.trial_id, "trial_id"),
        "object_key": args.object,
        "profile": args.profile,
        "desired_angle_deg": float(args.desired_angle_deg),
        "measured_angle_deg": float(args.measured_angle_deg),
        "rotation_axis": args.rotation_axis,
        "g1_positions": positions,
        "fully_detached": detached,
        "caught": caught,
        "hold_s": float(args.hold_s),
        "complete_demo_success": success,
        "video": args.video,
        "runner_summary": args.runner_summary,
        "notes": args.notes,
        "manual_label": True,
        "robot_connection_attempted_by_this_tool": False,
    }


def write_trial(trial: dict, output: Path) -> None:
    if output.exists():
        raise FileExistsError(f"refusing to overwrite trial: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(trial, indent=2) + "\n", encoding="utf-8")


def load_trials(input_root: Path) -> list[dict]:
    rows = []
    for path in sorted(input_root.rglob("*.trial.json")):
        row = json.loads(path.read_text(encoding="utf-8"))
        if row.get("schema") != TRIAL_SCHEMA:
            continue
        rows.append({**row, "record_path": str(path)})
    return rows


def summarize(trials: list[dict]) -> dict:
    groups: dict[tuple[str, str], list[dict]] = {}
    for trial in trials:
        groups.setdefault((trial["object_key"], trial["profile"]), []).append(trial)
    results = []
    for (object_key, profile), rows in sorted(groups.items()):
        angles = [float(row["measured_angle_deg"]) for row in rows]
        successes = sum(bool(row["complete_demo_success"]) for row in rows)
        results.append(
            {
                "object_key": object_key,
                "profile": profile,
                "trials": len(rows),
                "successes": successes,
                "catch_rate": successes / len(rows),
                "measured_angle_mean_deg": mean(angles),
                "measured_angle_std_deg": pstdev(angles),
                "trial_ids": [row["trial_id"] for row in rows],
            }
        )
    return {
        "schema": "xarm6_real_open_loop_trial_summary_v1",
        "trial_count": len(trials),
        "groups": results,
    }


def add_record_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--object", choices=OBJECTS, required=True)
    parser.add_argument("--trial-id", required=True)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--desired-angle-deg", type=float, required=True)
    parser.add_argument("--measured-angle-deg", type=float, required=True)
    parser.add_argument(
        "--rotation-axis",
        choices=("forward_tumble", "backward_tumble"),
        required=True,
    )
    parser.add_argument("--held-position", type=int, required=True)
    parser.add_argument("--release-position", type=int, required=True)
    parser.add_argument("--preclose-position", type=int, required=True)
    parser.add_argument("--close-position", type=int, required=True)
    parser.add_argument("--detached", choices=("yes", "no"), required=True)
    parser.add_argument("--caught", choices=("yes", "no"), required=True)
    parser.add_argument("--hold-s", type=float, required=True)
    parser.add_argument("--video", required=True)
    parser.add_argument("--runner-summary")
    parser.add_argument("--notes", default="")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--write", action="store_true")


def main() -> int:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)
    record = subparsers.add_parser("record")
    add_record_arguments(record)
    summary = subparsers.add_parser("summarize")
    summary.add_argument("--input-root", type=Path, required=True)
    summary.add_argument("--output", type=Path)
    args = parser.parse_args()

    if args.command == "record":
        trial = build_trial(args)
        print(json.dumps(trial, indent=2))
        if not args.write:
            print("preview only; no trial file was written")
            return 0
        write_trial(trial, args.output)
        print(f"saved real trial: {args.output}")
        return 0

    report = summarize(load_trials(args.input_root))
    rendered = json.dumps(report, indent=2) + "\n"
    print(rendered, end="")
    if args.output is not None:
        if args.output.exists():
            raise FileExistsError(f"refusing to overwrite summary: {args.output}")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
        print(f"saved trial summary: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
