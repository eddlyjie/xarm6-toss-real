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
OBJECT_ID_TO_KEY = {
    "yellow_cube_38mm_8g": "O0",
    "cuboid_44p5x46x30mm_20g": "O1",
    "cuboid_50p5x51x33p5mm_26p6g": "O2",
    "cuboid_57p5x58x38mm_37g": "O3",
}


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


def _g1_positions_from_plan(plan: dict) -> dict[str, int]:
    positions = {"held": plan.get("g1_held_position")}
    for event in plan.get("g1_events", []):
        name = event.get("name")
        if name in {"release", "preclose", "close"}:
            positions[name] = event.get("position")
    missing = [
        name
        for name in ("held", "release", "preclose", "close")
        if positions.get(name) is None
    ]
    if missing:
        raise ValueError(f"runner summary is missing G1 positions: {missing}")
    result = {}
    for name, value in positions.items():
        numeric = float(value)
        if not numeric.is_integer():
            raise ValueError(f"runner G1 {name} position must be an integer")
        result[name] = int(numeric)
    return result


def build_trial_from_runner(args: argparse.Namespace) -> dict:
    summary_path = Path(args.runner_summary)
    if not summary_path.is_file():
        raise FileNotFoundError(f"runner summary does not exist: {summary_path}")
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    plan = summary.get("plan")
    if not isinstance(plan, dict) or plan.get("schema") != "xarm6_open_loop_demo_plan_v1":
        raise ValueError("runner summary does not contain an open-loop demo plan")
    object_id = plan.get("object_id")
    if object_id not in OBJECT_ID_TO_KEY:
        raise ValueError(f"runner summary has unknown object_id: {object_id}")
    if plan.get("mode") not in {"cube", "object"}:
        raise ValueError("record-from-runner requires a cube/object recatch run")
    positions = _g1_positions_from_plan(plan)
    derived = argparse.Namespace(
        object=OBJECT_ID_TO_KEY[object_id],
        trial_id=args.trial_id,
        profile=plan["profile"],
        desired_angle_deg=float(plan["desired_angle_deg"]),
        measured_angle_deg=args.measured_angle_deg,
        rotation_axis=args.rotation_axis,
        held_position=positions["held"],
        release_position=positions["release"],
        preclose_position=positions["preclose"],
        close_position=positions["close"],
        detached=args.detached,
        caught=args.caught,
        hold_s=args.hold_s,
        video=args.video,
        runner_summary=str(summary_path),
        notes=args.notes,
    )
    trial = build_trial(derived)
    execution_error = summary.get("execution", {}).get("error")
    trial["runner_fields_auto_filled"] = True
    trial["runner_mode"] = plan["mode"]
    trial["runner_execution_error"] = execution_error
    if execution_error is not None:
        trial["complete_demo_success"] = False
    return trial


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


def add_runner_record_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--runner-summary", type=Path, required=True)
    parser.add_argument("--trial-id", required=True)
    parser.add_argument("--measured-angle-deg", type=float, required=True)
    parser.add_argument(
        "--rotation-axis",
        choices=("forward_tumble", "backward_tumble"),
        required=True,
    )
    parser.add_argument("--detached", choices=("yes", "no"), required=True)
    parser.add_argument("--caught", choices=("yes", "no"), required=True)
    parser.add_argument("--hold-s", type=float, required=True)
    parser.add_argument("--video", required=True)
    parser.add_argument("--notes", default="")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--write", action="store_true")


def main() -> int:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)
    record = subparsers.add_parser("record")
    add_record_arguments(record)
    runner_record = subparsers.add_parser("record-from-runner")
    add_runner_record_arguments(runner_record)
    summary = subparsers.add_parser("summarize")
    summary.add_argument("--input-root", type=Path, required=True)
    summary.add_argument("--output", type=Path)
    args = parser.parse_args()

    if args.command in {"record", "record-from-runner"}:
        trial = (
            build_trial(args)
            if args.command == "record"
            else build_trial_from_runner(args)
        )
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
