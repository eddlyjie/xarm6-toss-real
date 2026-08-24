#!/usr/bin/env python3
"""Export a successful Sim controller command trace as a 20 ms real reference."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from xarm6_toss.motion_limits import evaluate_joint_trajectory  # noqa: E402


def load_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def export_reference(
    trajectory_path: Path,
    summary_path: Path,
    *,
    profile_id: str,
) -> dict:
    summary = load_json(summary_path)
    if not summary.get("catch_stable", False):
        raise ValueError("only a stable-catch Sim run can become an open-loop reference")
    if not summary.get("catch_j235_only", False):
        raise ValueError("open-loop export requires the J2/J3/J5-only controller")
    rows = load_json(trajectory_path)
    ticks = [row for row in rows if row.get("arm_control_tick")]
    if len(ticks) < 2:
        raise ValueError("trajectory does not contain recorded 20 ms command ticks")
    start = float(ticks[0]["time_s"])
    samples = []
    for row in ticks:
        samples.append(
            {
                "time_s": float(row["time_s"]) - start,
                "phase": row["phase"],
                "joint_position_rad": row["arm_command_position_rad"],
                "joint_velocity_rad_s": row["arm_command_velocity_rad_s"],
                "joint_acceleration_rad_s2": row[
                    "arm_command_acceleration_rad_s2"
                ],
                "sim_gripper_drive_target_rad": row[
                    "gripper_command_drive_rad"
                ],
            }
        )
    periods = [
        samples[index]["time_s"] - samples[index - 1]["time_s"]
        for index in range(1, len(samples))
    ]
    if max(abs(period - 0.02) for period in periods) > 1.1e-3:
        raise ValueError("Sim command ticks are not a 20 ms reference")
    fixed_joint_indices = (0, 3, 5)
    fixed_positions = samples[0]["joint_position_rad"]
    for sample in samples:
        for joint_index in fixed_joint_indices:
            if abs(
                sample["joint_position_rad"][joint_index]
                - fixed_positions[joint_index]
            ) > 1.0e-6:
                raise ValueError("J1/J4/J6 must remain fixed in exported reference")
            if abs(sample["joint_velocity_rad_s"][joint_index]) > 1.0e-6:
                raise ValueError("J1/J4/J6 velocities must remain zero")
    limits = evaluate_joint_trajectory(
        [sample["joint_position_rad"] for sample in samples],
        [sample["joint_velocity_rad_s"] for sample in samples],
        [sample["joint_acceleration_rad_s2"] for sample in samples],
    )
    if not limits["joint_mechanical_limits_pass"]:
        raise ValueError("successful Sim command trace exceeds the real handoff envelope")
    return {
        "schema": "xarm6_open_loop_exported_timeline_v1",
        "profile_id": profile_id,
        "source_summary": str(summary_path),
        "source_trajectory": str(trajectory_path),
        "source_sim_measured_rotation_deg": summary.get(
            "free_flight_signed_tumble_rotation_deg"
        ),
        "source_sim_free_flight_s": summary.get(
            "continuous_free_flight_duration_s"
        ),
        "source_sim_catch_stable": True,
        "control_period_s": 0.02,
        "dynamic_joint_indices_zero_based": [1, 2, 4],
        "fixed_joint_indices_zero_based": [0, 3, 5],
        "reference_limit_evidence": limits,
        "samples": samples,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trajectory", type=Path, required=True)
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--profile-id", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    payload = export_reference(
        args.trajectory, args.summary, profile_id=args.profile_id
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(f"saved open-loop timeline: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
