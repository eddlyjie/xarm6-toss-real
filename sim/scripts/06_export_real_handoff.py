#!/usr/bin/env python3
"""Export the real-safe reference and three G1 timing candidates."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from xarm6_toss.control_reference import (  # noqa: E402
    QuinticJointSegment,
    generate_joint_reference,
)


DEFAULT_CONFIG = ROOT / "sim" / "configs" / "ballistic_throw_real_v1.json"
DEFAULT_OUTPUT = ROOT / "real_handoff"


def load_reference(path: Path):
    payload = json.loads(path.read_text(encoding="utf-8"))
    segments = tuple(
        QuinticJointSegment(
            phase=item["phase"],
            duration_s=float(item["duration_s"]),
            start_joint_rad=tuple(item["start_joint_rad"]),
            start_joint_velocity_rad_s=tuple(
                item["start_joint_velocity_rad_s"]
            ),
            end_joint_rad=tuple(item["end_joint_rad"]),
            end_joint_velocity_rad_s=tuple(
                item["end_joint_velocity_rad_s"]
            ),
        )
        for item in payload["reference_segments"]
    )
    samples = generate_joint_reference(
        segments, float(payload["control_period_s"])
    )
    return payload, samples


def sample_payload(sample, g1_target: float | None) -> dict:
    return {
        "time_s": sample.time_s,
        "phase": sample.phase,
        "joint_position_rad": list(sample.joint_position_rad),
        "joint_velocity_rad_s": list(sample.joint_velocity_rad_s),
        "joint_acceleration_rad_s2": list(sample.joint_acceleration_rad_s2),
        "g1_target_position": g1_target,
    }


def write_csv(path: Path, samples: list[dict]) -> None:
    vector_fields = (
        "joint_position_rad",
        "joint_velocity_rad_s",
        "joint_acceleration_rad_s2",
    )
    columns = ["time_s", "phase", "g1_target_position"]
    for field in vector_fields:
        columns.extend(f"{field}_{index}" for index in range(1, 7))
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        for sample in samples:
            row = {
                "time_s": sample["time_s"],
                "phase": sample["phase"],
                "g1_target_position": sample["g1_target_position"],
            }
            for field in vector_fields:
                row.update(
                    {
                        f"{field}_{index}": value
                        for index, value in enumerate(sample[field], 1)
                    }
                )
            writer.writerow(row)


def export_candidate(output: Path, name: str, release_time_s: float, samples):
    close_time_s = 0.58
    events = {
        round(release_time_s, 6): 520.0,
        round(close_time_s, 6): 370.0,
    }
    rows = [
        sample_payload(sample, events.get(round(sample.time_s, 6)))
        for sample in samples
    ]
    payload = {
        "schema": "xarm6_real_timeline_v1",
        "name": name,
        "control_period_s": 0.02,
        "joint_units": "radian",
        "g1_units": "UFACTORY position",
        "events": [
            {
                "time_s": release_time_s,
                "name": "release_partial_open",
                "g1_position": 520.0,
            },
            {
                "time_s": close_time_s,
                "name": "catch_close",
                "g1_position": 370.0,
            },
        ],
        "predicted_real_detach_time_s": release_time_s + 0.035,
        "samples": rows,
    }
    json_path = output / "timelines" / f"{name}.json"
    csv_path = output / "timelines" / f"{name}.csv"
    json_path.write_text(json.dumps(payload, indent=2) + "\n")
    write_csv(csv_path, rows)
    return {
        "name": name,
        "release_time_s": release_time_s,
        "predicted_real_detach_time_s": release_time_s + 0.035,
        "close_time_s": close_time_s,
        "json": str(json_path.relative_to(output)),
        "csv": str(csv_path.relative_to(output)),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    config, reference = load_reference(args.config)
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "timelines").mkdir(exist_ok=True)

    candidates = [
        export_candidate(args.output, "release_early", 0.48, reference),
        export_candidate(args.output, "nominal", 0.50, reference),
        export_candidate(args.output, "release_late", 0.52, reference),
    ]
    qd = np.asarray([sample.joint_velocity_rad_s for sample in reference])
    qdd = np.asarray([sample.joint_acceleration_rad_s2 for sample in reference])
    manifest = {
        "schema": "xarm6_real_handoff_v1",
        "task": "fixed grasp, low-energy self toss, camera-updated self catch",
        "versions": {
            "isaac_sim": "6.0.1",
            "isaac_lab": "6.1.16",
            "python": "3.12",
        },
        "asset": "../sim/assets/xarm6_g1/xarm6_g1.usd/xarm6_g1/xarm6_g1.usda",
        "source_config": str(args.config.relative_to(ROOT)),
        "control_period_s": float(config["control_period_s"]),
        "reference_sample_count": len(reference),
        "reference_duration_s": reference[-1].time_s,
        "peak_joint_speed_rad_s": float(np.max(np.abs(qd))),
        "peak_joint_acceleration_rad_s2": float(np.max(np.abs(qdd))),
        "g1_real": {
            "held_position": 370.0,
            "partial_open_position": 520.0,
            "partial_motion_s": 0.10,
            "measured_detach_delay_s_range": [0.025, 0.044],
            "sim_drive_direction_is_not_a_real_command": True
        },
        "timing": {
            "arm_tracking_delay_s": 0.09,
            "catch_servo_start_s": 0.52,
            "camera_control_end_s": 0.58,
            "prediction_horizon_s": 0.05,
            "catch_close_command_s": 0.58,
        },
        "predicted_sim_state": {
            "release_command_position_base_m": [0.19706, 0.01429, 0.38416],
            "release_command_velocity_base_m_s": [0.24738, 0.01800, 0.34241],
            "rendered_camera_detach_threshold_s": 0.565,
            "free_vertical_displacement_m": 0.00694,
        },
        "model": "../sim/models/intercept_residual_real_v1.json",
        "observation_schema": "../sim/models/intercept_residual_real_v1.json#feature_names",
        "camera_calibration": "../configs/global_camera_real.json",
        "wrist_camera_calibration": "../configs/wrist_camera_real.json",
        "dry_run": "../scripts/20_closed_loop_dry_run.py",
        "candidate_timelines": candidates,
        "evaluation": {
            "summary": "evaluation_summary.json",
            "perturbed_native_cohort": "../sim/outputs/native_cohort_v3/summary.json",
            "failure_analysis": "../sim/outputs/native_cohort_v3/failure_analysis.json",
            "real_candidate_3": "../sim/outputs/real_candidate_learned_3/summary.json",
        },
        "real_execution_status": "requires empty preview on the robot computer",
    }
    (args.output / "manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n"
    )
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
