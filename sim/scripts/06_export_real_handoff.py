#!/usr/bin/env python3
"""Export the validated outward xArm6 toss/catch candidate for real dry-run."""

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
from xarm6_toss_sim import URDFKinematics, load_real_setup  # noqa: E402


DEFAULT_CONFIG = ROOT / "sim" / "configs" / "outward_minimal_v1.json"
DEFAULT_RESULT = ROOT / "outputs" / "final_handoff_nominal"
DEFAULT_MODEL = ROOT / "sim" / "models" / "intercept_residual_native_outward_v1.json"
DEFAULT_OUTPUT = ROOT / "real_handoff"


def load_reference(path: Path, joint6_offset_rad: float):
    payload = json.loads(path.read_text(encoding="utf-8"))
    segments = []
    for item in payload["reference_segments"]:
        start_q = list(item["start_joint_rad"])
        end_q = list(item["end_joint_rad"])
        start_q[5] += joint6_offset_rad
        end_q[5] += joint6_offset_rad
        segments.append(
            QuinticJointSegment(
                phase=item["phase"],
                duration_s=float(item["duration_s"]),
                start_joint_rad=tuple(start_q),
                start_joint_velocity_rad_s=tuple(item["start_joint_velocity_rad_s"]),
                end_joint_rad=tuple(end_q),
                end_joint_velocity_rad_s=tuple(item["end_joint_velocity_rad_s"]),
                start_joint_acceleration_rad_s2=tuple(
                    item.get("start_joint_acceleration_rad_s2", (0.0,) * 6)
                ),
                end_joint_acceleration_rad_s2=tuple(
                    item.get("end_joint_acceleration_rad_s2", (0.0,) * 6)
                ),
            )
        )
    return payload, generate_joint_reference(
        tuple(segments), float(payload["control_period_s"])
    )


def sample_dict(sample) -> dict:
    return {
        "time_s": sample.time_s,
        "phase": sample.phase,
        "joint_position_rad": list(sample.joint_position_rad),
        "joint_velocity_rad_s": list(sample.joint_velocity_rad_s),
        "joint_acceleration_rad_s2": list(sample.joint_acceleration_rad_s2),
    }


def write_csv(path: Path, rows: list[dict]) -> None:
    columns = ["time_s", "phase"]
    for field in (
        "joint_position_rad",
        "joint_velocity_rad_s",
        "joint_acceleration_rad_s2",
    ):
        columns.extend(f"{field}_{index}" for index in range(1, 7))
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        for sample in rows:
            row = {"time_s": sample["time_s"], "phase": sample["phase"]}
            for field in columns[2:]:
                vector_name, index = field.rsplit("_", 1)
                row[field] = sample[vector_name][int(index) - 1]
            writer.writerow(row)


def first_bilateral_time(rows, detach_time_s):
    for row in rows:
        if (
            row["time_s"] > detach_time_s
            and row["left_finger_cube_contact_force_n"] > 0.01
            and row["right_finger_cube_contact_force_n"] > 0.01
        ):
            return float(row["time_s"])
    return None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--result", type=Path, default=DEFAULT_RESULT)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--joint6-roll-offset-rad", type=float, default=0.785398)
    args = parser.parse_args()

    setup = load_real_setup()
    kinematics = URDFKinematics(setup.urdf_path)
    source_config, reference = load_reference(
        args.config, args.joint6_roll_offset_rad
    )
    summary = json.loads((args.result / "summary.json").read_text(encoding="utf-8"))
    trajectory = json.loads((args.result / "trajectory.json").read_text(encoding="utf-8"))
    camera_rows = json.loads(
        (args.result / "camera_measurements.json").read_text(encoding="utf-8")
    )
    model = json.loads(args.model.read_text(encoding="utf-8"))

    args.output.mkdir(parents=True, exist_ok=True)
    rows = [sample_dict(sample) for sample in reference]
    timeline = {
        "schema": "xarm6_outward_toss_timeline_v1",
        "control_period_s": setup.control_period_s,
        "joint6_roll_offset_rad": args.joint6_roll_offset_rad,
        "g1_events_are_asynchronous_to_arm_ticks": True,
        "g1_events": [
            {"time_s": 0.69, "name": "release_partial_open", "position": setup.partial_open_gripper_position},
            {"time_s": 0.78, "name": "catch_close", "position": setup.close_gripper_position},
        ],
        "samples": rows,
    }
    (args.output / "nominal_timeline.json").write_text(
        json.dumps(timeline, indent=2) + "\n", encoding="utf-8"
    )
    write_csv(args.output / "nominal_timeline.csv", rows)

    controller = {
        "schema": "xarm6_outward_camera_ballistic_controller_v1",
        "fixed_cube_side_m": setup.cube_side_m,
        "nominal_cube_mass_kg": setup.nominal_cube_mass_kg,
        "arm": {
            "control_period_s": setup.control_period_s,
            "max_joint_speed_rad_s": setup.max_joint_speed_rad_s,
            "max_joint_acceleration_rad_s2": setup.max_joint_acceleration_rad_s2,
            "tracking_delay_s": setup.arm_tracking_delay_s,
        },
        "g1_real_positions": {
            "held": setup.held_gripper_position,
            "partial_open": setup.partial_open_gripper_position,
            "close": setup.close_gripper_position,
            "firmware_speed": 5000,
        },
        "timing_s": {
            "release_command": 0.69,
            "detach_prior": 0.030,
            "catch_servo_start": 0.72,
            "vision_control_end": 0.78,
            "catch_close_command": 0.78,
            "ballistic_intercept": 0.815,
        },
        "catch_servo": {
            "gain": 0.25,
            "max_joint_step_rad": 0.005,
        },
        "residual": {
            "model": "../sim/models/intercept_residual_native_outward_v1.json",
            "minimum_camera_samples": 3,
            "action_norm_limit_m": model["action_norm_limit_m"],
        },
        "camera": {
            "rate_hz": setup.third_view.fps,
            "receive_latency_s": 0.022,
            "third_view_serial": setup.third_view.serial,
            "wrist_serial": setup.wrist.serial,
            "missing_frames_propagate_ballistic_belief": True,
            "wrist_detection_required_for_catch": False,
        },
    }
    (args.output / "controller_config.json").write_text(
        json.dumps(controller, indent=2) + "\n", encoding="utf-8"
    )

    actual_q = np.asarray([row["arm_joint_position_rad"] for row in trajectory])
    lower, upper = kinematics.arm_limits
    margin = np.minimum(actual_q - lower, upper - actual_q)
    detach_time_s = float(summary["detach_time_s"])
    bilateral_time_s = first_bilateral_time(trajectory, detach_time_s)
    camera_counts = {}
    for source in ("third_view", "wrist"):
        source_rows = [row for row in camera_rows if row["source_camera"] == source]
        camera_counts[source] = {
            "frames_reported": len(source_rows),
            "detections": sum(row.get("detected", 0.0) > 0.5 for row in source_rows),
            "post_detach_detections": sum(
                row["time_s"] > detach_time_s and row.get("detected", 0.0) > 0.5
                for row in source_rows
            ),
        }
    report = {
        "schema": "xarm6_real_constraints_report_v1",
        "source_result": str(args.result.relative_to(ROOT)),
        "success": {
            "detach_detected": summary["detach_detected"],
            "catch_stable": summary["catch_stable"],
            "bilateral_contact_fraction": summary["bilateral_contact_fraction"],
            "maximum_separation_m": summary["maximum_separation_m"],
            "free_vertical_displacement_m": summary["free_vertical_displacement_m"],
            "detach_to_first_bilateral_contact_s": (
                None if bilateral_time_s is None else bilateral_time_s - detach_time_s
            ),
        },
        "geometry": {
            "release_hand_horizontal_radius_m": summary["release_tcp_horizontal_radius_m"],
            "release_outward_dot_m": summary["release_outward_dot_m"],
            "planned_tcp_horizontal_radius_m": source_config["release_geometry"]["tcp_horizontal_radius_m"],
            "planned_tcp_outward_dot_m": source_config["release_geometry"]["outward_dot_m"],
        },
        "limits": {
            "commanded_max_joint_speed_rad_s": summary["commanded_max_joint_speed_rad_s"],
            "allowed_max_joint_speed_rad_s": setup.max_joint_speed_rad_s,
            "commanded_max_joint_acceleration_rad_s2": summary["commanded_max_joint_acceleration_rad_s2"],
            "allowed_max_joint_acceleration_rad_s2": setup.max_joint_acceleration_rad_s2,
            "minimum_actual_joint_margin_rad": np.min(margin, axis=0).tolist(),
        },
        "delays_s": {
            "arm_tracking": setup.arm_tracking_delay_s,
            "g1_prior": summary["detach_delay_prior_s"],
            "g1_measured_sim_detach": summary["detach_delay_s"],
            "real_measured_detach_range": list(setup.detach_delay_range_s),
            "camera_receive": controller["camera"]["receive_latency_s"],
        },
        "policy": {
            "camera_counts": camera_counts,
            "camera_updates_after_detach": summary["camera_control_updates_after_detach"],
            "learned_updates_after_detach": summary["learned_control_updates_after_detach"],
            "intercept_mean_error_before_m": summary["intercept_mean_error_before_residual_m"],
            "intercept_mean_error_after_m": summary["intercept_mean_error_after_residual_m"],
            "spectator_used_for_control": summary["spectator_used_for_control"],
        },
    }
    (args.output / "real_constraints_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )

    manifest = {
        "schema": "xarm6_outward_real_handoff_v1",
        "source_config": str(args.config.relative_to(ROOT)),
        "source_result": str(args.result.relative_to(ROOT)),
        "timeline": "nominal_timeline.json",
        "controller": "controller_config.json",
        "constraints_report": "real_constraints_report.json",
        "videos": {
            "spectator": "../outputs/final_handoff_nominal/spectator.mp4",
            "spectator_slow": "../outputs/final_handoff_nominal/spectator_slow_0p4x.mp4",
            "third_view": "../outputs/final_handoff_nominal/spectator_third_view.mp4",
            "wrist": "../outputs/final_handoff_nominal/spectator_wrist.mp4",
            "three_view": "../outputs/final_handoff_nominal/three_view.mp4",
        },
        "validated_scope": "fixed 38 mm cube near 35 g; nominal calibration",
        "known_limits": [
            "25--45 g mass sweep was not 3/3; weigh/probe the real cube before timing transfer.",
            "Wrist camera did not see the cube during this short catch; third-view updates carried the belief.",
            "The sim catch window is timing-sensitive: 0.785 s close missed for the nominal opening.",
        ],
        "real_execution_status": "run disconnected dry-run and empty-gripper preview first",
    }
    (args.output / "manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(manifest, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
