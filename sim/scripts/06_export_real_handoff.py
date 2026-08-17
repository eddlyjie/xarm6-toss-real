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


DEFAULT_CONFIG = ROOT / "sim" / "configs" / "outward_vertical_real_detach_v7.json"
DEFAULT_RESULT = ROOT / "outputs" / "clear_probe_j_camera_seed_20260871_v2"
DEFAULT_PROBE_J_CONFIG = ROOT / "sim" / "configs" / "probe_j_fixed_cube_v1.json"
DEFAULT_MODEL = ROOT / "sim" / "models" / "intercept_residual_vertical_third_view_v1.json"
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
    parser.add_argument("--probe-j-config", type=Path, default=DEFAULT_PROBE_J_CONFIG)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--joint6-roll-offset-rad", type=float, default=0.0)
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
    probe_j_config = json.loads(args.probe_j_config.read_text(encoding="utf-8"))
    probe_j_evidence = json.loads(
        (args.result / "probe_j.json").read_text(encoding="utf-8")
    )
    clear_controller = next(
        item["controller"] for item in probe_j_config["catch_candidates"]
        if item["name"] == "clear_flight_upgrade"
    )
    model = json.loads(args.model.read_text(encoding="utf-8"))

    args.output.mkdir(parents=True, exist_ok=True)
    rows = [sample_dict(sample) for sample in reference]
    reference_qd = np.asarray([row["joint_velocity_rad_s"] for row in rows])
    reference_qdd = np.asarray(
        [row["joint_acceleration_rad_s2"] for row in rows]
    )
    reference_max_speed = float(np.max(np.abs(reference_qd)))
    reference_max_acceleration = float(np.max(np.abs(reference_qdd)))
    timeline = {
        "schema": "xarm6_outward_toss_timeline_v3",
        "control_period_s": setup.control_period_s,
        "joint6_roll_offset_rad": args.joint6_roll_offset_rad,
        "joint6_roll_is_baked_into_reference": True,
        "execution_envelope": source_config["execution_envelope"],
        "operator_approval_required": True,
        "reference_max_joint_speed_rad_s": reference_max_speed,
        "reference_max_joint_acceleration_rad_s2": reference_max_acceleration,
        "g1_events_are_asynchronous_to_arm_ticks": True,
        "g1_events": [
            {
                "time_s": summary["release_command_time_s"],
                "name": "release_partial_open",
                "position": setup.partial_open_gripper_position,
            },
            {
                "time_s": summary["catch_close_time_s"],
                "name": "stable_candidate_close",
                "position": setup.close_gripper_position,
            },
        ],
        "clear_flight_upgrade_g1_event": {
            "time_s": clear_controller["catch_close_time_s"],
            "name": "clear_flight_upgrade_close",
            "position": setup.close_gripper_position,
        },
        "arm_tracking_delay_compensation_s": summary["arm_tracking_delay_s"],
        "samples": rows,
    }
    (args.output / "nominal_timeline.json").write_text(
        json.dumps(timeline, indent=2) + "\n", encoding="utf-8"
    )
    write_csv(args.output / "nominal_timeline.csv", rows)

    controller = {
        "schema": "xarm6_outward_camera_ballistic_controller_v3",
        "fixed_cube_side_m": setup.cube_side_m,
        "nominal_cube_mass_kg": setup.nominal_cube_mass_kg,
        "arm": {
            "control_period_s": setup.control_period_s,
            "real_configured_transfer_cap": {
                "joint_speed_rad_s": setup.max_joint_speed_rad_s,
                "joint_acceleration_rad_s2": setup.max_joint_acceleration_rad_s2,
            },
            "candidate_command_cap": source_config["limits"],
            "reference_max_joint_speed_rad_s": reference_max_speed,
            "reference_max_joint_acceleration_rad_s2": reference_max_acceleration,
            "candidate_exceeds_transfer_cap": True,
            "operator_approval_required": True,
            "measured_tracking_delay_s": setup.arm_tracking_delay_s,
        },
        "g1_real_positions": {
            "held": setup.held_gripper_position,
            "partial_open": setup.partial_open_gripper_position,
            "close": setup.close_gripper_position,
            "firmware_speed": 5000,
        },
        "timing_s": {
            "release_command": summary["release_command_time_s"],
            "detach_prior": summary["detach_delay_prior_s"],
            "catch_servo_start": summary["catch_servo_start_time_s"],
            "vision_control_end": summary["vision_control_end_time_s"],
            "catch_close_command": summary["catch_close_time_s"],
            "ballistic_intercept": summary["catch_intercept_time_s"],
            "arm_tracking_delay_compensation": summary["arm_tracking_delay_s"],
        },
        "catch_servo": {
            "mode": "third_view_lateral_joint1",
            "gain": 0.75,
            "max_joint_step_rad": 0.035,
        },
        "residual": {
            "model": "../sim/models/intercept_residual_vertical_third_view_v1.json",
            "minimum_camera_samples": 1,
            "action_norm_limit_m": model["action_norm_limit_m"],
        },
        "camera": {
            "rate_hz": setup.third_view.fps,
            "receive_latency_s": 0.020,
            "third_view_serial": setup.third_view.serial,
            "wrist_serial": setup.wrist.serial,
            "flight_primary": "third_view",
            "wrist_role": "grasp_probe_and_opportunistic_flight_updates",
            "missing_frames_propagate_ballistic_belief": True,
            "wrist_detection_required_for_catch": False,
        },
        "probe_preflight": {
            "required_before_cube_throw": True,
            "source_config": "../toss_project_sim_handoff/toss_project/real_cube_demo/configs/probe.json",
            "empty_run_script": "../toss_project_sim_handoff/toss_project/real_cube_demo/scripts/07_probe_cube.py",
            "comparison_script": "../toss_project_sim_handoff/toss_project/real_cube_demo/scripts/08_compare_probe.py",
            "required_outputs": [
                "effective_payload_posterior",
                "held_probability",
                "slip_probability",
                "detach_timing_uncertainty",
            ],
            "sim_paired_probe_config": "../sim/configs/probe_j_fixed_cube_v1.json",
            "sim_paired_probe_used_for_control": summary["probe_used_for_control"],
            "sim_j_used_for_control": summary["j_used_for_control"],
            "sim_probe_gate_passed": summary["probe_gate_passed"],
            "sim_probe_posterior": summary["probe_posterior"],
            "sim_selected_catch_candidate": summary["selected_catch_candidate"],
            "sim_catch_candidate_ranking": summary["catch_candidate_ranking"],
            "real_paired_probe_completed": False,
            "current_sim_evidence_consumes_real_probe_output": False,
        },
        "spectator_used_for_control": False,
    }
    (args.output / "controller_config.json").write_text(
        json.dumps(controller, indent=2) + "\n", encoding="utf-8"
    )
    (args.output / "sim_probe_j_evidence.json").write_text(
        json.dumps(probe_j_evidence, indent=2) + "\n", encoding="utf-8"
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
        "schema": "xarm6_real_constraints_report_v3",
        "source_result": str(args.result.relative_to(ROOT)),
        "success": {
            "detach_detected": summary["detach_detected"],
            "catch_stable": summary["catch_stable"],
            "bilateral_contact_fraction": summary["bilateral_contact_fraction"],
            "continuous_free_flight_duration_s": summary["continuous_free_flight_duration_s"],
            "free_flight_rise_from_kinematic_release_m": summary["free_flight_rise_from_kinematic_release_m"],
            "free_flight_apex_is_internal": summary["free_flight_apex_is_internal"],
            "precontact_vertical_velocity_m_s": summary["precontact_vertical_velocity_m_s"],
            "obvious_free_flight": summary["obvious_free_flight"],
            "obvious_toss_success": summary["obvious_toss_success"],
            "maximum_separation_m": summary["maximum_separation_m"],
            "free_vertical_displacement_m": summary["free_vertical_displacement_m"],
            "detach_to_first_bilateral_contact_s": (
                None if bilateral_time_s is None else bilateral_time_s - detach_time_s
            ),
        },
        "geometry": {
            "release_hand_horizontal_radius_m": summary["release_tcp_horizontal_radius_m"],
            "release_outward_dot_m": summary["release_outward_dot_m"],
            "configured_gripper_base_position_m": source_config["release_geometry"]["gripper_base_position_m"],
            "baked_joint6_roll_rad": source_config["release_geometry"]["joint6_roll_rad"],
        },
        "limits": {
            "actual_max_joint_speed_rad_s": summary["actual_max_joint_speed_rad_s"],
            "actual_max_joint_acceleration_rad_s2": summary["actual_max_joint_acceleration_rad_s2"],
            "commanded_max_joint_speed_rad_s": summary["commanded_max_joint_speed_rad_s"],
            "commanded_max_joint_acceleration_rad_s2": summary["commanded_max_joint_acceleration_rad_s2"],
            "reference_max_joint_speed_rad_s": reference_max_speed,
            "reference_max_joint_acceleration_rad_s2": reference_max_acceleration,
            "candidate_command_cap": source_config["limits"],
            "real_configured_transfer_cap": {
                "joint_speed_rad_s": setup.max_joint_speed_rad_s,
                "joint_acceleration_rad_s2": setup.max_joint_acceleration_rad_s2,
            },
            "candidate_exceeds_transfer_cap": True,
            "operator_approval_required": True,
            "minimum_actual_joint_margin_rad": np.min(margin, axis=0).tolist(),
        },
        "delays_s": {
            "real_arm_tracking_measured": setup.arm_tracking_delay_s,
            "sim_explicit_command_delay": source_config["sim_actuator_calibration"]["explicit_command_delay_s"],
            "sim_measured_tracking_lag": source_config["sim_actuator_calibration"]["measured_best_tracking_lag_s"],
            "g1_prior": summary["detach_delay_prior_s"],
            "g1_measured_sim_detach": summary["detach_delay_s"],
            "real_measured_detach_range": list(setup.detach_delay_range_s),
            "camera_receive": controller["camera"]["receive_latency_s"],
        },
        "policy": {
            "flight_primary_camera": "third_view",
            "wrist_terminal_required": False,
            "camera_counts": camera_counts,
            "camera_updates_after_detach": summary["camera_control_updates_after_detach"],
            "learned_updates_after_detach": summary["learned_control_updates_after_detach"],
            "terminal_wrist_observation_count": summary["terminal_wrist_observation_count"],
            "intercept_mean_error_before_m": summary["intercept_mean_error_before_residual_m"],
            "intercept_mean_error_after_m": summary["intercept_mean_error_after_residual_m"],
            "intercept_residual_fraction_improved": summary["intercept_residual_fraction_improved"],
            "spectator_used_for_control": summary["spectator_used_for_control"],
        },
        "probe": controller["probe_preflight"],
    }
    (args.output / "real_constraints_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )

    manifest = {
        "schema": "xarm6_outward_real_handoff_v3",
        "source_config": str(args.config.relative_to(ROOT)),
        "source_result": str(args.result.relative_to(ROOT)),
        "primary_candidate": "stable_third_view_learned",
        "timeline": "nominal_timeline.json",
        "controller": "controller_config.json",
        "constraints_report": "real_constraints_report.json",
        "probe_j_evidence": "sim_probe_j_evidence.json",
        "videos": {
            "spectator": "../outputs/clear_probe_j_camera_seed_20260871_v2/spectator.mp4",
            "spectator_slow": "../outputs/clear_probe_j_camera_seed_20260871_v2/spectator_slow_0p25x.mp4",
            "third_view": "../outputs/clear_probe_j_camera_seed_20260871_v2/spectator_third_view.mp4",
            "wrist": "../outputs/clear_probe_j_camera_seed_20260871_v2/spectator_wrist.mp4",
            "clear_flight_camera_diagnostic": "../outputs/final_clear_camera_seed_20260841/spectator.mp4",
            "clear_flight_camera_diagnostic_zoom_slow": "../outputs/final_clear_camera_seed_20260841/spectator_zoom_slow_0p4x.mp4",
        },
        "evidence": {
            "probe_j_camera_learned": "3/3 catches with paired Probe, J selection, cameras, learned residual and 90 ms arm lag; 95/95/145 ms free flight",
            "clear_flight_physics": "1/1 stable catch; 245 ms free flight; apex and descending catch; no policy camera",
            "clear_flight_rendered_camera": "160 ms free flight and apex; final bilateral fraction 0.667; not a stable catch",
        },
        "validated_scope": "one fixed 38 mm cube near 35 g; nominal camera/dropout seeds",
        "known_limits": [
            "The 3/3 Probe/J candidate is the stable transfer baseline, not a strict obvious-flight success; only seed 20260863 reached an internal apex and descending renewed contact, with one rather than two post-apex spectator frames.",
            "The strict clear-flight stable success uses physics observation; rendered-camera clear flight is not yet a stable catch.",
            "The simulator consumes paired empty/held actuator effort and uses its posterior in J; the corresponding real paired-current Probe has not yet been run.",
            "Wrist has zero terminal detections in this reversed-wrist candidate; third_view carries flight tracking.",
            "The 1x reference exceeds the current 0.45/1.5 transfer cap, and sim actual acceleration peaks near 90.2 rad/s^2 despite the 20 rad/s^2 command cap; it is not approved for direct real execution.",
        ],
        "real_execution_status": "paired real Probe and disconnected dry-run required; only empty 0.25x preview is inside the current transfer cap; full-speed throw requires operator approval and acceleration retiming",
    }
    (args.output / "manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(manifest, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
