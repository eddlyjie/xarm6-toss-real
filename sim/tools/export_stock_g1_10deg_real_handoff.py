#!/usr/bin/env python3
"""Export the stock-G1 10-degree stable regrasp for the real runner."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import asdict
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from xarm6_toss.control_reference import QuinticJointSegment, generate_joint_reference  # noqa: E402
from xarm6_toss.motion_limits import evaluate_reference_samples  # noqa: E402


REFERENCE_PATH = ROOT / "sim/configs/pose_rotation_throwonly_r10cfh.json"
SUMMARY_PATH = ROOT / "docs/media/stock_g1_10deg_v86/summary.json"
CONTROLLER_TEMPLATE_PATH = ROOT / "real_handoff/j5_dynamic_regrasp_controller.json"
TIMELINE_OUTPUT = ROOT / "real_handoff/stock_g1_10deg_regrasp_timeline.json"
CONTROLLER_OUTPUT = ROOT / "real_handoff/stock_g1_10deg_regrasp_controller.json"
HOLD_UNTIL_S = 1.40
NOMINAL_DETACH_TIME_S = 0.660


def extended_samples(reference: dict) -> tuple[list[dict], dict]:
    generated = generate_joint_reference(
        tuple(
            QuinticJointSegment(**segment)
            for segment in reference["reference_segments"]
        ),
        float(reference["control_period_s"]),
    )
    limits = evaluate_reference_samples(generated)
    if not limits["joint_mechanical_limits_pass"]:
        raise RuntimeError(f"10-degree reference violates real limits: {limits}")
    samples = [asdict(sample) for sample in generated]
    period_s = float(reference["control_period_s"])
    final_position = list(samples[-1]["joint_position_rad"])
    next_time_s = round(float(samples[-1]["time_s"]) + period_s, 10)
    while next_time_s <= HOLD_UNTIL_S + 1.0e-9:
        samples.append(
            {
                "time_s": next_time_s,
                "phase": "catch_and_stable_hold_window",
                "joint_position_rad": final_position,
                "joint_velocity_rad_s": [0.0] * 6,
                "joint_acceleration_rad_s2": [0.0] * 6,
            }
        )
        next_time_s = round(next_time_s + period_s, 10)
    return samples, limits


def main() -> int:
    reference = json.loads(REFERENCE_PATH.read_text(encoding="utf-8"))
    summary = json.loads(SUMMARY_PATH.read_text(encoding="utf-8"))
    template = json.loads(CONTROLLER_TEMPLATE_PATH.read_text(encoding="utf-8"))
    samples, limits = extended_samples(reference)

    fixed_profile = {
        "name": "stock_g1_10deg_stable",
        "controller": {
            "catch_servo_start_time_s": 0.680,
            "catch_preclose_time_s": 0.840,
            "catch_preclose_drive_rad": 0.48,
            "catch_close_time_s": 0.920,
            "vision_control_end_time_s": 1.040,
            "catch_intercept_time_s": 1.000,
            "catch_lateral_only": False,
            "catch_hold_throw_joints": False,
            "catch_lock_wrist": True,
            "catch_position_bias_m": [0.0, 0.0, 0.0],
            "catch_drive_rad": 0.65,
        },
    }
    timeline = {
        "schema": "xarm6_stock_g1_10deg_real_timeline_v1",
        "profile_name": "stock_g1_10deg_stable_regrasp",
        "execution_mode": "dynamic_regrasp",
        "status": "sim_validated_real_unverified",
        "source_reference": str(REFERENCE_PATH.relative_to(ROOT)),
        "source_summary": str(SUMMARY_PATH.relative_to(ROOT)),
        "control_period_s": reference["control_period_s"],
        "nominal_sim_detach_time_s": NOMINAL_DETACH_TIME_S,
        "measured_real_arm_tracking_delay_s": reference[
            "measured_real_arm_tracking_delay_s"
        ],
        "operator_approval_required": True,
        "execution_envelope": reference["execution_envelope"],
        "reference_limit_evidence": limits,
        "wrist_branch": reference["wrist_branch"],
        "active_throw_joints_one_based": [2, 3, 5],
        "fixed_throw_joints_one_based": [1, 4, 6],
        "release_command_time_s": 0.620,
        "expected_real_detach_window_s": [0.645, 0.664],
        "catch_window_end_time_s": 1.040,
        "samples": samples,
    }

    controller = deepcopy(template)
    controller.update(
        {
            "schema": "xarm6_stock_g1_10deg_regrasp_controller_v1",
            "profile_name": "stock_g1_10deg_stable_regrasp",
            "execution_mode": "dynamic_regrasp",
            "status": "sim_validated_real_unverified",
            "source_summary": str(SUMMARY_PATH.relative_to(ROOT)),
            "reference_timeline": str(TIMELINE_OUTPUT.relative_to(ROOT)),
            "nominal_sim_detach_time_s": NOMINAL_DETACH_TIME_S,
            "fixed_control_profile": fixed_profile,
        }
    )
    controller["robot"].update(
        {
            "reference_peak_speed_rad_s": limits["max_joint_speed_rad_s"],
            "reference_peak_acceleration_rad_s2": limits[
                "max_joint_acceleration_rad_s2"
            ],
            "minimum_joint_margin_rad": limits["minimum_joint_margin_rad"],
        }
    )
    controller["g1_real"].update(
        {
            "release_command_time_s": 0.620,
            "preclose_from_observed_detach_s": 0.180,
            "final_close_from_observed_detach_s": 0.260,
        }
    )
    controller["ballistic_catch"].update(
        {
            "catch_servo_from_observed_detach_s": 0.020,
            "intercept_from_observed_detach_s": 0.340,
            "control_end_from_observed_detach_s": 0.380,
        }
    )
    controller["probe_j"].update(
        {
            "required": True,
            "selection_role": (
                "paired Probe gates cube execution; the frozen 10-degree "
                "profile supplies detach-relative catch timing"
            ),
        }
    )
    controller["sim_evidence"] = {
        "source_trial": "v86_fast_early_avoid",
        "signed_forward_rotation_deg": summary[
            "free_flight_signed_tumble_rotation_deg"
        ],
        "strict_free_flight_s": summary[
            "continuous_free_flight_duration_s"
        ],
        "axis_alignment": summary["tumble_axis_alignment"],
        "first_bilateral_contact_time_s": summary[
            "first_renewed_bilateral_contact_time_s"
        ],
        "bilateral_contact_fraction": summary[
            "bilateral_contact_fraction"
        ],
        "catch_stable": summary["catch_stable"],
        "reference_joint_mechanical_limits_pass": limits[
            "joint_mechanical_limits_pass"
        ],
        "real_verified": False,
    }

    TIMELINE_OUTPUT.write_text(
        json.dumps(timeline, indent=2) + "\n", encoding="utf-8"
    )
    CONTROLLER_OUTPUT.write_text(
        json.dumps(controller, indent=2) + "\n", encoding="utf-8"
    )
    print(f"timeline={TIMELINE_OUTPUT}")
    print(f"controller={CONTROLLER_OUTPUT}")
    print(f"samples={len(samples)} duration_s={samples[-1]['time_s']:.3f}")
    print(
        f"peak_speed={limits['max_joint_speed_rad_s']:.6f} "
        f"peak_acceleration={limits['max_joint_acceleration_rad_s2']:.6f}"
    )
    print("export only; no robot connection or command was sent")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
