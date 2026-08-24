#!/usr/bin/env python3
"""Plan an open-loop J2/J3/J5 intercept from an existing Sim flight."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import build_pose_rotation_ladder as ladder  # noqa: E402
from xarm6_toss.pose_conditioned_catch import (  # noqa: E402
    CatchPlanSettings,
    ballistic_continuation,
    plan_pose_conditioned_catch,
)
from xarm6_toss_sim.kinematics import URDFKinematics  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trajectory", type=Path, required=True)
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--desired-angle-deg", type=float, required=True)
    parser.add_argument(
        "--ballistic-anchor-time-s",
        type=float,
        default=None,
        help="Replace later cube states with contact-free flight from this measured time.",
    )
    parser.add_argument("--start-time-s", type=float, default=0.68)
    parser.add_argument("--activation-time-s", type=float, default=None)
    parser.add_argument("--earliest-intercept-time-s", type=float, default=0.84)
    parser.add_argument("--latest-intercept-time-s", type=float, default=1.04)
    parser.add_argument("--maximum-capture-position-error-m", type=float, default=0.019)
    parser.add_argument("--maximum-capture-velocity-error-m-s", type=float, default=0.10)
    parser.add_argument("--minimum-preintercept-distance-m", type=float, default=0.022)
    parser.add_argument(
        "--minimum-preintercept-vertical-clearance-m",
        type=float,
        default=0.008,
    )
    parser.add_argument(
        "--preintercept-clearance-start-time-s",
        type=float,
        default=None,
    )
    parser.add_argument(
        "--final-approach-duration-s",
        type=float,
        default=0.08,
    )
    parser.add_argument(
        "--actuator-response-alpha",
        type=float,
        nargs=3,
        default=None,
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--reference-output", type=Path, required=True)
    args = parser.parse_args()
    if args.desired_angle_deg <= 0.0:
        parser.error("desired angle must be positive")

    rows = json.loads(args.trajectory.read_text(encoding="utf-8"))
    ballistic_anchor_time_s = None
    if args.ballistic_anchor_time_s is not None:
        rows, ballistic_anchor_time_s = ballistic_continuation(
            rows,
            anchor_time_s=args.ballistic_anchor_time_s,
        )
    settings = CatchPlanSettings(
        maximum_capture_position_error_m=args.maximum_capture_position_error_m,
        maximum_capture_velocity_error_m_s=args.maximum_capture_velocity_error_m_s,
        minimum_preintercept_distance_m=args.minimum_preintercept_distance_m,
        minimum_preintercept_vertical_clearance_m=args.minimum_preintercept_vertical_clearance_m,
        preintercept_clearance_start_time_s=args.preintercept_clearance_start_time_s,
        final_approach_duration_s=args.final_approach_duration_s,
        actuator_response_alpha=(
            None if args.actuator_response_alpha is None
            else tuple(args.actuator_response_alpha)
        ),
    )
    summary = json.loads(args.summary.read_text(encoding="utf-8"))
    plan = plan_pose_conditioned_catch(
        rows,
        summary,
        URDFKinematics(ladder.base.URDF),
        desired_angle_deg=args.desired_angle_deg,
        start_time_s=args.start_time_s,
        earliest_intercept_time_s=args.earliest_intercept_time_s,
        latest_intercept_time_s=args.latest_intercept_time_s,
        settings=settings,
    )
    plan["ballistic_continuation"] = {
        "enabled": ballistic_anchor_time_s is not None,
        "anchor_time_s": ballistic_anchor_time_s,
        "gravity_m_s2": 9.81 if ballistic_anchor_time_s is not None else None,
        "source": "measured_postdetach_cube_pose_twist",
    }
    selected = plan["selected"]
    activation_time_s = (
        plan["start_time_s"]
        if args.activation_time_s is None
        else float(args.activation_time_s)
    )
    if activation_time_s > plan["start_time_s"] + 1.0e-9:
        parser.error("activation time cannot be later than planning start")
    prefix_samples = []
    prefix_time_s = activation_time_s
    fixed_reference = selected["samples"][0]["joint_position_rad"]
    while prefix_time_s < plan["start_time_s"] - 1.0e-9:
        row = min(rows, key=lambda item: abs(float(item["time_s"]) - prefix_time_s))
        prefix_position = list(row["arm_joint_position_rad"])
        prefix_velocity = list(row["arm_joint_velocity_rad_s"])
        for fixed_index in (0, 3, 5):
            prefix_position[fixed_index] = fixed_reference[fixed_index]
            prefix_velocity[fixed_index] = 0.0
        prefix_samples.append(
            {
                "time_s": prefix_time_s,
                "phase": "measured_safe_escape_prefix",
                "joint_position_rad": prefix_position,
                "joint_velocity_rad_s": prefix_velocity,
                "joint_acceleration_rad_s2": [0.0] * 6,
            }
        )
        prefix_time_s += settings.control_period_s
    reference_samples = prefix_samples + selected["samples"]
    reference = {
        "schema": "xarm6_offline_j235_catch_reference_v1",
        "desired_angle_deg": plan["desired_angle_deg"],
        "predicted_rotation_at_intercept_deg": selected[
            "predicted_rotation_at_intercept_deg"
        ],
        "start_time_s": activation_time_s,
        "planned_motion_start_time_s": plan["start_time_s"],
        "intercept_time_s": selected["intercept_time_s"],
        "capture_position_error_m": selected["capture_position_error_m"],
        "capture_velocity_error_m_s": selected["capture_velocity_error_m_s"],
        "capture_admitted": selected["capture_admitted"],
        "dynamic_joint_indices_zero_based": plan[
            "dynamic_joint_indices_zero_based"
        ],
        "ballistic_continuation": plan["ballistic_continuation"],
        "fixed_joint_indices_zero_based": plan["fixed_joint_indices_zero_based"],
        "samples": reference_samples,
        "real_g1_schedule": plan["real_g1_schedule"],
        "execution_note": (
            "Replay only after plan-only and empty-arm checks; no online object "
            "observation is required."
        ),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.reference_output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(plan, indent=2) + "\n", encoding="utf-8")
    args.reference_output.write_text(
        json.dumps(reference, indent=2) + "\n", encoding="utf-8"
    )
    print(
        json.dumps(
            {
                "desired_angle_deg": plan["desired_angle_deg"],
                "desired_angle_admitted": plan["desired_angle_admitted"],
                "selected_intercept_time_s": selected["intercept_time_s"],
                "predicted_rotation_deg": selected[
                    "predicted_rotation_at_intercept_deg"
                ],
                "capture_position_error_m": selected[
                    "capture_position_error_m"
                ],
                "capture_velocity_error_m_s": selected[
                    "capture_velocity_error_m_s"
                ],
                "capture_admitted": selected["capture_admitted"],
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
