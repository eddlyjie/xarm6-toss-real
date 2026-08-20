#!/usr/bin/env python3
"""Export the J5 forward-rotation reference and selected J controller."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from xarm6_toss.control_reference import (  # noqa: E402
    QuinticJointSegment,
    generate_joint_reference,
)
from xarm6_toss.motion_limits import evaluate_reference_samples  # noqa: E402


DEFAULT_REFERENCE = ROOT / "sim/configs/j5_forward_rotation_throwonly_1p6.json"
DEFAULT_PROBE_J = ROOT / "sim/configs/probe_j_j5_forward_rotation_v1.json"
DEFAULT_OUTPUT = ROOT / "real_handoff/j5_forward_rotation_timeline.json"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", type=Path, default=DEFAULT_REFERENCE)
    parser.add_argument("--probe-j", type=Path, default=DEFAULT_PROBE_J)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    reference_config = json.loads(args.reference.read_text(encoding="utf-8"))
    probe_j_config = json.loads(args.probe_j.read_text(encoding="utf-8"))
    samples = generate_joint_reference(
        tuple(
            QuinticJointSegment(**segment)
            for segment in reference_config["reference_segments"]
        ),
        float(reference_config["control_period_s"]),
    )
    limits = evaluate_reference_samples(samples)
    if not limits["joint_mechanical_limits_pass"]:
        raise RuntimeError(f"reference violates real transfer limits: {limits}")

    stable_candidate = next(
        candidate
        for candidate in probe_j_config["catch_candidates"]
        if candidate["name"] == "j5_forward_rotation_stable_0640"
    )
    controller = stable_candidate["controller"]
    payload = {
        "schema": "xarm6_j5_forward_rotation_real_handoff_v1",
        "status": "sim_validated_real_unverified",
        "control_period_s": reference_config["control_period_s"],
        "measured_real_arm_tracking_delay_s": reference_config[
            "measured_real_arm_tracking_delay_s"
        ],
        "operator_approval_required": True,
        "execution_envelope": reference_config["execution_envelope"],
        "reference_limit_evidence": limits,
        "wrist_branch": reference_config["wrist_branch"],
        "active_throw_joints_one_based": [2, 3, 5],
        "fixed_throw_joints_one_based": [1, 4, 6],
        "grasp_offset_hand_m": [0.004, 0.0, 0.024],
        "g1_events": [
            *reference_config["real_g1_events"],
            {
                "time_s": controller["catch_close_time_s"],
                "name": "j_selected_stable_close",
                "position": 370.0,
            },
        ],
        "detach_delay_range_s": [0.025, 0.044],
        "selected_controller": controller,
        "probe_j_config": str(args.probe_j.relative_to(ROOT)),
        "arm_policy_after_detach": (
            "actual_q_dq_release_prior_plus_ballistic_J1_J3_servo; "
            "J4_J6_locked"
        ),
        "camera_required_for_control": False,
        "samples": [
            {
                "time_s": sample.time_s,
                "phase": sample.phase,
                "joint_position_rad": list(sample.joint_position_rad),
                "joint_velocity_rad_s": list(sample.joint_velocity_rad_s),
                "joint_acceleration_rad_s2": list(
                    sample.joint_acceleration_rad_s2
                ),
            }
            for sample in samples
        ],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(payload, indent=2) + "\n", encoding="utf-8"
    )
    print(f"output={args.output}")
    print(f"samples={len(samples)} duration_s={samples[-1].time_s:.3f}")
    print(f"limits_pass={limits['joint_mechanical_limits_pass']}")
    print("export only; no robot connection or command was sent")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
