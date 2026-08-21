#!/usr/bin/env python3
"""Export the standard-G1 pose-rotation reference for a real throw-only check."""

from __future__ import annotations

import argparse
from dataclasses import asdict
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


DEFAULT_REFERENCE = ROOT / "sim/configs/pose_rotation_throwonly_r90.json"
DEFAULT_EVIDENCE = (
    ROOT / "docs/media/j5_forward_rotation/standard_g1_throwonly_v62.json"
)
DEFAULT_OUTPUT = (
    ROOT / "real_handoff/standard_g1_throwonly_11p5deg_timeline.json"
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", type=Path, default=DEFAULT_REFERENCE)
    parser.add_argument("--evidence", type=Path, default=DEFAULT_EVIDENCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    reference = json.loads(args.reference.read_text(encoding="utf-8"))
    evidence = json.loads(args.evidence.read_text(encoding="utf-8"))
    if evidence["release_mechanism"] != "standard_g1_no_insert":
        raise RuntimeError("real checkpoint must not depend on a release insert")
    samples = generate_joint_reference(
        tuple(
            QuinticJointSegment(**segment)
            for segment in reference["reference_segments"]
        ),
        float(reference["control_period_s"]),
    )
    limits = evaluate_reference_samples(samples)
    if not limits["joint_mechanical_limits_pass"]:
        raise RuntimeError(f"reference violates the real envelope: {limits}")

    payload = {
        "schema": "xarm6_standard_g1_throwonly_real_timeline_v1",
        "profile_name": "standard_g1_throwonly_11p5deg",
        "execution_mode": "throw_only",
        "status": "sim_validated_real_unverified",
        "source_reference": str(args.reference.relative_to(ROOT)),
        "source_evidence": str(args.evidence.relative_to(ROOT)),
        "control_period_s": reference["control_period_s"],
        "measured_real_arm_tracking_delay_s": reference[
            "measured_real_arm_tracking_delay_s"
        ],
        "operator_approval_required": True,
        "reference_limit_evidence": limits,
        "wrist_branch": reference["wrist_branch"],
        "active_throw_joints_one_based": [2, 3, 5],
        "fixed_throw_joints_one_based": [1, 4, 6],
        "release_command_time_s": evidence["release_command_time_s"],
        "expected_detach_window_s": evidence["expected_real_detach_window_s"],
        "samples": [asdict(sample) for sample in samples],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(payload, indent=2) + "\n", encoding="utf-8"
    )
    print(f"output={args.output}")
    print(f"samples={len(samples)} duration_s={samples[-1].time_s:.3f}")
    print(
        "peak_speed_rad_s="
        f"{limits['max_joint_speed_rad_s']:.6f} "
        "peak_acceleration_rad_s2="
        f"{limits['max_joint_acceleration_rad_s2']:.6f}"
    )
    print("export only; no robot connection or command was sent")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
