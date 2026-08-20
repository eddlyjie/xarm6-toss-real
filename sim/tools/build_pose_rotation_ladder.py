#!/usr/bin/env python3
"""Build real-envelope late-burst references for 30/60/90 degree skills."""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import build_forward_rotation_ladder as base  # noqa: E402
from xarm6_toss.control_reference import (  # noqa: E402
    QuinticJointSegment,
    generate_joint_reference,
)
from xarm6_toss.motion_limits import evaluate_reference_samples  # noqa: E402
from xarm6_toss_sim.kinematics import URDFKinematics  # noqa: E402


G1_OPEN_COMMAND_TIME_S = 0.580
MEASURED_COMMAND_TO_DETACH_S = 0.048
REFERENCE_PEAK_TIME_S = (
    G1_OPEN_COMMAND_TIME_S
    + MEASURED_COMMAND_TO_DETACH_S
    - base.TRACKING_DELAY_S
)
PREPOSITION_DURATION_S = 0.380
BURST_DURATION_S = 0.140
DETACH_PLATEAU_DURATION_S = 0.080
BRAKE_DURATION_S = 0.200
REVERSE_VELOCITY_SCALE = -0.49
REVERSE_STOP_DURATION_S = 0.120
MEASURED_ANGULAR_RETENTION = 0.3921103083502193

# This is the mechanically admitted v35 actual-detach pose. The optimized
# maximum uses the verified 1x joint caps and the real 1.6 m/s Cartesian gate.
PEAK_Q_RAD = np.asarray(
    [0.060919, -0.023663, -0.495031, 2.8797932657906435, 1.559916, -0.026179938779914945],
    dtype=float,
)
MAX_RELEASE_QD_RAD_S = np.asarray(
    [0.0, -0.4380367984403463, -1.73611027775, 0.0, 1.73611027775, 0.0],
    dtype=float,
)
SKILLS = (
    ("r30", 30.0, 0.55),
    ("r60", 60.0, 0.78),
    ("r90", 90.0, 1.00),
)


def segment_dict(segment: QuinticJointSegment) -> dict[str, object]:
    return {
        "phase": segment.phase,
        "duration_s": segment.duration_s,
        "start_joint_rad": base.vector(np.asarray(segment.start_joint_rad)),
        "start_joint_velocity_rad_s": base.vector(
            np.asarray(segment.start_joint_velocity_rad_s)
        ),
        "start_joint_acceleration_rad_s2": base.vector(
            np.asarray(segment.start_joint_acceleration_rad_s2)
        ),
        "end_joint_rad": base.vector(np.asarray(segment.end_joint_rad)),
        "end_joint_velocity_rad_s": base.vector(
            np.asarray(segment.end_joint_velocity_rad_s)
        ),
        "end_joint_acceleration_rad_s2": base.vector(
            np.asarray(segment.end_joint_acceleration_rad_s2)
        ),
    }


def build_candidate(
    kinematics: URDFKinematics,
    label: str,
    requested_rotation_deg: float,
    velocity_scale: float,
) -> tuple[Path, dict[str, object]]:
    start_q = np.deg2rad(base.START_DEG)
    release_qd = velocity_scale * MAX_RELEASE_QD_RAD_S
    burst_acceleration = release_qd / BURST_DURATION_S
    preburst_q = PEAK_Q_RAD - 0.5 * release_qd * BURST_DURATION_S
    plateau_end_q = PEAK_Q_RAD + release_qd * DETACH_PLATEAU_DURATION_S
    brake_end_qd = REVERSE_VELOCITY_SCALE * release_qd
    brake_acceleration = (
        brake_end_qd - release_qd
    ) / BRAKE_DURATION_S
    brake_end_q = plateau_end_q + 0.5 * (
        release_qd + brake_end_qd
    ) * BRAKE_DURATION_S
    stop_acceleration = -brake_end_qd / REVERSE_STOP_DURATION_S
    stop_q = brake_end_q + 0.5 * brake_end_qd * REVERSE_STOP_DURATION_S
    zeros = np.zeros(6, dtype=float)
    segments = (
        QuinticJointSegment(
            "preposition", PREPOSITION_DURATION_S,
            tuple(start_q), tuple(zeros), tuple(preburst_q), tuple(zeros),
            tuple(zeros), tuple(zeros),
        ),
        QuinticJointSegment(
            "late_burst", BURST_DURATION_S,
            tuple(preburst_q), tuple(zeros), tuple(PEAK_Q_RAD), tuple(release_qd),
            tuple(burst_acceleration), tuple(burst_acceleration),
        ),
        QuinticJointSegment(
            "detach_plateau", DETACH_PLATEAU_DURATION_S,
            tuple(PEAK_Q_RAD), tuple(release_qd), tuple(plateau_end_q), tuple(release_qd),
            tuple(zeros), tuple(zeros),
        ),
        QuinticJointSegment(
            "post_detach_brake", BRAKE_DURATION_S,
            tuple(plateau_end_q), tuple(release_qd), tuple(brake_end_q),
            tuple(brake_end_qd), tuple(brake_acceleration), tuple(brake_acceleration),
        ),
        QuinticJointSegment(
            "reverse_stop", REVERSE_STOP_DURATION_S,
            tuple(brake_end_q), tuple(brake_end_qd), tuple(stop_q), tuple(zeros),
            tuple(stop_acceleration), tuple(stop_acceleration),
        ),
    )
    samples = generate_joint_reference(segments, base.CONTROL_PERIOD_S)
    limits = evaluate_reference_samples(samples)
    if not limits["joint_mechanical_limits_pass"]:
        raise RuntimeError(f"{label} violates the real transfer envelope: {limits}")
    peak_sample = min(
        samples, key=lambda sample: abs(sample.time_s - REFERENCE_PEAK_TIME_S)
    )
    peak_q = np.asarray(peak_sample.joint_position_rad, dtype=float)
    peak_dq = np.asarray(peak_sample.joint_velocity_rad_s, dtype=float)
    transform = kinematics.forward(peak_q)
    axis = base.tumble_axis(transform)
    twist = kinematics.jacobian(peak_q) @ peak_dq
    axis_omega = float(np.dot(twist[3:], axis))
    tcp_speed = float(np.linalg.norm(twist[:3]))
    expected_cube_omega = MEASURED_ANGULAR_RETENTION * axis_omega
    expected_flight_for_target = math.radians(requested_rotation_deg) / expected_cube_omega

    _, config = base.build_candidate(kinematics, "1p6", 1.6, 0.75, -0.25)
    config.update(
        schema="xarm6_pose_rotation_throwonly_v1",
        name=f"pose_rotation_throwonly_{label}",
        reference_segments=[segment_dict(segment) for segment in segments],
    )
    config["kinematic_design"] = {
        "requested_rotation_deg": requested_rotation_deg,
        "velocity_scale": velocity_scale,
        "reference_peak_time_s": REFERENCE_PEAK_TIME_S,
        "release_joint_velocity_rad_s": base.vector(release_qd),
        "predicted_peak_tcp_position_m": base.vector(transform[:3, 3]),
        "predicted_peak_tcp_velocity_m_s": base.vector(twist[:3]),
        "predicted_peak_tcp_speed_m_s": tcp_speed,
        "predicted_tumble_axis_world": base.vector(axis),
        "predicted_hand_axis_omega_rad_s": axis_omega,
        "measured_g1_angular_retention_prior": MEASURED_ANGULAR_RETENTION,
        "predicted_cube_axis_omega_rad_s": expected_cube_omega,
        "predicted_flight_time_for_target_s": expected_flight_for_target,
        "reference_limit_evidence": limits,
    }
    config["gripper_events"] = [
        {"time_s": G1_OPEN_COMMAND_TIME_S, "name": "release_partial_open", "real_position": 520.0}
    ]
    config["sim_gripper_events"] = [
        {"time_s": G1_OPEN_COMMAND_TIME_S, "name": "release_partial_open", "drive_rad": 0.39}
    ]
    config["real_g1_events"] = [
        {"time_s": G1_OPEN_COMMAND_TIME_S, "name": "release_partial_open", "position": 520.0}
    ]
    config["validation_targets"] = {
        "requested_rotation_deg": requested_rotation_deg,
        "maximum_rotation_error_deg": 12.0,
        "minimum_continuous_free_flight_s": 0.12,
        "minimum_relative_separation_m": 0.025,
        "minimum_axis_alignment": 0.85,
    }
    config["lineage"] = {
        "goal": "goal.md v5",
        "release_transfer_prior": "docs/media/j5_forward_rotation/release_transfer_v49.json",
        "peak_pose_source": "v35_1p6_no_wrist_camera_oriented_ground actual detach",
        "generator": "sim/tools/build_pose_rotation_ladder.py",
    }
    return base.OUTPUT_DIR / f"pose_rotation_throwonly_{label}.json", config


def main() -> int:
    kinematics = URDFKinematics(base.URDF)
    for spec in SKILLS:
        path, config = build_candidate(kinematics, *spec)
        path.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
        design = config["kinematic_design"]
        print(
            f"{path}: hand_omega={design['predicted_hand_axis_omega_rad_s']:.3f}, "
            f"tcp_speed={design['predicted_peak_tcp_speed_m_s']:.3f}, "
            f"flight_prior={design['predicted_flight_time_for_target_s']:.3f}, "
            f"limits_pass={design['reference_limit_evidence']['joint_mechanical_limits_pass']}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
