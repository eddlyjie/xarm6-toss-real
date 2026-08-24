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
CONTACT_ALIGNED_DETACH_PLATEAU_DURATION_S = 0.060
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
# At the measured v62 detach pose this keeps forward hand omega near
# 3.28 rad/s while cancelling the cube-COM outward velocity induced by
# omega cross the 137.6 mm hand-to-cube offset.  J4/J6 remain static.
CATCHABLE_RELEASE_QD_RAD_S = np.asarray(
    [0.0, -0.897343, -1.395868, 0.0, 1.096753, 0.0],
    dtype=float,
)
FAST_CATCHABLE_RELEASE_QD_RAD_S = np.asarray(
    [0.0, -0.419, -1.495, 0.0, 1.744, 0.0],
    dtype=float,
)
CATCH_ALIGNED_RELEASE_QD_RAD_S = np.asarray(
    [0.0, -1.25933, 0.0, 0.0, 1.7448, 0.0],
    dtype=float,
)
MOMENTUM_COMPATIBLE_RELEASE_QD_RAD_S = np.asarray(
    [0.0, 0.235124, -0.600000, 0.0, 1.744800, 0.0],
    dtype=float,
)
UPWARD_MOMENTUM_RELEASE_QD_RAD_S = np.asarray(
    [0.0, -0.111191, -0.600000, 0.0, 1.744800, 0.0],
    dtype=float,
)
TWENTY_DEG_UPWARD_RELEASE_QD_RAD_S = np.asarray(
    [0.0, -0.456295, -0.600000, 0.0, 1.744800, 0.0],
    dtype=float,
)
TEN_DEG_POSE_CONDITIONED_RELEASE_QD_RAD_S = np.asarray(
    [0.0, -0.2908, -1.7448, 0.0, 1.1632, 0.0],
    dtype=float,
)
TEN_DEG_POSE_CONDITIONED_OFFSET_RAD = np.asarray(
    [0.0, -0.190, 0.050, 0.0, 0.0, 0.0],
    dtype=float,
)
TEN_DEG_HIGH_RELEASE_OFFSET_RAD = np.asarray(
    [0.0, -0.190, -0.070, 0.0, 0.0, 0.0],
    dtype=float,
)
FIFTEEN_DEG_POSE_OPTIMIZED_RELEASE_QD_RAD_S = np.asarray(
    [0.0, -0.4105, -1.7448, 0.0, 1.7448, 0.0],
    dtype=float,
)
FIFTEEN_DEG_POSE_OFFSET_RAD = np.asarray(
    [0.0, -0.190, 0.050, 0.0, 0.0, 0.0],
    dtype=float,
)
RESIDUAL_COMPENSATED_RELEASE_QD_RAD_S = np.asarray(
    [0.0, -0.942, -1.7448, 0.0, -0.174, 0.0],
    dtype=float,
)
TRACKING_COMPENSATED_RELEASE_QD_RAD_S = np.asarray(
    [0.0, 0.0, -1.605, 0.0, 1.7448, 0.0],
    dtype=float,
)
# Sim's controller hand body is not the URDF FK terminal frame.  Use the
# measured v62 world-frame hand-body-to-cube offset at detach for the COM
# velocity model; the wrist branch and grasp are frozen for this search.
MEASURED_CUBE_OFFSET_WORLD_M = np.asarray(
    [0.11830688, 0.04275623, 0.05539122], dtype=float
)
MEASURED_V62_DETACH_Q_RAD = np.asarray(
    [0.06064348, 0.00463218, -0.48111078, 2.87927103, 1.55711591, -0.02615020],
    dtype=float,
)
HIGH_RELEASE_JOINT_OFFSET_RAD = np.asarray(
    [0.0, -0.11, -0.11, 0.0, 0.0, 0.0],
    dtype=float,
)
TWENTY_DEG_RELEASE_JOINT_OFFSET_RAD = np.asarray(
    [0.0, -0.20, -0.20, 0.0, 0.0, 0.0], dtype=float
)
SKILLS = (
    ("r30", 30.0, 0.55, DETACH_PLATEAU_DURATION_S),
    ("r60", 60.0, 0.78, DETACH_PLATEAU_DURATION_S),
    ("r90", 90.0, 1.00, DETACH_PLATEAU_DURATION_S),
    ("r90_brake20", 90.0, 1.00, CONTACT_ALIGNED_DETACH_PLATEAU_DURATION_S),
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
    detach_plateau_duration_s: float = DETACH_PLATEAU_DURATION_S,
    release_qd_override: np.ndarray | None = None,
    joint_position_offset_rad: np.ndarray | None = None,
    preposition_duration_s: float = PREPOSITION_DURATION_S,
    burst_duration_s: float = BURST_DURATION_S,
    contact_burst_duration_s: float = 0.0,
    contact_burst_start_velocity_scale: float = 1.0,
) -> tuple[Path, dict[str, object]]:
    joint_position_offset = (
        np.zeros(6, dtype=float)
        if joint_position_offset_rad is None
        else np.asarray(joint_position_offset_rad, dtype=float)
    )
    start_q = np.deg2rad(base.START_DEG) + joint_position_offset
    peak_q = PEAK_Q_RAD + joint_position_offset
    # The deployable real-robot policy has exactly three dynamic arm joints.
    # Preserve the measured pre-throw values of J1/J4/J6 for every phase.
    fixed_joint_indices = np.asarray([0, 3, 5], dtype=int)
    peak_q[fixed_joint_indices] = start_q[fixed_joint_indices]
    release_qd = (
        velocity_scale * MAX_RELEASE_QD_RAD_S
        if release_qd_override is None
        else np.asarray(release_qd_override, dtype=float)
    )
    if not np.allclose(release_qd[fixed_joint_indices], 0.0, atol=0.0):
        raise ValueError("J1/J4/J6 release velocities must remain exactly zero")

    mid_qd = (
        contact_burst_start_velocity_scale * release_qd
        if contact_burst_duration_s > 0.0
        else release_qd
    )
    burst_acceleration = mid_qd / burst_duration_s
    burst_end_q = (
        peak_q - 0.5 * (mid_qd + release_qd) * contact_burst_duration_s
        if contact_burst_duration_s > 0.0
        else peak_q
    )
    preburst_q = burst_end_q - 0.5 * mid_qd * burst_duration_s
    plateau_end_q = peak_q + release_qd * detach_plateau_duration_s
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
    contact_burst_segments = (
        (
            QuinticJointSegment(
                "contact_burst", contact_burst_duration_s,
                tuple(burst_end_q), tuple(mid_qd), tuple(peak_q), tuple(release_qd),
                tuple((release_qd - mid_qd) / contact_burst_duration_s),
                tuple((release_qd - mid_qd) / contact_burst_duration_s),
            ),
        )
        if contact_burst_duration_s > 0.0
        else ()
    )
    segments = (
        QuinticJointSegment(
            "preposition", preposition_duration_s,
            tuple(start_q), tuple(zeros), tuple(preburst_q), tuple(zeros),
            tuple(zeros), tuple(zeros),
        ),
        QuinticJointSegment(
            "late_burst", burst_duration_s,
            tuple(preburst_q), tuple(zeros), tuple(burst_end_q), tuple(mid_qd),
            tuple(burst_acceleration), tuple(burst_acceleration),
        ),
        *contact_burst_segments,
        QuinticJointSegment(
            "detach_plateau", detach_plateau_duration_s,
            tuple(peak_q), tuple(release_qd), tuple(plateau_end_q), tuple(release_qd),
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
    reference_evaluation_time_s = max(
        REFERENCE_PEAK_TIME_S,
        preposition_duration_s + burst_duration_s + contact_burst_duration_s,
    )
    peak_sample = min(
        samples, key=lambda sample: abs(sample.time_s - reference_evaluation_time_s)
    )
    reference_peak_q = np.asarray(peak_sample.joint_position_rad, dtype=float)
    peak_dq = np.asarray(peak_sample.joint_velocity_rad_s, dtype=float)
    design_q = (
        reference_peak_q
        if release_qd_override is None
        else MEASURED_V62_DETACH_Q_RAD + joint_position_offset
    )
    transform = kinematics.forward(design_q)
    axis = base.tumble_axis(transform)
    twist = kinematics.jacobian(design_q) @ peak_dq
    axis_omega = float(np.dot(twist[3:], axis))
    tcp_speed = float(np.linalg.norm(twist[:3]))
    expected_cube_omega = MEASURED_ANGULAR_RETENTION * axis_omega
    predicted_cube_com_velocity = (
        twist[:3] + np.cross(twist[3:], MEASURED_CUBE_OFFSET_WORLD_M)
    )
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
        "reference_peak_time_s": reference_evaluation_time_s,
        "burst_duration_s": burst_duration_s,
        "contact_burst_duration_s": contact_burst_duration_s,
        "contact_burst_start_velocity_scale": contact_burst_start_velocity_scale,
        "detach_plateau_duration_s": detach_plateau_duration_s,
        "contact_phase_brake_advance_s": DETACH_PLATEAU_DURATION_S - detach_plateau_duration_s,
        "release_joint_velocity_rad_s": base.vector(release_qd),
        "kinematic_evaluation_state": (
            "reference_peak"
            if release_qd_override is None
            else "measured_v62_actual_detach_q"
        ),
        "joint_position_offset_rad": base.vector(joint_position_offset),
        "predicted_peak_tcp_position_m": base.vector(transform[:3, 3]),
        "predicted_peak_tcp_velocity_m_s": base.vector(twist[:3]),
        "predicted_peak_tcp_speed_m_s": tcp_speed,
        "predicted_tumble_axis_world": base.vector(axis),
        "predicted_cube_com_velocity_m_s": base.vector(
            predicted_cube_com_velocity
        ),
        "predicted_hand_axis_omega_rad_s": axis_omega,
        "measured_g1_angular_retention_prior": MEASURED_ANGULAR_RETENTION,
        "predicted_cube_axis_omega_rad_s": expected_cube_omega,
        "predicted_flight_time_for_target_s": expected_flight_for_target,
        "reference_limit_evidence": limits,
    }
    if preposition_duration_s != PREPOSITION_DURATION_S:
        config["kinematic_design"]["preposition_duration_s"] = (
            preposition_duration_s
        )
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
        "coordination": "20 ms brake advance only for r90_brake20",
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
    path, config = build_candidate(
        kinematics,
        "r10c",
        10.0,
        1.0,
        CONTACT_ALIGNED_DETACH_PLATEAU_DURATION_S,
        CATCHABLE_RELEASE_QD_RAD_S,
    )
    config["lineage"]["goal"] = "goal.md v6"
    config["lineage"]["coordination"] = "low-outward stock-G1 catchable reference"
    path.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
    path, config = build_candidate(
        kinematics,
        "r10ch",
        10.0,
        1.0,
        CONTACT_ALIGNED_DETACH_PLATEAU_DURATION_S,
        CATCHABLE_RELEASE_QD_RAD_S,
        HIGH_RELEASE_JOINT_OFFSET_RAD,
    )
    config["lineage"]["goal"] = "goal.md v6"
    config["lineage"]["coordination"] = (
        "118 mm higher low-outward stock-G1 catchable reference"
    )
    path.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
    path, config = build_candidate(
        kinematics,
        "r10cf",
        10.0,
        1.0,
        CONTACT_ALIGNED_DETACH_PLATEAU_DURATION_S,
        FAST_CATCHABLE_RELEASE_QD_RAD_S,
    )
    config["lineage"]["goal"] = "goal.md v6"
    config["lineage"]["coordination"] = (
        "joint-limit fast low-outward stock-G1 catchable reference"
    )
    path.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
    path, config = build_candidate(
        kinematics,
        "r10cfh",
        10.0,
        1.0,
        CONTACT_ALIGNED_DETACH_PLATEAU_DURATION_S,
        FAST_CATCHABLE_RELEASE_QD_RAD_S,
        HIGH_RELEASE_JOINT_OFFSET_RAD,
    )
    config["lineage"]["goal"] = "goal.md v6"
    config["lineage"]["coordination"] = (
        "118 mm higher joint-limit fast stock-G1 catchable reference"
    )
    path.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
    path, config = build_candidate(
        kinematics,
        "r10cfp",
        10.0,
        1.0,
        0.120,
        FAST_CATCHABLE_RELEASE_QD_RAD_S,
        HIGH_RELEASE_JOINT_OFFSET_RAD,
        0.320,
    )
    config["lineage"]["goal"] = "goal.md v6"
    config["lineage"]["coordination"] = (
        "60 ms earlier burst with an extended constant-velocity release plateau"
    )
    path.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
    path, config = build_candidate(
        kinematics,
        "r10cfa",
        10.0,
        1.0,
        0.120,
        CATCH_ALIGNED_RELEASE_QD_RAD_S,
        HIGH_RELEASE_JOINT_OFFSET_RAD,
        0.320,
    )
    config["lineage"]["goal"] = "goal.md v6"
    config["lineage"]["coordination"] = (
        "q3-zero catch-aligned release with an extended velocity plateau"
    )
    path.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
    path, config = build_candidate(
        kinematics,
        "r10cfm",
        10.0,
        1.0,
        0.120,
        MOMENTUM_COMPATIBLE_RELEASE_QD_RAD_S,
        HIGH_RELEASE_JOINT_OFFSET_RAD,
        0.320,
    )
    config["lineage"]["goal"] = "goal.md v6"
    config["lineage"]["coordination"] = (
        "bounded-q3 release optimized for 0.30 m/s horizontal cube speed"
    )
    path.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
    path, config = build_candidate(
        kinematics,
        "r10cfu",
        10.0,
        1.0,
        0.120,
        UPWARD_MOMENTUM_RELEASE_QD_RAD_S,
        HIGH_RELEASE_JOINT_OFFSET_RAD,
        0.320,
    )
    config["lineage"]["goal"] = "goal.md v6"
    config["lineage"]["coordination"] = (
        "bounded-q3 release optimized for upward velocity at 0.40 m/s horizontal cap"
    )
    path.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
    path, config = build_candidate(
        kinematics,
        "r20cfu",
        20.0,
        1.0,
        0.120,
        TWENTY_DEG_UPWARD_RELEASE_QD_RAD_S,
        TWENTY_DEG_RELEASE_JOINT_OFFSET_RAD,
        0.380,
    )
    config["lineage"]["goal"] = "goal.md v6"
    config["lineage"]["coordination"] = (
        "high-release bounded-q3 action optimized at a 0.50 m/s horizontal cap"
    )
    path.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
    path, config = build_candidate(
        kinematics,
        "r20cfa",
        20.0,
        1.0,
        0.120,
        TWENTY_DEG_UPWARD_RELEASE_QD_RAD_S,
        TWENTY_DEG_RELEASE_JOINT_OFFSET_RAD,
        0.440,
    )
    config["lineage"]["goal"] = "goal.md v6"
    config["lineage"]["coordination"] = (
        "physical detach aligned to the end of the late-burst acceleration"
    )
    path.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
    path, config = build_candidate(
        kinematics,
        "r20cfb",
        20.0,
        1.0,
        0.120,
        TWENTY_DEG_UPWARD_RELEASE_QD_RAD_S,
        TWENTY_DEG_RELEASE_JOINT_OFFSET_RAD,
        0.420,
        0.160,
    )
    config["lineage"]["goal"] = "goal.md v6"
    config["lineage"]["coordination"] = (
        "high-pose late burst ending at the physical-detach tracking phase"
    )
    path.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
    path, config = build_candidate(
        kinematics,
        "r20cfd",
        20.0,
        1.0,
        0.120,
        TWENTY_DEG_UPWARD_RELEASE_QD_RAD_S,
        TWENTY_DEG_RELEASE_JOINT_OFFSET_RAD,
        0.380,
        0.140,
        0.060,
        0.85,
    )
    config["lineage"]["goal"] = "goal.md v6"
    config["lineage"]["coordination"] = (
        "85-to-100-percent contact-window acceleration at the high release pose"
    )
    path.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
    path, config = build_candidate(
        kinematics,
        "r20fh",
        20.0,
        1.0,
        0.020,
        FAST_CATCHABLE_RELEASE_QD_RAD_S,
        TWENTY_DEG_RELEASE_JOINT_OFFSET_RAD,
    )
    config["lineage"]["goal"] = "goal.md v6"
    config["lineage"]["coordination"] = (
        "extra-high joint-limit fast stock-G1 20-degree reference"
    )
    path.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
    path, config = build_candidate(
        kinematics,
        "r10pc",
        10.0,
        1.0,
        CONTACT_ALIGNED_DETACH_PLATEAU_DURATION_S,
        TEN_DEG_POSE_CONDITIONED_RELEASE_QD_RAD_S,
        TEN_DEG_POSE_CONDITIONED_OFFSET_RAD,
    )
    config["lineage"]["goal"] = "goal.md 2026-08-24"
    config["lineage"]["coordination"] = (
        "10-degree selection from calibrated pose-conditioned J2/J3/J5 search"
    )
    config["kinematic_design"]["search_target_rotation_deg"] = 10.0
    config["kinematic_design"]["search_report"] = (
        "real_handoff/open_loop_cube/pose_conditioned_j235_search.json"
    )
    path.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
    path, config = build_candidate(
        kinematics,
        "r10air",
        10.0,
        1.0,
        CONTACT_ALIGNED_DETACH_PLATEAU_DURATION_S,
        TEN_DEG_POSE_CONDITIONED_RELEASE_QD_RAD_S,
        TEN_DEG_HIGH_RELEASE_OFFSET_RAD,
    )
    config["lineage"]["goal"] = "goal.md 2026-08-24"
    config["lineage"]["coordination"] = (
        "high-release J2/J3/J5 action selected for 10-degree tumble and "
        "additional catch height"
    )
    config["kinematic_design"]["search_target_rotation_deg"] = 10.0
    config["kinematic_design"]["design_hypothesis"] = (
        "raise the 10-degree intercept without increasing joint speed"
    )
    config["kinematic_design"]["search_report"] = (
        "real_handoff/open_loop_cube/pose_conditioned_j235_search.json"
    )
    path.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
    path, config = build_candidate(
        kinematics,
        "r12sep",
        12.0,
        1.0,
        CONTACT_ALIGNED_DETACH_PLATEAU_DURATION_S,
        FAST_CATCHABLE_RELEASE_QD_RAD_S,
        FIFTEEN_DEG_POSE_OFFSET_RAD,
    )
    config["lineage"]["goal"] = "goal.md 2026-08-24"
    config["lineage"]["coordination"] = (
        "r15 finger-clearance release pose with the lower-drift J2/J3/J5 "
        "velocity previously validated by the stock-G1 catch"
    )
    config["kinematic_design"]["design_hypothesis"] = (
        "preserve natural post-detach separation while reducing outward cube drift"
    )
    config["kinematic_design"]["measured_reference_family"] = (
        "cube8g_stock_g1_10deg_j235_centered"
    )
    path.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
    path, config = build_candidate(
        kinematics,
        "r15opt",
        15.0,
        1.0,
        CONTACT_ALIGNED_DETACH_PLATEAU_DURATION_S,
        FIFTEEN_DEG_POSE_OPTIMIZED_RELEASE_QD_RAD_S,
        FIFTEEN_DEG_POSE_OFFSET_RAD,
    )
    config["lineage"]["goal"] = "goal.md 2026-08-24"
    config["lineage"]["coordination"] = (
        "J2/J3/J5 release-pose search maximizing forward tumble while "
        "constraining TCP speed and cube-COM lateral velocity"
    )
    config["kinematic_design"]["search_target_rotation_deg"] = 15.0
    config["kinematic_design"]["search_measured_baseline_rotation_deg"] = 7.865743678202105
    path.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
    path, config = build_candidate(
        kinematics,
        "r10cx",
        10.0,
        1.0,
        CONTACT_ALIGNED_DETACH_PLATEAU_DURATION_S,
        RESIDUAL_COMPENSATED_RELEASE_QD_RAD_S,
    )
    config["lineage"]["goal"] = "goal.md v6"
    config["lineage"]["coordination"] = (
        "native-residual-compensated low-outward stock-G1 reference"
    )
    path.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
    path, config = build_candidate(
        kinematics,
        "r10cy",
        10.0,
        1.0,
        CONTACT_ALIGNED_DETACH_PLATEAU_DURATION_S,
        TRACKING_COMPENSATED_RELEASE_QD_RAD_S,
    )
    config["lineage"]["goal"] = "goal.md v6"
    config["lineage"]["coordination"] = (
        "three-native-run tracking-compensated stock-G1 reference"
    )
    path.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
