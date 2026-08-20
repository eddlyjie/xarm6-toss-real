#!/usr/bin/env python3
"""Build the fixed J4/J6, J2/J3/J5 forward-rotation throw ladder."""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from xarm6_toss.control_reference import (  # noqa: E402
    QuinticJointSegment,
    generate_joint_reference,
)
from xarm6_toss.motion_limits import evaluate_reference_samples  # noqa: E402
from xarm6_toss_sim.kinematics import URDFKinematics  # noqa: E402


URDF = ROOT / "sim/assets/xarm6_g1/xarm6_g1.urdf"
OUTPUT_DIR = ROOT / "sim/configs"
CONTROL_PERIOD_S = 0.020
TRACKING_DELAY_S = 0.080
SIM_OPEN_COMMAND_TIME_S = 0.612
REAL_OPEN_COMMAND_TIME_S = 0.585
DETACH_DELAY_PRIOR_S = 0.035
DETACH_REFERENCE_TIME_S = (
    REAL_OPEN_COMMAND_TIME_S + DETACH_DELAY_PRIOR_S - TRACKING_DELAY_S
)
ACCEL_DURATION_S = 0.460
FLICK_DURATION_S = 0.100
BRAKE_DURATION_S = 0.140
REVERSE_VELOCITY_SCALE = -0.38
RETRACT_STOP_DURATION_S = 0.140
BRAKE_START_REFERENCE_TIME_S = ACCEL_DURATION_S + FLICK_DURATION_S
DETACH_BRAKE_ELAPSED_S = (
    DETACH_REFERENCE_TIME_S - BRAKE_START_REFERENCE_TIME_S
)
if DETACH_BRAKE_ELAPSED_S <= 0.0:
    DETACH_VELOCITY_SCALE = 1.0
elif DETACH_BRAKE_ELAPSED_S < BRAKE_DURATION_S:
    DETACH_VELOCITY_SCALE = 1.0 + (
        (REVERSE_VELOCITY_SCALE - 1.0)
        * DETACH_BRAKE_ELAPSED_S
        / BRAKE_DURATION_S
    )
else:
    raise RuntimeError("predicted detach must precede the end of braking")

# The real UI/user-observed seed, with J4 pulled back from 175.4 to 165 degrees
# to retain the required handoff margin. J6 keeps the camera housing/cable below.
START_DEG = np.asarray([3.5, 9.8, -25.7, 165.0, 82.5, -1.5], dtype=float)

# label, desired target-axis omega, desired upward TCP speed, fixed J3 velocity.
# J2 and J5 are solved at the predicted physical-detach configuration.
LADDER = (
    ("0p8", 0.8, 0.40, -0.15),
    ("1p2", 1.2, 0.60, -0.20),
    ("1p6", 1.6, 0.75, -0.25),
    ("2p0", 2.0, 0.75, -0.25),
)


def tumble_axis(transform: np.ndarray) -> np.ndarray:
    finger_direction = transform[:3, 2]
    axis = np.cross(finger_direction, np.asarray([0.0, 0.0, 1.0]))
    norm = float(np.linalg.norm(axis))
    if norm <= 1.0e-6:
        raise RuntimeError("finger direction does not define a horizontal tumble axis")
    return axis / norm


def solve_detach_velocity(
    kinematics: URDFKinematics,
    start_q: np.ndarray,
    target_omega_rad_s: float,
    target_upward_m_s: float,
    joint3_velocity_rad_s: float,
) -> np.ndarray:
    # Solve the peak velocity before braking.  At predicted physical detach the
    # command is already decelerating, so the requested detach twist is divided
    # by the known linear brake velocity scale.
    velocity = np.zeros(6, dtype=float)
    velocity[2] = joint3_velocity_rad_s / DETACH_VELOCITY_SCALE
    brake_accel_scale = (
        REVERSE_VELOCITY_SCALE - 1.0
    ) / BRAKE_DURATION_S
    if DETACH_BRAKE_ELAPSED_S <= 0.0:
        detach_lead_s = 0.5 * ACCEL_DURATION_S + (
            DETACH_REFERENCE_TIME_S - ACCEL_DURATION_S
        )
    else:
        detach_lead_s = (
            0.5 * ACCEL_DURATION_S
            + FLICK_DURATION_S
            + DETACH_BRAKE_ELAPSED_S
            + 0.5 * brake_accel_scale * DETACH_BRAKE_ELAPSED_S**2
        )
    if detach_lead_s <= 0.0:
        raise RuntimeError("detach must occur after the acceleration segment midpoint")
    for _ in range(20):
        detach_q = start_q + detach_lead_s * velocity
        transform = kinematics.forward(detach_q)
        axis = tumble_axis(transform)
        jacobian = kinematics.jacobian(detach_q)
        system = np.asarray(
            [
                [jacobian[2, 1], jacobian[2, 4]],
                [
                    float(np.dot(jacobian[3:, 1], axis)),
                    float(np.dot(jacobian[3:, 4], axis)),
                ],
            ]
        )
        target = np.asarray(
            [
                target_upward_m_s / DETACH_VELOCITY_SCALE
                - jacobian[2, 2] * velocity[2],
                target_omega_rad_s / DETACH_VELOCITY_SCALE
                - float(np.dot(jacobian[3:, 2], axis)) * velocity[2],
            ]
        )
        velocity[[1, 4]] = np.linalg.solve(system, target)
    if not (
        velocity[1] < 0.0
        and velocity[2] < 0.0
        and velocity[4] > 0.0
        and velocity[3] == 0.0
        and velocity[5] == 0.0
    ):
        raise RuntimeError(f"unexpected J2/J3/J5 solution: {velocity.tolist()}")
    return velocity


def vector(values: np.ndarray) -> list[float]:
    return [float(value) for value in values]


def build_candidate(
    kinematics: URDFKinematics,
    label: str,
    target_omega_rad_s: float,
    target_upward_m_s: float,
    joint3_velocity_rad_s: float,
) -> tuple[Path, dict[str, object]]:
    start_q = np.deg2rad(START_DEG)
    release_velocity = solve_detach_velocity(
        kinematics,
        start_q,
        target_omega_rad_s,
        target_upward_m_s,
        joint3_velocity_rad_s,
    )
    accel = release_velocity / ACCEL_DURATION_S
    accel_end_q = start_q + 0.5 * release_velocity * ACCEL_DURATION_S
    flick_end_q = accel_end_q + release_velocity * FLICK_DURATION_S
    brake_end_velocity = REVERSE_VELOCITY_SCALE * release_velocity
    brake_accel = (
        brake_end_velocity - release_velocity
    ) / BRAKE_DURATION_S
    brake_end_q = flick_end_q + 0.5 * (
        release_velocity + brake_end_velocity
    ) * BRAKE_DURATION_S
    retract_stop_accel = -brake_end_velocity / RETRACT_STOP_DURATION_S
    retract_end_q = brake_end_q + 0.5 * (
        brake_end_velocity * RETRACT_STOP_DURATION_S
    )
    zeros = np.zeros(6, dtype=float)
    segments = (
        QuinticJointSegment(
            "throw_accel",
            ACCEL_DURATION_S,
            tuple(start_q),
            tuple(zeros),
            tuple(accel_end_q),
            tuple(release_velocity),
            tuple(accel),
            tuple(accel),
        ),
        QuinticJointSegment(
            "detach_flick",
            FLICK_DURATION_S,
            tuple(accel_end_q),
            tuple(release_velocity),
            tuple(flick_end_q),
            tuple(release_velocity),
            tuple(zeros),
            tuple(zeros),
        ),
        QuinticJointSegment(
            "post_detach_brake",
            BRAKE_DURATION_S,
            tuple(flick_end_q),
            tuple(release_velocity),
            tuple(brake_end_q),
            tuple(brake_end_velocity),
            tuple(brake_accel),
            tuple(brake_accel),
        ),
        QuinticJointSegment(
            "reverse_retract_stop",
            RETRACT_STOP_DURATION_S,
            tuple(brake_end_q),
            tuple(brake_end_velocity),
            tuple(retract_end_q),
            tuple(zeros),
            tuple(retract_stop_accel),
            tuple(retract_stop_accel),
        ),
    )
    samples = generate_joint_reference(segments, CONTROL_PERIOD_S)
    limits = evaluate_reference_samples(samples)
    if not limits["joint_mechanical_limits_pass"]:
        raise RuntimeError(f"{label} reference violates transfer limits: {limits}")
    detach_sample = min(
        samples, key=lambda sample: abs(sample.time_s - DETACH_REFERENCE_TIME_S)
    )
    detach_q = np.asarray(detach_sample.joint_position_rad, dtype=float)
    detach_dq = np.asarray(detach_sample.joint_velocity_rad_s, dtype=float)
    detach_transform = kinematics.forward(detach_q)
    axis = tumble_axis(detach_transform)
    detach_twist = kinematics.jacobian(detach_q) @ detach_dq
    angular = detach_twist[3:]
    axis_projection = float(np.dot(angular, axis))
    alignment = abs(axis_projection) / float(np.linalg.norm(angular))
    if abs(axis_projection - target_omega_rad_s) > 0.03 or alignment < 0.90:
        raise RuntimeError(
            f"{label} detach twist mismatch: projection={axis_projection}, alignment={alignment}"
        )

    def segment_dict(segment: QuinticJointSegment) -> dict[str, object]:
        return {
            "phase": segment.phase,
            "duration_s": segment.duration_s,
            "start_joint_rad": vector(np.asarray(segment.start_joint_rad)),
            "start_joint_velocity_rad_s": vector(
                np.asarray(segment.start_joint_velocity_rad_s)
            ),
            "start_joint_acceleration_rad_s2": vector(
                np.asarray(segment.start_joint_acceleration_rad_s2)
            ),
            "end_joint_rad": vector(np.asarray(segment.end_joint_rad)),
            "end_joint_velocity_rad_s": vector(
                np.asarray(segment.end_joint_velocity_rad_s)
            ),
            "end_joint_acceleration_rad_s2": vector(
                np.asarray(segment.end_joint_acceleration_rad_s2)
            ),
        }

    config = {
        "schema": "xarm6_g1_j5_forward_rotation_throwonly_v1",
        "name": f"j5_forward_rotation_throwonly_{label}",
        "physics_dt_s": 0.001,
        "control_period_s": CONTROL_PERIOD_S,
        "measured_real_arm_tracking_delay_s": TRACKING_DELAY_S,
        "execution_envelope": "verified_1x_joint_command_envelope_empty_run_required",
        "operator_approval_required_for_real_execution": True,
        "limits": {
            "joint_speed_rad_s": 1.74483445,
            "joint_acceleration_rad_s2": 13.0573925,
            "max_joint_step_rad": 0.0348967,
            "max_qdot_change_rad_s": 0.261148,
            "minimum_joint_margin_rad": 0.15,
        },
        "cube_physics": {
            "side_length_m_range": [0.035, 0.040],
            "mass_kg_range": [0.020, 0.050],
            "contact_offset_m": 0.0002,
            "static_friction": 1.2,
            "dynamic_friction": 0.9,
        },
        "gripper_sim": {
            "effort_limit_n": 4.0,
            "stiffness_n_m": 60.0,
            "damping_n_s_m": 5.0,
            "max_depenetration_velocity_m_s": 0.5,
            "contact_offset_m": 0.0002,
        },
        "wrist_branch": {
            "user_seed_joint_deg": vector(START_DEG),
            "joint4_static_deg": float(START_DEG[3]),
            "joint6_static_deg": float(START_DEG[5]),
            "joint4_role": "static_left_right_branch_only",
            "joint6_role": "static_camera_housing_underneath_only",
        },
        "kinematic_design": {
            "target_tumble_omega_rad_s": target_omega_rad_s,
            "target_upward_tcp_velocity_m_s": target_upward_m_s,
            "predicted_physical_detach_reference_time_s": DETACH_REFERENCE_TIME_S,
            "release_joint_velocity_rad_s": vector(release_velocity),
            "predicted_detach_tcp_position_m": vector(detach_transform[:3, 3]),
            "predicted_tumble_axis_world": vector(axis),
            "predicted_detach_twist": vector(detach_twist),
            "predicted_axis_projection_rad_s": axis_projection,
            "predicted_axis_alignment": alignment,
            "reference_limit_evidence": limits,
        },
        "reference_segments": [segment_dict(segment) for segment in segments],
        "gripper_events": [
            {"time_s": REAL_OPEN_COMMAND_TIME_S, "name": "release_partial_open", "real_position": 520.0}
        ],
        "sim_gripper_events": [
            {"time_s": SIM_OPEN_COMMAND_TIME_S, "name": "release_partial_open", "drive_rad": 0.39}
        ],
        "real_g1_events": [
            {"time_s": REAL_OPEN_COMMAND_TIME_S, "name": "release_partial_open", "position": 520.0}
        ],
        "state_estimator": {
            "mode": "actual_q_dq_release_prior_plus_ballistic_propagation",
            "camera_required_for_control": False,
            "detach_delay_range_s": [0.025, 0.044],
        },
        "validation_targets": {
            "minimum_continuous_free_flight_s": 0.12,
            "minimum_relative_separation_m": 0.025,
            "minimum_axis_alignment": 0.85,
            "minimum_throwonly_tumble_deg": 12.0,
        },
        "lineage": {
            "frozen_real_baseline": "REAL_ROBOT_TEST_20260817.md",
            "goal": "goal.md v4",
            "generator": "sim/tools/build_forward_rotation_ladder.py",
        },
    }
    path = OUTPUT_DIR / f"j5_forward_rotation_throwonly_{label}.json"
    return path, config


def main() -> int:
    kinematics = URDFKinematics(URDF)
    for spec in LADDER:
        path, config = build_candidate(kinematics, *spec)
        path.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
        design = config["kinematic_design"]
        print(
            f"{path}: omega={design['predicted_axis_projection_rad_s']:.3f} rad/s, "
            f"alignment={design['predicted_axis_alignment']:.3f}, "
            f"limits_pass={design['reference_limit_evidence']['joint_mechanical_limits_pass']}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
