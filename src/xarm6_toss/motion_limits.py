"""Mechanical transfer limits shared by the xArm6 simulator and handoff."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

import numpy as np


JOINT_LOWER_RAD = np.asarray(
    [-3.14, -1.92, -3.927, -3.14, -1.69297, -np.pi], dtype=float
)
JOINT_UPPER_RAD = np.asarray(
    [3.14, 2.0944, 0.19198, 3.14, np.pi, np.pi], dtype=float
)
ARM_EFFORT_LIMIT_NM = np.asarray([50.0, 50.0, 32.0, 32.0, 32.0, 20.0])
TRANSFER_MAX_JOINT_SPEED_RAD_S = 1.74483445
TRANSFER_MAX_JOINT_ACCELERATION_RAD_S2 = 13.0573925
TRANSFER_CONTROL_PERIOD_S = 0.020
TRANSFER_MAX_JOINT_STEP_RAD = 0.0348967
TRANSFER_MAX_QDOT_CHANGE_RAD_S = 0.261148
HANDOFF_MIN_JOINT_MARGIN_RAD = 0.15


def _matrix(values: Sequence[Sequence[float]], name: str) -> np.ndarray:
    matrix = np.asarray(values, dtype=float)
    if matrix.ndim != 2 or matrix.shape[1] != 6 or matrix.shape[0] == 0:
        raise ValueError(f"{name} must have shape (N, 6) with N > 0")
    if not np.all(np.isfinite(matrix)):
        raise ValueError(f"{name} must contain only finite values")
    return matrix


def evaluate_joint_trajectory(
    positions_rad: Sequence[Sequence[float]],
    velocities_rad_s: Sequence[Sequence[float]],
    accelerations_rad_s2: Sequence[Sequence[float]],
    *,
    efforts_nm: Sequence[Sequence[float]] | None = None,
) -> dict[str, Any]:
    """Return hard-limit evidence; no value is clipped into the envelope."""

    q = _matrix(positions_rad, "positions_rad")
    dq = _matrix(velocities_rad_s, "velocities_rad_s")
    ddq = _matrix(accelerations_rad_s2, "accelerations_rad_s2")
    if q.shape != dq.shape or q.shape != ddq.shape:
        raise ValueError("q, dq and ddq must have the same shape")

    lower_margin = q - JOINT_LOWER_RAD
    upper_margin = JOINT_UPPER_RAD - q
    per_joint_margin = np.min(np.minimum(lower_margin, upper_margin), axis=0)
    max_speed = float(np.max(np.abs(dq)))
    max_acceleration = float(np.max(np.abs(ddq)))
    max_step = 0.0 if len(q) < 2 else float(np.max(np.abs(np.diff(q, axis=0))))
    max_qdot_change = (
        0.0 if len(dq) < 2 else float(np.max(np.abs(np.diff(dq, axis=0))))
    )

    effort_pass = efforts_nm is None
    peak_effort = None
    if efforts_nm is not None:
        efforts = _matrix(efforts_nm, "efforts_nm")
        if efforts.shape != q.shape:
            raise ValueError("efforts_nm must have the same shape as q")
        peak_effort_array = np.max(np.abs(efforts), axis=0)
        peak_effort = peak_effort_array.tolist()
        effort_pass = bool(np.all(peak_effort_array <= ARM_EFFORT_LIMIT_NM + 1.0e-9))

    position_pass = bool(np.all(per_joint_margin >= -1.0e-9))
    handoff_margin_pass = bool(
        np.all(per_joint_margin >= HANDOFF_MIN_JOINT_MARGIN_RAD - 1.0e-9)
    )
    speed_pass = max_speed <= TRANSFER_MAX_JOINT_SPEED_RAD_S + 1.0e-9
    acceleration_pass = (
        max_acceleration <= TRANSFER_MAX_JOINT_ACCELERATION_RAD_S2 + 1.0e-9
    )
    step_pass = max_step <= TRANSFER_MAX_JOINT_STEP_RAD + 1.0e-9
    qdot_change_pass = (
        max_qdot_change <= TRANSFER_MAX_QDOT_CHANGE_RAD_S + 1.0e-9
    )
    return {
        "joint_hard_bounds_pass": position_pass,
        "handoff_joint_margin_pass": handoff_margin_pass,
        "minimum_joint_margin_rad": float(np.min(per_joint_margin)),
        "per_joint_minimum_margin_rad": per_joint_margin.tolist(),
        "joint_speed_pass": speed_pass,
        "max_joint_speed_rad_s": max_speed,
        "joint_acceleration_pass": acceleration_pass,
        "max_joint_acceleration_rad_s2": max_acceleration,
        "joint_step_pass": step_pass,
        "max_joint_step_rad": max_step,
        "qdot_change_pass": qdot_change_pass,
        "max_qdot_change_rad_s": max_qdot_change,
        "effort_pass": effort_pass,
        "per_joint_peak_effort_nm": peak_effort,
        "joint_mechanical_limits_pass": bool(
            position_pass
            and handoff_margin_pass
            and speed_pass
            and acceleration_pass
            and step_pass
            and qdot_change_pass
            and effort_pass
        ),
    }


def evaluate_reference_samples(samples) -> dict[str, Any]:
    return evaluate_joint_trajectory(
        [sample.joint_position_rad for sample in samples],
        [sample.joint_velocity_rad_s for sample in samples],
        [sample.joint_acceleration_rad_s2 for sample in samples],
    )
