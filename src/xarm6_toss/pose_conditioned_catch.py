"""Offline J2/J3/J5 intercept planning from a calibrated Sim flight.

The planner turns a deterministic detach/flight rollout into a bounded 20 ms
joint reference.  It does not use a camera or object state at execution time.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, Sequence

import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation

from xarm6_toss.motion_limits import (
    HANDOFF_MIN_JOINT_MARGIN_RAD,
    JOINT_LOWER_RAD,
    JOINT_UPPER_RAD,
    TRANSFER_CONTROL_PERIOD_S,
    TRANSFER_MAX_JOINT_ACCELERATION_RAD_S2,
    TRANSFER_MAX_JOINT_SPEED_RAD_S,
    evaluate_joint_trajectory,
)


DYNAMIC_JOINT_INDICES = (1, 2, 4)
FIXED_JOINT_INDICES = (0, 3, 5)


@dataclass(frozen=True)
class CatchPlanSettings:
    control_period_s: float = TRANSFER_CONTROL_PERIOD_S
    maximum_joint_speed_rad_s: float = TRANSFER_MAX_JOINT_SPEED_RAD_S
    maximum_joint_acceleration_rad_s2: float = (
        TRANSFER_MAX_JOINT_ACCELERATION_RAD_S2
    )
    maximum_capture_position_error_m: float = 0.019
    maximum_capture_velocity_error_m_s: float = 0.10
    minimum_preintercept_distance_m: float = 0.022
    minimum_preintercept_vertical_clearance_m: float = 0.008
    preintercept_clearance_start_time_s: float | None = None
    final_approach_duration_s: float = 0.08
    actuator_response_alpha: tuple[float, float, float] | None = None
    terminal_velocity_weight: float = 10_000.0
    acceleration_weight: float = 0.002
    acceleration_smoothness_weight: float = 0.01


def nearest_row(rows: Sequence[dict[str, Any]], time_s: float) -> dict[str, Any]:
    return min(rows, key=lambda row: abs(float(row["time_s"]) - float(time_s)))


def ballistic_continuation(
    rows: Sequence[dict[str, Any]],
    *,
    anchor_time_s: float,
    gravity_m_s2: float = 9.81,
) -> tuple[list[dict[str, Any]], float]:
    """Replace post-anchor object state with contact-free rigid-body flight.

    Failed catch controllers often touch the object before the desired
    intercept and therefore corrupt the remainder of a recorded trajectory.
    This continuation keeps the measured detach state, arm state and clock,
    while propagating only the cube pose/twist without robot contact.
    """

    if len(rows) < 2:
        raise ValueError("ballistic continuation needs at least two rows")
    times = np.asarray([float(row["time_s"]) for row in rows], dtype=float)
    if not np.all(np.isfinite(times)) or np.any(np.diff(times) < 0.0):
        raise ValueError("trajectory times must be finite and nondecreasing")
    if not np.isfinite(anchor_time_s) or not times[0] <= anchor_time_s <= times[-1]:
        raise ValueError("ballistic anchor must lie inside the trajectory")
    if not np.isfinite(gravity_m_s2) or gravity_m_s2 <= 0.0:
        raise ValueError("gravity must be finite and positive")

    anchor_index = int(np.argmin(np.abs(times - float(anchor_time_s))))
    anchor = rows[anchor_index]
    anchor_time = float(anchor["time_s"])
    position = np.asarray(anchor["cube_position_w_m"], dtype=float)
    velocity = np.asarray(anchor["cube_linear_velocity_w_m_s"], dtype=float)
    angular_velocity = np.asarray(
        anchor["cube_angular_velocity_w_rad_s"], dtype=float
    )
    quaternion_wxyz = np.asarray(anchor["cube_quaternion_wxyz"], dtype=float)
    if not np.all(
        np.isfinite(np.r_[position, velocity, angular_velocity, quaternion_wxyz])
    ):
        raise ValueError("ballistic anchor cube state must be finite")
    anchor_rotation = Rotation.from_quat(
        np.r_[quaternion_wxyz[1:], quaternion_wxyz[0]]
    )
    gravity = np.asarray([0.0, 0.0, -float(gravity_m_s2)])

    continued = []
    for source in rows:
        row = dict(source)
        elapsed = float(row["time_s"]) - anchor_time
        if elapsed > 0.0:
            row["cube_position_w_m"] = (
                position + velocity * elapsed + 0.5 * gravity * elapsed**2
            ).tolist()
            row["cube_linear_velocity_w_m_s"] = (
                velocity + gravity * elapsed
            ).tolist()
            rotation = (
                Rotation.from_rotvec(angular_velocity * elapsed) * anchor_rotation
            )
            xyzw = rotation.as_quat()
            row["cube_quaternion_wxyz"] = [
                float(xyzw[3]), *[float(value) for value in xyzw[:3]]
            ]
            row["cube_angular_velocity_w_rad_s"] = angular_velocity.tolist()
        continued.append(row)
    return continued, anchor_time


def infer_local_point(
    rows: Sequence[dict[str, Any]],
    kinematics,
    *,
    point_key: str,
    start_time_s: float,
    end_time_s: float,
) -> np.ndarray:
    """Infer a fixed tool-frame point from measured world-frame samples."""

    samples = []
    for row in rows:
        time_s = float(row["time_s"])
        if start_time_s <= time_s <= end_time_s:
            transform = kinematics.forward(
                np.asarray(row["arm_joint_position_rad"], dtype=float)
            )
            point_w = np.r_[np.asarray(row[point_key], dtype=float), 1.0]
            samples.append((np.linalg.inv(transform) @ point_w)[:3])
    if len(samples) < 3:
        raise ValueError("not enough calibration rows to infer the tool point")
    local = np.median(np.asarray(samples), axis=0)
    residuals = np.linalg.norm(np.asarray(samples) - local, axis=1)
    if float(np.max(residuals)) > 0.002:
        raise ValueError("tool point is inconsistent with the URDF kinematics")
    return local


def point_position(kinematics, joint_rad: np.ndarray, local_point: np.ndarray) -> np.ndarray:
    return (kinematics.forward(joint_rad) @ np.r_[local_point, 1.0])[:3]


def dynamic_point_jacobian(
    kinematics,
    joint_rad: np.ndarray,
    local_point: np.ndarray,
    *,
    epsilon: float = 1.0e-5,
) -> np.ndarray:
    base = point_position(kinematics, joint_rad, local_point)
    jacobian = np.zeros((3, len(DYNAMIC_JOINT_INDICES)), dtype=float)
    for column, joint_index in enumerate(DYNAMIC_JOINT_INDICES):
        moved = joint_rad.copy()
        moved[joint_index] += epsilon
        jacobian[:, column] = (
            point_position(kinematics, moved, local_point) - base
        ) / epsilon
    return jacobian


def integrate_accelerations(
    initial_position_rad: np.ndarray,
    initial_velocity_rad_s: np.ndarray,
    accelerations_rad_s2: np.ndarray,
    control_period_s: float,
) -> tuple[np.ndarray, np.ndarray]:
    acceleration = np.asarray(accelerations_rad_s2, dtype=float)
    velocity = (
        np.asarray(initial_velocity_rad_s, dtype=float)[None, :]
        + np.cumsum(acceleration, axis=0) * control_period_s
    )
    position = (
        np.asarray(initial_position_rad, dtype=float)[None, :]
        + np.cumsum(velocity, axis=0) * control_period_s
    )
    return position, velocity


def predict_first_order_actuator_response(
    initial_position_rad: np.ndarray,
    commanded_position_rad: np.ndarray,
    control_period_s: float,
    response_alpha: Sequence[float],
) -> tuple[np.ndarray, np.ndarray]:
    """Predict executed q/dq from a bounded position command reference."""

    command = np.asarray(commanded_position_rad, dtype=float)
    alpha = np.asarray(response_alpha, dtype=float)
    if alpha.shape != (command.shape[1],) or np.any(alpha <= 0.0) or np.any(alpha > 1.0):
        raise ValueError("actuator response alpha must contain one value in (0, 1] per joint")
    executed = np.zeros_like(command)
    velocity = np.zeros_like(command)
    state = np.asarray(initial_position_rad, dtype=float).copy()
    for index, target in enumerate(command):
        previous = state.copy()
        state = previous + alpha * (target - previous)
        executed[index] = state
        velocity[index] = (state - previous) / control_period_s
    return executed, velocity


def signed_rotation_deg(
    detach_row: dict[str, Any],
    target_row: dict[str, Any],
    axis_world: Sequence[float],
) -> float:
    def rotation(row):
        quaternion_wxyz = np.asarray(row["cube_quaternion_wxyz"], dtype=float)
        return Rotation.from_quat(np.r_[quaternion_wxyz[1:], quaternion_wxyz[0]])

    delta = rotation(target_row) * rotation(detach_row).inv()
    return math.degrees(float(np.dot(delta.as_rotvec(), axis_world)))


def optimize_intercept(
    rows: Sequence[dict[str, Any]],
    kinematics,
    local_finger_midpoint: np.ndarray,
    *,
    start_time_s: float,
    intercept_time_s: float,
    settings: CatchPlanSettings,
) -> dict[str, Any]:
    period = settings.control_period_s
    steps = round((intercept_time_s - start_time_s) / period)
    if steps < 2 or not np.isclose(start_time_s + steps * period, intercept_time_s):
        raise ValueError("catch interval must contain whole control periods")

    start_row = nearest_row(rows, start_time_s)
    target_row = nearest_row(rows, intercept_time_s)
    initial_full_q = np.asarray(start_row["arm_joint_position_rad"], dtype=float)
    initial_full_dq = np.asarray(start_row["arm_joint_velocity_rad_s"], dtype=float)
    q0 = initial_full_q[list(DYNAMIC_JOINT_INDICES)]
    dq0 = initial_full_dq[list(DYNAMIC_JOINT_INDICES)]
    fixed = initial_full_q[list(FIXED_JOINT_INDICES)].copy()
    target_position = np.asarray(target_row["cube_position_w_m"], dtype=float)
    target_velocity = np.asarray(target_row["cube_linear_velocity_w_m_s"], dtype=float)
    trajectory_times = start_time_s + np.arange(1, steps + 1) * period
    trajectory_cube_positions = np.asarray(
        [nearest_row(rows, time_s)["cube_position_w_m"] for time_s in trajectory_times],
        dtype=float,
    )
    clearance_end_time_s = intercept_time_s - settings.final_approach_duration_s
    clearance_start_time_s = (
        start_time_s if settings.preintercept_clearance_start_time_s is None
        else settings.preintercept_clearance_start_time_s
    )

    lower = JOINT_LOWER_RAD[list(DYNAMIC_JOINT_INDICES)] + HANDOFF_MIN_JOINT_MARGIN_RAD
    upper = JOINT_UPPER_RAD[list(DYNAMIC_JOINT_INDICES)] - HANDOFF_MIN_JOINT_MARGIN_RAD

    def full_joint(dynamic_q: np.ndarray) -> np.ndarray:
        result = initial_full_q.copy()
        result[list(DYNAMIC_JOINT_INDICES)] = dynamic_q
        result[list(FIXED_JOINT_INDICES)] = fixed
        return result

    def rollout(flat_acceleration: np.ndarray):
        acceleration = flat_acceleration.reshape(steps, len(DYNAMIC_JOINT_INDICES))
        position, velocity = integrate_accelerations(q0, dq0, acceleration, period)
        return position, velocity, acceleration

    def predicted_execution(position: np.ndarray, velocity: np.ndarray):
        if settings.actuator_response_alpha is None:
            return position, velocity
        return predict_first_order_actuator_response(
            q0,
            position,
            period,
            settings.actuator_response_alpha,
        )

    def objective(flat_acceleration: np.ndarray) -> float:
        position, velocity, acceleration = rollout(flat_acceleration)
        executed_position, executed_velocity = predicted_execution(position, velocity)
        final_q = full_joint(executed_position[-1])
        position_error = (
            point_position(kinematics, final_q, local_finger_midpoint)
            - target_position
        )
        velocity_error = (
            dynamic_point_jacobian(
                kinematics, final_q, local_finger_midpoint
            ) @ executed_velocity[-1]
            - target_velocity
        )
        acceleration_delta = np.diff(acceleration, axis=0)
        return float(
            1.0e6 * np.dot(position_error, position_error)
            + settings.terminal_velocity_weight
            * np.dot(velocity_error, velocity_error)
            + settings.acceleration_weight * np.sum(acceleration**2)
            + settings.acceleration_smoothness_weight
            * np.sum(acceleration_delta**2)
        )

    def joint_constraints(flat_acceleration: np.ndarray) -> np.ndarray:
        position, velocity, _ = rollout(flat_acceleration)
        return np.r_[
            settings.maximum_joint_speed_rad_s - 1.0e-8 - np.abs(velocity).ravel(),
            (position - lower).ravel(),
            (upper - position).ravel(),
        ]

    def trajectory_constraints(flat_acceleration: np.ndarray) -> np.ndarray:
        position, velocity, _ = rollout(flat_acceleration)
        executed_position, _ = predicted_execution(position, velocity)
        clearance = []
        for time_s, dynamic_q, cube_position in zip(
            trajectory_times, executed_position, trajectory_cube_positions, strict=True
        ):
            if clearance_start_time_s <= time_s <= clearance_end_time_s + 1.0e-9:
                finger_position = point_position(
                    kinematics, full_joint(dynamic_q), local_finger_midpoint
                )
                clearance.extend(
                    (
                        float(np.linalg.norm(finger_position - cube_position))
                        - settings.minimum_preintercept_distance_m,
                        float(cube_position[2] - finger_position[2])
                        - settings.minimum_preintercept_vertical_clearance_m,
                    )
                )
        return np.r_[
            joint_constraints(flat_acceleration),
            clearance,
        ]

    bounds = [
        (
            -settings.maximum_joint_acceleration_rad_s2,
            settings.maximum_joint_acceleration_rad_s2,
        )
    ] * (steps * len(DYNAMIC_JOINT_INDICES))
    initial = np.zeros(steps * len(DYNAMIC_JOINT_INDICES), dtype=float)
    warm_start = minimize(
        objective,
        initial,
        method="SLSQP",
        bounds=bounds,
        constraints={"type": "ineq", "fun": joint_constraints},
        options={"maxiter": 500, "ftol": 1.0e-8, "disp": False},
    )
    if warm_start.success:
        initial = warm_start.x
    result = minimize(
        objective,
        initial,
        method="SLSQP",
        bounds=bounds,
        constraints={"type": "ineq", "fun": trajectory_constraints},
        options={"maxiter": 800, "ftol": 1.0e-8, "disp": False},
    )
    position, velocity, acceleration = rollout(result.x)
    executed_position, executed_velocity = predicted_execution(position, velocity)
    full_q = np.repeat(initial_full_q[None, :], steps + 1, axis=0)
    full_dq = np.zeros_like(full_q)
    full_ddq = np.zeros_like(full_q)
    full_q[0] = initial_full_q
    full_dq[0] = initial_full_dq
    full_dq[0, list(FIXED_JOINT_INDICES)] = 0.0
    full_q[1:, list(DYNAMIC_JOINT_INDICES)] = position
    full_dq[1:, list(DYNAMIC_JOINT_INDICES)] = velocity
    full_ddq[1:, list(DYNAMIC_JOINT_INDICES)] = acceleration
    full_q[:, list(FIXED_JOINT_INDICES)] = fixed

    executed_full_q = full_q.copy()
    executed_full_dq = full_dq.copy()
    executed_full_q[1:, list(DYNAMIC_JOINT_INDICES)] = executed_position
    executed_full_dq[1:, list(DYNAMIC_JOINT_INDICES)] = executed_velocity
    final_point = point_position(
        kinematics, executed_full_q[-1], local_finger_midpoint
    )
    final_point_velocity = (
        dynamic_point_jacobian(
            kinematics, executed_full_q[-1], local_finger_midpoint
        ) @ executed_full_dq[-1, list(DYNAMIC_JOINT_INDICES)]
    )
    position_error_m = float(np.linalg.norm(final_point - target_position))
    velocity_error_m_s = float(np.linalg.norm(final_point_velocity - target_velocity))
    limits = evaluate_joint_trajectory(full_q, full_dq, full_ddq)
    preintercept_distances = []
    preintercept_vertical_clearances = []
    for time_s, q, cube_position in zip(
        trajectory_times, executed_full_q[1:], trajectory_cube_positions, strict=True
    ):
        if clearance_start_time_s <= time_s <= clearance_end_time_s + 1.0e-9:
            finger_position = point_position(kinematics, q, local_finger_midpoint)
            preintercept_distances.append(
                float(np.linalg.norm(finger_position - cube_position))
            )
            preintercept_vertical_clearances.append(
                float(cube_position[2] - finger_position[2])
            )
    minimum_distance = min(preintercept_distances)
    minimum_vertical_clearance = min(preintercept_vertical_clearances)
    admitted = bool(
        result.success
        and limits["joint_mechanical_limits_pass"]
        and position_error_m <= settings.maximum_capture_position_error_m
        and velocity_error_m_s <= settings.maximum_capture_velocity_error_m_s
        and minimum_distance >= settings.minimum_preintercept_distance_m - 1.0e-6
        and minimum_vertical_clearance
        >= settings.minimum_preintercept_vertical_clearance_m - 1.0e-6
    )
    times = start_time_s + np.arange(steps + 1) * period
    return {
        "optimizer_success": bool(result.success),
        "optimizer_message": str(result.message),
        "intercept_time_s": float(intercept_time_s),
        "capture_position_error_m": position_error_m,
        "capture_velocity_error_m_s": velocity_error_m_s,
        "capture_admitted": admitted,
        "minimum_preintercept_distance_m": minimum_distance,
        "minimum_preintercept_vertical_clearance_m": (
            minimum_vertical_clearance
        ),
        "target_cube_position_w_m": target_position.tolist(),
        "target_cube_velocity_w_m_s": target_velocity.tolist(),
        "planned_finger_midpoint_w_m": final_point.tolist(),
        "planned_finger_velocity_w_m_s": final_point_velocity.tolist(),
        "actuator_response_alpha": (
            None if settings.actuator_response_alpha is None
            else list(settings.actuator_response_alpha)
        ),
        "joint_limit_evidence": limits,
        "samples": [
            {
                "time_s": float(time_s),
                "phase": "offline_j235_intercept",
                "joint_position_rad": q.tolist(),
                "joint_velocity_rad_s": dq.tolist(),
                "joint_acceleration_rad_s2": ddq.tolist(),
            }
            for time_s, q, dq, ddq in zip(
                times, full_q, full_dq, full_ddq, strict=True
            )
        ],
    }


def plan_pose_conditioned_catch(
    rows: Sequence[dict[str, Any]],
    summary: dict[str, Any],
    kinematics,
    *,
    desired_angle_deg: float,
    start_time_s: float = 0.68,
    earliest_intercept_time_s: float = 0.84,
    latest_intercept_time_s: float = 1.04,
    settings: CatchPlanSettings = CatchPlanSettings(),
) -> dict[str, Any]:
    local_finger = infer_local_point(
        rows,
        kinematics,
        point_key="finger_midpoint_w_m",
        start_time_s=start_time_s - 0.12,
        end_time_s=start_time_s - 0.04,
    )
    detach_row = nearest_row(rows, float(summary["detach_time_s"]))
    axis = summary["release_transfer_evidence"]["target_axis_world"]
    times = np.arange(
        earliest_intercept_time_s,
        latest_intercept_time_s + settings.control_period_s / 2.0,
        settings.control_period_s,
    )
    candidates = []
    for intercept_time_s in times:
        candidate = optimize_intercept(
            rows,
            kinematics,
            local_finger,
            start_time_s=start_time_s,
            intercept_time_s=float(intercept_time_s),
            settings=settings,
        )
        target_row = nearest_row(rows, float(intercept_time_s))
        measured_angle = signed_rotation_deg(detach_row, target_row, axis)
        candidate["predicted_rotation_at_intercept_deg"] = measured_angle
        candidate["angle_error_deg"] = abs(measured_angle - desired_angle_deg)
        candidate["j_score"] = float(
            candidate["angle_error_deg"]
            + 40.0 * candidate["capture_position_error_m"]
            + 2.0 * candidate["capture_velocity_error_m_s"]
        )
        candidates.append(candidate)
    candidates.sort(
        key=lambda item: (
            not item["capture_admitted"],
            item["j_score"],
            item["intercept_time_s"],
        )
    )
    selected = candidates[0]
    return {
        "schema": "xarm6_pose_conditioned_j235_catch_plan_v1",
        "desired_angle_deg": float(desired_angle_deg),
        "dynamic_joint_indices_zero_based": list(DYNAMIC_JOINT_INDICES),
        "fixed_joint_indices_zero_based": list(FIXED_JOINT_INDICES),
        "start_time_s": float(start_time_s),
        "measured_arm_tracking_delay_s": float(summary["arm_tracking_delay_s"]),
        "local_finger_midpoint_m": local_finger.tolist(),
        "settings": settings.__dict__,
        "desired_angle_admitted": bool(
            selected["capture_admitted"] and selected["angle_error_deg"] <= 2.0
        ),
        "selected": selected,
        "candidates": candidates,
        "real_g1_schedule": {
            "preclose_command_time_s": max(
                start_time_s, selected["intercept_time_s"] - 0.18
            ),
            "close_command_time_s": (
                selected["intercept_time_s"] - 0.10279
            ),
            "measured_full_travel_s": 0.10279,
        },
    }


def load_catch_reference(path: Path) -> dict[str, Any]:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    if payload.get("schema") != "xarm6_offline_j235_catch_reference_v1":
        raise ValueError("unsupported offline catch reference schema")
    if payload.get("dynamic_joint_indices_zero_based") != list(DYNAMIC_JOINT_INDICES):
        raise ValueError("offline catch reference must control exactly J2/J3/J5")
    if payload.get("fixed_joint_indices_zero_based") != list(FIXED_JOINT_INDICES):
        raise ValueError("offline catch reference must preserve J1/J4/J6")
    if not payload.get("capture_admitted", False):
        raise ValueError("offline catch reference was not mechanically admitted")
    rows = payload.get("samples")
    if not isinstance(rows, list) or len(rows) < 2:
        raise ValueError("offline catch reference needs at least two samples")
    times = np.asarray([row["time_s"] for row in rows], dtype=float)
    positions = np.asarray([row["joint_position_rad"] for row in rows], dtype=float)
    velocities = np.asarray([row["joint_velocity_rad_s"] for row in rows], dtype=float)
    accelerations = np.asarray([row["joint_acceleration_rad_s2"] for row in rows], dtype=float)
    if positions.shape != (len(rows), 6):
        raise ValueError("offline catch q samples must have shape (N, 6)")
    if velocities.shape != positions.shape or accelerations.shape != positions.shape:
        raise ValueError("offline catch q/dq/ddq sample shapes must match")
    if not np.all(np.isfinite(np.c_[times, positions, velocities, accelerations])):
        raise ValueError("offline catch reference contains non-finite values")
    period = float(times[1] - times[0])
    if period <= 0.0 or not np.allclose(np.diff(times), period, atol=1.0e-9):
        raise ValueError("offline catch reference times must use one fixed period")
    if not np.isclose(times[0], float(payload["start_time_s"])):
        raise ValueError("offline catch start time does not match its first sample")
    if not np.isclose(times[-1], float(payload["intercept_time_s"])):
        raise ValueError("offline catch intercept time does not match its last sample")
    if not np.all(positions[:, list(FIXED_JOINT_INDICES)] == positions[0, list(FIXED_JOINT_INDICES)]):
        raise ValueError("offline catch reference moves J1/J4/J6")
    if not np.all(velocities[:, list(FIXED_JOINT_INDICES)] == 0.0):
        raise ValueError("offline catch reference gives J1/J4/J6 velocity")
    evidence = evaluate_joint_trajectory(positions, velocities, accelerations)
    if not evidence["joint_mechanical_limits_pass"]:
        raise ValueError("offline catch reference exceeds the real transfer envelope")
    payload["control_period_s"] = period
    payload["joint_limit_evidence"] = evidence
    return payload


def catch_reference_sample(reference: dict[str, Any], time_s: float) -> dict[str, Any]:
    rows = reference["samples"]
    period = float(reference["control_period_s"])
    index = int(round((float(time_s) - float(reference["start_time_s"])) / period))
    return rows[min(max(index, 0), len(rows) - 1)]
