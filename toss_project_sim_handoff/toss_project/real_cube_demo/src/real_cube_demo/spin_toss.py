"""Kinematics-aware spin-toss planning without object-property priors."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Callable

import numpy as np
from scipy.spatial.transform import Rotation

from .config import DEMO_ROOT


DEFAULT_SPIN_TOSS_PATH = DEMO_ROOT / "configs" / "spin_toss.json"


@dataclass(frozen=True)
class SpinTossSpec:
    execution_ready: bool
    execution_block_reason: str
    control_period_s: float
    release_joint_rad: tuple[float, ...]
    prethrow_path: str
    prethrow_drop_m: float
    prethrow_rotation_rad: float
    prethrow_duration_s: float
    nominal_upward_velocity_m_s: float
    nominal_spin_rate_rad_s: float
    spin_axis: str
    capture_mode: str
    free_flight_duration_s: float
    translation_stop_duration_s: float
    spin_stop_duration_s: float
    servo_tracking_delay_s: float
    final_hold_duration_s: float
    catch_drop_m: float
    catch_orientation_follow_fraction: float
    catch_linear_velocity_match_fraction: float
    catch_angular_velocity_match_fraction: float
    capture_absorb_duration_s: float
    detach_delay_s: float
    detach_delay_source: str
    held_gripper_position: float
    catch_open_position: float
    catch_close_lead_s: float
    max_joint_speed_rad_s: float
    max_joint_acceleration_rad_s2: float
    max_tcp_speed_m_s: float
    physics_source: str
    geometry_source: str
    grasp_offset_tool_m: tuple[float, ...]
    grasp_offset_source: str


@dataclass(frozen=True)
class SpinTossSample:
    time_s: float
    phase: str
    joint_rad: tuple[float, ...]
    joint_velocity_rad_s: tuple[float, ...]
    joint_acceleration_rad_s2: tuple[float, ...]


@dataclass(frozen=True)
class SpinTossPlan:
    samples: tuple[SpinTossSample, ...]
    release_time_s: float
    catch_time_s: float
    physical_release_time_s: float
    physical_catch_time_s: float
    gripper_release_command_time_s: float
    gripper_close_command_time_s: float
    start_joint_rad: tuple[float, ...]
    release_joint_rad: tuple[float, ...]
    catch_joint_rad: tuple[float, ...]
    release_joint_velocity_rad_s: tuple[float, ...]
    release_linear_joint_velocity_rad_s: tuple[float, ...]
    release_spin_joint_velocity_rad_s: tuple[float, ...]
    catch_joint_velocity_rad_s: tuple[float, ...]
    start_tcp_pose: tuple[float, ...]
    release_tcp_pose: tuple[float, ...]
    catch_tcp_pose: tuple[float, ...]
    nominal_release_twist: tuple[float, ...]
    nominal_catch_twist: tuple[float, ...]
    spin_axis_world: tuple[float, ...]
    predicted_object_rotation_rad: float
    object_relative_displacement_at_catch_m: tuple[float, ...]
    max_joint_speed_rad_s: float
    max_joint_acceleration_rad_s2: float
    max_tcp_speed_m_s: float
    release_tool_z_world: tuple[float, ...]
    gripper_base_below_object_m: float


def load_spin_toss_spec(path: Path = DEFAULT_SPIN_TOSS_PATH) -> SpinTossSpec:
    raw = json.loads(path.read_text(encoding="utf-8"))
    if raw.get("schema") != "real_cube_spin_toss_v1":
        raise ValueError("unsupported spin-toss configuration")
    release = tuple(float(value) for value in raw["release_joint_rad"])
    if len(release) != 6:
        raise ValueError("release_joint_rad must contain six values")
    return SpinTossSpec(
        execution_ready=bool(raw["execution_ready"]),
        execution_block_reason=str(raw["execution_block_reason"]),
        control_period_s=float(raw["control_period_s"]),
        release_joint_rad=release,
        prethrow_path=str(raw["prethrow_path"]),
        prethrow_drop_m=float(raw["prethrow_drop_m"]),
        prethrow_rotation_rad=float(raw["prethrow_rotation_rad"]),
        prethrow_duration_s=float(raw["prethrow_duration_s"]),
        nominal_upward_velocity_m_s=float(raw["nominal_upward_velocity_m_s"]),
        nominal_spin_rate_rad_s=float(raw["nominal_spin_rate_rad_s"]),
        spin_axis=str(raw["spin_axis"]),
        capture_mode=str(raw["capture_mode"]),
        free_flight_duration_s=float(raw["free_flight_duration_s"]),
        translation_stop_duration_s=float(raw["translation_stop_duration_s"]),
        spin_stop_duration_s=float(raw["spin_stop_duration_s"]),
        servo_tracking_delay_s=float(raw["servo_tracking_delay_s"]),
        final_hold_duration_s=float(raw["final_hold_duration_s"]),
        catch_drop_m=float(raw["catch_drop_m"]),
        catch_orientation_follow_fraction=float(
            raw["catch_orientation_follow_fraction"]
        ),
        catch_linear_velocity_match_fraction=float(
            raw["catch_linear_velocity_match_fraction"]
        ),
        catch_angular_velocity_match_fraction=float(
            raw["catch_angular_velocity_match_fraction"]
        ),
        capture_absorb_duration_s=float(raw["capture_absorb_duration_s"]),
        detach_delay_s=float(raw["detach_delay_s"]),
        detach_delay_source=str(raw["detach_delay_source"]),
        held_gripper_position=float(raw["held_gripper_position"]),
        catch_open_position=float(raw["catch_open_position"]),
        catch_close_lead_s=float(raw["catch_close_lead_s"]),
        max_joint_speed_rad_s=float(raw["max_joint_speed_rad_s"]),
        max_joint_acceleration_rad_s2=float(raw["max_joint_acceleration_rad_s2"]),
        max_tcp_speed_m_s=float(raw["max_tcp_speed_m_s"]),
        physics_source=str(raw["physics_source"]),
        geometry_source=str(raw["geometry_source"]),
        grasp_offset_tool_m=tuple(
            float(value) for value in raw["grasp_offset_tool_m"]
        ),
        grasp_offset_source=str(raw["grasp_offset_source"]),
    )


def pose_matrix(tcp_pose: tuple[float, ...]) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler("xyz", tcp_pose[3:6]).as_matrix()
    transform[:3, 3] = np.asarray(tcp_pose[:3], dtype=float) / 1000.0
    return transform


def matrix_pose(transform: np.ndarray) -> tuple[float, ...]:
    rpy = Rotation.from_matrix(transform[:3, :3]).as_euler("xyz")
    xyz_mm = transform[:3, 3] * 1000.0
    return tuple(float(value) for value in np.concatenate((xyz_mm, rpy)))


def numerical_jacobian(
    joint_rad: tuple[float, ...],
    forward_kinematics: Callable[[tuple[float, ...]], tuple[float, ...]],
    epsilon: float = 1e-4,
) -> np.ndarray:
    base = pose_matrix(forward_kinematics(joint_rad))
    jacobian = np.zeros((6, 6), dtype=float)
    for joint in range(6):
        perturbed = list(joint_rad)
        perturbed[joint] += epsilon
        moved = pose_matrix(forward_kinematics(tuple(perturbed)))
        jacobian[:3, joint] = (moved[:3, 3] - base[:3, 3]) / epsilon
        world_delta = moved[:3, :3] @ base[:3, :3].T
        jacobian[3:, joint] = (
            Rotation.from_matrix(world_delta).as_rotvec() / epsilon
        )
    return jacobian


def _quintic_coefficients(
    q0: np.ndarray,
    v0: np.ndarray,
    a0: np.ndarray,
    q1: np.ndarray,
    v1: np.ndarray,
    a1: np.ndarray,
    duration_s: float,
) -> np.ndarray:
    coefficients = np.zeros((6, q0.size), dtype=float)
    coefficients[0] = q0
    coefficients[1] = v0
    coefficients[2] = 0.5 * a0
    duration = duration_s
    matrix = np.asarray(
        [
            [duration**3, duration**4, duration**5],
            [3 * duration**2, 4 * duration**3, 5 * duration**4],
            [6 * duration, 12 * duration**2, 20 * duration**3],
        ],
        dtype=float,
    )
    right = np.stack(
        (
            q1 - coefficients[0] - coefficients[1] * duration - coefficients[2] * duration**2,
            v1 - coefficients[1] - 2 * coefficients[2] * duration,
            a1 - 2 * coefficients[2],
        )
    )
    coefficients[3:] = np.linalg.solve(matrix, right)
    return coefficients


def _cubic_coefficients(
    q0: np.ndarray,
    v0: np.ndarray,
    q1: np.ndarray,
    v1: np.ndarray,
    duration_s: float,
) -> np.ndarray:
    coefficients = np.zeros((6, q0.size), dtype=float)
    delta = q1 - q0
    coefficients[0] = q0
    coefficients[1] = v0
    coefficients[2] = (
        3.0 * delta / duration_s**2 - (2.0 * v0 + v1) / duration_s
    )
    coefficients[3] = (
        -2.0 * delta / duration_s**3 + (v0 + v1) / duration_s**2
    )
    return coefficients


def _evaluate(coefficients: np.ndarray, time_s: float) -> tuple[np.ndarray, ...]:
    powers = np.asarray([time_s**index for index in range(6)])
    velocity_powers = np.asarray(
        [0.0, 1.0, 2 * time_s, 3 * time_s**2, 4 * time_s**3, 5 * time_s**4]
    )
    acceleration_powers = np.asarray(
        [0.0, 0.0, 2.0, 6 * time_s, 12 * time_s**2, 20 * time_s**3]
    )
    return (
        powers @ coefficients,
        velocity_powers @ coefficients,
        acceleration_powers @ coefficients,
    )


def _delayed_stop_profile(
    q0: np.ndarray,
    v0: np.ndarray,
    time_s: float,
    stop_delay_s: np.ndarray,
    stop_duration_s: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    q = q0.copy()
    qd = v0.copy()
    qdd = np.zeros_like(v0)
    for joint in range(q0.size):
        delay = stop_delay_s[joint]
        duration = stop_duration_s[joint]
        if time_s <= delay:
            q[joint] += v0[joint] * time_s
        elif time_s < delay + duration:
            braking_time = time_s - delay
            q[joint] += (
                v0[joint] * delay
                + v0[joint] * braking_time
                - 0.5 * v0[joint] * braking_time**2 / duration
            )
            qd[joint] = v0[joint] * (1.0 - braking_time / duration)
            qdd[joint] = -v0[joint] / duration
        else:
            q[joint] += v0[joint] * (delay + 0.5 * duration)
            qd[joint] = 0.0
    return q, qd, qdd


def build_spin_toss_plan(
    spec: SpinTossSpec,
    forward_kinematics: Callable[[tuple[float, ...]], tuple[float, ...]],
    inverse_kinematics: Callable[[tuple[float, ...]], tuple[float, ...]],
    *,
    validate: bool = True,
) -> SpinTossPlan:
    release_q = np.asarray(spec.release_joint_rad, dtype=float)
    release_tcp_pose = forward_kinematics(spec.release_joint_rad)
    release_transform = pose_matrix(release_tcp_pose)
    axis_index = {"tool_x": 0, "tool_y": 1, "tool_z": 2}[spec.spin_axis]
    spin_axis = release_transform[:3, axis_index]

    gravity = 9.81
    observed_intercept = spec.capture_mode == "observed_ballistic_intercept"
    if spec.capture_mode == "in_gripper_regrasp" or observed_intercept:
        ideal_flight_s = spec.free_flight_duration_s
    else:
        ideal_flight_s = (
            spec.nominal_upward_velocity_m_s
            + math.sqrt(
                spec.nominal_upward_velocity_m_s**2
                + 2.0 * gravity * spec.catch_drop_m
            )
        ) / gravity
    flight_steps = max(3, round(ideal_flight_s / spec.control_period_s))
    flight_time_s = flight_steps * spec.control_period_s
    if spec.capture_mode == "in_gripper_regrasp" or observed_intercept:
        upward_velocity = spec.nominal_upward_velocity_m_s
    else:
        upward_velocity = (
            0.5 * gravity * flight_time_s
            - spec.catch_drop_m / flight_time_s
        )
    desired_linear_twist = np.concatenate(
        (np.asarray([0.0, 0.0, upward_velocity]), np.zeros(3, dtype=float))
    )
    desired_spin_twist = np.concatenate(
        (np.zeros(3, dtype=float), spin_axis * spec.nominal_spin_rate_rad_s)
    )
    jacobian = numerical_jacobian(spec.release_joint_rad, forward_kinematics)
    jacobian_inverse = np.linalg.pinv(jacobian, rcond=1e-3)
    release_linear_qd = jacobian_inverse @ desired_linear_twist
    release_spin_qd = jacobian_inverse @ desired_spin_twist
    release_qd = release_linear_qd + release_spin_qd
    actual_twist = jacobian @ release_qd
    pre_steps = max(3, round(spec.prethrow_duration_s / spec.control_period_s))
    pre_duration_s = pre_steps * spec.control_period_s

    start_transform = release_transform.copy()
    start_transform[2, 3] -= spec.prethrow_drop_m
    local_windup = np.zeros(3, dtype=float)
    local_windup[axis_index] = -spec.prethrow_rotation_rad
    start_transform[:3, :3] = (
        release_transform[:3, :3]
        @ Rotation.from_rotvec(local_windup).as_matrix()
    )
    start_q = np.asarray(inverse_kinematics(matrix_pose(start_transform)), dtype=float)

    predicted_rotation = float(np.linalg.norm(actual_twist[3:]) * flight_time_s)
    nominal_object_catch_twist = actual_twist.copy()
    nominal_object_catch_twist[2] -= gravity * flight_time_s
    object_position_at_catch = None
    if spec.capture_mode == "in_gripper_regrasp":
        translation_stop_delay_s = np.zeros(6, dtype=float)
        translation_stop_duration_s = np.full(
            6, spec.translation_stop_duration_s, dtype=float
        )
        spin_stop_delay_s = np.full(6, flight_time_s, dtype=float)
        spin_stop_duration_s = np.full(6, spec.spin_stop_duration_s, dtype=float)
        stop_end_s = max(
            spec.translation_stop_duration_s,
            flight_time_s + spec.spin_stop_duration_s,
        )
        stop_steps = round(stop_end_s / spec.control_period_s)
        catch_linear_q, catch_linear_qd, _ = _delayed_stop_profile(
            np.zeros(6, dtype=float),
            release_linear_qd,
            flight_time_s,
            translation_stop_delay_s,
            translation_stop_duration_s,
        )
        catch_spin_q, catch_spin_qd, _ = _delayed_stop_profile(
            np.zeros(6, dtype=float),
            release_spin_qd,
            flight_time_s,
            spin_stop_delay_s,
            spin_stop_duration_s,
        )
        catch_q = release_q + catch_linear_q + catch_spin_q
        catch_qd = catch_linear_qd + catch_spin_qd
    elif observed_intercept:
        grasp_offset_tool = np.asarray(spec.grasp_offset_tool_m, dtype=float)
        object_position_release = (
            release_transform[:3, 3]
            + release_transform[:3, :3] @ grasp_offset_tool
        )
        object_position_at_catch = (
            object_position_release
            + actual_twist[:3] * flight_time_s
            + np.asarray([0.0, 0.0, -0.5 * gravity * flight_time_s**2])
        )
        catch_rotation = (
            Rotation.from_rotvec(
                actual_twist[3:]
                * flight_time_s
                * spec.catch_orientation_follow_fraction
            ).as_matrix()
            @ release_transform[:3, :3]
        )
        catch_transform = np.eye(4)
        catch_transform[:3, :3] = catch_rotation
        catch_transform[:3, 3] = (
            object_position_at_catch - catch_rotation @ grasp_offset_tool
        )
        catch_q = np.asarray(
            inverse_kinematics(matrix_pose(catch_transform)), dtype=float
        )
        catch_angular_velocity = (
            nominal_object_catch_twist[3:]
            * spec.catch_angular_velocity_match_fraction
        )
        center_offset_world = catch_rotation @ grasp_offset_tool
        catch_tcp_linear_velocity = (
            nominal_object_catch_twist[:3]
            * spec.catch_linear_velocity_match_fraction
            - np.cross(catch_angular_velocity, center_offset_world)
        )
        desired_catch_twist = np.concatenate(
            (catch_tcp_linear_velocity, catch_angular_velocity)
        )
        catch_jacobian = numerical_jacobian(
            tuple(float(value) for value in catch_q), forward_kinematics
        )
        catch_qd = (
            np.linalg.pinv(catch_jacobian, rcond=1e-3) @ desired_catch_twist
        )
    else:
        catch_transform = release_transform.copy()
        catch_transform[2, 3] -= spec.catch_drop_m
        catch_transform[:3, :3] = (
            Rotation.from_rotvec(
                actual_twist[3:]
                * flight_time_s
                * spec.catch_orientation_follow_fraction
            ).as_matrix()
            @ release_transform[:3, :3]
        )
        catch_q = np.asarray(
            inverse_kinematics(matrix_pose(catch_transform)), dtype=float
        )
        desired_catch_twist = np.concatenate(
            (
                nominal_object_catch_twist[:3]
                * spec.catch_linear_velocity_match_fraction,
                nominal_object_catch_twist[3:]
                * spec.catch_angular_velocity_match_fraction,
            )
        )
        catch_jacobian = numerical_jacobian(
            tuple(float(value) for value in catch_q), forward_kinematics
        )
        catch_qd = (
            np.linalg.pinv(catch_jacobian, rcond=1e-3) @ desired_catch_twist
        )

    zeros = np.zeros(6, dtype=float)
    if spec.prethrow_path == "cartesian_vertical":
        vertical_coefficients = _quintic_coefficients(
            np.asarray([-spec.prethrow_drop_m]),
            np.zeros(1),
            np.zeros(1),
            np.zeros(1),
            np.asarray([upward_velocity]),
            np.zeros(1),
            pre_duration_s,
        )
        rotation_coefficients = _quintic_coefficients(
            np.asarray([-spec.prethrow_rotation_rad]),
            np.zeros(1),
            np.zeros(1),
            np.zeros(1),
            np.asarray([spec.nominal_spin_rate_rad_s]),
            np.zeros(1),
            pre_duration_s,
        )
    else:
        pre_coefficients = _quintic_coefficients(
            start_q, zeros, zeros, release_q, release_qd, zeros, pre_duration_s
        )
    if spec.capture_mode != "in_gripper_regrasp":
        post_coefficients = _cubic_coefficients(
            release_q, release_qd, catch_q, catch_qd, flight_time_s
        )
        absorb_steps = max(
            3, round(spec.capture_absorb_duration_s / spec.control_period_s)
        )
        absorb_duration_s = absorb_steps * spec.control_period_s
        settle_q = catch_q + 0.5 * absorb_duration_s * catch_qd
        absorb_coefficients = _quintic_coefficients(
            catch_q, catch_qd, zeros, settle_q, zeros, zeros, absorb_duration_s
        )

    samples: list[SpinTossSample] = []
    if spec.prethrow_path == "cartesian_vertical":
        local_axis = np.zeros(3, dtype=float)
        local_axis[axis_index] = 1.0
        for step in range(pre_steps + 1):
            local_time = step * spec.control_period_s
            vertical, vertical_velocity, _ = _evaluate(
                vertical_coefficients, local_time
            )
            rotation, rotation_velocity, _ = _evaluate(
                rotation_coefficients, local_time
            )
            target_transform = release_transform.copy()
            target_transform[2, 3] += vertical[0]
            target_transform[:3, :3] = (
                release_transform[:3, :3]
                @ Rotation.from_rotvec(local_axis * rotation[0]).as_matrix()
            )
            if step == 0:
                q = start_q
                qd = zeros
            elif step == pre_steps:
                q = release_q
                qd = release_qd
            else:
                q = np.asarray(
                    inverse_kinematics(matrix_pose(target_transform)), dtype=float
                )
                sample_jacobian = numerical_jacobian(
                    tuple(float(value) for value in q), forward_kinematics
                )
                desired_twist = np.concatenate(
                    (
                        np.asarray([0.0, 0.0, vertical_velocity[0]]),
                        spin_axis * rotation_velocity[0],
                    )
                )
                qd = (
                    np.linalg.pinv(sample_jacobian, rcond=1e-3)
                    @ desired_twist
                )
            samples.append(
                SpinTossSample(
                    time_s=local_time,
                    phase="prethrow",
                    joint_rad=tuple(float(value) for value in q),
                    joint_velocity_rad_s=tuple(float(value) for value in qd),
                    joint_acceleration_rad_s2=(0.0,) * 6,
                )
            )
    else:
        for step in range(pre_steps + 1):
            local_time = step * spec.control_period_s
            q, qd, qdd = _evaluate(pre_coefficients, local_time)
            samples.append(
                SpinTossSample(
                    time_s=local_time,
                    phase="prethrow",
                    joint_rad=tuple(float(value) for value in q),
                    joint_velocity_rad_s=tuple(float(value) for value in qd),
                    joint_acceleration_rad_s2=tuple(float(value) for value in qdd),
                )
            )
    if spec.capture_mode == "in_gripper_regrasp":
        for step in range(1, stop_steps + 1):
            local_time = step * spec.control_period_s
            linear_q, linear_qd, linear_qdd = _delayed_stop_profile(
                np.zeros(6, dtype=float),
                release_linear_qd,
                local_time,
                translation_stop_delay_s,
                translation_stop_duration_s,
            )
            spin_q, spin_qd, spin_qdd = _delayed_stop_profile(
                np.zeros(6, dtype=float),
                release_spin_qd,
                local_time,
                spin_stop_delay_s,
                spin_stop_duration_s,
            )
            q = release_q + linear_q + spin_q
            qd = linear_qd + spin_qd
            qdd = linear_qdd + spin_qdd
            samples.append(
                SpinTossSample(
                    time_s=pre_duration_s + local_time,
                    phase=(
                        "catch_transfer"
                        if local_time <= flight_time_s
                        else "capture_absorb"
                    ),
                    joint_rad=tuple(float(value) for value in q),
                    joint_velocity_rad_s=tuple(float(value) for value in qd),
                    joint_acceleration_rad_s2=tuple(float(value) for value in qdd),
                )
            )
    else:
        for step in range(1, flight_steps + 1):
            local_time = step * spec.control_period_s
            q, qd, qdd = _evaluate(post_coefficients, local_time)
            samples.append(
                SpinTossSample(
                    time_s=pre_duration_s + local_time,
                    phase="catch_transfer",
                    joint_rad=tuple(float(value) for value in q),
                    joint_velocity_rad_s=tuple(float(value) for value in qd),
                    joint_acceleration_rad_s2=tuple(float(value) for value in qdd),
                )
            )
        for step in range(1, absorb_steps + 1):
            local_time = step * spec.control_period_s
            q, qd, qdd = _evaluate(absorb_coefficients, local_time)
            samples.append(
                SpinTossSample(
                    time_s=pre_duration_s + flight_time_s + local_time,
                    phase="capture_absorb",
                    joint_rad=tuple(float(value) for value in q),
                    joint_velocity_rad_s=tuple(float(value) for value in qd),
                    joint_acceleration_rad_s2=tuple(float(value) for value in qdd),
                )
            )

    final_sample = samples[-1]
    hold_steps = round(spec.final_hold_duration_s / spec.control_period_s)
    for step in range(1, hold_steps + 1):
        samples.append(
            SpinTossSample(
                time_s=final_sample.time_s + step * spec.control_period_s,
                phase="final_hold",
                joint_rad=final_sample.joint_rad,
                joint_velocity_rad_s=(0.0,) * 6,
                joint_acceleration_rad_s2=(0.0,) * 6,
            )
        )

    planned_velocities = np.asarray(
        [sample.joint_velocity_rad_s for sample in samples], dtype=float
    )
    planned_accelerations = np.gradient(
        planned_velocities, spec.control_period_s, axis=0, edge_order=2
    )
    samples = [
        SpinTossSample(
            time_s=sample.time_s,
            phase=sample.phase,
            joint_rad=sample.joint_rad,
            joint_velocity_rad_s=sample.joint_velocity_rad_s,
            joint_acceleration_rad_s2=tuple(
                float(value) for value in planned_accelerations[index]
            ),
        )
        for index, sample in enumerate(samples)
    ]

    speed_peak_sample = max(
        samples,
        key=lambda sample: max(abs(value) for value in sample.joint_velocity_rad_s),
    )
    acceleration_peak_sample = max(
        samples,
        key=lambda sample: max(
            abs(value) for value in sample.joint_acceleration_rad_s2
        ),
    )
    max_joint_speed = max(
        abs(value) for value in speed_peak_sample.joint_velocity_rad_s
    )
    max_joint_acceleration = max(
        abs(value) for value in acceleration_peak_sample.joint_acceleration_rad_s2
    )
    tcp_positions = np.asarray(
        [
            forward_kinematics(sample.joint_rad)[:3]
            for sample in samples
        ],
        dtype=float,
    ) / 1000.0
    tcp_speeds = np.linalg.norm(
        np.diff(tcp_positions, axis=0) / spec.control_period_s, axis=1
    )
    max_tcp_speed = float(np.max(tcp_speeds))
    predicted_object_displacement = (
        actual_twist[:3] * flight_time_s
        + np.asarray([0.0, 0.0, -0.5 * gravity * flight_time_s**2])
    )
    start_tcp_pose = forward_kinematics(tuple(float(value) for value in start_q))
    catch_tcp_pose = forward_kinematics(tuple(float(value) for value in catch_q))
    catch_tcp_position = pose_matrix(catch_tcp_pose)[:3, 3]
    if object_position_at_catch is None:
        object_relative_displacement = predicted_object_displacement - (
            catch_tcp_position - release_transform[:3, 3]
        )
    else:
        object_relative_displacement = object_position_at_catch - catch_tcp_position
    release_tool_z_world = release_transform[:3, 2]
    release_object_position = (
        release_transform[:3, 3]
        + release_transform[:3, :3] @ np.asarray(spec.grasp_offset_tool_m)
    )
    gripper_base_position = (
        release_transform[:3, 3] - 0.172 * release_tool_z_world
    )

    if validate and max_joint_speed > spec.max_joint_speed_rad_s:
        raise RuntimeError(
            f"planned joint speed {max_joint_speed:.3f} rad/s exceeds "
            f"configured {spec.max_joint_speed_rad_s:.3f}; "
            f"peak phase={speed_peak_sample.phase}, t={speed_peak_sample.time_s:.3f} s, "
            f"release qd={[round(value, 3) for value in release_qd]}"
        )
    if validate and max_joint_acceleration > spec.max_joint_acceleration_rad_s2:
        raise RuntimeError(
            f"planned joint acceleration {max_joint_acceleration:.3f} rad/s^2 "
            f"exceeds configured {spec.max_joint_acceleration_rad_s2:.3f}; "
            f"peak phase={acceleration_peak_sample.phase}, "
            f"t={acceleration_peak_sample.time_s:.3f} s, "
            "qdd="
            f"{[round(value, 3) for value in acceleration_peak_sample.joint_acceleration_rad_s2]}, "
            f"release qd={[round(value, 3) for value in release_qd]}, "
            f"catch qd={[round(value, 3) for value in catch_qd]}"
        )
    if validate and max_tcp_speed > spec.max_tcp_speed_m_s:
        raise RuntimeError(
            f"planned TCP speed {max_tcp_speed:.3f} m/s exceeds "
            f"configured {spec.max_tcp_speed_m_s:.3f}"
        )

    release_time_s = pre_duration_s
    catch_time_s = release_time_s + flight_time_s
    physical_release_time_s = release_time_s + spec.servo_tracking_delay_s
    physical_catch_time_s = catch_time_s + spec.servo_tracking_delay_s
    return SpinTossPlan(
        samples=tuple(samples),
        release_time_s=release_time_s,
        catch_time_s=catch_time_s,
        physical_release_time_s=physical_release_time_s,
        physical_catch_time_s=physical_catch_time_s,
        gripper_release_command_time_s=(
            physical_release_time_s - spec.detach_delay_s
        ),
        gripper_close_command_time_s=(
            physical_catch_time_s - spec.catch_close_lead_s
        ),
        start_joint_rad=tuple(float(value) for value in start_q),
        release_joint_rad=tuple(float(value) for value in release_q),
        catch_joint_rad=tuple(float(value) for value in catch_q),
        release_joint_velocity_rad_s=tuple(float(value) for value in release_qd),
        release_linear_joint_velocity_rad_s=tuple(
            float(value) for value in release_linear_qd
        ),
        release_spin_joint_velocity_rad_s=tuple(
            float(value) for value in release_spin_qd
        ),
        catch_joint_velocity_rad_s=tuple(float(value) for value in catch_qd),
        start_tcp_pose=tuple(float(value) for value in start_tcp_pose),
        release_tcp_pose=tuple(float(value) for value in release_tcp_pose),
        catch_tcp_pose=tuple(float(value) for value in catch_tcp_pose),
        nominal_release_twist=tuple(float(value) for value in actual_twist),
        nominal_catch_twist=tuple(
            float(value) for value in nominal_object_catch_twist
        ),
        spin_axis_world=tuple(float(value) for value in spin_axis),
        predicted_object_rotation_rad=predicted_rotation,
        object_relative_displacement_at_catch_m=tuple(
            float(value) for value in object_relative_displacement
        ),
        max_joint_speed_rad_s=float(max_joint_speed),
        max_joint_acceleration_rad_s2=float(max_joint_acceleration),
        max_tcp_speed_m_s=max_tcp_speed,
        release_tool_z_world=tuple(
            float(value) for value in release_tool_z_world
        ),
        gripper_base_below_object_m=float(
            release_object_position[2] - gripper_base_position[2]
        ),
    )
