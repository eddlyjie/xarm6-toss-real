"""Version-independent q/dq references and G1 event timing for sim and real."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


ARM_DOF = 6
G1_REAL_POSITION_MIN = 0.0
G1_REAL_POSITION_MAX = 850.0
G1_DRIVE_JOINT_MAX_RAD = 0.85


def _joint_vector(values, name: str) -> np.ndarray:
    result = np.asarray(values, dtype=float)
    if result.shape != (ARM_DOF,):
        raise ValueError(f"{name} must contain {ARM_DOF} joint values")
    return result


def g1_position_to_drive_joint_rad(position: float) -> float:
    """Map the real G1 0..850 command to its official 0..0.85 URDF joint."""

    value = float(position)
    if not G1_REAL_POSITION_MIN <= value <= G1_REAL_POSITION_MAX:
        raise ValueError("G1 position must lie in [0, 850]")
    return value * (G1_DRIVE_JOINT_MAX_RAD / G1_REAL_POSITION_MAX)


def g1_drive_joint_rad_to_position(angle_rad: float) -> float:
    value = float(angle_rad)
    if not 0.0 <= value <= G1_DRIVE_JOINT_MAX_RAD:
        raise ValueError("G1 drive joint must lie in [0, 0.85] rad")
    return value * (G1_REAL_POSITION_MAX / G1_DRIVE_JOINT_MAX_RAD)


@dataclass(frozen=True)
class QuinticJointSegment:
    phase: str
    duration_s: float
    start_joint_rad: tuple[float, ...]
    start_joint_velocity_rad_s: tuple[float, ...]
    end_joint_rad: tuple[float, ...]
    end_joint_velocity_rad_s: tuple[float, ...]
    start_joint_acceleration_rad_s2: tuple[float, ...] = (0.0,) * ARM_DOF
    end_joint_acceleration_rad_s2: tuple[float, ...] = (0.0,) * ARM_DOF


@dataclass(frozen=True)
class JointReferenceSample:
    time_s: float
    phase: str
    joint_position_rad: tuple[float, ...]
    joint_velocity_rad_s: tuple[float, ...]
    joint_acceleration_rad_s2: tuple[float, ...]


@dataclass(frozen=True)
class GripperEvent:
    time_s: float
    name: str
    real_position: float

    @property
    def drive_joint_rad(self) -> float:
        return g1_position_to_drive_joint_rad(self.real_position)


def _quintic_coefficients(segment: QuinticJointSegment) -> np.ndarray:
    duration = float(segment.duration_s)
    if duration <= 0.0:
        raise ValueError("segment duration must be positive")
    q0 = _joint_vector(segment.start_joint_rad, "start_joint_rad")
    v0 = _joint_vector(
        segment.start_joint_velocity_rad_s,
        "start_joint_velocity_rad_s",
    )
    q1 = _joint_vector(segment.end_joint_rad, "end_joint_rad")
    v1 = _joint_vector(
        segment.end_joint_velocity_rad_s,
        "end_joint_velocity_rad_s",
    )
    a0 = _joint_vector(
        segment.start_joint_acceleration_rad_s2,
        "start_joint_acceleration_rad_s2",
    )
    a1 = _joint_vector(
        segment.end_joint_acceleration_rad_s2,
        "end_joint_acceleration_rad_s2",
    )
    coefficients = np.zeros((ARM_DOF, 6), dtype=float)
    coefficients[:, 0] = q0
    coefficients[:, 1] = v0
    coefficients[:, 2] = 0.5 * a0
    system = np.asarray(
        [
            [duration**3, duration**4, duration**5],
            [3.0 * duration**2, 4.0 * duration**3, 5.0 * duration**4],
            [6.0 * duration, 12.0 * duration**2, 20.0 * duration**3],
        ]
    )
    target = np.stack(
        (
            q1 - q0 - v0 * duration - 0.5 * a0 * duration**2,
            v1 - v0 - a0 * duration,
            a1 - a0,
        ),
        axis=0,
    )
    coefficients[:, 3:] = np.linalg.solve(system, target).T
    return coefficients


def _evaluate(coefficients: np.ndarray, time_s: float):
    t = float(time_s)
    position_basis = np.asarray([1.0, t, t**2, t**3, t**4, t**5])
    velocity_basis = np.asarray([0.0, 1.0, 2.0 * t, 3.0 * t**2, 4.0 * t**3, 5.0 * t**4])
    acceleration_basis = np.asarray(
        [0.0, 0.0, 2.0, 6.0 * t, 12.0 * t**2, 20.0 * t**3]
    )
    return (
        coefficients @ position_basis,
        coefficients @ velocity_basis,
        coefficients @ acceleration_basis,
    )


def generate_joint_reference(
    segments: tuple[QuinticJointSegment, ...],
    control_period_s: float,
) -> list[JointReferenceSample]:
    """Sample continuous paired q/dq segments without duplicate boundaries."""

    period = float(control_period_s)
    if period <= 0.0:
        raise ValueError("control period must be positive")
    samples: list[JointReferenceSample] = []
    elapsed = 0.0
    previous_end_q = None
    previous_end_velocity = None
    for segment_index, segment in enumerate(segments):
        start_q = _joint_vector(segment.start_joint_rad, "start_joint_rad")
        start_velocity = _joint_vector(
            segment.start_joint_velocity_rad_s,
            "start_joint_velocity_rad_s",
        )
        if previous_end_q is not None and (
            not np.allclose(start_q, previous_end_q)
            or not np.allclose(start_velocity, previous_end_velocity)
        ):
            raise ValueError("adjacent joint segments must be q/dq continuous")
        coefficients = _quintic_coefficients(segment)
        steps = round(segment.duration_s / period)
        if not np.isclose(steps * period, segment.duration_s):
            raise ValueError("segment duration must be a multiple of control period")
        first_step = 0 if segment_index == 0 else 1
        for step in range(first_step, steps + 1):
            local_time = step * period
            q, dq, ddq = _evaluate(coefficients, local_time)
            samples.append(
                JointReferenceSample(
                    time_s=elapsed + local_time,
                    phase=segment.phase,
                    joint_position_rad=tuple(float(value) for value in q),
                    joint_velocity_rad_s=tuple(float(value) for value in dq),
                    joint_acceleration_rad_s2=tuple(float(value) for value in ddq),
                )
            )
        previous_end_q = _joint_vector(segment.end_joint_rad, "end_joint_rad")
        previous_end_velocity = _joint_vector(
            segment.end_joint_velocity_rad_s,
            "end_joint_velocity_rad_s",
        )
        elapsed += segment.duration_s
    return samples
