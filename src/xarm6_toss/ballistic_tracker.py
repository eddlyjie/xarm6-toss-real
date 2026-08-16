"""Deployable encoder-prior plus gravity-constrained RGB-D flight tracker."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


def _vector3(value, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if array.shape != (3,) or not np.isfinite(array).all():
        raise ValueError(f"{name} must contain three finite values")
    return array


@dataclass(frozen=True)
class BallisticEstimate:
    time_s: float
    position_m: tuple[float, float, float]
    velocity_m_s: tuple[float, float, float]
    camera_sample_count: int
    fit_rms_m: float | None


class BallisticTracker:
    """Fit p/v from camera positions while treating gravity as known physics."""

    def __init__(
        self,
        gravity_m_s2=(0.0, 0.0, -9.81),
        max_camera_samples: int = 6,
    ) -> None:
        self.gravity = _vector3(gravity_m_s2, "gravity_m_s2")
        self.max_camera_samples = int(max_camera_samples)
        if self.max_camera_samples < 2:
            raise ValueError("max_camera_samples must be at least two")
        self._prior_time_s: float | None = None
        self._prior_position: np.ndarray | None = None
        self._prior_velocity: np.ndarray | None = None
        self._camera_times: list[float] = []
        self._camera_positions: list[np.ndarray] = []

    @property
    def camera_sample_count(self) -> int:
        return len(self._camera_times)

    def set_encoder_prior(self, time_s: float, position_m, velocity_m_s) -> None:
        if not math.isfinite(time_s):
            raise ValueError("prior time must be finite")
        self._prior_time_s = float(time_s)
        self._prior_position = _vector3(position_m, "position_m").copy()
        self._prior_velocity = _vector3(velocity_m_s, "velocity_m_s").copy()

    def add_camera_position(self, time_s: float, position_m) -> None:
        if not math.isfinite(time_s):
            raise ValueError("camera time must be finite")
        if self._camera_times and time_s <= self._camera_times[-1]:
            raise ValueError("camera measurements must have increasing times")
        self._camera_times.append(float(time_s))
        self._camera_positions.append(
            _vector3(position_m, "position_m").copy()
        )
        if len(self._camera_times) > self.max_camera_samples:
            self._camera_times.pop(0)
            self._camera_positions.pop(0)

    def _prior_at(self, time_s: float) -> tuple[np.ndarray, np.ndarray]:
        if self._prior_time_s is None:
            raise RuntimeError("ballistic tracker has no encoder prior")
        dt = float(time_s) - self._prior_time_s
        position = (
            self._prior_position
            + self._prior_velocity * dt
            + 0.5 * self.gravity * dt**2
        )
        velocity = self._prior_velocity + self.gravity * dt
        return position, velocity

    def estimate(self, time_s: float) -> BallisticEstimate:
        if not math.isfinite(time_s):
            raise ValueError("estimate time must be finite")
        query_time = float(time_s)
        if not self._camera_times:
            position, velocity = self._prior_at(query_time)
            fit_rms = None
        elif len(self._camera_times) == 1:
            measurement_time = self._camera_times[0]
            measurement_position = self._camera_positions[0]
            _, prior_velocity = self._prior_at(measurement_time)
            dt = query_time - measurement_time
            position = (
                measurement_position
                + prior_velocity * dt
                + 0.5 * self.gravity * dt**2
            )
            velocity = prior_velocity + self.gravity * dt
            fit_rms = None
        else:
            reference_time = self._camera_times[-1]
            relative_times = np.asarray(self._camera_times) - reference_time
            positions = np.asarray(self._camera_positions)
            gravity_displacement = (
                0.5 * relative_times[:, None] ** 2 * self.gravity
            )
            corrected_positions = positions - gravity_displacement
            design = np.column_stack(
                (np.ones(len(relative_times)), relative_times)
            )
            coefficients, _, _, _ = np.linalg.lstsq(
                design,
                corrected_positions,
                rcond=None,
            )
            reference_position = coefficients[0]
            reference_velocity = coefficients[1]
            fitted = design @ coefficients + gravity_displacement
            fit_rms = float(
                np.sqrt(np.mean(np.sum((fitted - positions) ** 2, axis=1)))
            )
            dt = query_time - reference_time
            position = (
                reference_position
                + reference_velocity * dt
                + 0.5 * self.gravity * dt**2
            )
            velocity = reference_velocity + self.gravity * dt
        return BallisticEstimate(
            time_s=query_time,
            position_m=tuple(float(value) for value in position),
            velocity_m_s=tuple(float(value) for value in velocity),
            camera_sample_count=len(self._camera_times),
            fit_rms_m=fit_rms,
        )
