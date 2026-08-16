"""Deployable camera-to-intercept controller; no simulator state is accepted."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

from .ballistic_tracker import BallisticTracker
from .intercept_residual import InterceptResidualPolicy, residual_features


GRAVITY_BASE_M_S2 = np.asarray([0.0, 0.0, -9.81])


def _vector3(value, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must contain three finite values")
    return result


@dataclass(frozen=True)
class InterceptCommand:
    source_camera: str
    observation_time_s: float
    time_since_release_s: float
    prediction_horizon_s: float
    camera_sample_count: int
    fit_rms_m: float | None
    estimated_position_base_m: tuple[float, float, float]
    estimated_velocity_base_m_s: tuple[float, float, float]
    nominal_intercept_base_m: tuple[float, float, float]
    learned_residual_m: tuple[float, float, float]
    learned_residual_applied: bool
    corrected_intercept_base_m: tuple[float, float, float]

    def as_dict(self) -> dict:
        return asdict(self)


class OnlineInterceptController:
    """Fuse an encoder detach prior and asynchronous timestamped camera poses."""

    def __init__(
        self,
        *,
        release_command_time_s: float,
        prediction_horizon_s: float,
        policy: InterceptResidualPolicy,
        intercept_time_s: float | None = None,
        minimum_camera_samples: int = 1,
    ) -> None:
        self.release_command_time_s = float(release_command_time_s)
        self.prediction_horizon_s = float(prediction_horizon_s)
        if self.prediction_horizon_s <= 0.0:
            raise ValueError("prediction_horizon_s must be positive")
        self.intercept_time_s = (
            None if intercept_time_s is None else float(intercept_time_s)
        )
        self.minimum_camera_samples = int(minimum_camera_samples)
        if self.minimum_camera_samples < 1:
            raise ValueError("minimum_camera_samples must be positive")
        self.policy = policy
        self.tracker = BallisticTracker()
        self._prior_time_s: float | None = None
        self._prior_position: np.ndarray | None = None
        self._prior_velocity: np.ndarray | None = None

    @classmethod
    def from_checkpoint(
        cls,
        checkpoint: str | Path,
        *,
        release_command_time_s: float,
        prediction_horizon_s: float,
        intercept_time_s: float | None = None,
        minimum_camera_samples: int = 1,
    ) -> "OnlineInterceptController":
        return cls(
            release_command_time_s=release_command_time_s,
            prediction_horizon_s=prediction_horizon_s,
            policy=InterceptResidualPolicy.load(checkpoint),
            intercept_time_s=intercept_time_s,
            minimum_camera_samples=minimum_camera_samples,
        )

    def set_encoder_detach_prior(
        self,
        time_s: float,
        position_base_m,
        velocity_base_m_s,
    ) -> None:
        self._prior_time_s = float(time_s)
        self._prior_position = _vector3(position_base_m, "position_base_m")
        self._prior_velocity = _vector3(velocity_base_m_s, "velocity_base_m_s")
        self.tracker.set_encoder_prior(
            time_s,
            self._prior_position,
            self._prior_velocity,
        )

    def _prior_at(self, time_s: float) -> tuple[np.ndarray, np.ndarray]:
        if self._prior_time_s is None:
            raise RuntimeError("encoder detach prior must be set first")
        age = float(time_s) - self._prior_time_s
        position = (
            self._prior_position
            + self._prior_velocity * age
            + 0.5 * GRAVITY_BASE_M_S2 * age**2
        )
        velocity = self._prior_velocity + GRAVITY_BASE_M_S2 * age
        return position, velocity

    def add_camera_position(
        self,
        source_camera: str,
        time_s: float,
        position_base_m,
    ) -> InterceptCommand:
        if source_camera not in {"third_view", "wrist"}:
            raise ValueError("source_camera must be third_view or wrist")
        observation_time = float(time_s)
        prior_position, prior_velocity = self._prior_at(observation_time)
        self.tracker.add_camera_position(observation_time, position_base_m)
        estimate = self.tracker.estimate(observation_time)
        position = np.asarray(estimate.position_m)
        velocity = np.asarray(estimate.velocity_m_s)
        horizon = self.prediction_horizon_s
        if self.intercept_time_s is not None:
            horizon = max(0.0, self.intercept_time_s - observation_time)
        nominal_intercept = (
            position
            + velocity * horizon
            + 0.5 * GRAVITY_BASE_M_S2 * horizon**2
        )
        feature = residual_features(
            time_since_release_s=(
                observation_time - self.release_command_time_s
            ),
            camera_sample_count=estimate.camera_sample_count,
            fit_rms_m=estimate.fit_rms_m,
            position_innovation_m=position - prior_position,
            velocity_innovation_m_s=velocity - prior_velocity,
        )
        learned_applied = (
            estimate.camera_sample_count >= self.minimum_camera_samples
        )
        residual = (
            self.policy.predict(feature)
            if learned_applied
            else np.zeros(3, dtype=float)
        )
        corrected = nominal_intercept + residual
        return InterceptCommand(
            source_camera=source_camera,
            observation_time_s=observation_time,
            time_since_release_s=(
                observation_time - self.release_command_time_s
            ),
            prediction_horizon_s=horizon,
            camera_sample_count=estimate.camera_sample_count,
            fit_rms_m=estimate.fit_rms_m,
            estimated_position_base_m=tuple(float(v) for v in position),
            estimated_velocity_base_m_s=tuple(float(v) for v in velocity),
            nominal_intercept_base_m=tuple(float(v) for v in nominal_intercept),
            learned_residual_m=tuple(float(v) for v in residual),
            learned_residual_applied=learned_applied,
            corrected_intercept_base_m=tuple(float(v) for v in corrected),
        )

    def add_global_camera_position(
        self, time_s: float, position_base_m
    ) -> InterceptCommand:
        return self.add_camera_position(
            "third_view", time_s, position_base_m
        )

    def add_wrist_camera_position(
        self, time_s: float, position_base_m
    ) -> InterceptCommand:
        return self.add_camera_position("wrist", time_s, position_base_m)
