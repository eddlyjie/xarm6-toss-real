"""Frozen small residual on top of the analytic ballistic intercept."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path

import numpy as np


FEATURE_NAMES = (
    "time_since_release_s",
    "camera_sample_count",
    "fit_rms_m",
    "position_innovation_x_m",
    "position_innovation_y_m",
    "position_innovation_z_m",
    "velocity_innovation_x_m_s",
    "velocity_innovation_y_m_s",
    "velocity_innovation_z_m_s",
)


def residual_features(
    *,
    time_since_release_s: float,
    camera_sample_count: int,
    fit_rms_m: float | None,
    position_innovation_m,
    velocity_innovation_m_s,
) -> np.ndarray:
    position = np.asarray(position_innovation_m, dtype=float)
    velocity = np.asarray(velocity_innovation_m_s, dtype=float)
    if position.shape != (3,) or velocity.shape != (3,):
        raise ValueError("position and velocity innovations must be 3-vectors")
    return np.asarray(
        [
            time_since_release_s,
            camera_sample_count,
            0.0 if fit_rms_m is None else fit_rms_m,
            *position,
            *velocity,
        ],
        dtype=float,
    )


@dataclass(frozen=True)
class InterceptResidualPolicy:
    feature_mean: np.ndarray
    feature_scale: np.ndarray
    weight: np.ndarray
    bias: np.ndarray
    action_norm_limit_m: float

    @classmethod
    def from_dict(cls, payload: dict) -> "InterceptResidualPolicy":
        if payload.get("schema") != "xarm6_intercept_residual_v1":
            raise ValueError("unsupported intercept residual schema")
        if tuple(payload["feature_names"]) != FEATURE_NAMES:
            raise ValueError("intercept residual feature order does not match runtime")
        policy = cls(
            feature_mean=np.asarray(payload["feature_mean"], dtype=float),
            feature_scale=np.asarray(payload["feature_scale"], dtype=float),
            weight=np.asarray(payload["weight"], dtype=float),
            bias=np.asarray(payload["bias"], dtype=float),
            action_norm_limit_m=float(payload["action_norm_limit_m"]),
        )
        feature_count = len(FEATURE_NAMES)
        if (
            policy.feature_mean.shape != (feature_count,)
            or policy.feature_scale.shape != (feature_count,)
            or policy.weight.shape != (3, feature_count)
            or policy.bias.shape != (3,)
        ):
            raise ValueError("intercept residual checkpoint has invalid dimensions")
        if np.any(policy.feature_scale <= 0.0):
            raise ValueError("intercept residual feature scales must be positive")
        if not math.isfinite(policy.action_norm_limit_m) or policy.action_norm_limit_m <= 0.0:
            raise ValueError("intercept residual action limit must be positive")
        return policy

    @classmethod
    def load(cls, path: str | Path) -> "InterceptResidualPolicy":
        return cls.from_dict(
            json.loads(Path(path).read_text(encoding="utf-8"))
        )

    def predict(self, feature) -> np.ndarray:
        feature = np.asarray(feature, dtype=float)
        if feature.shape != (len(FEATURE_NAMES),):
            raise ValueError("intercept residual feature has invalid dimensions")
        normalized = (feature - self.feature_mean) / self.feature_scale
        action = self.weight @ normalized + self.bias
        norm = float(np.linalg.norm(action))
        if norm > self.action_norm_limit_m:
            action = action * (self.action_norm_limit_m / norm)
        return action


def ridge_fit(
    features: np.ndarray,
    targets: np.ndarray,
    ridge: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    features = np.asarray(features, dtype=float)
    targets = np.asarray(targets, dtype=float)
    mean = np.mean(features, axis=0)
    scale = np.std(features, axis=0)
    scale[scale < 1.0e-9] = 1.0
    normalized = (features - mean) / scale
    design = np.column_stack((normalized, np.ones(len(normalized))))
    penalty = np.eye(design.shape[1]) * ridge
    penalty[-1, -1] = 0.0
    coefficient = np.linalg.solve(
        design.T @ design + penalty,
        design.T @ targets,
    )
    return mean, scale, coefficient[:-1].T, coefficient[-1]
