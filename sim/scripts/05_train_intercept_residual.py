#!/usr/bin/env python3
"""Train the small deployable intercept residual on randomized flight states."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np


XARM_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(XARM_ROOT / "src"))

from xarm6_toss.ballistic_tracker import BallisticTracker
from xarm6_toss.intercept_residual import (
    FEATURE_NAMES,
    InterceptResidualPolicy,
    residual_features,
    ridge_fit,
)


GRAVITY = np.asarray([0.0, 0.0, -9.81])
RELEASE_TIME_S = 0.30
LOOKAHEAD_S = 0.09
NOMINAL_RELEASE_POSITION_M = np.asarray([0.1878, 0.0125, 0.3736])
NOMINAL_RELEASE_VELOCITY_M_S = np.asarray([0.308, 0.036, 0.452])
RENDERED_CAMERA_BIAS_M = np.asarray([0.0, -0.013, 0.0105])
ACTION_LIMIT_M = 0.020


def propagate(position, velocity, dt):
    return (
        position + velocity * dt + 0.5 * GRAVITY * dt**2,
        velocity + GRAVITY * dt,
    )


def clip_action(action):
    norm = float(np.linalg.norm(action))
    if norm > ACTION_LIMIT_M:
        return action * (ACTION_LIMIT_M / norm)
    return action


def make_example(
    rng: np.random.Generator,
    *,
    release_time_s: float = RELEASE_TIME_S,
    lookahead_s: float = LOOKAHEAD_S,
    nominal_release_position_m=NOMINAL_RELEASE_POSITION_M,
    nominal_release_velocity_m_s=NOMINAL_RELEASE_VELOCITY_M_S,
):
    true_release_position = (
        nominal_release_position_m
        + rng.normal(scale=[0.003, 0.003, 0.003])
    )
    true_release_velocity = (
        nominal_release_velocity_m_s
        + rng.normal(scale=[0.050, 0.035, 0.080])
    )
    prior_release_position = (
        true_release_position
        + rng.normal(scale=[0.004, 0.004, 0.004])
    )
    prior_release_velocity = (
        true_release_velocity
        + rng.normal(scale=[0.080, 0.055, 0.130])
    )
    camera_bias = (
        RENDERED_CAMERA_BIAS_M
        + rng.normal(scale=[0.007, 0.007, 0.006])
    )
    observation_time = release_time_s + float(
        rng.choice([0.04, 0.06, 0.08])
    )
    tracker = BallisticTracker(max_camera_samples=4)
    tracker.set_encoder_prior(
        release_time_s,
        prior_release_position,
        prior_release_velocity,
    )
    candidate_times = np.arange(
        release_time_s + 0.02, observation_time + 1.0e-9, 0.02
    )
    kept = []
    for camera_time in candidate_times:
        if rng.random() < 0.16:
            continue
        true_position, _ = propagate(
            true_release_position,
            true_release_velocity,
            camera_time - release_time_s,
        )
        measurement = (
            true_position
            + camera_bias
            + rng.normal(scale=[0.0025, 0.0025, 0.0025])
        )
        tracker.add_camera_position(float(camera_time), measurement)
        kept.append(camera_time)
    if not kept:
        camera_time = observation_time
        true_position, _ = propagate(
            true_release_position,
            true_release_velocity,
            camera_time - release_time_s,
        )
        tracker.add_camera_position(
            camera_time,
            true_position + camera_bias + rng.normal(scale=0.0025, size=3),
        )
    estimate = tracker.estimate(observation_time)
    estimated_position = np.asarray(estimate.position_m)
    estimated_velocity = np.asarray(estimate.velocity_m_s)
    prior_position, prior_velocity = propagate(
        prior_release_position,
        prior_release_velocity,
        observation_time - release_time_s,
    )
    feature = residual_features(
        time_since_release_s=observation_time - release_time_s,
        camera_sample_count=estimate.camera_sample_count,
        fit_rms_m=estimate.fit_rms_m,
        position_innovation_m=estimated_position - prior_position,
        velocity_innovation_m_s=estimated_velocity - prior_velocity,
    )
    predicted_intercept, _ = propagate(
        estimated_position,
        estimated_velocity,
        lookahead_s,
    )
    true_position_now, true_velocity_now = propagate(
        true_release_position,
        true_release_velocity,
        observation_time - release_time_s,
    )
    true_intercept, _ = propagate(
        true_position_now,
        true_velocity_now,
        lookahead_s,
    )
    target = clip_action(true_intercept - predicted_intercept)
    return feature, target, true_intercept - predicted_intercept


def make_dataset(rng, count, **example_kwargs):
    examples = [
        make_example(rng, **example_kwargs) for _ in range(count)
    ]
    return tuple(np.asarray(values) for values in zip(*examples))


def metrics(policy, features, raw_targets):
    actions = np.asarray([policy.predict(feature) for feature in features])
    before = np.linalg.norm(raw_targets, axis=1)
    after = np.linalg.norm(raw_targets - actions, axis=1)
    return {
        "mean_error_before_m": float(np.mean(before)),
        "mean_error_after_m": float(np.mean(after)),
        "p95_error_before_m": float(np.quantile(before, 0.95)),
        "p95_error_after_m": float(np.quantile(after, 0.95)),
        "fraction_improved": float(np.mean(after < before)),
        "mean_action_norm_m": float(np.mean(np.linalg.norm(actions, axis=1))),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--train-samples", type=int, default=12000)
    parser.add_argument("--validation-samples", type=int, default=3000)
    parser.add_argument("--seed", type=int, default=20260816)
    parser.add_argument("--ridge", type=float, default=5.0)
    parser.add_argument("--release-time-s", type=float, default=RELEASE_TIME_S)
    parser.add_argument("--lookahead-s", type=float, default=LOOKAHEAD_S)
    parser.add_argument(
        "--nominal-release-position-m", type=float, nargs=3,
        default=NOMINAL_RELEASE_POSITION_M.tolist(),
    )
    parser.add_argument(
        "--nominal-release-velocity-m-s", type=float, nargs=3,
        default=NOMINAL_RELEASE_VELOCITY_M_S.tolist(),
    )
    args = parser.parse_args()
    rng = np.random.default_rng(args.seed)
    example_kwargs = {
        "release_time_s": args.release_time_s,
        "lookahead_s": args.lookahead_s,
        "nominal_release_position_m": np.asarray(args.nominal_release_position_m),
        "nominal_release_velocity_m_s": np.asarray(args.nominal_release_velocity_m_s),
    }
    train_features, train_targets, train_raw = make_dataset(
        rng, args.train_samples, **example_kwargs
    )
    validation_features, _, validation_raw = make_dataset(
        rng, args.validation_samples, **example_kwargs
    )
    mean, scale, weight, bias = ridge_fit(
        train_features,
        train_targets,
        args.ridge,
    )
    payload = {
        "schema": "xarm6_intercept_residual_v1",
        "feature_names": list(FEATURE_NAMES),
        "feature_mean": mean.tolist(),
        "feature_scale": scale.tolist(),
        "weight": weight.tolist(),
        "bias": bias.tolist(),
        "action_norm_limit_m": ACTION_LIMIT_M,
        "release_time_s": args.release_time_s,
        "lookahead_s": args.lookahead_s,
        "nominal_release_position_m": args.nominal_release_position_m,
        "nominal_release_velocity_m_s": args.nominal_release_velocity_m_s,
        "training_seed": args.seed,
        "training_samples": args.train_samples,
        "validation_samples": args.validation_samples,
        "randomization": {
            "camera_rate_hz": 50.0,
            "camera_dropout_probability": 0.16,
            "rendered_camera_bias_center_m": RENDERED_CAMERA_BIAS_M.tolist(),
            "camera_noise_std_m": 0.0025,
            "release_velocity_std_m_s": [0.050, 0.035, 0.080],
            "encoder_prior_velocity_error_std_m_s": [0.080, 0.055, 0.130]
        },
    }
    policy = InterceptResidualPolicy.from_dict(payload)
    payload["train_metrics"] = metrics(policy, train_features, train_raw)
    payload["validation_metrics"] = metrics(
        policy,
        validation_features,
        validation_raw,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(payload["validation_metrics"], indent=2))
    print(f"checkpoint={args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
