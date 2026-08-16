#!/usr/bin/env python3
"""Fit the bounded intercept residual from native rendered rollouts."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np


XARM_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(XARM_ROOT / "src"))

from xarm6_toss.intercept_residual import (  # noqa: E402
    FEATURE_NAMES,
    InterceptResidualPolicy,
    ridge_fit,
)


def load_rollout(path: Path, minimum_camera_samples: int):
    root = path if path.is_dir() else path.parent
    measurement_path = (
        path if path.is_file() else root / "camera_measurements.json"
    )
    summary = json.loads((root / "summary.json").read_text(encoding="utf-8"))
    detach_time_s = summary["detach_time_s"]
    measurements = json.loads(measurement_path.read_text(encoding="utf-8"))
    rows = []
    for measurement in measurements:
        if (
            "residual_feature" not in measurement
            or "intercept_residual_target_m" not in measurement
            or measurement["time_s"] <= detach_time_s
            or measurement["ballistic_camera_sample_count"]
            < minimum_camera_samples
        ):
            continue
        rows.append(
            (
                np.asarray(measurement["residual_feature"], dtype=float),
                np.asarray(
                    measurement["intercept_residual_target_m"], dtype=float
                ),
            )
        )
    if not rows:
        raise RuntimeError(f"no eligible residual samples in {root}")
    return root, rows


def policy_metrics(policy, rows):
    targets = np.asarray([target for _, target in rows])
    actions = np.asarray([policy.predict(feature) for feature, _ in rows])
    before = np.linalg.norm(targets, axis=1)
    after = np.linalg.norm(targets - actions, axis=1)
    return {
        "sample_count": len(rows),
        "mean_error_before_m": float(np.mean(before)),
        "mean_error_after_m": float(np.mean(after)),
        "p95_error_before_m": float(np.quantile(before, 0.95)),
        "p95_error_after_m": float(np.quantile(after, 0.95)),
        "fraction_improved": float(np.mean(after < before)),
        "mean_action_norm_m": float(
            np.mean(np.linalg.norm(actions, axis=1))
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("inputs", type=Path, nargs="+")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--minimum-camera-samples", type=int, default=3)
    parser.add_argument("--ridge", type=float, default=20.0)
    parser.add_argument("--action-norm-limit-m", type=float, default=0.012)
    args = parser.parse_args()
    if len(args.inputs) < 2:
        raise ValueError("at least two native rollouts are required")

    loaded = [
        load_rollout(path, args.minimum_camera_samples)
        for path in args.inputs
    ]
    training_rows = [row for _, rows in loaded[:-1] for row in rows]
    validation_rows = loaded[-1][1]
    features = np.asarray([feature for feature, _ in training_rows])
    targets = np.asarray([target for _, target in training_rows])
    mean, scale, weight, bias = ridge_fit(features, targets, args.ridge)
    payload = {
        "schema": "xarm6_intercept_residual_v1",
        "feature_names": list(FEATURE_NAMES),
        "feature_mean": mean.tolist(),
        "feature_scale": scale.tolist(),
        "weight": weight.tolist(),
        "bias": bias.tolist(),
        "action_norm_limit_m": args.action_norm_limit_m,
        "native_training": {
            "minimum_camera_samples": args.minimum_camera_samples,
            "ridge": args.ridge,
            "training_rollouts": [str(root) for root, _ in loaded[:-1]],
            "validation_rollout": str(loaded[-1][0]),
            "sim_truth_used_only_for_training_label": True,
        },
    }
    policy = InterceptResidualPolicy.from_dict(payload)
    payload["train_metrics"] = policy_metrics(policy, training_rows)
    payload["validation_metrics"] = policy_metrics(policy, validation_rows)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(payload, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(payload["train_metrics"], indent=2))
    print(json.dumps(payload["validation_metrics"], indent=2))
    print(f"checkpoint={args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
