#!/usr/bin/env python3
"""Build a continuous object/pose-conditioned J2/J3/J5 warm-start candidate."""

from __future__ import annotations

import argparse
import copy
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


DEFAULT_DATASET = ROOT / "sim/data/four_object_pose_endpoints.json"
DYNAMIC_JOINTS = (1, 2, 4)
FIXED_JOINTS = (0, 3, 5)


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def principal_inertia(dimensions_m: list[float], mass_kg: float) -> list[float]:
    x_m, y_m, z_m = (float(value) for value in dimensions_m)
    return [
        mass_kg * (y_m**2 + z_m**2) / 12.0,
        mass_kg * (x_m**2 + z_m**2) / 12.0,
        mass_kg * (x_m**2 + y_m**2) / 12.0,
    ]


def load_dataset(path: Path = DEFAULT_DATASET) -> dict:
    payload = load_json(path)
    objects = []
    for row in payload["objects"]:
        profile = load_json(ROOT / row["object_profile"])
        dimensions = [float(value) for value in profile["dimensions_m"]]
        mass = float(profile["mass_kg"])
        inertia = profile.get("principal_inertia_kg_m2") or principal_inertia(
            dimensions, mass
        )
        endpoints = {}
        for name in ("low", "high"):
            endpoint = dict(row[name])
            summary = load_json(ROOT / endpoint["summary"])
            if not summary["catch_stable"]:
                raise ValueError(f"dataset endpoint is not a stable catch: {endpoint['summary']}")
            endpoint["measured_rotation_deg"] = float(
                summary["free_flight_rotation_deg"]
            )
            endpoint["signed_rotation_deg"] = float(
                summary["free_flight_signed_tumble_rotation_deg"]
            )
            endpoint["free_flight_s"] = float(
                summary["continuous_free_flight_duration_s"]
            )
            endpoints[name] = endpoint
        if endpoints["high"]["measured_rotation_deg"] <= endpoints["low"]["measured_rotation_deg"]:
            raise ValueError(f"high endpoint must exceed low endpoint for {profile['object_id']}")
        calibration_points = [
            {"action_alpha": 0.0, "measured_rotation_deg": endpoints["low"]["measured_rotation_deg"]},
            {"action_alpha": 1.0, "measured_rotation_deg": endpoints["high"]["measured_rotation_deg"]},
        ]
        for trial in row.get("additional_stable_trials", []):
            summary = load_json(ROOT / trial["summary"])
            if not summary.get("catch_stable", False):
                raise ValueError(f"additional response trial is not a stable catch: {trial['summary']}")
            calibration_points.append({
                "action_alpha": float(trial["action_alpha"]),
                "measured_rotation_deg": float(summary["free_flight_rotation_deg"]),
                "summary": trial["summary"],
            })
        calibration_points.sort(key=lambda point: point["action_alpha"])
        objects.append(
            {
                "object_profile": row["object_profile"],
                "object_id": profile["object_id"],
                "dimensions_m": dimensions,
                "mass_kg": mass,
                "principal_inertia_kg_m2": [float(value) for value in inertia],
                "grip_width_m": float(profile.get("grasp", {}).get("gripped_dimension_m", dimensions[1])),
                "sim_open_loop_calibration": profile.get("sim_open_loop_calibration"),
                "calibration_points": calibration_points,
                **endpoints,
            }
        )
    return {"schema": payload["schema"], "objects": objects}


def lerp(a: float, b: float, alpha: float) -> float:
    return (1.0 - alpha) * float(a) + alpha * float(b)


def object_row(dataset: dict, object_id: str) -> dict:
    for row in dataset["objects"]:
        if row["object_id"] == object_id:
            return row
    raise KeyError(f"unknown object_id: {object_id}")


def response_angle(row: dict, alpha: float) -> float:
    points = row["calibration_points"]
    return float(np.interp(
        float(alpha),
        [point["action_alpha"] for point in points],
        [point["measured_rotation_deg"] for point in points],
    ))


def action_for_angle(row: dict, desired_angle_deg: float) -> float:
    points = sorted(
        row["calibration_points"], key=lambda point: point["measured_rotation_deg"]
    )
    return float(np.clip(np.interp(
        float(desired_angle_deg),
        [point["measured_rotation_deg"] for point in points],
        [point["action_alpha"] for point in points],
    ), 0.0, 1.0))


def score_candidate(row: dict, desired_angle_deg: float, alpha: float) -> dict:
    predicted = response_angle(row, alpha)
    terms = {
        "angle_error_deg": abs(predicted - desired_angle_deg),
        "action_strength_cost": 0.05 * alpha,
        "extrapolation_cost": 20.0 * max(0.0, -alpha, alpha - 1.0),
    }
    return {
        "alpha": float(alpha),
        "predicted_rotation_deg": predicted,
        "j_terms": terms,
        "j": float(sum(terms.values())),
    }


def compare_methods(dataset: dict, row: dict, desired_angle_deg: float) -> dict:
    cube = object_row(dataset, "yellow_cube_38mm_8g")
    cube_span = cube["high"]["measured_rotation_deg"] - cube["low"]["measured_rotation_deg"]
    alpha_m0 = np.clip(
        (desired_angle_deg - cube["low"]["measured_rotation_deg"]) / cube_span,
        0.0,
        1.0,
    )
    cube_inertia = cube["principal_inertia_kg_m2"][1]
    inertia_scale = math.sqrt(row["principal_inertia_kg_m2"][1] / cube_inertia)
    alpha_m1 = float(np.clip(alpha_m0 * inertia_scale, 0.0, 1.0))
    grid = np.linspace(0.0, 1.0, 11)
    m2_candidates = [score_candidate(row, desired_angle_deg, float(value)) for value in grid]
    m2 = min(m2_candidates, key=lambda item: (item["j"], item["alpha"]))
    alpha_m3 = action_for_angle(row, desired_angle_deg)
    return {
        "M0_fixed_o0_pose_policy": score_candidate(row, desired_angle_deg, float(alpha_m0)),
        "M1_inertia_scaled": {
            **score_candidate(row, desired_angle_deg, alpha_m1),
            "inertia_scale": inertia_scale,
        },
        "M2_search_only": {**m2, "grid_size": len(grid)},
        "M3_object_pose_conditioned": score_candidate(row, desired_angle_deg, alpha_m3),
    }


def sampled_reference(config: dict):
    return generate_joint_reference(
        tuple(
            QuinticJointSegment(**segment)
            for segment in config["reference_segments"]
        ),
        float(config["control_period_s"]),
    )


def interpolate_arm_config(row: dict, alpha: float, desired_angle_deg: float) -> dict:
    low = load_json(ROOT / row["low"]["arm_config"])
    high = load_json(ROOT / row["high"]["arm_config"])
    if len(low["reference_segments"]) != len(high["reference_segments"]):
        raise ValueError("endpoint arm configs have different segment counts")
    config = copy.deepcopy(low)
    config["name"] = f"pose_conditioned_{row['object_id']}_{desired_angle_deg:g}deg"
    control_period_s = float(config["control_period_s"])
    if not math.isclose(control_period_s, float(high["control_period_s"])):
        raise ValueError("endpoint arm configs use different control periods")
    low_samples = sampled_reference(low)
    high_samples = sampled_reference(high)
    if len(low_samples) != len(high_samples):
        raise ValueError("endpoint arm configs have different sampled durations")

    # Interpolate executable q/dq/ddq at each 20 ms control tick.  Interpolating
    # the five coarse quintic segments directly is unsafe: the low/high phase
    # boundaries occur at different times and their midpoint can overshoot both
    # endpoint speed and acceleration limits.  Tick-space interpolation is a
    # convex combination of two already validated command references.
    blended_samples = []
    for low_sample, high_sample in zip(low_samples, high_samples, strict=True):
        if not math.isclose(low_sample.time_s, high_sample.time_s, abs_tol=1.0e-12):
            raise ValueError("endpoint arm configs have different sample times")
        blended_samples.append(
            {
                "time_s": low_sample.time_s,
                "joint_position_rad": [
                    lerp(a, b, alpha)
                    for a, b in zip(low_sample.joint_position_rad, high_sample.joint_position_rad, strict=True)
                ],
                "joint_velocity_rad_s": [
                    lerp(a, b, alpha)
                    for a, b in zip(low_sample.joint_velocity_rad_s, high_sample.joint_velocity_rad_s, strict=True)
                ],
                "joint_acceleration_rad_s2": [
                    lerp(a, b, alpha)
                    for a, b in zip(low_sample.joint_acceleration_rad_s2, high_sample.joint_acceleration_rad_s2, strict=True)
                ],
            }
        )
    interpolated = []
    for index, (start, end) in enumerate(
        zip(blended_samples[:-1], blended_samples[1:], strict=True)
    ):
        interpolated.append({
            "phase": f"pose_conditioned_tick_{index:03d}",
            "duration_s": control_period_s,
            "start_joint_rad": start["joint_position_rad"],
            "start_joint_velocity_rad_s": start["joint_velocity_rad_s"],
            "start_joint_acceleration_rad_s2": start["joint_acceleration_rad_s2"],
            "end_joint_rad": end["joint_position_rad"],
            "end_joint_velocity_rad_s": end["joint_velocity_rad_s"],
            "end_joint_acceleration_rad_s2": end["joint_acceleration_rad_s2"],
        })
    config["reference_segments"] = interpolated
    config.pop("kinematic_design", None)
    config["pose_conditioned_generation"] = {
        "method": "supervised_low_high_control_tick_interpolation",
        "object_id": row["object_id"],
        "object_context": {
            "dimensions_m": row["dimensions_m"],
            "mass_kg": row["mass_kg"],
            "principal_inertia_kg_m2": row["principal_inertia_kg_m2"],
            "grip_width_m": row["grip_width_m"],
        },
        "desired_angle_deg": desired_angle_deg,
        "response_model": "piecewise_linear_stable_same_object_trials",
        "alpha": alpha,
        "calibration_points": row["calibration_points"],
        "low_config": row["low"]["arm_config"],
        "high_config": row["high"]["arm_config"],
    }
    samples = generate_joint_reference(
        tuple(QuinticJointSegment(**segment) for segment in interpolated),
        control_period_s,
    )
    limits = evaluate_reference_samples(samples)
    if not limits["joint_mechanical_limits_pass"]:
        raise ValueError(f"interpolated arm reference exceeds the handoff envelope: {limits}")
    config["pose_conditioned_generation"]["reference_limit_evidence"] = limits
    return config


def build_candidate(dataset: dict, object_id: str, desired_angle_deg: float) -> tuple[dict, dict]:
    row = object_row(dataset, object_id)
    methods = compare_methods(dataset, row, desired_angle_deg)
    selected = methods["M3_object_pose_conditioned"]
    alpha = selected["alpha"]
    config = interpolate_arm_config(row, alpha, desired_angle_deg)
    calibration = row.get("sim_open_loop_calibration")
    if not calibration:
        raise ValueError(f"object has no successful Sim open-loop calibration: {object_id}")
    timing = {
        key: lerp(row["low"][key], row["high"][key], alpha)
        for key in (
            "catch_intercept_time_s",
            "catch_preclose_time_s",
            "catch_close_time_s",
        )
    }
    report = {
        "schema": "xarm6_object_pose_conditioned_candidate_v1",
        "object_id": object_id,
        "object_profile": row["object_profile"],
        "desired_angle_deg": desired_angle_deg,
        "training_boundary": "piecewise_interpolation_among_stable_same_object_trials",
        "formal_unseen_generalization": False,
        "methods": methods,
        "selected_method": "M3_object_pose_conditioned",
        "selected_action": {
            "alpha": alpha,
            **timing,
            **calibration,
        },
        "predicted_rotation_deg": selected["predicted_rotation_deg"],
    }
    return config, report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", type=Path, default=DEFAULT_DATASET)
    parser.add_argument("--object-id", required=True)
    parser.add_argument("--desired-angle-deg", type=float, required=True)
    parser.add_argument("--output-config", type=Path, required=True)
    parser.add_argument("--output-report", type=Path, required=True)
    args = parser.parse_args()
    dataset = load_dataset(args.dataset)
    config, report = build_candidate(
        dataset, args.object_id, float(args.desired_angle_deg)
    )
    args.output_config.parent.mkdir(parents=True, exist_ok=True)
    args.output_report.parent.mkdir(parents=True, exist_ok=True)
    args.output_config.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
    args.output_report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
