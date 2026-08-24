"""Planning and validation for camera-free xArm6 object demos.

This module is side-effect free: importing it never connects to a robot.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

from .motion_limits import evaluate_joint_trajectory
from .real_dynamic_regrasp import resample_timeline


PROFILE_SCHEMA = "xarm6_open_loop_flip_profile_v1"
OBJECT_SCHEMA = "xarm6_demo_object_v1"
CATALOG_PATH = Path("configs/open_loop_flip/catalog.json")
EXECUTION_MODES = {"empty_arm", "empty_g1", "throw_only", "cube", "object"}


class DeploymentPlanError(ValueError):
    """Raised when a requested demo has no deployable exact profile."""


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def profile_for_angle(root: Path, desired_angle_deg: float) -> Path:
    catalog = load_json(root / CATALOG_PATH)
    for item in catalog["profiles"]:
        if abs(float(item["desired_angle_deg"]) - desired_angle_deg) <= 1.0e-9:
            if item["profile"] is None:
                raise DeploymentPlanError(
                    f"{desired_angle_deg:g} degree profile is {item['status']} and has no trajectory"
                )
            return root / item["profile"]
    raise DeploymentPlanError(
        f"no exact profile for {desired_angle_deg:g} degrees; nearest-angle fallback is disabled"
    )


def _validate_object(config: dict[str, Any]) -> None:
    if config.get("schema") != OBJECT_SCHEMA:
        raise DeploymentPlanError("unsupported object profile schema")
    dimensions = np.asarray(config.get("dimensions_m"), dtype=float)
    if dimensions.shape != (3,) or np.any(dimensions <= 0.0):
        raise DeploymentPlanError("object dimensions must contain three positive values")
    mass_kg = float(config.get("mass_kg", -1.0))
    if not np.isfinite(mass_kg) or mass_kg <= 0.0:
        raise DeploymentPlanError("object mass must be positive and finite")


def _validate_profile(profile: dict[str, Any], timeline: dict[str, Any]) -> None:
    if profile.get("schema") != PROFILE_SCHEMA:
        raise DeploymentPlanError("unsupported open-loop profile schema")
    if abs(float(profile.get("control_period_s", 0.0)) - 0.02) > 1.0e-12:
        raise DeploymentPlanError("real demo profiles must use the measured 20 ms period")
    samples = timeline.get("samples")
    if not isinstance(samples, list) or len(samples) < 2:
        raise DeploymentPlanError("timeline must contain at least two samples")
    times = np.asarray([sample["time_s"] for sample in samples], dtype=float)
    if np.any(np.diff(times) <= 0.0):
        raise DeploymentPlanError("timeline sample times must be strictly increasing")
    for field in ("joint_position_rad", "joint_velocity_rad_s"):
        values = np.asarray([sample[field] for sample in samples], dtype=float)
        if values.shape != (len(samples), 6) or not np.all(np.isfinite(values)):
            raise DeploymentPlanError(f"timeline {field} must have finite shape (N, 6)")
    events = profile["g1"]["events"]
    event_times = [float(event["time_s"]) for event in events]
    if event_times != sorted(event_times) or event_times[-1] > times[-1] + 1.0e-9:
        raise DeploymentPlanError("G1 events must be ordered and inside the arm timeline")
    allowed = set(profile.get("hardware_modes_allowed", []))
    if not allowed or not allowed <= EXECUTION_MODES:
        raise DeploymentPlanError("profile hardware_modes_allowed is invalid")
    held_position = profile["g1"].get("held_position")
    missing_positions = [event["name"] for event in events if event.get("position") is None]
    if held_position is None and allowed & {"empty_g1", "throw_only", "cube", "object"}:
        raise DeploymentPlanError(
            "profile enables G1 motion before held position is calibrated"
        )
    if missing_positions and allowed & {"empty_g1", "throw_only", "cube", "object"}:
        raise DeploymentPlanError(
            "profile enables G1 motion before real positions are calibrated: "
            + ", ".join(missing_positions)
        )


def events_for_mode(
    profile: dict[str, Any], mode: str, *, speed_scale: float
) -> list[dict[str, Any]]:
    if mode == "empty_arm":
        return []
    events = profile["g1"]["events"]
    if mode == "throw_only":
        events = [event for event in events if event["name"] == "release"]
    return [
        {**event, "time_s": float(event["time_s"]) / speed_scale}
        for event in events
    ]


def _display_path(path: Path, root: Path) -> str:
    try:
        return str(path.resolve().relative_to(root.resolve()))
    except ValueError:
        return str(path.resolve())


def prepare_deployment(
    root: Path,
    *,
    desired_angle_deg: float | None = None,
    profile_path: Path | None = None,
    speed_scale: float = 1.0,
    mode: str = "plan",
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    root = root.resolve()
    if profile_path is None:
        if desired_angle_deg is None:
            raise DeploymentPlanError("desired_angle_deg or profile_path is required")
        profile_path = profile_for_angle(root, float(desired_angle_deg))
    elif not profile_path.is_absolute():
        profile_path = root / profile_path
    if speed_scale not in {0.25, 0.5, 1.0}:
        raise DeploymentPlanError("speed_scale must be 0.25, 0.5, or 1.0")
    if mode != "plan" and mode not in EXECUTION_MODES:
        raise DeploymentPlanError(f"unsupported execution mode: {mode}")

    profile = load_json(profile_path)
    timeline_path = root / profile["timeline"]
    object_path = root / profile["object_profile"]
    timeline = load_json(timeline_path)
    object_config = load_json(object_path)
    _validate_object(object_config)
    _validate_profile(profile, timeline)
    if desired_angle_deg is not None and abs(
        float(profile["desired_angle_deg"]) - float(desired_angle_deg)
    ) > 1.0e-9:
        raise DeploymentPlanError("profile desired angle does not match the request")
    if mode != "plan" and mode not in profile["hardware_modes_allowed"]:
        raise DeploymentPlanError(
            f"profile {profile['profile_id']} does not allow {mode}; status={profile['status']}"
        )
    if mode in {"throw_only", "cube", "object"} and speed_scale != 1.0:
        raise DeploymentPlanError("object trials require the validated 1.0 time scale")

    samples = resample_timeline(timeline["samples"], speed_scale=speed_scale)
    source_times = np.asarray(
        [sample["time_s"] for sample in timeline["samples"]], dtype=float
    )
    source_acceleration = np.asarray(
        [sample["joint_acceleration_rad_s2"] for sample in timeline["samples"]],
        dtype=float,
    )
    for sample in samples:
        source_time = float(sample["source_time_s"])
        sample["joint_acceleration_rad_s2"] = np.asarray(
            [
                np.interp(source_time, source_times, source_acceleration[:, joint])
                for joint in range(6)
            ]
        ) * speed_scale**2
    q = [sample["joint_position_rad"] for sample in samples]
    dq = [sample["joint_velocity_rad_s"] for sample in samples]
    ddq = [sample["joint_acceleration_rad_s2"] for sample in samples]
    limits = evaluate_joint_trajectory(q, dq, ddq)
    if not limits["joint_mechanical_limits_pass"]:
        raise DeploymentPlanError("resampled reference exceeds the handoff joint envelope")
    events = events_for_mode(profile, mode, speed_scale=speed_scale)
    plan = {
        "schema": "xarm6_open_loop_demo_plan_v1",
        "mode": mode,
        "profile_id": profile["profile_id"],
        "profile_status": profile["status"],
        "desired_angle_deg": float(profile["desired_angle_deg"]),
        "object_id": object_config["object_id"],
        "object_dimensions_m": [float(value) for value in object_config["dimensions_m"]],
        "object_mass_kg": float(object_config["mass_kg"]),
        "profile": _display_path(profile_path, root),
        "source_timeline": profile["timeline"],
        "speed_scale": speed_scale,
        "control_period_s": 0.02,
        "duration_s": float(samples[-1]["time_s"]),
        "sample_count": len(samples),
        "g1_speed": float(profile["g1"]["speed"]),
        "g1_held_position": profile["g1"].get("held_position"),
        "g1_events": events,
        "joint_limits": limits,
        "online_probe": False,
        "online_vision": False,
        "online_ballistic_correction": False,
    }
    return plan, samples
