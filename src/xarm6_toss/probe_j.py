"""Deployable paired-signal Probe posterior and catch-candidate J."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Mapping, Sequence

import numpy as np

from .method import CatchObjectiveTerms, catch_objective


@dataclass(frozen=True)
class ProbePosterior:
    effective_payload_mean_kg: float
    effective_payload_std_kg: float
    com_offset_mean_m: tuple[float, float, float]
    com_offset_std_m: tuple[float, float, float]
    held_probability: float
    slip_probability: float
    projected_width_m: float
    detach_time_std_s: float
    payload_signal_nm: float
    gripper_contact_signal_nm: float
    effort_residual_mean_nm: tuple[float, ...]
    effort_residual_dynamic_rms_nm: tuple[float, ...]
    sample_count: int

    def as_dict(self) -> dict:
        return asdict(self)


def probe_joint_offset_rad(
    elapsed_s: float,
    *,
    duration_s: float,
    amplitude_rad: float,
    frequency_hz: float,
) -> float:
    """Return a bounded excitation that starts and finishes at zero."""

    if elapsed_s <= 0.0 or elapsed_s >= duration_s:
        return 0.0
    envelope = math.sin(math.pi * elapsed_s / duration_s) ** 2
    return (
        float(amplitude_rad)
        * envelope
        * math.sin(2.0 * math.pi * float(frequency_hz) * elapsed_s)
    )


def _aligned(values, sample_count: int, width: int) -> np.ndarray:
    result = np.asarray(values, dtype=float)[:sample_count]
    if result.shape != (sample_count, width):
        raise ValueError(f"expected paired signal shape {(sample_count, width)}")
    return result


def estimate_probe_posterior(
    *,
    empty_arm_effort_nm,
    held_arm_effort_nm,
    empty_gripper_effort_nm,
    held_gripper_effort_nm,
    held_joint_velocity_rad_s,
    held_gripper_position,
    projected_width_m: float,
    calibration: Mapping,
) -> ProbePosterior:
    """Estimate a broad posterior without object mass or simulator state."""

    sample_count = min(
        len(empty_arm_effort_nm),
        len(held_arm_effort_nm),
        len(empty_gripper_effort_nm),
        len(held_gripper_effort_nm),
        len(held_joint_velocity_rad_s),
        len(held_gripper_position),
    )
    if sample_count < 8:
        raise ValueError("paired Probe requires at least eight aligned samples")
    empty_arm = _aligned(empty_arm_effort_nm, sample_count, 6)
    held_arm = _aligned(held_arm_effort_nm, sample_count, 6)
    empty_gripper = np.asarray(empty_gripper_effort_nm, dtype=float)[:sample_count]
    held_gripper = np.asarray(held_gripper_effort_nm, dtype=float)[:sample_count]
    held_velocity = _aligned(held_joint_velocity_rad_s, sample_count, 6)
    held_drive = np.asarray(held_gripper_position, dtype=float)[:sample_count]

    residual = held_arm - empty_arm
    residual_mean = np.mean(residual, axis=0)
    residual_dynamic = residual - residual_mean
    residual_dynamic_rms = np.sqrt(np.mean(residual_dynamic**2, axis=0))
    selected_joints = np.asarray(
        calibration.get("payload_joint_indices", [1, 2, 4]), dtype=int
    )
    payload_signal = float(np.linalg.norm(residual_mean[selected_joints]))

    mass_range = np.asarray(calibration["payload_mass_range_kg"], dtype=float)
    signal_range = np.asarray(
        calibration["payload_signal_range_nm"], dtype=float
    )
    signal_fraction = float(
        np.clip(
            (payload_signal - signal_range[0])
            / (signal_range[1] - signal_range[0]),
            0.0,
            1.0,
        )
    )
    payload_mean = float(
        mass_range[0] + signal_fraction * (mass_range[1] - mass_range[0])
    )
    signal_noise = float(
        np.linalg.norm(residual_dynamic_rms[selected_joints])
        / math.sqrt(sample_count)
    )
    payload_std = max(
        float(calibration.get("payload_std_floor_kg", 0.006)),
        signal_noise
        / (signal_range[1] - signal_range[0])
        * (mass_range[1] - mass_range[0]),
    )
    payload_std = min(payload_std, 0.5 * float(np.ptp(mass_range)))

    com_matrix = np.asarray(
        calibration.get("com_offset_matrix_m_per_nm", np.zeros((3, 6))),
        dtype=float,
    )
    com_offset = com_matrix @ residual_mean
    com_std = np.full(
        3, float(calibration.get("com_offset_std_m", 0.008)), dtype=float
    )

    gripper_residual = held_gripper - empty_gripper
    gripper_contact_signal = float(abs(np.mean(gripper_residual)))
    contact_center = float(calibration["gripper_contact_signal_center_nm"])
    contact_scale = float(calibration["gripper_contact_signal_scale_nm"])
    held_probability = float(
        1.0 / (1.0 + math.exp(-(gripper_contact_signal - contact_center) / contact_scale))
    )
    gripper_drift = float(np.ptp(held_drive))
    effort_instability = float(np.mean(residual_dynamic_rms[selected_joints]))
    gripper_drift_excess = max(
        0.0,
        gripper_drift
        - float(calibration.get("slip_gripper_drift_center", 0.0)),
    )
    effort_instability_excess = max(
        0.0,
        effort_instability
        - float(calibration.get("slip_effort_rms_center_nm", 0.0)),
    )
    slip_probability = float(
        np.clip(
            gripper_drift_excess / float(calibration["slip_gripper_drift_scale"])
            + effort_instability_excess
            / float(calibration["slip_effort_rms_scale_nm"])
            + max(0.0, float(np.max(np.abs(held_velocity))) - 0.5) * 0.1,
            0.0,
            1.0,
        )
    )
    detach_time_std = (
        float(calibration.get("detach_time_std_floor_s", 0.006))
        + float(calibration.get("detach_mass_std_gain_s_per_kg", 0.5))
        * payload_std
        + float(calibration.get("detach_slip_gain_s", 0.015))
        * slip_probability
    )
    return ProbePosterior(
        effective_payload_mean_kg=payload_mean,
        effective_payload_std_kg=payload_std,
        com_offset_mean_m=tuple(float(value) for value in com_offset),
        com_offset_std_m=tuple(float(value) for value in com_std),
        held_probability=held_probability,
        slip_probability=slip_probability,
        projected_width_m=float(projected_width_m),
        detach_time_std_s=float(detach_time_std),
        payload_signal_nm=payload_signal,
        gripper_contact_signal_nm=gripper_contact_signal,
        effort_residual_mean_nm=tuple(float(value) for value in residual_mean),
        effort_residual_dynamic_rms_nm=tuple(
            float(value) for value in residual_dynamic_rms
        ),
        sample_count=sample_count,
    )


def select_catch_candidate(
    posterior: ProbePosterior,
    candidates: Sequence[Mapping],
) -> tuple[Mapping, list[dict]]:
    """Rank executable candidates with J after applying Probe uncertainty."""

    ranking = []
    for candidate in candidates:
        base = dict(candidate["objective_terms"])
        uncertainty = (
            float(base.pop("base_cvar_failure"))
            + float(candidate["payload_std_gain"])
            * posterior.effective_payload_std_kg
            + float(candidate["detach_std_gain"])
            * posterior.detach_time_std_s
        )
        catch_probability = float(base.pop("catch_probability")) * (
            0.5 + 0.5 * posterior.held_probability
        )
        slip_risk = float(base.pop("slip_risk", 0.0))
        terms = CatchObjectiveTerms(
            **base,
            slip_risk=slip_risk
            + posterior.slip_probability,
            cvar_failure=uncertainty,
            catch_probability=max(1.0e-4, min(1.0, catch_probability)),
        )
        score = catch_objective(terms)
        ranking.append(
            {
                "name": candidate["name"],
                "J": float(score),
                "terms": asdict(terms),
            }
        )
    ranking.sort(key=lambda item: (item["J"], item["name"]))
    selected_name = ranking[0]["name"]
    selected = next(item for item in candidates if item["name"] == selected_name)
    return selected, ranking
