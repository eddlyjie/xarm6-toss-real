from __future__ import annotations

import numpy as np

from xarm6_toss.probe_j import (
    ProbePosterior,
    estimate_probe_posterior,
    probe_joint_offset_rad,
    select_catch_candidate,
)


CALIBRATION = {
    "payload_joint_indices": [1, 2, 4],
    "payload_mass_range_kg": [0.020, 0.050],
    "payload_signal_range_nm": [0.01, 0.10],
    "payload_std_floor_kg": 0.004,
    "com_offset_matrix_m_per_nm": [[0.0] * 6] * 3,
    "com_offset_std_m": 0.008,
    "gripper_contact_signal_center_nm": 0.02,
    "gripper_contact_signal_scale_nm": 0.01,
    "slip_gripper_drift_center": 0.04,
    "slip_gripper_drift_scale": 0.02,
    "slip_effort_rms_center_nm": 0.01,
    "slip_effort_rms_scale_nm": 2.0,
    "detach_time_std_floor_s": 0.006,
    "detach_mass_std_gain_s_per_kg": 0.5,
    "detach_slip_gain_s": 0.015,
}


def paired_signals(payload_signal: float):
    samples = 40
    empty = np.zeros((samples, 6))
    held = np.zeros((samples, 6))
    held[:, 1] = payload_signal
    phase = np.linspace(0.0, 2.0 * np.pi, samples)
    held[:, 2] = 0.002 * np.sin(phase)
    return dict(
        empty_arm_effort_nm=empty,
        held_arm_effort_nm=held,
        empty_gripper_effort_nm=np.zeros(samples),
        held_gripper_effort_nm=np.full(samples, 0.06),
        held_joint_velocity_rad_s=np.zeros((samples, 6)),
        held_gripper_position=np.full(samples, 0.56),
        projected_width_m=0.038,
        calibration=CALIBRATION,
    )


def test_probe_profile_is_bounded_and_returns_to_center():
    values = [
        probe_joint_offset_rad(
            time_s,
            duration_s=0.4,
            amplitude_rad=0.04,
            frequency_hz=2.0,
        )
        for time_s in np.linspace(0.0, 0.4, 101)
    ]
    assert values[0] == 0.0
    assert values[-1] == 0.0
    assert max(abs(value) for value in values) <= 0.04


def test_paired_effort_changes_payload_without_true_mass_input():
    light = estimate_probe_posterior(**paired_signals(0.02))
    heavy = estimate_probe_posterior(**paired_signals(0.08))
    assert light.effective_payload_mean_kg < heavy.effective_payload_mean_kg
    assert light.projected_width_m == 0.038
    assert light.held_probability > 0.9
    assert light.slip_probability < 0.1
    assert light.sample_count == 40


def posterior(*, payload_std: float, detach_std: float) -> ProbePosterior:
    return ProbePosterior(
        effective_payload_mean_kg=0.035,
        effective_payload_std_kg=payload_std,
        com_offset_mean_m=(0.0, 0.0, 0.0),
        com_offset_std_m=(0.008, 0.008, 0.008),
        held_probability=0.98,
        slip_probability=0.01,
        projected_width_m=0.038,
        detach_time_std_s=detach_std,
        payload_signal_nm=0.05,
        gripper_contact_signal_nm=0.06,
        effort_residual_mean_nm=(0.0,) * 6,
        effort_residual_dynamic_rms_nm=(0.0,) * 6,
        sample_count=40,
    )


CANDIDATES = [
    {
        "name": "stable",
        "payload_std_gain": 2.0,
        "detach_std_gain": 2.0,
        "objective_terms": {
            "task_grasp_error": 0.0,
            "relative_contact_velocity": 0.20,
            "impact_energy": 0.10,
            "slip_risk": 0.03,
            "arm_motion_cost": 0.10,
            "base_cvar_failure": 0.10,
            "catch_probability": 0.92,
        },
    },
    {
        "name": "clear",
        "payload_std_gain": 25.0,
        "detach_std_gain": 20.0,
        "objective_terms": {
            "task_grasp_error": 0.0,
            "relative_contact_velocity": 0.05,
            "impact_energy": 0.03,
            "slip_risk": 0.03,
            "arm_motion_cost": 0.05,
            "base_cvar_failure": 0.02,
            "catch_probability": 0.97,
        },
    },
]


def test_probe_uncertainty_changes_j_selection():
    selected_low, ranking_low = select_catch_candidate(
        posterior(payload_std=0.001, detach_std=0.002), CANDIDATES
    )
    selected_high, ranking_high = select_catch_candidate(
        posterior(payload_std=0.012, detach_std=0.020), CANDIDATES
    )
    assert selected_low["name"] == "clear"
    assert selected_high["name"] == "stable"
    assert ranking_low[0]["J"] < ranking_low[1]["J"]
    assert ranking_high[0]["J"] < ranking_high[1]["J"]
