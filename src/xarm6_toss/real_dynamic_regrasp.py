"""Hardware-independent core of the real xArm6 dynamic-regrasp runner."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np

from .probe_j import ProbePosterior, select_catch_candidate


GRAVITY_M_S2 = 9.81
NOMINAL_SIM_DETACH_TIME_S = 0.615


@dataclass(frozen=True)
class DetachEvent:
    time_s: float
    gripper_position: float
    source: str

    def as_dict(self) -> dict[str, float | str]:
        return asdict(self)


@dataclass(frozen=True)
class ReleaseState:
    time_s: float
    position_base_m: tuple[float, float, float]
    velocity_base_m_s: tuple[float, float, float]

    def as_dict(self) -> dict[str, object]:
        return asdict(self)


class G1PositionDetachObserver:
    """Detect release from a camera-calibrated G1 actual-position threshold."""

    def __init__(
        self,
        *,
        command_time_s: float,
        held_position: float,
        open_position: float,
        detach_position_threshold: float | None,
        fallback_delay_s: float,
    ):
        self.command_time_s = float(command_time_s)
        self.held_position = float(held_position)
        self.open_position = float(open_position)
        self.detach_position_threshold = (
            None
            if detach_position_threshold is None
            else float(detach_position_threshold)
        )
        self.fallback_delay_s = float(fallback_delay_s)
        self.opening_direction = math.copysign(
            1.0, self.open_position - self.held_position
        )
        self.event: DetachEvent | None = None

    def observe(self, time_s: float, gripper_position: float) -> DetachEvent | None:
        if self.event is not None or time_s < self.command_time_s:
            return self.event
        threshold_crossed = (
            self.detach_position_threshold is not None
            and self.opening_direction
            * (gripper_position - self.detach_position_threshold)
            >= 0.0
        )
        fallback_due = (
            time_s >= self.command_time_s + self.fallback_delay_s
        )
        if threshold_crossed or fallback_due:
            self.event = DetachEvent(
                time_s=float(time_s),
                gripper_position=float(gripper_position),
                source=(
                    "calibrated_g1_position"
                    if threshold_crossed
                    else "measured_delay_fallback"
                ),
            )
        return self.event


def release_state_from_arm(
    kinematics,
    joint_position_rad,
    joint_velocity_rad_s,
    grasp_offset_gripper_base_link_m,
    *,
    time_s: float,
    target_link: str = "xarm_gripper_base_link",
) -> ReleaseState:
    q = np.asarray(joint_position_rad, dtype=float)
    qd = np.asarray(joint_velocity_rad_s, dtype=float)
    offset_hand = np.asarray(
        grasp_offset_gripper_base_link_m, dtype=float
    )
    transform = kinematics.forward(q, target_link=target_link)
    jacobian = kinematics.jacobian(q, target_link=target_link)
    offset_world = transform[:3, :3] @ offset_hand
    position = transform[:3, 3] + offset_world
    hand_linear = jacobian[:3] @ qd
    hand_angular = jacobian[3:] @ qd
    velocity = hand_linear + np.cross(hand_angular, offset_world)
    return ReleaseState(
        time_s=float(time_s),
        position_base_m=tuple(float(value) for value in position),
        velocity_base_m_s=tuple(float(value) for value in velocity),
    )


def ballistic_position(release: ReleaseState, query_time_s: float) -> np.ndarray:
    duration_s = float(query_time_s) - release.time_s
    if duration_s < 0.0:
        raise ValueError("ballistic query must not precede detach")
    gravity = np.asarray([0.0, 0.0, -GRAVITY_M_S2])
    return (
        np.asarray(release.position_base_m)
        + np.asarray(release.velocity_base_m_s) * duration_s
        + 0.5 * gravity * duration_s**2
    )


def damped_catch_delta(
    position_jacobian,
    position_error_m,
    *,
    gain: float = 0.75,
    damping: float = 1.0e-3,
    maximum_step_rad: float = 0.035,
) -> np.ndarray:
    jacobian = np.asarray(position_jacobian, dtype=float)
    error = np.asarray(position_error_m, dtype=float)
    delta = jacobian.T @ np.linalg.solve(
        jacobian @ jacobian.T + damping * np.eye(3), error
    )
    return np.clip(gain * delta, -maximum_step_rad, maximum_step_rad)


def catch_position_target(
    kinematics,
    joint_position_rad,
    intercept_position_base_m,
    grasp_offset_gripper_base_link_m,
    *,
    controlled_joint_count: int = 3,
    target_link: str = "xarm_gripper_base_link",
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    q = np.asarray(joint_position_rad, dtype=float)
    transform = kinematics.forward(q, target_link=target_link)
    full_jacobian = kinematics.jacobian(q, target_link=target_link)
    offset_world = transform[:3, :3] @ np.asarray(
        grasp_offset_gripper_base_link_m, dtype=float
    )
    desired_hand_position = (
        np.asarray(intercept_position_base_m, dtype=float) - offset_world
    )
    error = desired_hand_position - transform[:3, 3]
    jacobian = full_jacobian[:3, :controlled_joint_count]
    delta = damped_catch_delta(jacobian, error)
    return delta, error, jacobian


def advance_limited_command(
    commanded_position_rad,
    commanded_velocity_rad_s,
    proposed_position_rad,
    *,
    control_period_s: float,
    maximum_speed_rad_s: float,
    maximum_acceleration_rad_s2: float,
) -> tuple[np.ndarray, np.ndarray]:
    position = np.asarray(commanded_position_rad, dtype=float)
    velocity = np.asarray(commanded_velocity_rad_s, dtype=float)
    proposed = np.asarray(proposed_position_rad, dtype=float)
    raw_velocity = (proposed - position) / control_period_s
    speed_limited = np.clip(
        raw_velocity, -maximum_speed_rad_s, maximum_speed_rad_s
    )
    acceleration = np.clip(
        (speed_limited - velocity) / control_period_s,
        -maximum_acceleration_rad_s2,
        maximum_acceleration_rad_s2,
    )
    next_velocity = velocity + acceleration * control_period_s
    next_position = position + next_velocity * control_period_s
    return next_position, next_velocity


def controller_offsets(
    controller: Mapping[str, object],
    nominal_detach_time_s: float = NOMINAL_SIM_DETACH_TIME_S,
) -> dict[str, float | None]:
    def offset(name: str) -> float | None:
        value = controller.get(name)
        return None if value is None else float(value) - nominal_detach_time_s

    return {
        "catch_servo": offset("catch_servo_start_time_s"),
        "preclose": offset("catch_preclose_time_s"),
        "final_close": offset("catch_close_time_s"),
        "intercept": offset("catch_intercept_time_s"),
        "control_end": offset("vision_control_end_time_s"),
    }


def resample_timeline(
    samples: Sequence[Mapping[str, object]],
    *,
    speed_scale: float,
    control_period_s: float = 0.02,
) -> list[dict[str, object]]:
    source_time = np.asarray([item["time_s"] for item in samples], dtype=float)
    source_q = np.asarray(
        [item["joint_position_rad"] for item in samples], dtype=float
    )
    source_qd = np.asarray(
        [item["joint_velocity_rad_s"] for item in samples], dtype=float
    )
    duration = source_time[-1] / speed_scale
    times = np.arange(0.0, duration + 0.5 * control_period_s, control_period_s)
    result = []
    for time_s in times:
        source_query = min(time_s * speed_scale, source_time[-1])
        q = np.asarray(
            [
                np.interp(source_query, source_time, source_q[:, joint])
                for joint in range(6)
            ]
        )
        qd = np.asarray(
            [
                np.interp(source_query, source_time, source_qd[:, joint])
                for joint in range(6)
            ]
        ) * speed_scale
        phase_index = int(np.searchsorted(source_time, source_query, side="right") - 1)
        result.append(
            {
                "time_s": float(time_s),
                "source_time_s": float(source_query),
                "phase": str(samples[max(0, phase_index)]["phase"]),
                "joint_position_rad": q,
                "joint_velocity_rad_s": qd,
            }
        )
    return result


def real_probe_posterior(
    comparison: Mapping[str, object],
    cube_probe: Mapping[str, object],
    *,
    calibration: Mapping[str, object],
    projected_width_m: float = 0.035,
) -> tuple[ProbePosterior, dict[str, float]]:
    current_mean = np.asarray(comparison["current_residual_mean"], dtype=float)
    current_noise = np.asarray(
        comparison["current_residual_dynamic_rms"], dtype=float
    )
    selected = np.asarray([1, 2, 4])
    current_signal = float(np.linalg.norm(current_mean[selected]))
    current_noise_level = float(np.linalg.norm(current_noise[selected]))
    signal_to_noise = current_signal / max(current_noise_level, 1.0e-6)
    effort_mean = np.asarray(comparison["effort_residual_mean"], dtype=float)
    effort_noise = np.asarray(
        comparison["effort_residual_dynamic_rms"], dtype=float
    )
    payload_signal = float(np.linalg.norm(effort_mean[selected]))
    signal_range = np.asarray(calibration["payload_signal_range_nm"], dtype=float)
    mass_range = np.asarray(calibration["payload_mass_range_kg"], dtype=float)
    signal_fraction = float(
        np.clip(
            (payload_signal - signal_range[0]) / np.ptp(signal_range), 0.0, 1.0
        )
    )
    payload_mean = float(mass_range[0] + signal_fraction * np.ptp(mass_range))
    effort_standard_error = float(
        np.linalg.norm(effort_noise[selected])
        / math.sqrt(max(1, int(comparison["samples"])))
    )
    drift = abs(
        float(cube_probe["gripper_position_after"])
        - float(cube_probe["gripper_position_before"])
    )
    held_from_signal = 1.0 / (1.0 + math.exp(-(signal_to_noise - 0.5) / 0.35))
    held_from_drift = 1.0 / (1.0 + math.exp((drift - 20.0) / 5.0))
    held_probability = float(held_from_signal * held_from_drift)
    slip_probability = float(
        np.clip(
            drift / 25.0
            + max(
                0.0,
                current_noise_level / max(current_signal, 1.0e-6) - 0.5,
            )
            * 0.5,
            0.0,
            1.0,
        )
    )
    payload_std_from_effort = (
        effort_standard_error / np.ptp(signal_range) * np.ptp(mass_range)
    )
    payload_std = float(
        np.clip(
            max(payload_std_from_effort, 0.006 + 0.009 / (1.0 + signal_to_noise)),
            float(calibration["payload_std_floor_kg"]),
            0.5 * np.ptp(mass_range),
        )
    )
    detach_std = 0.006 + 0.5 * payload_std + 0.015 * slip_probability
    posterior = ProbePosterior(
        effective_payload_mean_kg=payload_mean,
        effective_payload_std_kg=payload_std,
        com_offset_mean_m=(0.0, 0.0, 0.0),
        com_offset_std_m=(0.008, 0.008, 0.008),
        held_probability=held_probability,
        slip_probability=slip_probability,
        projected_width_m=float(projected_width_m),
        detach_time_std_s=float(detach_std),
        payload_signal_nm=payload_signal,
        gripper_contact_signal_nm=1.0 - drift / 150.0,
        effort_residual_mean_nm=tuple(float(value) for value in effort_mean),
        effort_residual_dynamic_rms_nm=tuple(float(value) for value in effort_noise),
        sample_count=int(comparison["samples"]),
    )
    return posterior, {
        "arm_current_signal": current_signal,
        "arm_current_dynamic_noise": current_noise_level,
        "arm_current_signal_to_noise": signal_to_noise,
        "arm_effort_payload_signal_nm": payload_signal,
        "effective_payload_mean_kg": payload_mean,
        "effective_payload_std_kg": payload_std,
        "g1_position_drift": drift,
    }


def load_real_probe_selection(
    comparison_path: Path,
    probe_j_config_path: Path,
) -> tuple[Mapping[str, object], dict[str, object]]:
    comparison_path = Path(comparison_path)
    comparison = json.loads(comparison_path.read_text(encoding="utf-8"))
    cube_path = Path(str(comparison["cube_probe"]))
    if not cube_path.is_absolute():
        candidates = [parent / cube_path for parent in comparison_path.parents]
        cube_path = next(
            (
                candidate
                for candidate in candidates
                if (candidate / "summary.json").is_file()
            ),
            candidates[0],
        )
    cube_summary = json.loads((cube_path / "summary.json").read_text(encoding="utf-8"))
    config = json.loads(Path(probe_j_config_path).read_text(encoding="utf-8"))
    posterior, features = real_probe_posterior(
        comparison,
        cube_summary,
        calibration=config["calibration"],
        projected_width_m=config["projected_width_m"],
    )
    selected, ranking = select_catch_candidate(
        posterior, config["catch_candidates"]
    )
    evidence = {
        "comparison": str(comparison_path),
        "posterior": posterior.as_dict(),
        "features": features,
        "ranking": ranking,
        "selected_candidate": selected["name"],
        "gate_passed": bool(
            posterior.held_probability >= config["minimum_held_probability"]
            and posterior.slip_probability <= config["maximum_slip_probability"]
        ),
    }
    return selected, evidence
