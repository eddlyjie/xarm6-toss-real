"""Nominal 6-D free-flight physics for a released cube.

The ideal propagator intentionally does not require object mass or side length:
gravity-only translation is mass independent, and a uniform cube has isotropic
inertia, so its torque-free world angular velocity is constant. Probe and
Detach posteriors enter as uncertainty/residuals around this nominal model.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from scipy.spatial.transform import Rotation


GRAVITY_M_S2 = 9.81


def _vector3(value, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if array.shape != (3,) or not np.isfinite(array).all():
        raise ValueError(f"{name} must contain three finite values")
    return array


def _rotation3(value) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if array.shape != (3, 3) or not np.isfinite(array).all():
        raise ValueError("rotation_world_object must be a finite 3x3 matrix")
    return array


@dataclass(frozen=True)
class FlightState:
    position_m: tuple[float, float, float]
    rotation_world_object: tuple[tuple[float, float, float], ...]
    linear_velocity_m_s: tuple[float, float, float]
    angular_velocity_world_rad_s: tuple[float, float, float]

    def arrays(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        return (
            _vector3(self.position_m, "position_m"),
            _rotation3(self.rotation_world_object),
            _vector3(self.linear_velocity_m_s, "linear_velocity_m_s"),
            _vector3(
                self.angular_velocity_world_rad_s,
                "angular_velocity_world_rad_s",
            ),
        )


def make_flight_state(
    position_m,
    rotation_world_object,
    linear_velocity_m_s,
    angular_velocity_world_rad_s,
) -> FlightState:
    position = _vector3(position_m, "position_m")
    rotation = _rotation3(rotation_world_object)
    linear = _vector3(linear_velocity_m_s, "linear_velocity_m_s")
    angular = _vector3(
        angular_velocity_world_rad_s, "angular_velocity_world_rad_s"
    )
    return FlightState(
        position_m=tuple(float(value) for value in position),
        rotation_world_object=tuple(
            tuple(float(value) for value in row) for row in rotation
        ),
        linear_velocity_m_s=tuple(float(value) for value in linear),
        angular_velocity_world_rad_s=tuple(float(value) for value in angular),
    )


def propagate_cube(state: FlightState, time_s: float) -> FlightState:
    """Propagate the nominal gravity-only, torque-free uniform-cube state."""
    if not math.isfinite(time_s) or time_s < 0.0:
        raise ValueError("time_s must be finite and non-negative")
    position, rotation, linear, angular = state.arrays()
    gravity = np.asarray([0.0, 0.0, -GRAVITY_M_S2])
    propagated_position = position + linear * time_s + 0.5 * gravity * time_s**2
    propagated_linear = linear + gravity * time_s
    delta_rotation = Rotation.from_rotvec(angular * time_s).as_matrix()
    propagated_rotation = delta_rotation @ rotation
    return make_flight_state(
        propagated_position,
        propagated_rotation,
        propagated_linear,
        angular,
    )


def flight_time_to_height(
    release_z_m: float,
    upward_velocity_m_s: float,
    catch_z_m: float,
) -> float:
    """Return the positive ballistic time at which the cube reaches catch_z."""
    values = (release_z_m, upward_velocity_m_s, catch_z_m)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("release height, velocity, and catch height must be finite")
    discriminant = upward_velocity_m_s**2 + 2.0 * GRAVITY_M_S2 * (
        release_z_m - catch_z_m
    )
    if discriminant < 0.0:
        raise ValueError("the requested catch height is not reached by this flight")
    time_s = (upward_velocity_m_s + math.sqrt(discriminant)) / GRAVITY_M_S2
    if time_s <= 0.0:
        raise ValueError("the requested catch event is not after release")
    return time_s


def angular_velocity_for_target_rotation(
    axis_world,
    angle_rad: float,
    flight_time_s: float,
) -> tuple[float, float, float]:
    axis = _vector3(axis_world, "axis_world")
    norm = float(np.linalg.norm(axis))
    if norm == 0.0:
        raise ValueError("axis_world must be non-zero")
    if not math.isfinite(angle_rad):
        raise ValueError("angle_rad must be finite")
    if not math.isfinite(flight_time_s) or flight_time_s <= 0.0:
        raise ValueError("flight_time_s must be finite and positive")
    angular = axis / norm * (angle_rad / flight_time_s)
    return tuple(float(value) for value in angular)


def nominal_object_twist(
    gripper_linear_velocity_m_s,
    gripper_angular_velocity_world_rad_s,
    gripper_to_object_world_m,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Rigid-contact object twist immediately before nominal detach."""
    linear = _vector3(gripper_linear_velocity_m_s, "gripper_linear_velocity_m_s")
    angular = _vector3(
        gripper_angular_velocity_world_rad_s,
        "gripper_angular_velocity_world_rad_s",
    )
    lever = _vector3(gripper_to_object_world_m, "gripper_to_object_world_m")
    object_linear = linear + np.cross(angular, lever)
    return (
        tuple(float(value) for value in object_linear),
        tuple(float(value) for value in angular),
    )



def continuous_free_flight_evidence(
    records,
    baseline_position_hand_m,
    *,
    release_height_m: float | None = None,
    contact_force_threshold_n: float = 0.05,
    spectator_rate_hz: float = 60.0,
) -> dict[str, object]:
    """Measure the longest uninterrupted post-release contact-free arc.

    A short force flicker cannot be reported as a long toss: the selected arc
    ends at the first subsequent finger contact. Catch stability remains a
    separate requirement evaluated by the native runner.
    """

    baseline = _vector3(baseline_position_hand_m, "baseline_position_hand_m")
    if release_height_m is not None:
        release_height_m = float(release_height_m)
        if not math.isfinite(release_height_m):
            raise ValueError("release_height_m must be finite")
    postrelease = [
        record
        for record in records
        if record.get("phase") in ("flight", "catch")
    ]
    if not postrelease:
        return {
            "continuous_free_flight_detected": False,
            "continuous_free_flight_duration_s": 0.0,
            "obvious_free_flight": False,
        }

    def force(record, key):
        value = float(record[key])
        if not math.isfinite(value):
            raise ValueError(f"{key} must be finite")
        return value

    def contact_free(record):
        return (
            force(record, "left_finger_cube_contact_force_n")
            <= contact_force_threshold_n
            and force(record, "right_finger_cube_contact_force_n")
            <= contact_force_threshold_n
        )

    runs = []
    start = None
    for index, record in enumerate(postrelease):
        if contact_free(record):
            if start is None:
                start = index
        elif start is not None:
            runs.append((start, index - 1, index))
            start = None
    if start is not None:
        runs.append((start, len(postrelease) - 1, None))
    if not runs:
        return {
            "continuous_free_flight_detected": False,
            "continuous_free_flight_duration_s": 0.0,
            "obvious_free_flight": False,
        }

    def run_duration(run):
        first, last, contact = run
        end_time = float(
            postrelease[contact if contact is not None else last]["time_s"]
        )
        return end_time - float(postrelease[first]["time_s"])

    sustained_runs = [run for run in runs if run_duration(run) >= 0.05]
    if sustained_runs:
        start_index, last_free_index, contact_index = min(
            sustained_runs, key=lambda run: run[0]
        )
    else:
        start_index, last_free_index, contact_index = max(
            runs, key=lambda run: (run_duration(run), -run[0])
        )
    free_records = postrelease[start_index : last_free_index + 1]
    start_time = float(free_records[0]["time_s"])
    end_time = float(
        postrelease[contact_index]["time_s"]
        if contact_index is not None
        else free_records[-1]["time_s"]
    )
    apex_local_index = max(
        range(len(free_records)),
        key=lambda index: float(free_records[index]["cube_position_w_m"][2]),
    )
    apex_record = free_records[apex_local_index]
    apex_time = float(apex_record["time_s"])
    first_record = free_records[0]
    last_record = free_records[-1]
    first_contact_record = (
        None if contact_index is None else postrelease[contact_index]
    )

    separations = [
        float(
            np.linalg.norm(
                _vector3(record["cube_position_hand_m"], "cube_position_hand_m")
                - baseline
            )
        )
        for record in free_records
    ]
    q_start = np.asarray(first_record["cube_quaternion_wxyz"], dtype=float)
    q_end = np.asarray(last_record["cube_quaternion_wxyz"], dtype=float)
    q_start /= np.linalg.norm(q_start)
    q_end /= np.linalg.norm(q_end)
    rotation_rad = 2.0 * math.acos(
        float(np.clip(abs(np.dot(q_start, q_end)), 0.0, 1.0))
    )
    spin_path_rad = 0.0
    for before, after in zip(free_records, free_records[1:]):
        dt = float(after["time_s"]) - float(before["time_s"])
        omega_before = np.linalg.norm(
            _vector3(
                before["cube_angular_velocity_w_rad_s"],
                "cube_angular_velocity_w_rad_s",
            )
        )
        omega_after = np.linalg.norm(
            _vector3(
                after["cube_angular_velocity_w_rad_s"],
                "cube_angular_velocity_w_rad_s",
            )
        )
        spin_path_rad += 0.5 * float(omega_before + omega_after) * dt

    bilateral_time = None
    for record in postrelease[last_free_index + 1 :]:
        if (
            force(record, "left_finger_cube_contact_force_n")
            > contact_force_threshold_n
            and force(record, "right_finger_cube_contact_force_n")
            > contact_force_threshold_n
        ):
            bilateral_time = float(record["time_s"])
            break

    duration_s = end_time - start_time
    apex_to_contact_s = None if first_contact_record is None else end_time - apex_time
    approach_velocity_z = (
        None
        if first_contact_record is None
        else float(last_record["cube_linear_velocity_w_m_s"][2])
    )
    apex_is_internal = 0 < apex_local_index < len(free_records) - 1
    rise_after_detach_m = float(
        apex_record["cube_position_w_m"][2]
        - first_record["cube_position_w_m"][2]
    )
    rise_from_release_m = float(apex_record["cube_position_w_m"][2]) - (
        float(first_record["cube_position_w_m"][2]) if release_height_m is None else release_height_m
    )
    has_descending_contact = (
        approach_velocity_z is not None and approach_velocity_z < 0.0
    )
    post_apex_frames = (
        0
        if apex_to_contact_s is None
        else max(0, int(math.floor(apex_to_contact_s * spectator_rate_hz + 1e-9)))
    )
    obvious = (
        duration_s >= 0.12
        and rise_from_release_m >= 0.04
        and apex_is_internal
        and has_descending_contact
        and post_apex_frames >= 2
    )
    return {
        "continuous_free_flight_detected": True,
        "continuous_free_flight_start_time_s": start_time,
        "continuous_free_flight_end_time_s": end_time,
        "continuous_free_flight_duration_s": duration_s,
        "continuous_free_flight_sample_count": len(free_records),
        "continuous_free_flight_max_separation_m": max(separations),
        "free_flight_apex_time_s": apex_time,
        "free_flight_apex_height_m": float(apex_record["cube_position_w_m"][2]),
        "free_flight_rise_after_detach_m": rise_after_detach_m,
        "free_flight_rise_from_kinematic_release_m": rise_from_release_m,
        "free_flight_apex_is_internal": apex_is_internal,
        "first_renewed_contact_time_s": (
            None if first_contact_record is None else end_time
        ),
        "first_renewed_bilateral_contact_time_s": bilateral_time,
        "precontact_vertical_velocity_m_s": approach_velocity_z,
        "post_apex_spectator_frame_count": post_apex_frames,
        "free_flight_rotation_rad": rotation_rad,
        "free_flight_spin_path_rad": spin_path_rad,
        "obvious_free_flight": obvious,
    }
