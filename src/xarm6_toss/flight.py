"""Nominal 6-D free-flight physics for a released cube.

The ideal propagator intentionally does not require object mass or side length:
gravity-only translation is mass independent, and a uniform cube has isotropic
inertia, so its torque-free world angular velocity is constant. Probe and
Detach posteriors enter as uncertainty/residuals around this nominal model.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import math

import numpy as np
from scipy.spatial.transform import Rotation


GRAVITY_M_S2 = 9.81
VISIBLE_SPIN_MIN_ROTATION_RAD = math.radians(8.0)
VISIBLE_SPIN_MIN_FLIGHT_S = 0.12
DETACH_DEBOUNCE_S = 0.005

TUMBLE_MIN_AXIS_ALIGNMENT = 0.85
TUMBLE_MIN_APEX_ROTATION_RAD = math.radians(5.0)
TUMBLE_MIN_FLIGHT_ROTATION_RAD = math.radians(12.0)
STRICT_CONTACT_BODY_NAMES = (
    "left_finger",
    "right_finger",
    "left_outer_knuckle",
    "right_outer_knuckle",
    "left_inner_knuckle",
    "right_inner_knuckle",
    "gripper_base",
    "link_eef",
    "link6",
    "link5",
    "link4",
    "wrist_camera_proxy",
    "ground",
)


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


def _rotation_from_wxyz(value, name: str) -> np.ndarray:
    quaternion = np.asarray(value, dtype=float)
    if quaternion.shape != (4,) or not np.isfinite(quaternion).all():
        raise ValueError(f"{name} must contain four finite values")
    norm = float(np.linalg.norm(quaternion))
    if norm == 0.0:
        raise ValueError(f"{name} must be non-zero")
    w, x, y, z = quaternion / norm
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ]
    )


def cube_ground_clearance_m(
    center_position_m,
    quaternion_wxyz,
    side_length_m: float,
    ground_height_m: float = 0.0,
) -> float:
    """Clearance from an oriented cube's lowest corner to a horizontal ground."""
    center = _vector3(center_position_m, "center_position_m")
    side_length = float(side_length_m)
    ground_height = float(ground_height_m)
    if not math.isfinite(side_length) or side_length <= 0.0:
        raise ValueError("side_length_m must be finite and positive")
    if not math.isfinite(ground_height):
        raise ValueError("ground_height_m must be finite")
    rotation = _rotation_from_wxyz(quaternion_wxyz, "quaternion_wxyz")
    vertical_half_extent = 0.5 * side_length * float(
        np.sum(np.abs(rotation[2, :]))
    )
    return float(center[2] - vertical_half_extent - ground_height)


def _robot_contact_forces(record) -> tuple[dict[str, float], bool]:
    mapped = record.get("robot_cube_contact_forces_n")
    if isinstance(mapped, Mapping):
        forces = {str(name): float(value) for name, value in mapped.items()}
        if not all(math.isfinite(value) for value in forces.values()):
            raise ValueError("robot_cube_contact_forces_n must be finite")
        complete = all(name in forces for name in STRICT_CONTACT_BODY_NAMES)
        return forces, complete
    forces = {
        "left_finger": float(record["left_finger_cube_contact_force_n"]),
        "right_finger": float(record["right_finger_cube_contact_force_n"]),
    }
    return forces, False


def _empty_tumble_evidence() -> dict[str, object]:
    return {
        "strict_contact_source_complete": False,
        "strict_contact_free_flight": False,
        "finger_direction_world": None,
        "tumble_axis_world": None,
        "detach_angular_velocity_world_rad_s": None,
        "tumble_axis_alignment": 0.0,
        "detach_to_apex_tumble_rotation_rad": 0.0,
        "detach_to_apex_tumble_rotation_deg": 0.0,
        "free_flight_signed_tumble_rotation_rad": 0.0,
        "free_flight_signed_tumble_rotation_deg": 0.0,
        "free_flight_non_target_rotation_rad": 0.0,
        "target_axis_tumble": False,
    }


def _integrated_axis_rotation(records, axis_world: np.ndarray) -> tuple[float, float]:
    signed_rotation = 0.0
    non_target_rotation = 0.0
    for before, after in zip(records, records[1:]):
        dt = float(after["time_s"]) - float(before["time_s"])
        omega_before = _vector3(
            before["cube_angular_velocity_w_rad_s"], "cube_angular_velocity_w_rad_s"
        )
        omega_after = _vector3(
            after["cube_angular_velocity_w_rad_s"], "cube_angular_velocity_w_rad_s"
        )
        projected_before = float(np.dot(omega_before, axis_world))
        projected_after = float(np.dot(omega_after, axis_world))
        signed_rotation += 0.5 * (projected_before + projected_after) * dt
        residual_before = omega_before - projected_before * axis_world
        residual_after = omega_after - projected_after * axis_world
        non_target_rotation += 0.5 * (
            float(np.linalg.norm(residual_before))
            + float(np.linalg.norm(residual_after))
        ) * dt
    return signed_rotation, non_target_rotation


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


def g1_release_response(
    held_drive_rad: float,
    open_target_rad: float,
    current_drive_rad: float,
    current_effort_nm: float,
    *,
    drive_delta_threshold_rad: float | None = None,
    opening_effort_threshold_nm: float | None = None,
) -> tuple[bool, float, float]:
    """Detect the first measured G1 response in the commanded opening direction.

    G1 position and motor-current/effort are available on the real system.  The
    response event is therefore a deployable replacement for reading the cube's
    simulator contact state when freezing the encoder/FK release prior.
    """
    opening_direction = math.copysign(1.0, open_target_rad - held_drive_rad)
    drive_progress_rad = opening_direction * (
        current_drive_rad - held_drive_rad
    )
    opening_effort_nm = opening_direction * current_effort_nm
    drive_triggered = (
        drive_delta_threshold_rad is not None
        and drive_progress_rad >= drive_delta_threshold_rad
    )
    effort_triggered = (
        opening_effort_threshold_nm is not None
        and opening_effort_nm >= opening_effort_threshold_nm
    )
    return (
        bool(drive_triggered or effort_triggered),
        float(drive_progress_rad),
        float(opening_effort_nm),
    )


def release_transfer_evidence(
    records,
    *,
    release_motion_start_time_s: float,
    detach_time_s: float | None,
    target_axis_world,
) -> dict[str, object]:
    """Measure the release residual from a rigid hand prior to actual detach.

    The nominal Detach prior assumes the cube shares the measured hand twist.
    This evidence reports how the finite G1 contact-release window changes that
    twist before the first sustained contact-free sample.
    """

    if detach_time_s is None or target_axis_world is None:
        return {"available": False}
    axis = _vector3(target_axis_world, "target_axis_world")
    axis_norm = float(np.linalg.norm(axis))
    if axis_norm == 0.0:
        return {"available": False}
    axis /= axis_norm
    ordered = sorted(records, key=lambda record: float(record["time_s"]))
    preopen_candidates = [
        record
        for record in ordered
        if float(record["time_s"]) <= release_motion_start_time_s + 1.0e-9
    ]
    predetach_candidates = [
        record
        for record in ordered
        if float(record["time_s"]) < detach_time_s - 1.0e-9
    ]
    detach_candidates = [
        record
        for record in ordered
        if float(record["time_s"]) >= detach_time_s - 1.0e-9
    ]
    if not preopen_candidates or not predetach_candidates or not detach_candidates:
        return {"available": False}

    preopen = preopen_candidates[-1]
    predetach = predetach_candidates[-1]
    detach = detach_candidates[0]
    postdetach_candidates = [
        record
        for record in detach_candidates
        if float(record["time_s"]) >= detach_time_s + DETACH_DEBOUNCE_S - 1.0e-9
    ]
    postdetach = postdetach_candidates[0] if postdetach_candidates else detach

    def vector(record, key):
        return _vector3(record[key], key)

    def projected(record, key):
        return float(np.dot(vector(record, key), axis))

    cube_preopen_axis = projected(preopen, "cube_angular_velocity_w_rad_s")
    cube_predetach_axis = projected(predetach, "cube_angular_velocity_w_rad_s")
    cube_detach_axis = projected(detach, "cube_angular_velocity_w_rad_s")
    cube_postdetach_axis = projected(postdetach, "cube_angular_velocity_w_rad_s")
    hand_detach_angular = vector(detach, "hand_angular_velocity_w_rad_s")
    hand_detach_axis = float(np.dot(hand_detach_angular, axis))
    lever_world = (
        vector(detach, "cube_position_w_m")
        - vector(detach, "hand_position_w_m")
    )
    nominal_linear = (
        vector(detach, "hand_linear_velocity_w_m_s")
        + np.cross(hand_detach_angular, lever_world)
    )
    actual_linear = vector(detach, "cube_linear_velocity_w_m_s")
    actual_angular = vector(detach, "cube_angular_velocity_w_rad_s")
    angular_residual = actual_angular - hand_detach_angular

    def ratio(numerator: float, denominator: float) -> float | None:
        if abs(denominator) <= 1.0e-9:
            return None
        return float(numerator / denominator)

    return {
        "available": True,
        "target_axis_world": axis.tolist(),
        "release_motion_start_time_s": float(release_motion_start_time_s),
        "preopen_sample_time_s": float(preopen["time_s"]),
        "predetach_sample_time_s": float(predetach["time_s"]),
        "detach_sample_time_s": float(detach["time_s"]),
        "postdetach_debounce_sample_time_s": float(postdetach["time_s"]),
        "release_contact_window_s": float(detach["time_s"])
        - float(preopen["time_s"]),
        "cube_preopen_axis_omega_rad_s": cube_preopen_axis,
        "cube_predetach_axis_omega_rad_s": cube_predetach_axis,
        "cube_detach_axis_omega_rad_s": cube_detach_axis,
        "cube_postdetach_axis_omega_rad_s": cube_postdetach_axis,
        "hand_detach_axis_omega_rad_s": hand_detach_axis,
        "cube_axis_angular_retention_from_preopen": ratio(
            cube_detach_axis, cube_preopen_axis
        ),
        "cube_axis_angular_transfer_from_hand_at_detach": ratio(
            cube_detach_axis, hand_detach_axis
        ),
        "detach_linear_velocity_residual_w_m_s": (
            actual_linear - nominal_linear
        ).tolist(),
        "detach_angular_velocity_residual_w_rad_s": angular_residual.tolist(),
        "detach_axis_angular_velocity_residual_rad_s": float(
            np.dot(angular_residual, axis)
        ),
    }



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
    ends at the first subsequent robot contact. Catch stability remains a
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
            "free_flight_rotation_rad": 0.0,
            "free_flight_rotation_deg": 0.0,
            "visible_spin": False,
            **_empty_tumble_evidence(),
        }

    def force(record, key):
        value = float(record[key])
        if not math.isfinite(value):
            raise ValueError(f"{key} must be finite")
        return value

    def contact_free(record):
        forces, _ = _robot_contact_forces(record)
        return all(value <= contact_force_threshold_n for value in forces.values())

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
            "free_flight_rotation_rad": 0.0,
            "free_flight_rotation_deg": 0.0,
            "visible_spin": False,
            **_empty_tumble_evidence(),
        }

    def run_duration(run):
        first, last, contact = run
        end_time = float(
            postrelease[contact if contact is not None else last]["time_s"]
        )
        return end_time - float(postrelease[first]["time_s"])

    sustained_runs = [
        run for run in runs if run_duration(run) >= DETACH_DEBOUNCE_S
    ]
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
    strict_contact_source_complete = all(
        _robot_contact_forces(record)[1]
        for record in postrelease[
            start_index : (contact_index + 1 if contact_index is not None else last_free_index + 1)
        ]
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

    tumble = _empty_tumble_evidence()
    if "hand_quaternion_wxyz" in first_record:
        hand_rotation = _rotation_from_wxyz(
            first_record["hand_quaternion_wxyz"], "hand_quaternion_wxyz"
        )
        finger_direction_world = hand_rotation @ np.asarray([0.0, 0.0, 1.0])
        tumble_axis_world = np.cross(
            finger_direction_world, np.asarray([0.0, 0.0, 1.0])
        )
        tumble_axis_norm = float(np.linalg.norm(tumble_axis_world))
        if tumble_axis_norm > 1.0e-6:
            tumble_axis_world /= tumble_axis_norm
            detach_omega = _vector3(
                first_record["cube_angular_velocity_w_rad_s"],
                "cube_angular_velocity_w_rad_s",
            )
            detach_omega_norm = float(np.linalg.norm(detach_omega))
            axis_alignment = (
                0.0
                if detach_omega_norm == 0.0
                else abs(float(np.dot(detach_omega / detach_omega_norm, tumble_axis_world)))
            )
            signed_tumble, non_target = _integrated_axis_rotation(
                free_records, tumble_axis_world
            )
            apex_tumble, _ = _integrated_axis_rotation(
                free_records[: apex_local_index + 1], tumble_axis_world
            )
            target_axis_tumble = (
                strict_contact_source_complete
                and axis_alignment >= TUMBLE_MIN_AXIS_ALIGNMENT
                and abs(apex_tumble) >= TUMBLE_MIN_APEX_ROTATION_RAD
                and abs(signed_tumble) >= TUMBLE_MIN_FLIGHT_ROTATION_RAD
                and non_target <= abs(signed_tumble)
            )
            tumble = {
                "strict_contact_source_complete": strict_contact_source_complete,
                "strict_contact_free_flight": strict_contact_source_complete,
                "finger_direction_world": finger_direction_world.tolist(),
                "tumble_axis_world": tumble_axis_world.tolist(),
                "detach_angular_velocity_world_rad_s": detach_omega.tolist(),
                "tumble_axis_alignment": axis_alignment,
                "detach_to_apex_tumble_rotation_rad": apex_tumble,
                "detach_to_apex_tumble_rotation_deg": math.degrees(apex_tumble),
                "free_flight_signed_tumble_rotation_rad": signed_tumble,
                "free_flight_signed_tumble_rotation_deg": math.degrees(signed_tumble),
                "free_flight_non_target_rotation_rad": non_target,
                "target_axis_tumble": target_axis_tumble,
            }

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
    visible_spin = (
        duration_s >= VISIBLE_SPIN_MIN_FLIGHT_S
        and rotation_rad >= VISIBLE_SPIN_MIN_ROTATION_RAD
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
        "free_flight_rotation_deg": math.degrees(rotation_rad),
        "free_flight_spin_path_rad": spin_path_rad,
        "visible_spin": visible_spin,
        "obvious_free_flight": obvious,
        **tumble,
    }
