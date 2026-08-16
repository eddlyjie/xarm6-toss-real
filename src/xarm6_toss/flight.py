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
