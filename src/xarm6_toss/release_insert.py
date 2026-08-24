"""Geometry helpers for printable passive release inserts."""

from __future__ import annotations

import math


def strike_return_command(
    elapsed_s: float,
    travel_m: float,
    strike_s: float,
    return_s: float | None,
    hold_s: float = 0.0,
) -> tuple[float, float]:
    """Position and velocity for a one-shot spring-tab strike and return."""
    if elapsed_s <= 0.0:
        return 0.0, 0.0
    if elapsed_s < strike_s:
        progress = elapsed_s / strike_s
        return -travel_m * progress, -travel_m / strike_s
    if return_s is None:
        return -travel_m, 0.0
    return_elapsed_s = elapsed_s - strike_s - hold_s
    if return_elapsed_s <= 0.0:
        return -travel_m, 0.0
    if return_elapsed_s < return_s:
        progress = return_elapsed_s / return_s
        return -travel_m * (1.0 - progress), travel_m / return_s
    return 0.0, 0.0


def d_roller_mesh(
    radius_m: float,
    chord_m: float,
    length_m: float,
    arc_segments: int = 24,
) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]]]:
    """Return a convex D-profile roller extruded along local X.

    The flat chord faces local +Y.  Mirroring the complete body with a 180
    degree rotation about X gives the matching right-finger part.
    """
    if not 0.0 <= chord_m < radius_m:
        raise ValueError("D-roller chord must satisfy 0 <= chord < radius")
    if arc_segments < 4:
        raise ValueError("D-roller arc requires at least four segments")

    theta = math.acos(chord_m / radius_m)
    arc_span = 2.0 * (math.pi - theta)
    profile = [
        (
            radius_m * math.cos(theta + arc_span * index / arc_segments),
            radius_m * math.sin(theta + arc_span * index / arc_segments),
        )
        for index in range(arc_segments + 1)
    ]
    half_length = 0.5 * length_m
    vertices = [
        (x, y, z)
        for x in (-half_length, half_length)
        for y, z in profile
    ]
    count = len(profile)
    faces: list[tuple[int, int, int]] = []

    # The profile is counter-clockwise when viewed from +X.
    for index in range(1, count - 1):
        faces.append((0, index + 1, index))
        faces.append((count, count + index, count + index + 1))
    for index in range(count):
        next_index = (index + 1) % count
        faces.append((index, next_index, count + next_index))
        faces.append((index, count + next_index, count + index))
    return vertices, faces
