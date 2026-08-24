"""Small, offline-only helpers for measuring marked-cube image rotation."""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np


def marker_edge_angle_deg(corners: Iterable[Iterable[float]]) -> float:
    """Return clockwise image-plane angle of ArUco corner 0 -> corner 1."""
    points = np.asarray(corners, dtype=float)
    if points.shape != (4, 2) or not np.all(np.isfinite(points)):
        raise ValueError("marker corners must have finite shape (4, 2)")
    edge = points[1] - points[0]
    if float(np.linalg.norm(edge)) <= 1.0e-9:
        raise ValueError("marker top edge has zero length")
    return math.degrees(math.atan2(float(edge[1]), float(edge[0])))


def unwrap_angle_deg(raw_angles_deg: Iterable[float]) -> np.ndarray:
    values = np.asarray(list(raw_angles_deg), dtype=float)
    if values.ndim != 1 or values.size == 0 or not np.all(np.isfinite(values)):
        raise ValueError("angles must be a nonempty finite vector")
    return np.rad2deg(np.unwrap(np.deg2rad(values)))


def summarize_measurements(rows: list[dict]) -> dict:
    if not rows:
        raise ValueError("no marker detections in the requested video interval")
    raw = [float(row["raw_angle_deg"]) for row in rows]
    unwrapped = unwrap_angle_deg(raw)
    relative = unwrapped - unwrapped[0]
    for row, angle, relative_angle in zip(rows, unwrapped, relative):
        row["unwrapped_angle_deg"] = float(angle)
        row["relative_angle_deg"] = float(relative_angle)
    peak_index = int(np.argmax(np.abs(relative)))
    return {
        "detected_frame_count": len(rows),
        "first_time_s": float(rows[0]["time_s"]),
        "last_time_s": float(rows[-1]["time_s"]),
        "signed_rotation_first_to_last_deg": float(relative[-1]),
        "peak_absolute_rotation_deg": float(abs(relative[peak_index])),
        "peak_time_s": float(rows[peak_index]["time_s"]),
        "minimum_relative_angle_deg": float(np.min(relative)),
        "maximum_relative_angle_deg": float(np.max(relative)),
    }
