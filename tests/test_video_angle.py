from pathlib import Path
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from xarm6_toss.video_angle import (
    marker_edge_angle_deg,
    summarize_measurements,
    unwrap_angle_deg,
)


def corners_for_angle(angle_deg: float) -> np.ndarray:
    angle = np.deg2rad(angle_deg)
    edge = np.asarray([np.cos(angle), np.sin(angle)])
    normal = np.asarray([-edge[1], edge[0]])
    origin = np.asarray([100.0, 100.0])
    return np.asarray(
        [origin, origin + edge, origin + edge + normal, origin + normal]
    )


@pytest.mark.parametrize("angle", [-70.0, -5.0, 0.0, 12.5, 85.0])
def test_marker_edge_angle(angle):
    assert marker_edge_angle_deg(corners_for_angle(angle)) == pytest.approx(angle)


def test_unwrap_crosses_180_degree_boundary():
    result = unwrap_angle_deg([170.0, 179.0, -175.0, -160.0])
    assert result.tolist() == pytest.approx([170.0, 179.0, 185.0, 200.0])


def test_summary_reports_signed_and_peak_rotation():
    rows = [
        {"frame_index": 0, "time_s": 0.0, "raw_angle_deg": 170.0},
        {"frame_index": 1, "time_s": 0.01, "raw_angle_deg": -175.0},
        {"frame_index": 2, "time_s": 0.02, "raw_angle_deg": -160.0},
    ]
    summary = summarize_measurements(rows)
    assert summary["signed_rotation_first_to_last_deg"] == pytest.approx(30.0)
    assert summary["peak_absolute_rotation_deg"] == pytest.approx(30.0)
    assert rows[-1]["relative_angle_deg"] == pytest.approx(30.0)


def test_invalid_corners_and_empty_measurements_fail():
    with pytest.raises(ValueError):
        marker_edge_angle_deg(np.zeros((3, 2)))
    with pytest.raises(ValueError):
        summarize_measurements([])
