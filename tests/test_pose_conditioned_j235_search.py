from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
spec = importlib.util.spec_from_file_location(
    "pose_search", ROOT / "sim/tools/search_pose_conditioned_j235.py"
)
search = importlib.util.module_from_spec(spec)
spec.loader.exec_module(search)

from xarm6_toss_sim.kinematics import URDFKinematics  # noqa: E402


def test_retention_calibration_uses_actual_transfer_evidence(tmp_path):
    paths = []
    for index, (hand, cube) in enumerate(((3.0, 0.45), (4.0, 0.64))):
        path = tmp_path / f"summary_{index}.json"
        path.write_text(
            json.dumps(
                {
                    "release_transfer_evidence": {
                        "hand_detach_axis_omega_rad_s": hand,
                        "cube_postdetach_axis_omega_rad_s": cube,
                    },
                    "free_flight_rotation_deg": 8.0 + index,
                    "catch_stable": True,
                }
            ),
            encoding="utf-8",
        )
        paths.append(path)
    calibration = search.load_retention_calibration(paths)
    assert calibration["angular_retention"] == pytest.approx(0.155)
    assert len(calibration["observations"]) == 2


def test_candidate_has_only_j2_j3_j5_and_respects_motion_constraints():
    kinematics = URDFKinematics(search.ladder.base.URDF)
    offset = np.asarray([0.0, -0.19, 0.05, 0.0, 0.0, 0.0])
    candidate = search.candidate_from_state(
        kinematics,
        offset,
        np.asarray([-0.4105, -1.7448, 1.7448]),
        desired_angle_deg=15.0,
        angular_retention=0.155,
    )
    assert candidate is not None
    velocity = candidate["release_joint_velocity_rad_s"]
    assert [velocity[index] for index in (0, 3, 5)] == [0.0, 0.0, 0.0]
    assert candidate["release_height_m"] >= 0.40
    assert candidate["predicted_axis_alignment"] >= 0.92
    assert candidate["predicted_tcp_speed_m_s"] <= 1.60
    assert candidate["predicted_rotation_deg"] > 13.0


def test_search_selects_angle_conditioned_candidates():
    kinematics = URDFKinematics(search.ladder.base.URDF)
    report = search.search_angle(
        kinematics,
        desired_angle_deg=12.0,
        angular_retention=0.155,
        pose_offsets=np.asarray([-0.19, -0.11]),
        velocity_values=np.asarray([-1.7448, -0.4105, 0.0, 1.7448]),
        top_k=3,
    )
    assert report["candidate_count"] > 0
    assert len(report["top_candidates"]) == 3
    selected = report["selected"]
    assert selected["desired_angle_deg"] == 12.0
    assert abs(selected["predicted_rotation_deg"] - 12.0) < 3.0
    assert set(selected["j_breakdown"]) == {
        "angle_error_deg",
        "lateral_penalty",
        "height_penalty",
        "motion_penalty",
    }


def test_invalid_release_direction_is_rejected():
    kinematics = URDFKinematics(search.ladder.base.URDF)
    assert search.candidate_from_state(
        kinematics,
        np.zeros(6),
        np.asarray([0.2, -0.5, 1.0]),
        10.0,
        0.155,
    ) is None

