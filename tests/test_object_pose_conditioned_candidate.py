from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
spec = importlib.util.spec_from_file_location(
    "object_pose_policy",
    ROOT / "sim/tools/build_object_pose_conditioned_candidate.py",
)
policy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(policy)


def test_dataset_loads_four_objects_with_stable_ordered_endpoints():
    dataset = policy.load_dataset()
    assert len(dataset["objects"]) == 4
    for row in dataset["objects"]:
        assert row["low"]["measured_rotation_deg"] < row["high"]["measured_rotation_deg"]
        assert len(row["dimensions_m"]) == 3
        assert row["mass_kg"] > 0.0
        assert row["principal_inertia_kg_m2"][1] > 0.0


@pytest.mark.parametrize(
    "object_id",
    [
        "yellow_cube_38mm_8g",
        "cuboid_44p5x46x30mm_20g",
        "cuboid_50p5x51x33p5mm_26p6g",
        "cuboid_57p5x58x38mm_37g",
    ],
)
def test_m3_continuous_proposal_hits_midpoint_and_preserves_j235(object_id):
    dataset = policy.load_dataset()
    row = policy.object_row(dataset, object_id)
    desired = 0.5 * (
        row["low"]["measured_rotation_deg"]
        + row["high"]["measured_rotation_deg"]
    )
    config, report = policy.build_candidate(dataset, object_id, desired)
    selected = report["methods"]["M3_object_pose_conditioned"]
    assert 0.0 < selected["alpha"] < 1.0
    assert selected["predicted_rotation_deg"] == pytest.approx(desired)
    assert selected["j_terms"]["angle_error_deg"] == pytest.approx(0.0)
    assert config["pose_conditioned_generation"]["reference_limit_evidence"][
        "joint_mechanical_limits_pass"
    ]
    assert len(config["reference_segments"]) == 45
    for segment in config["reference_segments"]:
        for field in (
            "start_joint_rad",
            "end_joint_rad",
            "start_joint_velocity_rad_s",
            "end_joint_velocity_rad_s",
        ):
            values = np.asarray(segment[field])
            if "velocity" in field:
                np.testing.assert_allclose(values[list(policy.FIXED_JOINTS)], 0.0)


@pytest.mark.parametrize("alpha", [0.0, 0.37, 1.0])
def test_tick_interpolation_is_convex_and_reproduces_endpoint_commands(alpha):
    dataset = policy.load_dataset()
    row = policy.object_row(dataset, "cuboid_44p5x46x30mm_20g")
    desired = policy.response_angle(row, alpha)
    config, _ = policy.build_candidate(dataset, row["object_id"], desired)
    actual = policy.sampled_reference(config)
    low = policy.sampled_reference(policy.load_json(policy.ROOT / row["low"]["arm_config"]))
    high = policy.sampled_reference(policy.load_json(policy.ROOT / row["high"]["arm_config"]))
    assert len(actual) == len(low) == len(high)
    for blended, low_sample, high_sample in zip(actual, low, high, strict=True):
        np.testing.assert_allclose(
            blended.joint_position_rad,
            (1.0 - alpha) * np.asarray(low_sample.joint_position_rad)
            + alpha * np.asarray(high_sample.joint_position_rad),
            atol=1.0e-12,
        )
        np.testing.assert_allclose(
            blended.joint_velocity_rad_s,
            (1.0 - alpha) * np.asarray(low_sample.joint_velocity_rad_s)
            + alpha * np.asarray(high_sample.joint_velocity_rad_s),
            atol=1.0e-12,
        )


def test_m2_is_discrete_while_m3_is_continuous_for_off_grid_target():
    dataset = policy.load_dataset()
    row = policy.object_row(dataset, "cuboid_44p5x46x30mm_20g")
    desired = policy.response_angle(row, 0.37)
    methods = policy.compare_methods(dataset, row, desired)
    assert methods["M2_search_only"]["alpha"] == pytest.approx(0.4)
    assert methods["M3_object_pose_conditioned"]["alpha"] == pytest.approx(0.37)
    assert methods["M3_object_pose_conditioned"]["j"] < methods["M2_search_only"]["j"]


def test_additional_stable_trial_corrects_nonlinear_o2_response():
    dataset = policy.load_dataset()
    row = policy.object_row(dataset, "cuboid_50p5x51x33p5mm_26p6g")
    measured_mid = row["calibration_points"][1]
    assert measured_mid["action_alpha"] == pytest.approx(0.4833576883176665)
    assert measured_mid["measured_rotation_deg"] == pytest.approx(4.776156485982997)
    corrected = policy.action_for_angle(row, 5.5)
    assert corrected > measured_mid["action_alpha"]
    assert policy.response_angle(row, corrected) == pytest.approx(5.5)


def test_unknown_object_is_rejected():
    with pytest.raises(KeyError, match="unknown object_id"):
        policy.object_row(policy.load_dataset(), "missing")
