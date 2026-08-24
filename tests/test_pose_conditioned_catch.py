from __future__ import annotations

import json
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from xarm6_toss.pose_conditioned_catch import (
    DYNAMIC_JOINT_INDICES,
    FIXED_JOINT_INDICES,
    ballistic_continuation,
    catch_reference_sample,
    infer_local_point,
    integrate_accelerations,
    load_catch_reference,
    signed_rotation_deg,
)


class TranslationKinematics:
    def forward(self, joint_rad):
        joint = np.asarray(joint_rad, dtype=float)
        transform = np.eye(4)
        transform[:3, 3] = joint[:3]
        return transform


def test_integrate_accelerations_uses_new_velocity_for_each_step():
    position, velocity = integrate_accelerations(
        np.asarray([0.0, 1.0, -1.0]),
        np.asarray([0.0, 0.5, 0.0]),
        np.asarray([[1.0, -1.0, 0.5], [1.0, -1.0, 0.5]]),
        0.02,
    )
    assert velocity[-1] == pytest.approx([0.04, 0.46, 0.02])
    assert position[-1] == pytest.approx([0.0012, 1.0188, -0.9994])


def test_infer_local_point_recovers_fixed_offset():
    local = np.asarray([0.01, -0.02, 0.03])
    rows = []
    for index in range(5):
        joint = np.asarray([0.1 * index, -0.02 * index, 0.03 * index, 0, 0, 0])
        world = joint[:3] + local
        rows.append(
            {
                "time_s": 0.1 * index,
                "arm_joint_position_rad": joint.tolist(),
                "finger_midpoint_w_m": world.tolist(),
            }
        )
    result = infer_local_point(
        rows,
        TranslationKinematics(),
        point_key="finger_midpoint_w_m",
        start_time_s=0.0,
        end_time_s=0.4,
    )
    assert result == pytest.approx(local)


def test_signed_rotation_is_about_requested_world_axis():
    def row(angle_deg):
        xyzw = Rotation.from_euler("z", angle_deg, degrees=True).as_quat()
        return {"cube_quaternion_wxyz": [xyzw[3], *xyzw[:3]]}

    assert signed_rotation_deg(row(5.0), row(35.0), [0.0, 0.0, 1.0]) == pytest.approx(30.0)
    assert signed_rotation_deg(row(5.0), row(35.0), [1.0, 0.0, 0.0]) == pytest.approx(0.0)


def test_ballistic_continuation_replaces_only_post_anchor_cube_state():
    rows = []
    for index, time_s in enumerate((0.0, 0.1, 0.2)):
        rows.append(
            {
                "time_s": time_s,
                "arm_joint_position_rad": [float(index)] * 6,
                "cube_position_w_m": [99.0, 99.0, 99.0],
                "cube_linear_velocity_w_m_s": [99.0, 99.0, 99.0],
                "cube_angular_velocity_w_rad_s": [99.0, 99.0, 99.0],
                "cube_quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
            }
        )
    rows[1].update(
        cube_position_w_m=[1.0, 2.0, 3.0],
        cube_linear_velocity_w_m_s=[1.0, 2.0, 3.0],
        cube_angular_velocity_w_rad_s=[0.0, 0.0, np.pi],
    )

    result, anchor_time = ballistic_continuation(rows, anchor_time_s=0.101)

    assert anchor_time == pytest.approx(0.1)
    assert result[0] == rows[0]
    assert result[1] == rows[1]
    assert result[2]["arm_joint_position_rad"] == [2.0] * 6
    assert result[2]["cube_position_w_m"] == pytest.approx(
        [1.1, 2.2, 3.25095]
    )
    assert result[2]["cube_linear_velocity_w_m_s"] == pytest.approx(
        [1.0, 2.0, 2.019]
    )
    assert result[2]["cube_angular_velocity_w_rad_s"] == pytest.approx(
        [0.0, 0.0, np.pi]
    )
    rotation = Rotation.from_quat(
        [*result[2]["cube_quaternion_wxyz"][1:], result[2]["cube_quaternion_wxyz"][0]]
    )
    assert rotation.as_rotvec() == pytest.approx([0.0, 0.0, 0.1 * np.pi])


@pytest.mark.parametrize("anchor", [-0.1, 0.3, float("nan")])
def test_ballistic_continuation_rejects_invalid_anchor(anchor):
    rows = [{"time_s": 0.0}, {"time_s": 0.2}]
    with pytest.raises(ValueError):
        ballistic_continuation(rows, anchor_time_s=anchor)


def test_joint_partition_is_exactly_j235():
    assert DYNAMIC_JOINT_INDICES == (1, 2, 4)
    assert FIXED_JOINT_INDICES == (0, 3, 5)


def valid_reference():
    rows = []
    for index in range(3):
        rows.append(
            {
                "time_s": 0.68 + 0.02 * index,
                "phase": "offline_j235_intercept",
                "joint_position_rad": [0.0, 0.1 + 0.001 * index, -0.5, 2.8, 1.4, 0.0],
                "joint_velocity_rad_s": [0.0, 0.05, 0.0, 0.0, 0.0, 0.0],
                "joint_acceleration_rad_s2": [0.0] * 6,
            }
        )
    return {
        "schema": "xarm6_offline_j235_catch_reference_v1",
        "start_time_s": 0.68,
        "intercept_time_s": 0.72,
        "capture_admitted": True,
        "dynamic_joint_indices_zero_based": [1, 2, 4],
        "fixed_joint_indices_zero_based": [0, 3, 5],
        "samples": rows,
    }


def test_offline_reference_loads_and_samples(tmp_path):
    path = tmp_path / "catch.json"
    path.write_text(json.dumps(valid_reference()), encoding="utf-8")
    reference = load_catch_reference(path)
    assert reference["control_period_s"] == pytest.approx(0.02)
    assert reference["joint_limit_evidence"]["joint_mechanical_limits_pass"]
    assert catch_reference_sample(reference, 0.701)["time_s"] == pytest.approx(0.70)
    assert catch_reference_sample(reference, 9.0)["time_s"] == pytest.approx(0.72)


@pytest.mark.parametrize("mutation", ["move_fixed", "fixed_velocity", "not_admitted"])
def test_offline_reference_rejects_invalid_transfer(tmp_path, mutation):
    payload = valid_reference()
    if mutation == "move_fixed":
        payload["samples"][1]["joint_position_rad"][0] = 0.01
    elif mutation == "fixed_velocity":
        payload["samples"][1]["joint_velocity_rad_s"][3] = 0.01
    else:
        payload["capture_admitted"] = False
    path = tmp_path / "catch.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ValueError):
        load_catch_reference(path)
