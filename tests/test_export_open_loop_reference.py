import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "export_open_loop_reference",
    ROOT / "sim/tools/export_open_loop_reference.py",
)
EXPORTER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(EXPORTER)


def write_json(path, payload):
    path.write_text(json.dumps(payload), encoding="utf-8")


def command_row(time_s, position, *, tick=True):
    return {
        "time_s": time_s,
        "phase": "catch",
        "arm_control_tick": tick,
        "arm_command_position_rad": [0.0, position, position, 0.0, position, 0.0],
        "arm_command_velocity_rad_s": [0.0, 0.1, 0.1, 0.0, 0.1, 0.0],
        "arm_command_acceleration_rad_s2": [0.0, 0.2, 0.2, 0.0, 0.2, 0.0],
        "gripper_command_drive_rad": 0.65,
    }


def test_exporter_keeps_only_20ms_command_ticks(tmp_path):
    trajectory = tmp_path / "trajectory.json"
    summary = tmp_path / "summary.json"
    write_json(
        trajectory,
        [
            command_row(0.50, 0.0),
            command_row(0.51, 9.0, tick=False),
            command_row(0.52, 0.002),
            command_row(0.54, 0.004),
        ],
    )
    write_json(
        summary,
        {
            "catch_stable": True,
            "catch_j235_only": True,
            "free_flight_signed_tumble_rotation_deg": 10.2,
            "continuous_free_flight_duration_s": 0.3,
        },
    )

    payload = EXPORTER.export_reference(
        trajectory, summary, profile_id="cube_10deg_open_loop"
    )
    assert [sample["time_s"] for sample in payload["samples"]] == pytest.approx(
        [0.0, 0.02, 0.04]
    )
    assert payload["samples"][1]["joint_position_rad"] == [
        0.0,
        0.002,
        0.002,
        0.0,
        0.002,
        0.0,
    ]
    assert payload["dynamic_joint_indices_zero_based"] == [1, 2, 4]
    assert payload["reference_limit_evidence"]["joint_mechanical_limits_pass"]


def test_exporter_rejects_failed_catch(tmp_path):
    trajectory = tmp_path / "trajectory.json"
    summary = tmp_path / "summary.json"
    write_json(trajectory, [command_row(0.0, 0.0), command_row(0.02, 0.0)])
    write_json(summary, {"catch_stable": False})
    with pytest.raises(ValueError, match="stable-catch"):
        EXPORTER.export_reference(trajectory, summary, profile_id="failed")
