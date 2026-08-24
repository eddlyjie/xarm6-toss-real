from __future__ import annotations

import importlib.util
from pathlib import Path
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "measure_g1_position", ROOT / "scripts/30_measure_g1_position.py"
)
measurement = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(measurement)


def test_default_is_dry_run_and_does_not_read_hardware_config(tmp_path):
    result = subprocess.run(
        [
            sys.executable,
            str(ROOT / "scripts/30_measure_g1_position.py"),
            "--object",
            "O2",
            "--purpose",
            "held",
            "--position",
            "390",
            "--hardware-config",
            str(tmp_path / "absent.json"),
        ],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    assert '"robot_connection_attempted": false' in result.stdout
    assert '"arm_motion_requested": false' in result.stdout
    assert "no robot connection or command was sent" in result.stdout


def test_measurement_uses_only_gripper_path():
    class FakeClient:
        def __init__(self):
            self.prepared = 0

        def prepare_gripper_only(self):
            self.prepared += 1

    class FakeRobot:
        def __init__(self):
            self.client = FakeClient()
            self.positions = [370.0, 431.0]
            self.commands = []

        def gripper_position(self, *, check_baud):
            assert check_baud is False
            return self.positions.pop(0)

        def set_gripper_position(self, position):
            self.commands.append(position)

    robot = FakeRobot()
    result = measurement.measure_position(robot, 430)

    assert robot.client.prepared == 1
    assert robot.commands == [430.0]
    assert result == {
        "target_position": 430,
        "reported_before": 370.0,
        "reported_after": 431.0,
        "absolute_target_error": 1.0,
    }


@pytest.mark.parametrize("value", [-1, 851])
def test_position_outside_g1_range_is_rejected(value):
    with pytest.raises(ValueError, match="0..850"):
        measurement.g1_position(value)


def test_source_has_no_arm_motion_entrypoint():
    source = (ROOT / "scripts/30_measure_g1_position.py").read_text(encoding="utf-8")
    assert "prepare_motion(" not in source
    assert "move_joints(" not in source
    assert "servo_j(" not in source
