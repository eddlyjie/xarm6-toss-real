import importlib.util
import json
from pathlib import Path
import subprocess
from types import SimpleNamespace
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from xarm6_toss.open_loop_demo import (  # noqa: E402
    DeploymentPlanError,
    prepare_deployment,
    profile_for_angle,
)


def test_cube_profile_uses_current_measured_mass():
    config = json.loads(
        (ROOT / "configs/objects/cube_38mm_8g.json").read_text()
    )
    assert config["dimensions_m"] == [0.038, 0.038, 0.038]
    assert config["mass_kg"] == 0.008


def test_catalog_uses_exact_profiles_without_angle_fallback():
    assert profile_for_angle(ROOT, 0).name == "baseline_0deg.json"
    assert profile_for_angle(ROOT, 5).name == "low_5deg.json"
    assert profile_for_angle(ROOT, 6.5).name == "medium_6p5deg.json"
    assert profile_for_angle(ROOT, 8).name == "high_8deg.json"
    assert profile_for_angle(ROOT, 10).name == "sim_10deg.json"
    with pytest.raises(DeploymentPlanError, match="planned"):
        profile_for_angle(ROOT, 20)
    with pytest.raises(DeploymentPlanError, match="no exact profile"):
        profile_for_angle(ROOT, 7)


def test_real_baseline_builds_open_loop_cube_plan():
    plan, samples = prepare_deployment(
        ROOT, desired_angle_deg=0, mode="cube"
    )
    assert plan["profile_status"] == "real_micro_toss_baseline"
    assert plan["object_mass_kg"] == 0.008
    assert [event["time_s"] for event in plan["g1_events"]] == [0.636, 0.72]
    assert plan["joint_limits"]["joint_mechanical_limits_pass"]
    assert len(samples) == plan["sample_count"]
    assert not plan["online_probe"]
    assert not plan["online_vision"]
    assert not plan["online_ballistic_correction"]


def test_slow_empty_preview_scales_timeline_and_g1_events():
    plan, _ = prepare_deployment(
        ROOT, desired_angle_deg=5, speed_scale=0.25, mode="empty_g1"
    )
    assert plan["duration_s"] == pytest.approx(7.28)
    assert [event["time_s"] for event in plan["g1_events"]] == pytest.approx(
        [2.48, 3.04, 3.24]
    )


def test_legacy_10deg_profile_does_not_claim_open_loop_cube_permission():
    with pytest.raises(DeploymentPlanError, match="does not allow cube"):
        prepare_deployment(ROOT, desired_angle_deg=10, mode="cube")
    plan, _ = prepare_deployment(
        ROOT, desired_angle_deg=10, mode="throw_only"
    )
    assert [event["name"] for event in plan["g1_events"]] == ["release"]


def test_cube38_low_medium_high_profiles_are_deployable_and_distinct():
    measured = []
    for desired_angle in (5.0, 6.5, 8.0):
        plan, samples = prepare_deployment(
            ROOT, desired_angle_deg=desired_angle, mode="cube"
        )
        assert plan["profile_status"] == (
            "sim_validated_open_loop_reference_real_unverified"
        )
        assert plan["sample_count"] == 92
        assert plan["joint_limits"]["joint_mechanical_limits_pass"]
        assert [event["name"] for event in plan["g1_events"]] == [
            "release", "preclose", "close"
        ]
        profile = json.loads((ROOT / plan["profile"]).read_text())
        measured.append(profile["evidence"]["sim_measured_rotation_deg"])
        schedule = json.loads(
            (ROOT / profile["g1_schedule"]).read_text()
        )
        assert schedule["events"] == profile["g1"]["events"]
        for joint_index in (0, 3, 5):
            assert all(
                sample["joint_position_rad"][joint_index]
                == samples[0]["joint_position_rad"][joint_index]
                for sample in samples
            )
    assert measured == sorted(measured)
    assert all(after - before > 1.0 for before, after in zip(measured, measured[1:]))


@pytest.mark.parametrize(
    "profile_path, object_id, measured_angle",
    [
        (
            "configs/open_loop_flip/cuboid30/low_3deg.json",
            "cuboid_44p5x46x30mm_20g",
            2.955258208553455,
        ),
        (
            "configs/open_loop_flip/cuboid33/low_5deg.json",
            "cuboid_50p5x51x33p5mm_26p6g",
            4.614634194842018,
        ),
        (
            "configs/open_loop_flip/cuboid38/low_4p5deg.json",
            "cuboid_57p5x58x38mm_37g",
            4.398300045151973,
        ),
    ],
)
def test_cuboid_transfer_profiles_are_plan_and_empty_arm_ready_only(
    profile_path, object_id, measured_angle
):
    plan, samples = prepare_deployment(
        ROOT, profile_path=Path(profile_path), mode="plan"
    )
    profile = json.loads((ROOT / profile_path).read_text())
    assert plan["object_id"] == object_id
    assert len(plan["object_dimensions_m"]) == 3
    assert plan["profile_status"] == (
        "sim_validated_arm_reference_g1_calibration_required"
    )
    assert profile["evidence"]["sim_measured_rotation_deg"] == pytest.approx(
        measured_angle
    )
    assert profile["evidence"]["method_stage"] == (
        "M0_fixed_replay_transfer_candidate"
    )
    assert plan["sample_count"] == len(samples) == 92
    assert plan["joint_limits"]["joint_mechanical_limits_pass"]
    assert all(event["position"] is None for event in plan["g1_events"])
    empty_plan, _ = prepare_deployment(
        ROOT, profile_path=Path(profile_path), mode="empty_arm"
    )
    assert empty_plan["g1_events"] == []
    with pytest.raises(DeploymentPlanError, match="does not allow object"):
        prepare_deployment(
            ROOT, profile_path=Path(profile_path), mode="object"
        )


@pytest.mark.parametrize(
    "profile_path, expected_angle",
    [
        ("configs/open_loop_flip/cuboid30/high_6p5deg.json", 6.570222380603423),
        ("configs/open_loop_flip/cuboid33/high_6p5deg.json", 6.446333217235788),
        ("configs/open_loop_flip/cuboid38/high_6p5deg.json", 6.851779273103772),
    ],
)
def test_cuboid_high_profiles_are_distinct_stable_arm_references(
    profile_path, expected_angle
):
    plan, samples = prepare_deployment(
        ROOT, profile_path=Path(profile_path), mode="plan"
    )
    profile = json.loads((ROOT / profile_path).read_text())
    assert profile["evidence"]["sim_measured_rotation_deg"] == pytest.approx(
        expected_angle
    )
    assert profile["evidence"]["sim_obvious_toss_success"]
    assert plan["sample_count"] == len(samples) == 92
    assert plan["joint_limits"]["joint_mechanical_limits_pass"]
    assert [event["time_s"] for event in plan["g1_events"]] == [0.62, 0.80, 0.86]


def test_8deg_profile_is_a_j235_open_loop_cube_reference():
    plan, samples = prepare_deployment(
        ROOT, desired_angle_deg=8, mode="cube"
    )
    assert plan["profile_status"] == (
        "sim_validated_open_loop_reference_real_unverified"
    )
    assert plan["sample_count"] == 92
    assert plan["control_period_s"] == 0.02
    assert [event["name"] for event in plan["g1_events"]] == [
        "release",
        "preclose",
        "close",
    ]
    assert plan["joint_limits"]["joint_mechanical_limits_pass"]
    for joint_index in (0, 3, 5):
        assert all(sample["joint_position_rad"][joint_index] == samples[0]["joint_position_rad"][joint_index] for sample in samples)


def test_runner_is_plan_only_by_default(tmp_path):
    output = tmp_path / "baseline_plan.json"
    result = subprocess.run(
        [
            sys.executable,
            str(ROOT / "scripts/24_run_cube_open_loop_demo.py"),
            "--angle-deg",
            "0",
            "--hardware-config",
            str(tmp_path / "intentionally_absent_hardware.json"),
            "--output-plan",
            str(output),
        ],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    assert '"mode": "plan"' in result.stdout
    assert "no robot connection or command was sent" in result.stdout
    saved = json.loads(output.read_text())
    assert saved["profile_id"] == "cube_real_micro_toss_baseline"
    assert saved["mode"] == "plan"


def test_fixed_g1_events_and_arm_reference_share_one_host_clock(monkeypatch, tmp_path):
    spec = importlib.util.spec_from_file_location(
        "open_loop_runner", ROOT / "scripts/24_run_cube_open_loop_demo.py"
    )
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)

    class FakeClock:
        now = 0.0

        def monotonic(self):
            return self.now

        def sleep(self, duration):
            self.now += duration

    class FakeRobot:
        def __init__(self):
            self.gripper = []
            self.reported_gripper = [370.0, 371.0]
            self.arm = []
            self.controller_samples = 0
            self.modes = []

        def enter_servo_mode(self):
            self.modes.append("servo")

        def enter_position_mode(self):
            self.modes.append("position")

        def command_gripper_position(self, position):
            self.gripper.append(position)

        def gripper_position(self, *, check_baud):
            assert check_baud is False
            return self.reported_gripper.pop(0)

        def controller_status(self):
            self.controller_samples += 1
            return {
                "connected": True,
                "mode": 0,
                "state": 0,
                "err_warn_code": [0, 0],
                "sample": self.controller_samples,
            }

        def servo_j(self, joint_rad):
            self.arm.append(joint_rad)

        def reported_joint_signals(self):
            return {
                "joint_position_rad": [0.0] * 6,
                "joint_velocity_rad_s": [0.0] * 6,
                "joint_effort": [0.0] * 6,
                "motor_current": [0.0] * 6,
            }

    clock = FakeClock()
    monkeypatch.setattr(runner, "time", clock)
    robot = FakeRobot()
    samples = [
        {
            "time_s": 0.0,
            "phase": "throw",
            "joint_position_rad": [0.0] * 6,
            "joint_velocity_rad_s": [0.2] * 6,
        },
        {
            "time_s": 0.02,
            "phase": "brake",
            "joint_position_rad": [0.1] * 6,
            "joint_velocity_rad_s": [-0.1] * 6,
        },
    ]
    events = [
        {"name": "release", "time_s": 0.005, "position": 520},
        {"name": "close", "time_s": 0.015, "position": 370},
    ]
    records, result = runner.execute_reference(
        robot, samples, events, observe_g1=True, observe_controller=True
    )

    assert result["error"] is None
    assert robot.modes == ["servo", "position"]
    assert robot.gripper == [520.0, 370.0]
    assert len(robot.arm) == 2
    assert len(records) == 2
    assert [event["name"] for event in result["g1_events"]] == ["release", "close"]
    assert result["g1_position_samples"] == [
        {"label": "before_servo", "time_s": 0.0, "position": 370.0, "error": None},
        {"label": "after_servo", "time_s": 0.52, "position": 371.0, "error": None},
    ]
    assert records[0]["reference_joint_velocity_rad_s"] == [0.2] * 6
    assert records[1]["reference_joint_velocity_rad_s"] == [-0.1] * 6
    assert [row["label"] for row in result["controller_status_samples"]] == [
        "before_servo",
        "after_servo",
    ]
    assert [row["status"]["sample"] for row in result["controller_status_samples"]] == [
        1,
        2,
    ]
    signals = tmp_path / "signals.csv"
    runner.write_signals(signals, records)
    header = signals.read_text(encoding="utf-8").splitlines()[0].split(",")
    assert "reference_joint_velocity_rad_s_1" in header
    assert "reference_joint_velocity_rad_s_6" in header
    assert "joint_velocity_rad_s_1" in header


def test_runner_requires_hardware_g1_speed_to_match_profile():
    spec = importlib.util.spec_from_file_location(
        "open_loop_runner_speed", ROOT / "scripts/24_run_cube_open_loop_demo.py"
    )
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
    plan = {"g1_speed": 5000}

    runner.validate_hardware_g1_speed(
        plan, SimpleNamespace(gripper_speed=5000)
    )
    with pytest.raises(RuntimeError, match="profile requires 5000"):
        runner.validate_hardware_g1_speed(
            plan, SimpleNamespace(gripper_speed=3000)
        )


def test_complete_8deg_reference_executes_on_fake_hardware(monkeypatch):
    spec = importlib.util.spec_from_file_location(
        "open_loop_runner_full", ROOT / "scripts/24_run_cube_open_loop_demo.py"
    )
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
    plan, samples = prepare_deployment(
        ROOT, desired_angle_deg=8, mode="cube"
    )

    class FakeClock:
        now = 0.0

        def monotonic(self):
            return self.now

        def sleep(self, duration):
            self.now += duration

    class FakeRobot:
        def __init__(self):
            self.gripper = []
            self.arm = []
            self.modes = []

        def enter_servo_mode(self):
            self.modes.append("servo")

        def enter_position_mode(self):
            self.modes.append("position")

        def command_gripper_position(self, position):
            self.gripper.append(position)

        def servo_j(self, joint_rad):
            self.arm.append(tuple(joint_rad))

        def reported_joint_signals(self):
            return {
                "joint_position_rad": list(self.arm[-1]),
                "joint_velocity_rad_s": [0.0] * 6,
                "joint_effort": [0.0] * 6,
                "motor_current": [0.0] * 6,
            }

    clock = FakeClock()
    monkeypatch.setattr(runner, "time", clock)
    robot = FakeRobot()
    records, result = runner.execute_reference(
        robot, samples, plan["g1_events"]
    )

    assert result["error"] is None
    assert robot.modes == ["servo", "position"]
    assert robot.gripper == [520.0, 441.0, 370.0]
    assert len(robot.arm) == len(samples) == 92
    assert len(records) == 92
    assert records[-1]["reference_time_s"] == pytest.approx(1.82)
    assert records[-1]["reference_joint_velocity_rad_s"] == list(
        samples[-1]["joint_velocity_rad_s"]
    )
    for joint_index in (0, 3, 5):
        assert all(command[joint_index] == robot.arm[0][joint_index] for command in robot.arm)
