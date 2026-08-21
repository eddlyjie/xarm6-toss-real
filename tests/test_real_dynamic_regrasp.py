import importlib.util
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from xarm6_toss.real_dynamic_regrasp import (
    G1PositionDetachObserver,
    advance_limited_command,
    ballistic_position,
    controller_offsets,
    damped_catch_delta,
    estimate_arm_tracking_delay,
    load_real_probe_selection,
    real_probe_posterior,
    release_state_from_arm,
    resample_timeline,
)
from xarm6_toss_sim.kinematics import URDFKinematics


def test_g1_position_observer_uses_calibrated_crossing():
    observer = G1PositionDetachObserver(
        command_time_s=0.585,
        held_position=370,
        open_position=520,
        detach_position_threshold=432,
        fallback_delay_s=0.044,
    )
    assert observer.observe(0.600, 410) is None
    event = observer.observe(0.618, 433)
    assert event is not None
    assert event.source == "calibrated_g1_position"
    assert event.time_s == 0.618


def test_g1_position_observer_has_measured_delay_fallback():
    observer = G1PositionDetachObserver(
        command_time_s=0.585,
        held_position=370,
        open_position=520,
        detach_position_threshold=None,
        fallback_delay_s=0.035,
    )
    event = observer.observe(0.620, 420)
    assert event is not None
    assert event.source == "measured_delay_fallback"


def test_g1_position_observer_waits_through_calibrated_detach_range():
    observer = G1PositionDetachObserver(
        command_time_s=0.585,
        held_position=370,
        open_position=520,
        detach_position_threshold=500,
        fallback_delay_s=0.035,
        calibrated_position_timeout_s=0.044,
    )
    assert observer.observe(0.620, 420) is None
    event = observer.observe(0.630, 430)
    assert event is not None
    assert event.source == "calibrated_position_timeout_fallback"
    assert event.time_s == 0.630


def test_release_state_preserves_actual_arm_sample_and_replays_ballistic_prior():
    class FakeKinematics:
        def forward(self, joint_rad, *, target_link):
            assert target_link == "xarm_gripper_base_link"
            transform = np.eye(4)
            transform[:3, :3] = [
                [0.0, 0.0, 1.0],
                [0.0, 1.0, 0.0],
                [-1.0, 0.0, 0.0],
            ]
            transform[:3, 3] = [0.4, 0.0, 0.3]
            return transform

        def jacobian(self, joint_rad, *, target_link):
            assert target_link == "xarm_gripper_base_link"
            return np.eye(6)

    q = [0.1, -0.2, 0.3, 0.4, -0.5, 0.6]
    qd = [0.2, 0.3, 0.4, 0.0, 2.0, 0.0]
    release = release_state_from_arm(
        FakeKinematics(),
        q,
        qd,
        [0.0, 0.0, 0.1],
        time_s=0.1,
    )

    assert release.joint_position_rad == tuple(q)
    assert release.joint_velocity_rad_s == tuple(qd)
    np.testing.assert_allclose(release.position_base_m, [0.5, 0.0, 0.3])
    np.testing.assert_allclose(release.velocity_base_m_s, [0.2, 0.3, 0.2])
    np.testing.assert_allclose(
        release.hand_angular_velocity_base_rad_s,
        [0.0, 2.0, 0.0],
    )
    np.testing.assert_allclose(release.finger_direction_base, [1.0, 0.0, 0.0])
    np.testing.assert_allclose(
        release.forward_tumble_axis_base,
        [0.0, -1.0, 0.0],
    )
    assert release.angular_velocity_source == "arm_fk_jacobian_rigid_grasp_prior"
    assert release.rigid_grasp_forward_angular_velocity_prior_rad_s == -2.0
    assert release.j5_rigid_grasp_forward_angular_velocity_rad_s == -2.0
    assert release.hand_forward_axis_alignment == 1.0
    np.testing.assert_allclose(
        ballistic_position(release, 0.2),
        [0.52, 0.03, 0.27095],
    )
    assert release.as_dict()["joint_position_rad"] == tuple(q)


def test_v47_actual_detach_state_matches_native_kinematics_and_labels_angular_prior():
    kinematics = URDFKinematics(
        ROOT
        / "toss_project_sim_handoff"
        / "toss_project"
        / "real_cube_demo"
        / "urdf"
        / "xarm6_with_gripper_g1.urdf"
    )
    actual_q = [
        0.060926616191864014,
        -0.02366306260228157,
        -0.4950313866138458,
        2.8794138431549072,
        1.5599160194396973,
        -0.02615748904645443,
    ]
    actual_qd = [
        0.0004886507522314787,
        -0.8923970460891724,
        -0.25304409861564636,
        -0.0026499798987060785,
        0.4636121690273285,
        5.009653847309892e-08,
    ]
    release = release_state_from_arm(
        kinematics,
        actual_q,
        actual_qd,
        [0.004000254, 0.0, 0.13743645],
        time_s=0.615,
    )

    np.testing.assert_allclose(
        release.position_base_m,
        [0.5160556436, 0.0920265093, 0.3338039517],
        atol=5.0e-6,
    )
    np.testing.assert_allclose(
        release.velocity_base_m_s,
        [-0.0269825887, 0.0095730051, 0.6951870918],
        atol=2.0e-3,
    )
    np.testing.assert_allclose(
        release.hand_angular_velocity_base_rad_s,
        [0.1998614967, -1.5839785337, 0.0623716041],
        atol=5.0e-6,
    )
    np.testing.assert_allclose(
        release.forward_tumble_axis_base,
        [0.3511706015, -0.9363114912, 0.0],
        atol=2.0e-6,
    )
    assert abs(
        release.rigid_grasp_forward_angular_velocity_prior_rad_s
        - 1.553283071
    ) < 1.0e-8
    cube_detach_omega = np.asarray([0.0960338712, -0.5319713354, 0.0032464722])
    cube_forward_omega = float(
        np.dot(cube_detach_omega, release.forward_tumble_axis_base)
    )
    assert cube_forward_omega < (
        0.5 * release.rigid_grasp_forward_angular_velocity_prior_rad_s
    )


def test_v47_first_catch_delta_matches_native_controller():
    jacobian = np.asarray(
        [
            [-0.0494543575, 0.00779744424, -0.277522892],
            [0.400730342, 0.000474124099, -0.0169310141],
            [1.20460246e-08, -0.402998, -0.357261121],
        ]
    )
    error = np.asarray([-0.00479221344, 0.00166748464, -0.0417577326])
    delta = damped_catch_delta(jacobian, error)
    np.testing.assert_allclose(
        delta,
        [0.00359585369, 0.035, 0.0146487672],
        atol=1.0e-8,
    )


def test_limited_command_matches_speed_and_acceleration_envelope():
    position, velocity = advance_limited_command(
        np.zeros(3),
        np.zeros(3),
        np.asarray([0.1, -0.1, 0.01]),
        control_period_s=0.02,
        maximum_speed_rad_s=1.74483445,
        maximum_acceleration_rad_s2=13.0573925,
    )
    np.testing.assert_allclose(
        velocity, [0.26114785, -0.26114785, 0.26114785]
    )
    np.testing.assert_allclose(position, velocity * 0.02)


def test_timeline_resampling_preserves_20ms_servo_rate():
    timeline = json.loads(
        (ROOT / "real_handoff" / "j5_forward_rotation_timeline.json").read_text()
    )
    samples = resample_timeline(timeline["samples"], speed_scale=0.25)
    periods = np.diff([sample["time_s"] for sample in samples])
    np.testing.assert_allclose(periods, 0.02, atol=1.0e-12)
    assert abs(samples[-1]["time_s"] - 3.36) < 1.0e-9
    assert np.max(
        np.abs([sample["joint_velocity_rad_s"] for sample in samples])
    ) <= 0.25 * timeline["reference_limit_evidence"]["max_joint_speed_rad_s"] + 1e-9


def test_tracking_delay_estimator_recovers_measured_command_lag():
    times = np.arange(0.0, 1.001, 0.02)
    command = np.zeros((len(times), 6))
    command[:, 1] = np.sin(2.0 * np.pi * times)
    command[:, 2] = 0.4 * np.cos(2.0 * np.pi * 0.7 * times)
    command[:, 4] = 0.25 * np.sin(2.0 * np.pi * 1.3 * times + 0.2)
    true_delay_s = 0.08
    actual = np.column_stack(
        [
            np.interp(times - true_delay_s, times, command[:, joint])
            for joint in range(6)
        ]
    )
    records = [
        {
            "command_time_s": float(time_s),
            "time_s": float(time_s),
            "command_joint_rad": command[index].tolist(),
            "joint_position_rad": actual[index].tolist(),
        }
        for index, time_s in enumerate(times)
    ]

    estimate = estimate_arm_tracking_delay(
        records,
        maximum_delay_s=0.14,
        delay_step_s=0.005,
    )

    assert estimate["status"] == "estimated"
    assert abs(estimate["estimated_delay_s"] - true_delay_s) < 1.0e-12
    assert estimate["active_joint_indices"] == [2, 3, 5]
    assert estimate["fit_rms_rad"] < 1.0e-12
    assert estimate["rms_rad_by_joint"][0] is None


def test_controller_offsets_are_detach_relative():
    config = json.loads(
        (ROOT / "sim" / "configs" / "probe_j_j5_dynamic_regrasp_v2.json").read_text()
    )
    dynamic = config["catch_candidates"][0]["controller"]
    offsets = controller_offsets(dynamic)
    assert abs(offsets["catch_servo"] - 0.005) < 1e-12
    assert abs(offsets["preclose"] - 0.065) < 1e-12
    assert abs(offsets["final_close"] - 0.165) < 1e-12
    assert abs(offsets["intercept"] - 0.185) < 1e-12
    assert abs(offsets["control_end"] - 0.215) < 1e-12


def test_real_paired_probe_changes_posterior_with_signal_quality():
    base = {
        "samples": 80,
        "current_residual_mean": [0, 0.25, 0.15, 0, 0.20, 0],
        "current_residual_dynamic_rms": [0, 0.02, 0.02, 0, 0.02, 0],
        "effort_residual_mean": [0, 0.9, 0.2, 0, 0.2, 0],
        "effort_residual_dynamic_rms": [0.1] * 6,
    }
    held = {
        "gripper_position_before": 370,
        "gripper_position_after": 372,
    }
    calibration = json.loads(
        (ROOT / "sim" / "configs" / "probe_j_j5_dynamic_regrasp_v2.json").read_text()
    )["calibration"]
    strong, strong_features = real_probe_posterior(
        base, held, calibration=calibration
    )
    noisy_payload = dict(base)
    noisy_payload["current_residual_dynamic_rms"] = [0, 0.3, 0.3, 0, 0.3, 0]
    noisy, noisy_features = real_probe_posterior(
        noisy_payload, held, calibration=calibration
    )
    assert 0.02 < strong.effective_payload_mean_kg < 0.05
    assert strong.held_probability > noisy.held_probability
    assert strong.effective_payload_std_kg < noisy.effective_payload_std_kg
    assert (
        strong_features["arm_current_signal_to_noise"]
        > noisy_features["arm_current_signal_to_noise"]
    )


def test_real_probe_loader_resolves_real_demo_relative_output_path(tmp_path):
    demo = tmp_path / "real_cube_demo"
    cube_dir = demo / "outputs" / "probe_trials" / "cube_trial"
    comparison_dir = demo / "outputs" / "probe_comparisons" / "comparison_trial"
    cube_dir.mkdir(parents=True)
    comparison_dir.mkdir(parents=True)
    (cube_dir / "summary.json").write_text(
        json.dumps(
            {
                "gripper_position_before": 370,
                "gripper_position_after": 372,
            }
        )
    )
    comparison_path = comparison_dir / "summary.json"
    comparison_path.write_text(
        json.dumps(
            {
                "cube_probe": "outputs/probe_trials/cube_trial",
                "samples": 80,
                "current_residual_mean": [0, 0.25, 0.15, 0, 0.20, 0],
                "current_residual_dynamic_rms": [0, 0.02, 0.02, 0, 0.02, 0],
                "effort_residual_mean": [0] * 6,
                "effort_residual_dynamic_rms": [0.1] * 6,
            }
        )
    )

    selected, evidence = load_real_probe_selection(
        comparison_path,
        ROOT / "sim" / "configs" / "probe_j_j5_dynamic_regrasp_v2.json",
    )

    assert selected["name"] == "dynamic_5deg_g1_observer"
    assert evidence["gate_passed"] is True


def test_real_runner_executes_detach_relative_sequence_without_hardware():
    spec = importlib.util.spec_from_file_location(
        "xarm6_real_runner", ROOT / "scripts" / "22_run_j5_dynamic_regrasp.py"
    )
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)

    class FakeClock:
        def __init__(self):
            self.now = 0.0

        def monotonic(self):
            return self.now

        def sleep(self, duration_s):
            self.now += max(0.0, duration_s)

    class FakeRobot:
        def __init__(self, initial_q, fail_after_arm_commands=None):
            self.q = np.asarray(initial_q, dtype=float)
            self.gripper = 370.0
            self.gripper_commands = []
            self.arm_commands = []
            self.position_mode_entered = False
            self.gripper_reads = 0
            self.fail_after_arm_commands = fail_after_arm_commands

        def reported_joint_signals(self):
            return {
                "joint_position_rad": self.q.tolist(),
                "joint_velocity_rad_s": [0.0] * 6,
                "joint_effort": [0.0] * 6,
                "motor_current": [0.0] * 6,
            }

        def enter_servo_mode(self):
            return None

        def enter_position_mode(self):
            self.position_mode_entered = True

        def command_gripper_position(self, position):
            self.gripper_commands.append(float(position))
            self.gripper = 440.0 if position == 520 else float(position)

        def gripper_position(self, *, check_baud):
            assert check_baud is False
            self.gripper_reads += 1
            return self.gripper

        def servo_j(self, joint_rad):
            if len(self.arm_commands) == self.fail_after_arm_commands:
                raise RuntimeError(
                    "C60: Linear speed exceeded limit in servo_j mode"
                )
            self.q = np.asarray(joint_rad, dtype=float)
            self.arm_commands.append(self.q.copy())

    class FakeKinematics:
        def forward(self, joint_rad, *, target_link):
            assert target_link == "xarm_gripper_base_link"
            transform = np.eye(4)
            transform[:3, :3] = [
                [0.0, 0.0, 1.0],
                [0.0, 1.0, 0.0],
                [-1.0, 0.0, 0.0],
            ]
            transform[:3, 3] = np.asarray(joint_rad, dtype=float)[:3]
            return transform

        def jacobian(self, joint_rad, *, target_link):
            assert target_link == "xarm_gripper_base_link"
            return np.eye(6)

    class LaggedFakeRobot(FakeRobot):
        actual_tracking_offset_rad = np.asarray(
            [0.01, -0.02, 0.03, 0.01, -0.01, 0.02]
        )

        def reported_joint_signals(self):
            signals = super().reported_joint_signals()
            signals["joint_position_rad"] = (
                np.asarray(signals["joint_position_rad"])
                - self.actual_tracking_offset_rad
            ).tolist()
            return signals

    timeline = runner.load_json(
        ROOT / "real_handoff" / "j5_forward_rotation_timeline.json"
    )
    samples = runner.resample_timeline(timeline["samples"], speed_scale=1.0)
    controller = runner.load_json(
        ROOT / "real_handoff" / "j5_dynamic_regrasp_controller.json"
    )
    controller["runtime_detach_position_threshold"] = 432.0
    probe_j = runner.load_json(
        ROOT / "sim" / "configs" / "probe_j_j5_dynamic_regrasp_v2.json"
    )
    selected = probe_j["catch_candidates"][0]
    robot = LaggedFakeRobot(samples[0]["joint_position_rad"])
    clock = FakeClock()
    runner.time = clock

    records, execution = runner.execute_timeline(
        robot,
        samples,
        selected,
        controller,
        FakeKinematics(),
        speed_scale=1.0,
        operate_g1=True,
        enable_catch=True,
    )

    assert robot.gripper_commands == [520.0, 441.0, 370.0]
    assert len(robot.arm_commands) == len(samples) == len(records)
    assert execution["error"] is None
    assert robot.position_mode_entered is True
    assert execution["detach_event"]["source"] == "calibrated_g1_position"
    assert execution["detach_event"]["time_s"] >= (
        controller["g1_real"]["release_command_time_s"]
        + controller["detach_observer"]["position_poll_start_after_release_s"]
    )
    assert execution["timing"]["g1_read_count"] == robot.gripper_reads == 1
    assert execution["release_state"] is not None
    assert execution["first_catch_update"] is not None
    assert any(record["catch_active"] for record in records)
    catch_start = execution["first_catch_update"]
    assert catch_start["command_seed_source"] == "preserved_nominal_reference"
    np.testing.assert_allclose(
        np.asarray(catch_start["command_position_before_update_rad"])
        - np.asarray(catch_start["actual_position_rad"]),
        robot.actual_tracking_offset_rad,
    )
    assert np.max(
        np.abs(catch_start["command_velocity_before_update_rad_s"])
    ) > 0.0
    assert execution["arm_tracking"]["status"] == "estimated"
    assert execution["timing"]["maximum_control_period_s"] <= 0.0200001

    failed_robot = FakeRobot(
        samples[0]["joint_position_rad"], fail_after_arm_commands=34
    )
    runner.time = FakeClock()
    partial_records, failed_execution = runner.execute_timeline(
        failed_robot,
        samples,
        selected,
        controller,
        FakeKinematics(),
        speed_scale=1.0,
        operate_g1=True,
        enable_catch=True,
    )

    assert len(partial_records) == 34
    assert failed_execution["error"] == {
        "type": "RuntimeError",
        "message": "C60: Linear speed exceeded limit in servo_j mode",
    }
    assert failed_robot.position_mode_entered is True


def test_plan_payload_uses_the_supplied_controller_config():
    spec = importlib.util.spec_from_file_location(
        "xarm6_real_runner_plan", ROOT / "scripts" / "22_run_j5_dynamic_regrasp.py"
    )
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
    timeline = runner.load_json(
        ROOT / "real_handoff" / "j5_forward_rotation_timeline.json"
    )
    probe_j = runner.load_json(
        ROOT / "sim" / "configs" / "probe_j_j5_dynamic_regrasp_v2.json"
    )
    controller = runner.load_json(
        ROOT / "real_handoff" / "j5_dynamic_regrasp_controller.json"
    )
    controller["grasp_offset_gripper_base_link_m"] = [0.01, 0.02, 0.03]

    payload = runner.plan_payload(
        timeline,
        probe_j["catch_candidates"][0],
        {"status": "test"},
        controller,
        None,
        {"status": "test"},
        speed_scale=0.25,
    )

    assert payload["grasp_offset"]["cube_center_m"] == [0.01, 0.02, 0.03]
    assert payload["detach_observer_timing_s"] == {
        "position_poll_start": 0.020,
        "position_poll_period": 0.005,
        "uncalibrated_fallback": 0.035,
        "calibrated_position_timeout": 0.044,
    }


def test_runner_sets_and_verifies_proven_linear_speed_factor():
    spec = importlib.util.spec_from_file_location(
        "xarm6_real_runner_factor", ROOT / "scripts" / "22_run_j5_dynamic_regrasp.py"
    )
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)

    class FakeRobot:
        def __init__(self):
            self.factor = 1.2
            self.set_calls = []

        def linear_speed_limit_factor(self):
            return self.factor

        def set_linear_speed_limit_factor(self, factor):
            self.factor = factor
            self.set_calls.append(factor)

    robot = FakeRobot()
    setup = runner.configure_linear_speed_limit(robot, 1.6)

    assert robot.set_calls == [1.6]
    assert setup == {
        "linear_speed_limit_factor_before": 1.2,
        "linear_speed_limit_factor_required": 1.6,
        "linear_speed_limit_factor_verified": 1.6,
    }


def test_standard_g1_throwonly_profile_is_separate_from_probe_j_recatch():
    spec = importlib.util.spec_from_file_location(
        "xarm6_real_runner_throwonly",
        ROOT / "scripts" / "22_run_j5_dynamic_regrasp.py",
    )
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
    controller = runner.load_json(
        ROOT
        / "real_handoff"
        / "standard_g1_throwonly_11p5deg_controller.json"
    )
    selected, evidence = runner.select_control_profile(controller, None)

    assert selected["name"] == "standard_g1_throwonly_11p5deg"
    assert all(value is None for value in selected["controller"].values())
    assert evidence["status"] == "not_used_for_throw_only"
    assert controller["sim_evidence"]["release_mechanism"] == (
        "standard_g1_no_insert"
    )
    assert controller["real_trial"]["recatch_enabled"] is False


def test_standard_g1_throwonly_timeline_stays_in_real_command_envelope():
    timeline = json.loads(
        (
            ROOT
            / "real_handoff"
            / "standard_g1_throwonly_11p5deg_timeline.json"
        ).read_text()
    )
    controller = json.loads(
        (
            ROOT
            / "real_handoff"
            / "standard_g1_throwonly_11p5deg_controller.json"
        ).read_text()
    )
    assert timeline["execution_mode"] == "throw_only"
    assert timeline["reference_limit_evidence"]["joint_mechanical_limits_pass"]
    assert timeline["reference_limit_evidence"]["max_joint_speed_rad_s"] <= 1.74483445
    assert controller["g1_real"]["release_command_time_s"] == 0.62


def test_stock_g1_10deg_profile_is_executable_regrasp_not_throwonly():
    spec = importlib.util.spec_from_file_location(
        "xarm6_real_runner_10deg",
        ROOT / "scripts" / "22_run_j5_dynamic_regrasp.py",
    )
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
    timeline = runner.load_json(
        ROOT / "real_handoff" / "stock_g1_10deg_regrasp_timeline.json"
    )
    controller = runner.load_json(
        ROOT / "real_handoff" / "stock_g1_10deg_regrasp_controller.json"
    )
    selected, evidence = runner.select_control_profile(controller, None)
    offsets = runner.selected_controller_offsets(controller, selected)

    assert controller.get("execution_mode", "dynamic_regrasp") == (
        "dynamic_regrasp"
    )
    assert selected["name"] == "stock_g1_10deg_stable"
    assert evidence["selection_mode"] == (
        "probe_gate_plus_fixed_sim_validated_profile"
    )
    assert abs(offsets["catch_servo"] - 0.020) < 1.0e-12
    assert abs(offsets["preclose"] - 0.180) < 1.0e-12
    assert abs(offsets["final_close"] - 0.260) < 1.0e-12
    assert abs(offsets["intercept"] - 0.340) < 1.0e-12
    assert abs(offsets["control_end"] - 0.380) < 1.0e-12
    assert timeline["samples"][-1]["time_s"] >= 1.40
    assert timeline["reference_limit_evidence"]["joint_mechanical_limits_pass"]
    assert controller["g1_real"]["release_command_time_s"] == 0.62
    assert controller["sim_evidence"]["catch_stable"] is True
    assert abs(
        controller["sim_evidence"]["signed_forward_rotation_deg"]
        - 9.840416328251768
    ) < 1.0e-12
