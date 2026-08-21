#!/usr/bin/env python3
"""Run the v47 J5 dynamic regrasp on the real xArm6, or inspect it offline."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
import json
from pathlib import Path
import sys
import time

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from xarm6_toss.real_dynamic_regrasp import (  # noqa: E402
    G1PositionDetachObserver,
    advance_limited_command,
    ballistic_position,
    catch_position_target,
    controller_offsets,
    estimate_arm_tracking_delay,
    load_real_probe_selection,
    release_state_from_arm,
    resample_timeline,
)
from xarm6_toss_sim.kinematics import URDFKinematics  # noqa: E402


REAL_DEMO = (
    ROOT
    / "toss_project_sim_handoff"
    / "toss_project"
    / "real_cube_demo"
)
TIMELINE_PATH = ROOT / "real_handoff" / "j5_forward_rotation_timeline.json"
CONTROLLER_PATH = ROOT / "real_handoff" / "j5_dynamic_regrasp_controller.json"
PROBE_J_PATH = ROOT / "sim" / "configs" / "probe_j_j5_dynamic_regrasp_v2.json"
URDF_PATH = REAL_DEMO / "urdf" / "xarm6_with_gripper_g1.urdf"
NOMINAL_CANDIDATE = "dynamic_5deg_g1_observer"
VECTOR_FIELDS = (
    "command_joint_rad",
    "joint_position_rad",
    "joint_velocity_rad_s",
    "joint_effort",
    "motor_current",
)


def latest_result(pattern: str) -> Path | None:
    matches = sorted(REAL_DEMO.glob(pattern))
    return None if not matches else matches[-1]


def load_json(path: Path) -> dict:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def detach_threshold(path: Path | None) -> tuple[float | None, dict[str, object]]:
    if path is None:
        return None, {"status": "measured_delay_fallback_only"}
    result = load_json(path)
    threshold = result.get("gravity_corrected_release_gripper_position")
    return (
        None if threshold is None else float(threshold),
        {
            "status": (
                "calibrated_g1_position"
                if threshold is not None
                else "calibration_missing_position_trace"
            ),
            "source": str(path),
            "gravity_corrected_release_estimate_s": result.get(
                "gravity_corrected_release_estimate_s"
            ),
            "gravity_corrected_release_gripper_position": threshold,
        },
    )


def select_candidate(
    probe_comparison: Path | None,
    *,
    probe_j_path: Path = PROBE_J_PATH,
    nominal_candidate: str = NOMINAL_CANDIDATE,
) -> tuple[dict, dict[str, object]]:
    config = load_json(probe_j_path)
    if probe_comparison is None:
        selected = next(
            item
            for item in config["catch_candidates"]
            if item["name"] == nominal_candidate
        )
        return selected, {
            "status": "no_real_probe_plan_only",
            "selected_candidate": selected["name"],
            "gate_passed": None,
        }
    selected, evidence = load_real_probe_selection(
        probe_comparison, probe_j_path
    )
    return dict(selected), {"status": "real_paired_probe", **evidence}


def select_control_profile(
    controller_config: dict,
    probe_comparison: Path | None,
) -> tuple[dict, dict[str, object]]:
    """Select Probe/J only for a recatch profile.

    A frozen throw-only checkpoint has no catch decision and must not be
    reported as if Probe/J changed its command.
    """
    if controller_config.get("execution_mode") == "throw_only":
        name = str(controller_config["profile_name"])
        return (
            {
                "name": name,
                "controller": {
                    "catch_servo_start_time_s": None,
                    "catch_preclose_time_s": None,
                    "catch_close_time_s": None,
                    "catch_intercept_time_s": None,
                    "vision_control_end_time_s": None,
                },
            },
            {
                "status": "not_used_for_throw_only",
                "selected_candidate": name,
                "gate_passed": None,
            },
        )
    probe_j_path = Path(
        controller_config.get("probe_j_config", PROBE_J_PATH)
    )
    if not probe_j_path.is_absolute():
        probe_j_path = ROOT / probe_j_path
    nominal_candidate = controller_config.get("probe_j", {}).get(
        "selected_nominal_candidate", NOMINAL_CANDIDATE
    )
    probe_selected, evidence = select_candidate(
        probe_comparison,
        probe_j_path=probe_j_path,
        nominal_candidate=str(nominal_candidate),
    )
    fixed_profile = controller_config.get("fixed_control_profile")
    if fixed_profile is None:
        return probe_selected, evidence
    evidence = {
        **evidence,
        "probe_ranked_candidate": evidence["selected_candidate"],
        "selected_candidate": fixed_profile["name"],
        "selection_mode": "probe_gate_plus_fixed_sim_validated_profile",
    }
    return dict(fixed_profile), evidence


def selected_controller_offsets(
    controller_config: dict,
    selected: dict,
) -> dict[str, float | None]:
    nominal_detach_time_s = controller_config.get(
        "nominal_sim_detach_time_s"
    )
    if nominal_detach_time_s is None:
        return controller_offsets(selected["controller"])
    return controller_offsets(
        selected["controller"],
        nominal_detach_time_s=float(nominal_detach_time_s),
    )


def plan_payload(
    timeline: dict,
    selected: dict,
    probe_evidence: dict,
    controller_config: dict,
    threshold: float | None,
    calibration: dict,
    *,
    speed_scale: float,
) -> dict[str, object]:
    samples = resample_timeline(
        timeline["samples"], speed_scale=speed_scale
    )
    offsets = selected_controller_offsets(controller_config, selected)
    positions = controller_config["g1_real"]
    throw_only = controller_config.get("execution_mode") == "throw_only"
    return {
        "schema": "xarm6_j5_dynamic_regrasp_real_plan_v1",
        "robot_commands_sent": 0,
        "speed_scale": speed_scale,
        "duration_s": samples[-1]["time_s"],
        "control_period_s": 0.02,
        "required_linear_speed_limit_factor": controller_config["robot"][
            "linear_speed_limit_factor"
        ],
        "selected_candidate": selected["name"],
        "probe": probe_evidence,
        "detach_calibration": calibration,
        "detach_position_threshold": threshold,
        "detach_observer_timing_s": {
            "position_poll_start": controller_config["detach_observer"][
                "position_poll_start_after_release_s"
            ],
            "position_poll_period": controller_config["detach_observer"][
                "position_poll_period_s"
            ],
            "uncalibrated_fallback": controller_config["detach_observer"][
                "fallback_delay_s"
            ],
            "calibrated_position_timeout": controller_config["detach_observer"][
                "calibrated_position_timeout_s"
            ],
        },
        "release_command_time_s": positions["release_command_time_s"] / speed_scale,
        "detach_relative_offsets_s": {
            key: None if value is None else value / speed_scale
            for key, value in offsets.items()
        },
        "g1_positions": {
            "held": positions["held_position"],
            "partial_open": positions["partial_open_position"],
            "preclose": (
                None if throw_only else positions["preclose_position_initial_mapping"]
            ),
            "final_close": (
                None if throw_only else positions["final_close_position"]
            ),
        },
        "grasp_offset": {
            "frame": "xarm_gripper_base_link",
            "cube_center_m": controller_config["grasp_offset_gripper_base_link_m"],
        },
        "reference_peak_speed_rad_s": float(
            np.max(
                np.abs(
                    [sample["joint_velocity_rad_s"] for sample in samples]
                )
            )
        ),
    }


def configure_linear_speed_limit(robot, required_factor: float) -> dict[str, float]:
    before = robot.linear_speed_limit_factor()
    if abs(before - required_factor) > 1.0e-6:
        robot.set_linear_speed_limit_factor(required_factor)
    after = robot.linear_speed_limit_factor()
    if abs(after - required_factor) > 1.0e-6:
        raise RuntimeError(
            "linear_spd_limit_factor did not reach "
            f"{required_factor:g}; controller reports {after:g}"
        )
    print(
        "linear_spd_limit_factor: "
        f"before={before:g}, required={required_factor:g}, verified={after:g}"
    )
    return {
        "linear_speed_limit_factor_before": before,
        "linear_speed_limit_factor_required": required_factor,
        "linear_speed_limit_factor_verified": after,
    }


def write_signals(path: Path, records: list[dict]) -> None:
    columns = [
        "time_s",
        "command_time_s",
        "reference_time_s",
        "phase",
        "g1_event",
        "g1_position",
        "g1_read_duration_s",
        "catch_active",
    ]
    for field in VECTOR_FIELDS:
        columns.extend(f"{field}_{joint}" for joint in range(1, 7))
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        for record in records:
            row = {key: record.get(key) for key in columns[:8]}
            for field in VECTOR_FIELDS:
                row.update(
                    {
                        f"{field}_{index + 1}": value
                        for index, value in enumerate(record[field])
                    }
                )
            writer.writerow(row)


def execute_timeline(
    robot,
    samples: list[dict],
    selected: dict,
    controller_config: dict,
    kinematics: URDFKinematics,
    *,
    speed_scale: float,
    operate_g1: bool,
    enable_catch: bool,
) -> tuple[list[dict], dict[str, object]]:
    if enable_catch and speed_scale != 1.0:
        raise ValueError("cube recatch uses the validated 1.0 speed scale")
    g1 = controller_config["g1_real"]
    ballistic = controller_config["ballistic_catch"]
    threshold = controller_config["runtime_detach_position_threshold"]
    detach_observer_config = controller_config["detach_observer"]
    release_command_time_s = float(g1["release_command_time_s"]) / speed_scale
    offsets = {
        key: None if value is None else value / speed_scale
        for key, value in selected_controller_offsets(
            controller_config, selected
        ).items()
    }
    observer = G1PositionDetachObserver(
        command_time_s=release_command_time_s,
        held_position=g1["held_position"],
        open_position=g1["partial_open_position"],
        detach_position_threshold=threshold,
        fallback_delay_s=float(detach_observer_config["fallback_delay_s"]),
        calibrated_position_timeout_s=float(
            detach_observer_config["calibrated_position_timeout_s"]
        ),
    )
    grasp_offset = np.asarray(
        controller_config["grasp_offset_gripper_base_link_m"], dtype=float
    )
    records = []
    g1_events = []
    g1_read_durations = []
    g1_position = float(g1["held_position"])
    g1_read_duration_s = 0.0
    release_sent = not operate_g1
    preclose_sent = not (operate_g1 and enable_catch)
    final_close_sent = not (operate_g1 and enable_catch)
    detach_event = None
    release_state = None
    intercept_position = None
    catch_started = False
    catch_target = None
    catch_start_state = None
    fixed_wrist = None
    commanded_position = np.asarray(samples[0]["joint_position_rad"], dtype=float)
    commanded_velocity = np.asarray(samples[0]["joint_velocity_rad_s"], dtype=float)
    latest_signals = robot.reported_joint_signals()
    first_catch_update = None
    next_g1_poll_s = release_command_time_s + float(
        detach_observer_config["position_poll_start_after_release_s"]
    )
    pending_event_label = ""

    robot.enter_servo_mode()
    execution_error = None
    start_host_s = time.monotonic()
    try:
        for sample in samples:
            target_host_s = start_host_s + sample["time_s"]
            while True:
                elapsed = time.monotonic() - start_host_s
                if not release_sent and elapsed >= release_command_time_s:
                    robot.command_gripper_position(g1["partial_open_position"])
                    release_sent = True
                    pending_event_label = "release_partial_open"
                    g1_events.append(
                        {"name": pending_event_label, "time_s": elapsed}
                    )
                    continue
                if (
                    release_sent
                    and detach_event is None
                    and elapsed >= next_g1_poll_s
                ):
                    read_start = time.monotonic()
                    g1_position = robot.gripper_position(check_baud=False)
                    g1_read_duration_s = time.monotonic() - read_start
                    g1_read_durations.append(g1_read_duration_s)
                    observation_time_s = time.monotonic() - start_host_s
                    next_g1_poll_s = observation_time_s + float(
                        detach_observer_config["position_poll_period_s"]
                    )
                    event = observer.observe(observation_time_s, g1_position)
                    if event is not None:
                        detach_event = event
                        latest_signals = robot.reported_joint_signals()
                        release_state = release_state_from_arm(
                            kinematics,
                            latest_signals["joint_position_rad"],
                            latest_signals["joint_velocity_rad_s"],
                            grasp_offset,
                            time_s=event.time_s,
                        )
                        if enable_catch:
                            intercept_time_s = (
                                event.time_s + float(offsets["intercept"])
                            )
                            intercept_position = ballistic_position(
                                release_state, intercept_time_s
                            )
                        g1_events.append(
                            {"name": "detach_observed", **event.as_dict()}
                        )
                    continue
                if detach_event is not None:
                    preclose_due = (
                        offsets["preclose"] is not None
                        and elapsed >= detach_event.time_s + offsets["preclose"]
                    )
                    if not preclose_sent and preclose_due:
                        robot.command_gripper_position(
                            g1["preclose_position_initial_mapping"]
                        )
                        preclose_sent = True
                        pending_event_label = "dynamic_preclose"
                        g1_events.append(
                            {"name": pending_event_label, "time_s": elapsed}
                        )
                        continue
                    final_due = elapsed >= (
                        detach_event.time_s + float(offsets["final_close"])
                    )
                    if not final_close_sent and final_due:
                        robot.command_gripper_position(g1["final_close_position"])
                        final_close_sent = True
                        pending_event_label = "dynamic_final_close"
                        g1_events.append(
                            {"name": pending_event_label, "time_s": elapsed}
                        )
                        continue
                remaining = target_host_s - time.monotonic()
                if remaining <= 0.0:
                    break
                time.sleep(min(remaining, 0.001))

            elapsed = time.monotonic() - start_host_s
            nominal_position = np.asarray(sample["joint_position_rad"], dtype=float)
            nominal_velocity = np.asarray(sample["joint_velocity_rad_s"], dtype=float)
            catch_active = bool(
                enable_catch
                and detach_event is not None
                and elapsed >= detach_event.time_s + float(offsets["catch_servo"])
            )
            if catch_active and not catch_started:
                fixed_wrist = commanded_position[3:].copy()
                catch_start_state = {
                    "command_seed_source": "preserved_nominal_reference",
                    "command_position_before_update_rad": (
                        commanded_position.tolist()
                    ),
                    "command_velocity_before_update_rad_s": (
                        commanded_velocity.tolist()
                    ),
                    "actual_position_rad": list(
                        latest_signals["joint_position_rad"]
                    ),
                    "actual_velocity_rad_s": list(
                        latest_signals["joint_velocity_rad_s"]
                    ),
                }
                catch_started = True
            if catch_active:
                control_end_s = detach_event.time_s + float(offsets["control_end"])
                if elapsed <= control_end_s or catch_target is None:
                    actual_q = np.asarray(
                        latest_signals["joint_position_rad"], dtype=float
                    )
                    delta, error, jacobian = catch_position_target(
                        kinematics,
                        actual_q,
                        intercept_position,
                        grasp_offset,
                    )
                    catch_target = commanded_position.copy()
                    catch_target[:3] += delta
                    catch_target[3:] = fixed_wrist
                    if first_catch_update is None:
                        first_catch_update = {
                            **catch_start_state,
                            "time_s": elapsed,
                            "position_error_m": error.tolist(),
                            "joint_delta_rad": delta.tolist(),
                            "position_jacobian": jacobian.tolist(),
                        }
                commanded_position, commanded_velocity = advance_limited_command(
                    commanded_position,
                    commanded_velocity,
                    catch_target,
                    control_period_s=0.02,
                    maximum_speed_rad_s=float(
                        ballistic["maximum_joint_speed_rad_s"]
                    ),
                    maximum_acceleration_rad_s2=float(
                        ballistic["maximum_joint_acceleration_rad_s2"]
                    ),
                )
            else:
                commanded_position = nominal_position
                commanded_velocity = nominal_velocity

            command_time_s = time.monotonic() - start_host_s
            robot.servo_j(tuple(float(value) for value in commanded_position))
            latest_signals = robot.reported_joint_signals()
            records.append(
                {
                    "time_s": time.monotonic() - start_host_s,
                    "command_time_s": command_time_s,
                    "reference_time_s": sample["time_s"],
                    "phase": sample["phase"],
                    "g1_event": pending_event_label,
                    "g1_position": g1_position,
                    "g1_read_duration_s": g1_read_duration_s,
                    "catch_active": catch_active,
                    "command_joint_rad": commanded_position.tolist(),
                    **latest_signals,
                }
            )
            pending_event_label = ""
        time.sleep(0.5 if enable_catch else 0.08)
    except Exception as error:
        execution_error = {
            "type": type(error).__name__,
            "message": str(error),
        }
    finally:
        robot.enter_position_mode()

    record_times = np.asarray([record["time_s"] for record in records])
    periods = np.diff(record_times)
    timing = {
        "mean_control_period_s": None if not periods.size else float(np.mean(periods)),
        "maximum_control_period_s": None if not periods.size else float(np.max(periods)),
        "maximum_servo_lateness_s": float(
            np.max(
                record_times
                - [
                    sample["time_s"]
                    for sample in samples[: len(records)]
                ]
            )
        ),
        "g1_read_count": len(g1_read_durations),
        "mean_g1_read_duration_s": (
            None
            if not g1_read_durations
            else float(np.mean(g1_read_durations))
        ),
        "maximum_g1_read_duration_s": (
            None
            if not g1_read_durations
            else float(np.max(g1_read_durations))
        ),
    }
    arm_tracking = estimate_arm_tracking_delay(records)
    return records, {
        "trajectory_start_host_s": start_host_s,
        "error": execution_error,
        "timing": timing,
        "arm_tracking": arm_tracking,
        "g1_events": g1_events,
        "detach_event": None if detach_event is None else detach_event.as_dict(),
        "release_state": None if release_state is None else release_state.as_dict(),
        "intercept_position_base_m": (
            None if intercept_position is None else intercept_position.tolist()
        ),
        "first_catch_update": first_catch_update,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeline", type=Path, default=TIMELINE_PATH)
    parser.add_argument("--controller", type=Path, default=CONTROLLER_PATH)
    parser.add_argument("--probe-comparison", type=Path, default=None)
    parser.add_argument("--detach-result", type=Path, default=None)
    parser.add_argument("--speed-scale", type=float, choices=(0.25, 0.5, 1.0), default=1.0)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--execute-empty-arm", action="store_true")
    modes.add_argument("--execute-empty-g1", action="store_true")
    modes.add_argument("--execute-throw-only", action="store_true")
    modes.add_argument("--execute-cube", action="store_true")
    parser.add_argument("--no-camera-recording", action="store_true")
    args = parser.parse_args()

    if args.probe_comparison is None:
        args.probe_comparison = latest_result(
            "outputs/probe_comparisons/*/summary.json"
        )
    if args.detach_result is None:
        args.detach_result = latest_result(
            "outputs/detach_trials/*/detach_result.json"
        )
    timeline = load_json(args.timeline)
    controller_config = load_json(args.controller)
    selected, probe_evidence = select_control_profile(
        controller_config, args.probe_comparison
    )
    threshold, calibration = detach_threshold(args.detach_result)
    execution_mode = controller_config.get("execution_mode", "dynamic_regrasp")
    payload = plan_payload(
        timeline,
        selected,
        probe_evidence,
        controller_config,
        threshold,
        calibration,
        speed_scale=args.speed_scale,
    )
    print(json.dumps(payload, indent=2))
    execute_requested = any(
        (
            args.execute_empty_arm,
            args.execute_empty_g1,
            args.execute_throw_only,
            args.execute_cube,
        )
    )
    if not execute_requested:
        print("plan only; no robot connection or command was sent")
        return 0
    if execution_mode == "throw_only" and (
        args.execute_empty_g1 or args.execute_cube
    ):
        raise RuntimeError(
            "throw-only controller permits only empty-arm or soft-mat throw-only execution"
        )
    if args.execute_cube and not probe_evidence.get("gate_passed", False):
        raise RuntimeError("real paired Probe gate did not pass")
    if args.execute_cube and threshold is None:
        raise RuntimeError(
            "run 10_measure_detach.py after pulling this version, then pass its detach_result.json"
        )
    if (args.execute_cube or args.execute_throw_only) and args.speed_scale != 1.0:
        raise RuntimeError("cube trials use the validated 1.0 speed scale")

    controller_config["runtime_detach_position_threshold"] = threshold
    samples = resample_timeline(
        timeline["samples"], speed_scale=args.speed_scale
    )
    kinematics = URDFKinematics(URDF_PATH)

    demo_src = REAL_DEMO / "src"
    if str(demo_src) not in sys.path:
        sys.path.insert(0, str(demo_src))
    from real_cube_demo.config import load_hardware  # noqa: E402
    from real_cube_demo.robot import PickPlaceRobot  # noqa: E402

    hardware = load_hardware()
    condition = (
        "cube"
        if args.execute_cube
        else "throw_only"
        if args.execute_throw_only
        else "empty_g1"
        if args.execute_empty_g1
        else "empty_arm"
    )
    operate_g1 = condition != "empty_arm"
    enable_catch = condition in {"cube", "empty_g1"}
    camera_recorder = None
    camera_metadata = None

    with PickPlaceRobot(hardware) as robot:
        robot.prepare_motion()
        robot_setup = configure_linear_speed_limit(
            robot,
            float(controller_config["robot"]["linear_speed_limit_factor"]),
        )
        try:
            robot.move_joints(
                tuple(samples[0]["joint_position_rad"]),
                "J5 dynamic-regrasp start",
            )
            if operate_g1:
                robot.open_gripper()
                if condition in {"cube", "throw_only"}:
                    input(
                        "Place the light cube at the fixed grasp point, remove "
                        "your hand, then press Enter: "
                    )
                robot.set_gripper_position(
                    controller_config["g1_real"]["held_position"]
                )
            input(
                "Workspace clear, soft mat below, and e-stop operator ready; press Enter to run: "
            )
            if condition in {"cube", "throw_only"} and not args.no_camera_recording:
                from real_cube_demo.realsense import MotionCameraRecorder  # noqa: E402

                global_camera = next(
                    camera for camera in hardware.cameras if camera.role == "global"
                )
                camera_recorder = MotionCameraRecorder(global_camera)
                camera_recorder.start()
            try:
                records, execution = execute_timeline(
                    robot,
                    samples,
                    selected,
                    controller_config,
                    kinematics,
                    speed_scale=args.speed_scale,
                    operate_g1=operate_g1,
                    enable_catch=enable_catch,
                )
                if execution["error"] is not None:
                    robot.stop()
            finally:
                if camera_recorder is not None:
                    camera_recorder.stop()
        except Exception:
            robot.stop()
            raise

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = REAL_DEMO / "outputs" / "j5_dynamic_regrasp" / f"{stamp}_{condition}"
    output_dir.mkdir(parents=True, exist_ok=False)
    if camera_recorder is not None:
        camera_metadata = camera_recorder.save(
            output_dir, execution["trajectory_start_host_s"]
        )
    summary = {
        **payload,
        "status": "failed" if execution["error"] is not None else "completed",
        "robot_commands_sent": len(records),
        "condition": condition,
        "robot_setup": robot_setup,
        "execution": execution,
        "global_camera": camera_metadata,
        "recorded_samples": len(records),
    }
    write_signals(output_dir / "signals.csv", records)
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    print(f"saved dynamic-regrasp run: {output_dir}")
    if execution["error"] is not None:
        raise RuntimeError(
            f"timeline failed with {execution['error']['type']}; "
            f"partial log saved at {output_dir}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
