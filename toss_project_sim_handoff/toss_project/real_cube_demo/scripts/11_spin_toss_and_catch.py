#!/usr/bin/env python3
"""Plan and execute a probe-conditioned spinning self toss-and-catch."""

import argparse
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import time

import numpy as np

import _bootstrap  # noqa: F401
from real_cube_demo.config import DEMO_ROOT, load_hardware
from real_cube_demo.realsense import MotionCameraRecorder
from real_cube_demo.robot import PickPlaceRobot
from real_cube_demo.spin_toss import build_spin_toss_plan, load_spin_toss_spec


VECTOR_FIELDS = (
    "command_joint_rad",
    "joint_position_rad",
    "joint_velocity_rad_s",
    "joint_effort",
    "motor_current",
)


def latest_probe_belief() -> dict:
    comparisons = sorted((DEMO_ROOT / "outputs" / "probe_comparisons").glob("*/summary.json"))
    if not comparisons:
        return {
            "status": "no_probe_comparison",
            "physical_mean": None,
            "physical_covariance": None,
        }
    source = comparisons[-1]
    features = json.loads(source.read_text(encoding="utf-8"))
    return {
        "status": "untrained_feature_posterior",
        "source": str(source.relative_to(DEMO_ROOT)),
        "physical_mean": None,
        "physical_covariance": None,
        "features": features,
    }


def plan_summary(plan, spec) -> dict:
    return {
        "control_period_s": spec.control_period_s,
        "prethrow_path": spec.prethrow_path,
        "release_time_s": plan.release_time_s,
        "catch_time_s": plan.catch_time_s,
        "physical_release_time_s": plan.physical_release_time_s,
        "physical_catch_time_s": plan.physical_catch_time_s,
        "free_flight_duration_s": plan.catch_time_s - plan.release_time_s,
        "gripper_release_command_time_s": plan.gripper_release_command_time_s,
        "gripper_close_command_time_s": plan.gripper_close_command_time_s,
        "start_joint_rad": list(plan.start_joint_rad),
        "release_joint_rad": list(plan.release_joint_rad),
        "catch_joint_rad": list(plan.catch_joint_rad),
        "release_joint_velocity_rad_s": list(plan.release_joint_velocity_rad_s),
        "release_linear_joint_velocity_rad_s": list(
            plan.release_linear_joint_velocity_rad_s
        ),
        "release_spin_joint_velocity_rad_s": list(
            plan.release_spin_joint_velocity_rad_s
        ),
        "catch_joint_velocity_rad_s": list(plan.catch_joint_velocity_rad_s),
        "start_tcp_pose": list(plan.start_tcp_pose),
        "release_tcp_pose": list(plan.release_tcp_pose),
        "catch_tcp_pose": list(plan.catch_tcp_pose),
        "nominal_release_twist": list(plan.nominal_release_twist),
        "nominal_catch_twist": list(plan.nominal_catch_twist),
        "spin_axis_world": list(plan.spin_axis_world),
        "predicted_object_rotation_rad": plan.predicted_object_rotation_rad,
        "object_relative_displacement_at_catch_m": list(
            plan.object_relative_displacement_at_catch_m
        ),
        "max_joint_speed_rad_s": plan.max_joint_speed_rad_s,
        "max_joint_acceleration_rad_s2": plan.max_joint_acceleration_rad_s2,
        "max_tcp_speed_m_s": plan.max_tcp_speed_m_s,
        "release_tool_z_world": list(plan.release_tool_z_world),
        "gripper_base_below_object_m": plan.gripper_base_below_object_m,
        "held_gripper_position": spec.held_gripper_position,
        "catch_open_position": spec.catch_open_position,
        "detach_delay_s": spec.detach_delay_s,
        "detach_delay_source": spec.detach_delay_source,
        "catch_drop_m": spec.catch_drop_m,
        "capture_absorb_duration_s": spec.capture_absorb_duration_s,
        "capture_mode": spec.capture_mode,
        "spin_axis": spec.spin_axis,
        "physics_source": spec.physics_source,
        "geometry_source": spec.geometry_source,
        "grasp_offset_tool_m": list(spec.grasp_offset_tool_m),
        "grasp_offset_source": spec.grasp_offset_source,
        "probe_belief": latest_probe_belief(),
    }


def print_summary(summary: dict) -> None:
    twist = summary["nominal_release_twist"]
    print(
        "release velocity: "
        f"linear={[round(value, 3) for value in twist[:3]]} m/s, "
        f"angular={[round(value, 3) for value in twist[3:]]} rad/s"
    )
    print(
        f"free flight={summary['free_flight_duration_s']:.3f} s, "
        f"predicted rotation={summary['predicted_object_rotation_rad']:.3f} rad"
    )
    print(
        "EE roll-pitch-yaw start -> release -> catch: "
        + " -> ".join(
            str([round(math.degrees(value), 1) for value in summary[name][3:]])
            for name in ("start_tcp_pose", "release_tcp_pose", "catch_tcp_pose")
        )
        + " deg"
    )
    print(
        f"prethrow path={summary['prethrow_path']}; "
        "TCP xyz start -> release: "
        + " -> ".join(
            str([round(value, 1) for value in summary[name][:3]])
            for name in ("start_tcp_pose", "release_tcp_pose")
        )
        + " mm"
    )
    relative = summary["object_relative_displacement_at_catch_m"]
    print(
        "predicted object-minus-TCP offset at catch: "
        f"{[round(1000.0 * value, 1) for value in relative]} mm"
    )
    tool_z = summary["release_tool_z_world"]
    print(
        "release gripper direction: "
        f"tool-z={[round(value, 3) for value in tool_z]}, "
        f"{math.degrees(math.asin(tool_z[2])):.1f} deg above horizon; "
        f"gripper base {1000.0 * summary['gripper_base_below_object_m']:.0f} mm "
        "below object"
    )
    catch_twist = summary["nominal_catch_twist"]
    print(
        "object velocity at catch: "
        f"linear={[round(value, 3) for value in catch_twist[:3]]} m/s, "
        f"angular={[round(value, 3) for value in catch_twist[3:]]} rad/s"
    )
    print(
        f"trajectory peaks: joint speed={summary['max_joint_speed_rad_s']:.3f} rad/s, "
        f"joint acceleration={summary['max_joint_acceleration_rad_s2']:.3f} rad/s^2, "
        f"TCP speed={summary['max_tcp_speed_m_s']:.3f} m/s"
    )
    print(
        f"G1 events: open command t={summary['gripper_release_command_time_s']:.3f} s, "
        f"close command t={summary['gripper_close_command_time_s']:.3f} s; "
        f"interval={summary['gripper_close_command_time_s'] - summary['gripper_release_command_time_s']:.3f} s; "
        f"physical release/catch={summary['physical_release_time_s']:.3f}/"
        f"{summary['physical_catch_time_s']:.3f} s"
    )
    print(f"probe belief: {summary['probe_belief']['status']}")


def write_signals(path: Path, samples: list[dict]) -> None:
    columns = ["time_s", "phase", "gripper_event"]
    for field in VECTOR_FIELDS:
        columns.extend(f"{field}_{joint}" for joint in range(1, 7))
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        for sample in samples:
            row = {
                "time_s": sample["time_s"],
                "phase": sample["phase"],
                "gripper_event": sample["gripper_event"],
            }
            for field in VECTOR_FIELDS:
                row.update(
                    {
                        f"{field}_{joint + 1}": value
                        for joint, value in enumerate(sample[field])
                    }
                )
            writer.writerow(row)


def tracking_summary(samples: list[dict]) -> dict:
    errors = [
        [
            measured - commanded
            for measured, commanded in zip(
                sample["joint_position_rad"], sample["command_joint_rad"]
            )
        ]
        for sample in samples
    ]
    periods = [
        samples[index]["time_s"] - samples[index - 1]["time_s"]
        for index in range(1, len(samples))
    ]
    times = np.asarray([sample["time_s"] for sample in samples], dtype=float)
    command = np.asarray(
        [sample["command_joint_rad"] for sample in samples], dtype=float
    )
    measured = np.asarray(
        [sample["joint_position_rad"] for sample in samples], dtype=float
    )
    lag_candidates = np.arange(0.0, 0.151, 0.005)
    estimated_lag_s = []
    lag_aligned_rms = []
    for joint in range(6):
        candidate_errors = []
        for lag in lag_candidates:
            eligible = times >= lag
            delayed_command = np.interp(
                times[eligible] - lag, times, command[:, joint]
            )
            rms = float(
                np.sqrt(
                    np.mean((measured[eligible, joint] - delayed_command) ** 2)
                )
            )
            candidate_errors.append((rms, float(lag)))
        rms, lag = min(candidate_errors)
        estimated_lag_s.append(lag)
        lag_aligned_rms.append(rms)

    return {
        "feedback_source": "get_joint_states",
        "mean_sample_period_s": sum(periods) / len(periods),
        "max_abs_error_rad": [
            max(abs(error[joint]) for error in errors) for joint in range(6)
        ],
        "rms_error_rad": [
            math.sqrt(
                sum(error[joint] ** 2 for error in errors) / len(errors)
            )
            for joint in range(6)
        ],
        "estimated_tracking_delay_s": estimated_lag_s,
        "lag_aligned_rms_error_rad": lag_aligned_rms,
    }


def execute_trajectory(robot, plan, spec, *, operate_gripper: bool) -> tuple[list[dict], dict]:
    records = []
    release_sent = not operate_gripper
    close_sent = not operate_gripper
    release_event = ""
    robot.enter_servo_mode()
    start = time.monotonic()
    event_timing = {
        "trajectory_start_host_s": start,
        "planned_release_command_host_s": (
            start + plan.gripper_release_command_time_s
        ),
        "planned_close_command_host_s": start + plan.gripper_close_command_time_s,
        "actual_release_command_host_s": None,
        "actual_close_command_host_s": None,
    }
    try:
        for sample in plan.samples:
            target_time = start + sample.time_s
            while True:
                elapsed = time.monotonic() - start
                if not release_sent and elapsed >= plan.gripper_release_command_time_s:
                    robot.command_gripper_position(spec.catch_open_position)
                    event_timing["actual_release_command_host_s"] = time.monotonic()
                    release_sent = True
                    release_event = "open_for_release"
                    continue
                if not close_sent and elapsed >= plan.gripper_close_command_time_s:
                    robot.command_gripper_position(spec.held_gripper_position)
                    event_timing["actual_close_command_host_s"] = time.monotonic()
                    close_sent = True
                    release_event = "close_for_catch"
                    continue
                remaining = target_time - time.monotonic()
                if remaining <= 0.0:
                    break
                time.sleep(min(remaining, 0.001))

            robot.servo_j(sample.joint_rad)
            signals = robot.reported_joint_signals()
            signals.update(
                time_s=time.monotonic() - start,
                phase=sample.phase,
                gripper_event=release_event,
                command_joint_rad=list(sample.joint_rad),
            )
            release_event = ""
            records.append(signals)
        time.sleep(0.08)
    finally:
        robot.enter_position_mode()
    return records, event_timing


def main() -> None:
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--inspect-release", action="store_true")
    mode.add_argument("--execute-empty", action="store_true")
    mode.add_argument("--execute-empty-gripper", action="store_true")
    mode.add_argument("--execute-cube", action="store_true")
    args = parser.parse_args()

    hardware = load_hardware()
    spec = load_spin_toss_spec()
    global_camera = next(
        camera for camera in hardware.cameras if camera.role == "global"
    )
    with PickPlaceRobot(hardware) as robot:
        inverse_kinematics = lambda pose: robot.inverse_kinematics(
            pose, ref_joint_rad=spec.release_joint_rad
        )
        plan = build_spin_toss_plan(
            spec, robot.forward_kinematics, inverse_kinematics
        )
        summary = plan_summary(plan, spec)
        print_summary(summary)
        if (
            not args.inspect_release
            and not args.execute_empty
            and not args.execute_empty_gripper
            and not args.execute_cube
        ):
            print("plan only; no motion command was sent")
            return

        if not spec.execution_ready:
            raise RuntimeError(
                f"motion disabled for this plan: {spec.execution_block_reason}"
            )

        robot.prepare_motion()
        try:
            if args.inspect_release:
                robot.move_joints(
                    spec.release_joint_rad, "upward release inspection"
                )
                print(
                    "release posture reached; this was setup relocation, not the "
                    "toss path; no toss or gripper command was sent"
                )
                return
            if args.execute_cube:
                robot.move_joints(spec.release_joint_rad, "cube handoff")
                robot.open_gripper()
                input("Hold the cube between the fingers, then press Enter to close: ")
                robot.set_gripper_position(spec.held_gripper_position)
                input("Remove your hand, keep the soft mat below, then press Enter: ")
            elif args.execute_empty_gripper:
                input("Keep the G1 fingers empty, then press Enter to move them to 370: ")
                robot.set_gripper_position(spec.held_gripper_position)

            robot.move_joints(plan.start_joint_rad, "spin-toss start")
            if args.execute_empty or args.execute_empty_gripper:
                input("Empty gripper and workspace clear; press Enter to run the arm trajectory: ")
            temperatures_before = list(robot.arm.temperatures[:6])
            recorder = MotionCameraRecorder(global_camera)
            print(f"recording global camera {global_camera.serial} at 60 Hz")
            recorder.start()
            try:
                records, event_timing = execute_trajectory(
                    robot,
                    plan,
                    spec,
                    operate_gripper=(args.execute_cube or args.execute_empty_gripper),
                )
            finally:
                recorder.stop()
            temperatures_after = list(robot.arm.temperatures[:6])
            final_gripper_position = robot.gripper_position(check_baud=False)

            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            condition = (
                "cube"
                if args.execute_cube
                else "empty_gripper"
                if args.execute_empty_gripper
                else "empty"
            )
            output_dir = DEMO_ROOT / "outputs" / "spin_toss" / f"{stamp}_{condition}"
            output_dir.mkdir(parents=True, exist_ok=True)
            summary.update(
                condition=condition,
                temperatures_before_c=temperatures_before,
                temperatures_after_c=temperatures_after,
                final_gripper_position=final_gripper_position,
                recorded_samples=len(records),
                gripper_event_timing=event_timing,
                tracking=tracking_summary(records),
            )
            write_signals(output_dir / "signals.csv", records)
            camera_metadata = recorder.save(
                output_dir, event_timing["trajectory_start_host_s"]
            )
            summary["global_camera"] = camera_metadata
            (output_dir / "summary.json").write_text(
                json.dumps(summary, indent=2) + "\n", encoding="utf-8"
            )
            print(f"saved spin-toss run: {output_dir}")
            print(f"joint temperatures: {temperatures_before} -> {temperatures_after} C")
            print(
                "direct-feedback max tracking error: "
                f"{[round(value, 4) for value in summary['tracking']['max_abs_error_rad']]} rad"
            )
            print(
                "estimated tracking delay: "
                f"{[round(value, 3) for value in summary['tracking']['estimated_tracking_delay_s']]} s; "
                "lag-aligned RMS: "
                f"{[round(value, 4) for value in summary['tracking']['lag_aligned_rms_error_rad']]} rad"
            )

            if args.execute_cube:
                input("Hold the cube if it was caught, then press Enter to open the gripper: ")
                robot.open_gripper()
            elif args.execute_empty_gripper:
                robot.open_gripper()
        except Exception:
            robot.stop()
            raise


if __name__ == "__main__":
    main()
