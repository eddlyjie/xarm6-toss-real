#!/usr/bin/env python3
"""Plan or execute an offline-selected open-loop object toss profile on xArm6.

The default is plan-only. Robot motion requires an explicit --execute-* flag.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
import json
from pathlib import Path
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from xarm6_toss.open_loop_demo import prepare_deployment  # noqa: E402


REAL_DEMO = ROOT / "toss_project_sim_handoff" / "toss_project" / "real_cube_demo"
DEFAULT_HARDWARE_CONFIG = REAL_DEMO / "configs" / "hardware.json"


def execution_mode(args: argparse.Namespace) -> str:
    if args.execute_empty_arm:
        return "empty_arm"
    if args.execute_empty_g1:
        return "empty_g1"
    if args.execute_throw_only:
        return "throw_only"
    if args.execute_cube:
        return "cube"
    if args.execute_object:
        return "object"
    return "plan"


def configure_linear_speed_limit(robot, required_factor: float = 1.6) -> dict:
    before = float(robot.linear_speed_limit_factor())
    if abs(before - required_factor) > 1.0e-6:
        robot.set_linear_speed_limit_factor(required_factor)
    after = float(robot.linear_speed_limit_factor())
    if abs(after - required_factor) > 1.0e-6:
        raise RuntimeError(
            f"linear speed limit factor is {after:g}, expected {required_factor:g}"
        )
    return {"before": before, "required": required_factor, "verified": after}


def validate_hardware_g1_speed(plan: dict, hardware) -> None:
    expected = float(plan["g1_speed"])
    actual = float(hardware.gripper_speed)
    if abs(actual - expected) > 1.0e-9:
        raise RuntimeError(
            f"hardware G1 speed is {actual:g}, profile requires {expected:g}"
        )


def _sample_g1_position(robot, label: str, time_s: float) -> dict:
    try:
        return {
            "label": label,
            "time_s": float(time_s),
            "position": float(robot.gripper_position(check_baud=False)),
            "error": None,
        }
    except Exception as exc:
        return {
            "label": label,
            "time_s": float(time_s),
            "position": None,
            "error": {"type": type(exc).__name__, "message": str(exc)},
        }


def _sample_controller_status(robot, label: str, time_s: float) -> dict:
    try:
        return {
            "label": label,
            "time_s": float(time_s),
            "status": robot.controller_status(),
            "error": None,
        }
    except Exception as exc:
        return {
            "label": label,
            "time_s": float(time_s),
            "status": None,
            "error": {"type": type(exc).__name__, "message": str(exc)},
        }


def execute_reference(
    robot,
    samples: list[dict],
    events: list[dict],
    *,
    observe_g1: bool = False,
    observe_controller: bool = False,
) -> tuple[list[dict], dict]:
    records: list[dict] = []
    event_records: list[dict] = []
    g1_position_samples: list[dict] = []
    controller_status_samples: list[dict] = []
    next_event = 0
    if observe_g1:
        g1_position_samples.append(_sample_g1_position(robot, "before_servo", 0.0))
    if observe_controller:
        controller_status_samples.append(
            _sample_controller_status(robot, "before_servo", 0.0)
        )
    robot.enter_servo_mode()
    start = time.monotonic()
    error = None
    try:
        for sample in samples:
            target = start + float(sample["time_s"])
            while True:
                elapsed = time.monotonic() - start
                while next_event < len(events) and elapsed >= float(events[next_event]["time_s"]):
                    event = events[next_event]
                    robot.command_gripper_position(float(event["position"]))
                    event_records.append(
                        {
                            "name": event["name"],
                            "scheduled_time_s": float(event["time_s"]),
                            "command_time_s": time.monotonic() - start,
                            "position": float(event["position"]),
                        }
                    )
                    next_event += 1
                    elapsed = time.monotonic() - start
                remaining = target - time.monotonic()
                if remaining <= 0.0:
                    break
                time.sleep(min(remaining, 0.001))

            command_time = time.monotonic() - start
            command = tuple(float(value) for value in sample["joint_position_rad"])
            reference_velocity = [
                float(value) for value in sample["joint_velocity_rad_s"]
            ]
            robot.servo_j(command)
            actual = robot.reported_joint_signals()
            records.append(
                {
                    "reference_time_s": float(sample["time_s"]),
                    "command_time_s": command_time,
                    "phase": sample.get("phase", "reference"),
                    "command_joint_rad": list(command),
                    "reference_joint_velocity_rad_s": reference_velocity,
                    **actual,
                }
            )
        time.sleep(0.5)
    except Exception as exc:
        error = {"type": type(exc).__name__, "message": str(exc)}
    finally:
        robot.enter_position_mode()
    if observe_g1:
        g1_position_samples.append(
            _sample_g1_position(robot, "after_servo", time.monotonic() - start)
        )
    if observe_controller:
        controller_status_samples.append(
            _sample_controller_status(robot, "after_servo", time.monotonic() - start)
        )
    return records, {
        "error": error,
        "g1_events": event_records,
        "g1_position_samples": g1_position_samples,
        "controller_status_samples": controller_status_samples,
        "duration_s": time.monotonic() - start,
    }


def write_signals(path: Path, records: list[dict]) -> None:
    vector_fields = (
        "command_joint_rad",
        "reference_joint_velocity_rad_s",
        "joint_position_rad",
        "joint_velocity_rad_s",
        "joint_effort",
        "motor_current",
    )
    columns = ["reference_time_s", "command_time_s", "phase"]
    for field in vector_fields:
        columns.extend(f"{field}_{index}" for index in range(1, 7))
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        for record in records:
            row = {key: record.get(key) for key in columns[:3]}
            for field in vector_fields:
                values = record.get(field, [None] * 6)
                row.update(
                    {f"{field}_{index + 1}": value for index, value in enumerate(values)}
                )
            writer.writerow(row)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--angle-deg", type=float, default=None)
    parser.add_argument("--profile", type=Path, default=None)
    parser.add_argument("--output-plan", type=Path, default=None)
    parser.add_argument("--hardware-config", type=Path, default=DEFAULT_HARDWARE_CONFIG)
    parser.add_argument("--speed-scale", type=float, choices=(0.25, 0.5, 1.0), default=1.0)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--execute-empty-arm", action="store_true")
    modes.add_argument("--execute-empty-g1", action="store_true")
    modes.add_argument("--execute-throw-only", action="store_true")
    modes.add_argument("--execute-cube", action="store_true")
    modes.add_argument("--execute-object", action="store_true")
    args = parser.parse_args()

    mode = execution_mode(args)
    desired_angle_deg = args.angle_deg
    if desired_angle_deg is None and args.profile is None:
        desired_angle_deg = 0.0
    plan, samples = prepare_deployment(
        ROOT,
        desired_angle_deg=desired_angle_deg,
        profile_path=args.profile,
        speed_scale=args.speed_scale,
        mode=mode,
    )
    print(json.dumps(plan, indent=2))
    if args.output_plan is not None:
        args.output_plan.parent.mkdir(parents=True, exist_ok=True)
        args.output_plan.write_text(
            json.dumps(plan, indent=2) + "\n", encoding="utf-8"
        )
        print(f"saved deployment plan: {args.output_plan}")
    if mode == "plan":
        print("plan only; no robot connection or command was sent")
        return 0

    demo_src = REAL_DEMO / "src"
    if str(demo_src) not in sys.path:
        sys.path.insert(0, str(demo_src))
    from real_cube_demo.config import load_hardware  # noqa: E402
    from real_cube_demo.robot import PickPlaceRobot  # noqa: E402

    hardware = load_hardware(args.hardware_config.resolve())
    validate_hardware_g1_speed(plan, hardware)
    operate_g1 = mode != "empty_arm"
    with PickPlaceRobot(hardware) as robot:
        robot.prepare_motion()
        setup = configure_linear_speed_limit(robot)
        try:
            robot.move_joints(
                tuple(samples[0]["joint_position_rad"]),
                f"open-loop {plan['desired_angle_deg']:g} degree start",
            )
            if operate_g1:
                robot.open_gripper()
                if mode in {"cube", "object", "throw_only"}:
                    dimensions_mm = [
                        round(float(value) * 1000.0, 1)
                        for value in plan["object_dimensions_m"]
                    ]
                    input(
                        f"Place {plan['object_id']} ({dimensions_mm} mm, "
                        f"{plan['object_mass_kg'] * 1000.0:g} g) at its marked grasp depth, "
                        "remove your hand, then press Enter: "
                    )
                robot.set_gripper_position(float(plan["g1_held_position"]))
            input("Workspace clear, soft mat placed, e-stop operator ready; press Enter to run: ")
            records, execution = execute_reference(
                robot,
                samples,
                plan["g1_events"],
                observe_g1=operate_g1,
                observe_controller=True,
            )
            if execution["error"] is not None:
                robot.stop()
        except Exception:
            robot.stop()
            raise

    output_root = REAL_DEMO / "outputs" / "open_loop_object_demo"
    condition = mode
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = output_root / f"{stamp}_{plan['profile_id']}_{condition}"
    output_dir.mkdir(parents=True, exist_ok=False)
    write_signals(output_dir / "signals.csv", records)
    summary = {"plan": plan, "robot_setup": setup, "execution": execution}
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8"
    )
    print(f"saved open-loop run: {output_dir}")
    if execution["error"] is not None:
        raise RuntimeError(f"open-loop execution failed: {execution['error']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
