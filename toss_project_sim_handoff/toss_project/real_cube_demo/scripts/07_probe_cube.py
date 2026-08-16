#!/usr/bin/env python3
"""Run a small two-axis wrist probe while recording proprioceptive signals."""

import argparse
import csv
from datetime import datetime
import json
import math
import time

import _bootstrap  # noqa: F401
from real_cube_demo.config import DEMO_ROOT, load_hardware, load_probe_plan
from real_cube_demo.robot import PickPlaceRobot


VECTOR_FIELDS = (
    "command_joint_rad",
    "joint_position_rad",
    "joint_velocity_rad_s",
    "joint_effort",
    "motor_current",
)


def target_at(plan, elapsed_s: float) -> tuple[str, tuple[float, ...]]:
    center = list(plan.center_joint_rad)
    if elapsed_s < plan.settle_s:
        return "settle", tuple(center)
    local_time = elapsed_s - plan.settle_s
    for motion in plan.motions:
        if local_time < motion.duration_s:
            envelope = math.sin(math.pi * local_time / motion.duration_s) ** 2
            offset = (
                motion.amplitude_rad
                * envelope
                * math.sin(2.0 * math.pi * motion.frequency_hz * local_time)
            )
            center[motion.joint_index - 1] += offset
            return motion.name, tuple(center)
        local_time -= motion.duration_s
    return "recover", tuple(center)


def summarize(samples: list[dict], center: tuple[float, ...]) -> dict:
    effort_range = []
    current_range = []
    max_tracking_error = []
    max_velocity = []
    for joint in range(6):
        effort = [sample["joint_effort"][joint] for sample in samples]
        current = [sample["motor_current"][joint] for sample in samples]
        tracking = [
            abs(sample["command_joint_rad"][joint] - sample["joint_position_rad"][joint])
            for sample in samples
        ]
        velocity = [abs(sample["joint_velocity_rad_s"][joint]) for sample in samples]
        effort_range.append(max(effort) - min(effort))
        current_range.append(max(current) - min(current))
        max_tracking_error.append(max(tracking))
        max_velocity.append(max(velocity))
    return {
        "samples": len(samples),
        "center_joint_rad": list(center),
        "joint_effort_range": effort_range,
        "motor_current_range": current_range,
        "max_tracking_error_rad": max_tracking_error,
        "max_joint_velocity_rad_s": max_velocity,
    }


def write_csv(path, samples: list[dict]) -> None:
    columns = ["time_s", "phase"]
    for field in VECTOR_FIELDS:
        columns.extend(f"{field}_{joint}" for joint in range(1, 7))
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        for sample in samples:
            row = {"time_s": sample["time_s"], "phase": sample["phase"]}
            for field in VECTOR_FIELDS:
                row.update(
                    {
                        f"{field}_{joint + 1}": value
                        for joint, value in enumerate(sample[field])
                    }
                )
            writer.writerow(row)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--condition", choices=("cube", "empty"), default="cube")
    parser.add_argument("--gripper-position", type=float, default=370.0)
    args = parser.parse_args()
    hardware = load_hardware()
    plan = load_probe_plan()
    print(f"probe center: {[round(value, 4) for value in plan.center_joint_rad]}")
    for motion in plan.motions:
        print(
            f"{motion.name}: joint {motion.joint_index}, "
            f"amplitude={motion.amplitude_rad:.3f} rad, "
            f"frequency={motion.frequency_hz:.2f} Hz, duration={motion.duration_s:.1f} s"
        )
    print(f"total duration: {plan.duration_s:.1f} s at {1 / plan.control_period_s:.0f} Hz")
    if not args.execute:
        print(f"dry-run only; pass --execute to run the {args.condition} probe")
        return

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = DEMO_ROOT / "outputs" / "probes" / f"{stamp}_{args.condition}"
    output_dir.mkdir(parents=True, exist_ok=True)
    samples: list[dict] = []

    with PickPlaceRobot(hardware) as robot:
        robot.prepare_motion()
        try:
            robot.move_joints(plan.center_joint_rad, "probe handoff")
            if args.condition == "cube":
                robot.open_gripper()
                input("Put the cube between the fingers, remove your hand, then press Enter: ")
                robot.close_gripper()
            else:
                robot.set_gripper_position(args.gripper_position)
            gripper_before = robot.gripper_position()

            robot.enter_servo_mode()
            start = time.monotonic()
            next_tick = start
            while time.monotonic() - start < plan.duration_s:
                elapsed = time.monotonic() - start
                phase, command = target_at(plan, elapsed)
                robot.servo_j(command)
                signals = robot.reported_joint_signals()
                signals.update(
                    time_s=elapsed,
                    phase=phase,
                    command_joint_rad=list(command),
                )
                samples.append(signals)
                next_tick += plan.control_period_s
                time.sleep(max(0.0, next_tick - time.monotonic()))

            robot.servo_j(plan.center_joint_rad)
            time.sleep(0.3)
            robot.enter_position_mode()
            gripper_after = robot.gripper_position()

            if args.condition == "cube":
                input("Hold the cube, then press Enter to open the gripper: ")
            robot.open_gripper()
        except Exception:
            robot.stop()
            raise

    summary = summarize(samples, plan.center_joint_rad)
    summary["condition"] = args.condition
    summary["gripper_position_before"] = gripper_before
    summary["gripper_position_after"] = gripper_after
    write_csv(output_dir / "signals.csv", samples)
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    print(f"saved probe: {output_dir}")
    print(
        "joint effort ranges: "
        f"{[round(value, 5) for value in summary['joint_effort_range']]}"
    )
    print(
        "motor current ranges: "
        f"{[round(value, 5) for value in summary['motor_current_range']]}"
    )
    print(
        "max tracking error: "
        f"{[round(value, 5) for value in summary['max_tracking_error_rad']]} rad"
    )


if __name__ == "__main__":
    main()
