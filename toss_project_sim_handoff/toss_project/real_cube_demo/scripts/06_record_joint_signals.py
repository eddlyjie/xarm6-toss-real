#!/usr/bin/env python3
"""Record proprioceptive contact signals without commanding robot motion."""

import argparse
import csv
from datetime import datetime
import time

import _bootstrap  # noqa: F401
from real_cube_demo.config import DEMO_ROOT, load_hardware
from real_cube_demo.robot import PickPlaceRobot


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--rate", type=float, default=25.0)
    args = parser.parse_args()

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = DEMO_ROOT / "outputs" / "joint_signals"
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / f"signals_{stamp}.csv"
    vector_fields = (
        "joint_position_rad",
        "joint_velocity_rad_s",
        "joint_effort",
        "motor_current",
    )
    columns = ["time_s"]
    for field in vector_fields:
        columns.extend(f"{field}_{joint}" for joint in range(1, 7))
    columns.append("gripper_position")

    effort_samples: list[list[float]] = []
    current_samples: list[list[float]] = []
    period = 1.0 / args.rate
    with output_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        with PickPlaceRobot(load_hardware()) as robot:
            start = time.monotonic()
            next_sample = start
            while time.monotonic() - start < args.seconds:
                signals = robot.joint_signals()
                row = {"time_s": time.monotonic() - start}
                for field in vector_fields:
                    row.update(
                        {
                            f"{field}_{joint + 1}": value
                            for joint, value in enumerate(signals[field])
                        }
                    )
                row["gripper_position"] = signals["gripper_position"]
                writer.writerow(row)
                effort_samples.append(signals["joint_effort"])
                current_samples.append(signals["motor_current"])
                next_sample += period
                time.sleep(max(0.0, next_sample - time.monotonic()))

    ranges = [
        max(sample[joint] for sample in effort_samples)
        - min(sample[joint] for sample in effort_samples)
        for joint in range(6)
    ]
    current_ranges = [
        max(sample[joint] for sample in current_samples)
        - min(sample[joint] for sample in current_samples)
        for joint in range(6)
    ]
    print(f"saved {len(effort_samples)} samples: {output_path}")
    print(f"joint effort ranges: {[round(value, 5) for value in ranges]}")
    print(f"motor current ranges: {[round(value, 5) for value in current_ranges]}")


if __name__ == "__main__":
    main()
