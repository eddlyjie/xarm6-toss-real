#!/usr/bin/env python3
"""Subtract an empty-gripper Probe from a cube Probe."""

import argparse
import csv
from datetime import datetime
import json
from pathlib import Path

import _bootstrap  # noqa: F401
from real_cube_demo.config import DEMO_ROOT


def read_signals(directory: Path) -> list[dict[str, str]]:
    with (directory / "signals.csv").open(encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cube", type=Path, required=True)
    parser.add_argument("--empty", type=Path, required=True)
    args = parser.parse_args()
    cube_rows = read_signals(args.cube)
    empty_rows = read_signals(args.empty)
    sample_count = min(len(cube_rows), len(empty_rows))

    columns = ["time_s", "phase"]
    for field in ("joint_effort_residual", "motor_current_residual"):
        columns.extend(f"{field}_{joint}" for joint in range(1, 7))

    residual_rows = []
    effort_values = [[] for _ in range(6)]
    current_values = [[] for _ in range(6)]
    position_difference = [[] for _ in range(6)]
    velocity_difference = [[] for _ in range(6)]
    for cube, empty in zip(cube_rows[:sample_count], empty_rows[:sample_count]):
        row = {"time_s": cube["time_s"], "phase": cube["phase"]}
        for joint in range(1, 7):
            effort = float(cube[f"joint_effort_{joint}"]) - float(
                empty[f"joint_effort_{joint}"]
            )
            current = float(cube[f"motor_current_{joint}"]) - float(
                empty[f"motor_current_{joint}"]
            )
            row[f"joint_effort_residual_{joint}"] = effort
            row[f"motor_current_residual_{joint}"] = current
            effort_values[joint - 1].append(effort)
            current_values[joint - 1].append(current)
            position_difference[joint - 1].append(
                float(cube[f"joint_position_rad_{joint}"])
                - float(empty[f"joint_position_rad_{joint}"])
            )
            velocity_difference[joint - 1].append(
                float(cube[f"joint_velocity_rad_s_{joint}"])
                - float(empty[f"joint_velocity_rad_s_{joint}"])
            )
        residual_rows.append(row)

    def rms(values: list[float]) -> float:
        return (sum(value * value for value in values) / len(values)) ** 0.5

    def mean(values: list[float]) -> float:
        return sum(values) / len(values)

    def dynamic_rms(values: list[float]) -> float:
        center = mean(values)
        return rms([value - center for value in values])

    summary = {
        "samples": sample_count,
        "cube_probe": str(args.cube),
        "empty_probe": str(args.empty),
        "position_difference_rms_rad": [
            rms(values) for values in position_difference
        ],
        "velocity_difference_rms_rad_s": [
            rms(values) for values in velocity_difference
        ],
        "effort_residual_mean": [mean(values) for values in effort_values],
        "effort_residual_dynamic_rms": [
            dynamic_rms(values) for values in effort_values
        ],
        "effort_residual_rms": [rms(values) for values in effort_values],
        "effort_residual_peak_abs": [
            max(abs(value) for value in values) for values in effort_values
        ],
        "current_residual_mean": [mean(values) for values in current_values],
        "current_residual_dynamic_rms": [
            dynamic_rms(values) for values in current_values
        ],
        "current_residual_rms": [rms(values) for values in current_values],
        "current_residual_peak_abs": [
            max(abs(value) for value in values) for values in current_values
        ],
    }

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = DEMO_ROOT / "outputs" / "probe_comparisons" / stamp
    output_dir.mkdir(parents=True, exist_ok=True)
    with (output_dir / "residual_signals.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(residual_rows)
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    print(f"saved comparison: {output_dir}")
    print(
        "effort residual mean: "
        f"{[round(value, 5) for value in summary['effort_residual_mean']]}"
    )
    print(
        "effort residual dynamic RMS: "
        f"{[round(value, 5) for value in summary['effort_residual_dynamic_rms']]}"
    )
    print(
        "current residual mean: "
        f"{[round(value, 5) for value in summary['current_residual_mean']]}"
    )
    print(
        "current residual dynamic RMS: "
        f"{[round(value, 5) for value in summary['current_residual_dynamic_rms']]}"
    )


if __name__ == "__main__":
    main()
