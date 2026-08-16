#!/usr/bin/env python3
"""Execute the J2/J3/J5 forward-upward throw trajectory with an empty gripper."""

import argparse
import csv
from datetime import datetime
import json
from pathlib import Path
import time

import numpy as np

import _bootstrap  # noqa: F401
from real_cube_demo.config import DEMO_ROOT, load_hardware
from real_cube_demo.local_kinematics import LocalXArmFK
from real_cube_demo.robot import PickPlaceRobot
from real_cube_demo.spin_toss import (
    _evaluate,
    _quintic_coefficients,
    pose_matrix,
)


CONFIG_PATH = DEMO_ROOT / "configs" / "natural_j5_candidate.json"
CONTROL_PERIOD_S = 0.02


def load_candidate(path: Path = CONFIG_PATH) -> dict:
    candidate = json.loads(path.read_text(encoding="utf-8"))
    if candidate["schema"] != "real_cube_natural_j5_candidate_v1":
        raise ValueError("unsupported natural J5 candidate configuration")
    return candidate


def build_plan(candidate: dict) -> list[dict]:
    start_q = np.asarray(candidate["start_joint_rad"], dtype=float)
    release_q = np.asarray(candidate["release_joint_rad"], dtype=float)
    release_qd = np.asarray(
        candidate["release_joint_velocity_rad_s"], dtype=float
    )
    throw_duration = float(candidate["throw_duration_s"])
    follow_duration = float(candidate["followthrough_duration_s"])
    hold_duration = float(candidate["final_hold_duration_s"])
    zeros = np.zeros(6)

    throw_coefficients = _quintic_coefficients(
        start_q,
        zeros,
        zeros,
        release_q,
        release_qd,
        zeros,
        throw_duration,
    )
    follow_q = release_q + 0.5 * follow_duration * release_qd
    follow_coefficients = _quintic_coefficients(
        release_q,
        release_qd,
        zeros,
        follow_q,
        zeros,
        zeros,
        follow_duration,
    )

    samples = []
    throw_steps = round(throw_duration / CONTROL_PERIOD_S)
    for step in range(throw_steps + 1):
        time_s = step * CONTROL_PERIOD_S
        q, qd, qdd = _evaluate(throw_coefficients, time_s)
        samples.append(
            {"time_s": time_s, "phase": "throw", "q": q, "qd": qd, "qdd": qdd}
        )

    follow_steps = round(follow_duration / CONTROL_PERIOD_S)
    for step in range(1, follow_steps + 1):
        local_time = step * CONTROL_PERIOD_S
        q, qd, qdd = _evaluate(follow_coefficients, local_time)
        samples.append(
            {
                "time_s": throw_duration + local_time,
                "phase": "followthrough",
                "q": q,
                "qd": qd,
                "qdd": qdd,
            }
        )

    hold_steps = round(hold_duration / CONTROL_PERIOD_S)
    for step in range(1, hold_steps + 1):
        samples.append(
            {
                "time_s": throw_duration + follow_duration + step * CONTROL_PERIOD_S,
                "phase": "hold",
                "q": follow_q.copy(),
                "qd": zeros.copy(),
                "qdd": zeros.copy(),
            }
        )
    return samples


def plan_summary(candidate: dict, samples: list[dict]) -> dict:
    fk = LocalXArmFK()
    joints = np.asarray([sample["q"] for sample in samples])
    velocities = np.asarray([sample["qd"] for sample in samples])
    accelerations = np.asarray([sample["qdd"] for sample in samples])
    tcp = np.asarray(
        [fk.forward_kinematics(tuple(sample["q"]))[:3] for sample in samples]
    )
    transforms = [
        pose_matrix(fk.forward_kinematics(tuple(sample["q"])))
        for sample in samples
    ]
    tool_z = np.asarray([transform[:3, 2] for transform in transforms])
    tool_z_elevation_deg = np.degrees(np.arcsin(tool_z[:, 2]))
    release_index = round(float(candidate["throw_duration_s"]) / CONTROL_PERIOD_S)
    return {
        "active_joints": candidate["active_joints"],
        "fixed_joints": candidate["fixed_joints"],
        "start_joint_rad": joints[0].tolist(),
        "release_joint_rad": joints[release_index].tolist(),
        "followthrough_joint_rad": joints[-1].tolist(),
        "start_tcp_mm": tcp[0].tolist(),
        "release_tcp_mm": tcp[release_index].tolist(),
        "followthrough_tcp_mm": tcp[-1].tolist(),
        "start_tool_z_world": tool_z[0].tolist(),
        "release_tool_z_world": tool_z[release_index].tolist(),
        "followthrough_tool_z_world": tool_z[-1].tolist(),
        "start_tool_z_elevation_deg": float(tool_z_elevation_deg[0]),
        "release_tool_z_elevation_deg": float(
            tool_z_elevation_deg[release_index]
        ),
        "followthrough_tool_z_elevation_deg": float(
            tool_z_elevation_deg[-1]
        ),
        "release_twist": candidate["predicted_release_twist"],
        "max_joint_speed_rad_s": float(np.max(np.abs(velocities))),
        "max_joint_acceleration_rad_s2": float(np.max(np.abs(accelerations))),
        "joint_path_rad": float(np.sum(np.abs(np.diff(joints, axis=0)))),
        "maximum_joint_swing_rad": float(np.max(np.ptp(joints, axis=0))),
        "minimum_tcp_z_mm": float(np.min(tcp[:, 2])),
        "maximum_tcp_z_mm": float(np.max(tcp[:, 2])),
        "total_duration_s": samples[-1]["time_s"],
    }


def print_summary(summary: dict) -> None:
    print(
        f"active joints={summary['active_joints']}; fixed joints={summary['fixed_joints']}"
    )
    print(
        "TCP start -> release -> followthrough: "
        + " -> ".join(
            str([round(value, 1) for value in summary[name]])
            for name in (
                "start_tcp_mm",
                "release_tcp_mm",
                "followthrough_tcp_mm",
            )
        )
        + " mm"
    )
    print(
        "tool-z elevation start -> release -> followthrough: "
        f"{summary['start_tool_z_elevation_deg']:.1f} -> "
        f"{summary['release_tool_z_elevation_deg']:.1f} -> "
        f"{summary['followthrough_tool_z_elevation_deg']:.1f} deg; "
        "positive means the gripper points upward"
    )
    twist = summary["release_twist"]
    print(
        f"release linear={[round(value, 3) for value in twist[:3]]} m/s, "
        f"angular={[round(value, 3) for value in twist[3:]]} rad/s"
    )
    print(
        f"peaks: qdot={summary['max_joint_speed_rad_s']:.3f} rad/s, "
        f"qdd={summary['max_joint_acceleration_rad_s2']:.3f} rad/s^2; "
        f"TCP z={summary['minimum_tcp_z_mm']:.1f}.."
        f"{summary['maximum_tcp_z_mm']:.1f} mm"
    )
    print(
        f"motion: path={summary['joint_path_rad']:.3f} rad, "
        f"max swing={summary['maximum_joint_swing_rad']:.3f} rad, "
        f"duration={summary['total_duration_s']:.2f} s"
    )


def execute(robot, samples: list[dict]) -> list[dict]:
    records = []
    robot.enter_servo_mode()
    start_time = time.monotonic()
    try:
        for sample in samples:
            target_time = start_time + sample["time_s"]
            remaining = target_time - time.monotonic()
            while remaining > 0.0:
                time.sleep(min(remaining, 0.001))
                remaining = target_time - time.monotonic()
            robot.servo_j(tuple(float(value) for value in sample["q"]))
            signals = robot.reported_joint_signals()
            records.append(
                {
                    "time_s": time.monotonic() - start_time,
                    "phase": sample["phase"],
                    "command_joint_rad": sample["q"].tolist(),
                    **signals,
                }
            )
    finally:
        robot.enter_position_mode()
    return records


def save_run(summary: dict, records: list[dict]) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = DEMO_ROOT / "outputs" / "natural_throw" / f"{stamp}_empty"
    output_dir.mkdir(parents=True, exist_ok=False)
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    columns = ["time_s", "phase"]
    vector_fields = (
        "command_joint_rad",
        "joint_position_rad",
        "joint_velocity_rad_s",
        "joint_effort",
        "motor_current",
    )
    for field in vector_fields:
        columns.extend(f"{field}_{joint}" for joint in range(1, 7))
    with (output_dir / "joint_tracking.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        for record in records:
            row = {"time_s": record["time_s"], "phase": record["phase"]}
            for field in vector_fields:
                row.update(
                    {
                        f"{field}_{index + 1}": value
                        for index, value in enumerate(record[field])
                    }
                )
            writer.writerow(row)
    return output_dir


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute-empty", action="store_true")
    args = parser.parse_args()

    candidate = load_candidate()
    samples = build_plan(candidate)
    summary = plan_summary(candidate, samples)
    print_summary(summary)
    if not args.execute_empty:
        print("plan only; no robot connection or motion command was sent")
        return
    if not candidate["empty_throw_execution_ready"]:
        raise RuntimeError("empty throw execution is disabled in the candidate config")

    hardware = load_hardware()
    start_q = np.asarray(candidate["start_joint_rad"], dtype=float)
    with PickPlaceRobot(hardware) as robot:
        current_q = np.asarray(robot.joint_signals()["joint_position_rad"][:6])
        difference = float(np.max(np.abs(current_q - start_q)))
        if difference > 0.08:
            raise RuntimeError(
                "robot is not at the natural throw start; "
                f"max joint difference={difference:.3f} rad"
            )
        input(
            "Empty gripper, workspace clear, and soft mat below; "
            "press Enter to execute the forward-upward throw motion: "
        )
        robot.prepare_motion()
        records = execute(robot, samples)
    output_dir = save_run(summary, records)
    print(f"saved empty throw run: {output_dir}")


if __name__ == "__main__":
    main()
