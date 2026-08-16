#!/usr/bin/env python3
"""Slow, static inspection of the natural J5 throw start and release poses."""

import argparse
from datetime import datetime
import json
import math
from pathlib import Path

import numpy as np

import _bootstrap  # noqa: F401
from real_cube_demo.config import DEMO_ROOT, load_hardware
from real_cube_demo.local_kinematics import LocalXArmFK
from real_cube_demo.realsense import capture_camera
from real_cube_demo.robot import PickPlaceRobot
from real_cube_demo.spin_toss import pose_matrix


CONFIG_PATH = DEMO_ROOT / "configs" / "natural_j5_candidate.json"


def load_candidate(path: Path = CONFIG_PATH) -> dict:
    candidate = json.loads(path.read_text(encoding="utf-8"))
    if candidate["schema"] != "real_cube_natural_j5_candidate_v1":
        raise ValueError("unsupported natural J5 candidate configuration")
    return candidate


def joint_tuple(candidate: dict, name: str) -> tuple[float, ...]:
    return tuple(float(value) for value in candidate[name])


def print_pose(
    robot,
    local_fk: LocalXArmFK,
    name: str,
    joint_rad: tuple[float, ...],
) -> None:
    tcp = robot.forward_kinematics(joint_rad)
    transform = pose_matrix(local_fk.forward_kinematics(joint_rad))
    tool_z = transform[:3, 2]
    elevation_deg = math.degrees(math.asin(float(tool_z[2])))
    print(f"{name} q: {[round(value, 4) for value in joint_rad]}")
    print(
        f"{name} TCP: {[round(value, 1) for value in tcp[:3]]} mm, "
        f"RPY={[round(float(np.degrees(value)), 1) for value in tcp[3:]]} deg"
    )
    print(
        f"{name} tool-z: {[round(float(value), 3) for value in tool_z]}, "
        f"elevation={elevation_deg:.1f} deg "
        "(positive means the gripper points upward)"
    )


def capture_pose_cameras(hardware, pose_name: str) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = (
        DEMO_ROOT / "outputs" / "natural_pose_inspections" / f"{stamp}_{pose_name}"
    )
    for camera in hardware.cameras:
        print(f"capturing {camera.role} camera {camera.serial}")
        capture_camera(camera, output_dir)
    print(f"saved pose inspection: {output_dir}")
    return output_dir


def main() -> None:
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--execute-start", action="store_true")
    mode.add_argument("--execute-revised-start", action="store_true")
    mode.add_argument(
        "--execute-start-from-last-followthrough",
        action="store_true",
    )
    mode.add_argument("--execute-release", action="store_true")
    mode.add_argument("--capture-cube-at-start", action="store_true")
    parser.add_argument("--capture-cameras", action="store_true")
    args = parser.parse_args()

    candidate = load_candidate()
    hardware = load_hardware()
    handoff_q = joint_tuple(candidate, "handoff_joint_rad")
    previous_start_q = joint_tuple(candidate, "previous_start_joint_rad")
    start_q = joint_tuple(candidate, "start_joint_rad")
    release_q = joint_tuple(candidate, "release_joint_rad")
    local_fk = LocalXArmFK()

    with PickPlaceRobot(hardware) as robot:
        print_pose(robot, local_fk, "handoff", handoff_q)
        print_pose(robot, local_fk, "start", start_q)
        print_pose(robot, local_fk, "release", release_q)
        if (
            not args.execute_start
            and not args.execute_revised_start
            and not args.execute_start_from_last_followthrough
            and not args.execute_release
            and not args.capture_cube_at_start
        ):
            print("plan only; no motion or gripper command was sent")
            return

        current_q = np.asarray(robot.joint_signals()["joint_position_rad"][:6])
        if args.execute_start:
            required_q = np.asarray(handoff_q)
            required_name = "handoff"
        elif (
            args.execute_revised_start
            or args.execute_start_from_last_followthrough
        ):
            required_q = np.asarray(previous_start_q)
            required_name = "last empty-throw followthrough"
        else:
            required_q = np.asarray(start_q)
            required_name = "start"
        distance = float(np.max(np.abs(current_q - required_q)))
        if distance > 0.08:
            raise RuntimeError(
                f"robot is not at the required {required_name} pose; "
                f"max joint difference={distance:.3f} rad"
            )

        if args.capture_cube_at_start:
            robot.prepare_motion()
            robot.open_gripper()
            input(
                "Put the cube between the fingers, keep a soft mat below, "
                "then press Enter to close: "
            )
            robot.set_gripper_position(
                float(candidate["held_gripper_position"])
            )
            input("Remove your hand, then press Enter to capture both cameras: ")
            capture_pose_cameras(hardware, "start_with_cube")
            input("Hold the cube, then press Enter to open the gripper: ")
            robot.open_gripper()
            print("cube visibility capture complete; the arm did not move")
            return

        target_name = (
            "start"
            if (
                args.execute_start
                or args.execute_revised_start
                or args.execute_start_from_last_followthrough
            )
            else "release"
        )
        target_q = start_q if target_name == "start" else release_q
        input(
            f"Workspace clear; press Enter to move slowly from {required_name} "
            f"to {target_name}: "
        )
        robot.prepare_motion()
        robot.move_joints(
            target_q,
            f"natural throw {target_name} inspection",
            speed_rad_s=float(candidate["inspection_speed_rad_s"]),
            acceleration_rad_s2=float(
                candidate["inspection_acceleration_rad_s2"]
            ),
        )
        if args.capture_cameras:
            capture_pose_cameras(hardware, target_name)
        print(
            f"{target_name} inspection complete; no throw or gripper command was sent"
        )


if __name__ == "__main__":
    main()
