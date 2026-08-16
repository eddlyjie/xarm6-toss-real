#!/usr/bin/env python3
"""Run the fixed-pose cube pick-and-place sequence; defaults to dry-run."""

import argparse

import _bootstrap  # noqa: F401
from real_cube_demo.config import load_hardware, load_plan
from real_cube_demo.robot import PickPlaceRobot


SEQUENCE = (
    "open gripper",
    "home",
    "pregrasp",
    "grasp",
    "close gripper",
    "lift",
    "preplace",
    "place",
    "open gripper",
    "preplace",
    "home",
)


def print_plan(plan) -> None:
    print("sequence:")
    for index, action in enumerate(SEQUENCE, start=1):
        print(f"  {index:02d}. {action}")
    print("joint poses:")
    for name, pose in plan.poses.items():
        print(f"  {name}: {None if pose is None else [round(v, 4) for v in pose]}")


def execute(robot: PickPlaceRobot, poses) -> None:
    robot.open_gripper()
    robot.move_joints(poses["home"], "home")
    robot.move_joints(poses["pregrasp"], "pregrasp")
    robot.move_joints(poses["grasp"], "grasp")
    robot.close_gripper()
    robot.move_joints(poses["lift"], "lift")
    robot.move_joints(poses["preplace"], "preplace")
    robot.move_joints(poses["place"], "place")
    robot.open_gripper()
    robot.move_joints(poses["preplace"], "preplace")
    robot.move_joints(poses["home"], "home")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    hardware = load_hardware()
    plan = load_plan()
    print_plan(plan)
    if not args.execute:
        print("dry-run only; teach all poses before passing --execute")
        return
    if plan.missing_poses:
        raise RuntimeError(f"record missing poses: {', '.join(plan.missing_poses)}")

    with PickPlaceRobot(hardware) as robot:
        robot.prepare_motion()
        try:
            execute(robot, plan.poses)
        except Exception:
            robot.stop()
            raise


if __name__ == "__main__":
    main()

