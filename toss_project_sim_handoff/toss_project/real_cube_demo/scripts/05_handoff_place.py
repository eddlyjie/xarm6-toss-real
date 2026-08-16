#!/usr/bin/env python3
"""Take a hand-delivered cube and place it at one fixed table location."""

import argparse

import _bootstrap  # noqa: F401
from real_cube_demo.config import load_handoff_plan, load_hardware
from real_cube_demo.robot import PickPlaceRobot


def print_plan(plan) -> None:
    print(f"handoff joint pose: {[round(value, 4) for value in plan.handoff_joint_rad]}")
    print(f"preplace joint pose: {[round(value, 4) for value in plan.preplace_joint_rad]}")
    print(f"preplace TCP: {[round(value, 4) for value in plan.preplace_tcp]}")
    print(f"place TCP: {[round(value, 4) for value in plan.place_tcp]}")
    print("sequence: handoff -> open -> wait for cube -> close -> preplace -> place -> open")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    hardware = load_hardware()
    plan = load_handoff_plan()
    print_plan(plan)
    if not args.execute:
        print("dry-run only; pass --execute to run the fixed handoff-and-place motion")
        return

    with PickPlaceRobot(hardware) as robot:
        robot.prepare_motion()
        try:
            robot.move_joints(plan.handoff_joint_rad, "handoff")
            robot.open_gripper()
            input("Put the cube between the fingers, remove your hand, then press Enter: ")
            robot.close_gripper()
            robot.move_joints(plan.preplace_joint_rad, "preplace")
            robot.move_tcp(
                plan.place_tcp,
                "place",
                plan.tcp_speed_mm_s,
                plan.tcp_acceleration_mm_s2,
            )
            robot.open_gripper()
            robot.move_tcp(
                plan.preplace_tcp,
                "preplace",
                plan.tcp_speed_mm_s,
                plan.tcp_acceleration_mm_s2,
            )
            robot.move_joints(plan.handoff_joint_rad, "handoff")
        except Exception:
            robot.stop()
            raise


if __name__ == "__main__":
    main()
