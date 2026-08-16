#!/usr/bin/env python3
"""Record the robot's current joint and TCP pose without commanding motion."""

import argparse

import _bootstrap  # noqa: F401
from real_cube_demo.config import POSE_NAMES, load_hardware, save_recorded_pose
from real_cube_demo.robot import PickPlaceRobot


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--name", choices=POSE_NAMES, required=True)
    args = parser.parse_args()

    with PickPlaceRobot(load_hardware()) as robot:
        snapshot = robot.client.snapshot()
    joints = list(snapshot.joint_rad[:6])
    tcp = list(snapshot.tcp_pose)
    save_recorded_pose(args.name, joints, tcp)
    print(f"recorded {args.name}")
    print(f"joint_rad: {[round(value, 6) for value in joints]}")
    print(f"tcp_pose: {[round(value, 3) for value in tcp]}")


if __name__ == "__main__":
    main()
