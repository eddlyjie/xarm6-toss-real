#!/usr/bin/env python3
"""Read-only inventory: this script does not enable or move the robot."""

import pprint

import _bootstrap  # noqa: F401
from real_cube_demo.config import load_hardware
from real_cube_demo.realsense import connected_devices
from real_cube_demo.robot import PickPlaceRobot


def main() -> None:
    hardware = load_hardware()
    print("RealSense devices:")
    pprint.pp(connected_devices())
    print("\nxArm and gripper:")
    with PickPlaceRobot(hardware) as robot:
        pprint.pp(robot.inventory())


if __name__ == "__main__":
    main()

