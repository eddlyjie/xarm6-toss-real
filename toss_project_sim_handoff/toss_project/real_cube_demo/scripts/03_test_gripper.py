#!/usr/bin/env python3
"""Open and close the confirmed G1 gripper; defaults to dry-run."""

import argparse
import time

import _bootstrap  # noqa: F401
from real_cube_demo.config import load_hardware
from real_cube_demo.robot import PickPlaceRobot


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    hardware = load_hardware()
    print(
        f"G1 gripper plan: open={hardware.gripper_open_position:g}, "
        f"close={hardware.gripper_closed_position:g}, speed={hardware.gripper_speed:g}"
    )
    if not args.execute:
        print("dry-run only; pass --execute after checking the clear workspace")
        return

    with PickPlaceRobot(hardware) as robot:
        robot.prepare_motion()
        try:
            for name, command in (
                ("open", robot.open_gripper),
                ("close", robot.close_gripper),
                ("open", robot.open_gripper),
            ):
                start = time.monotonic()
                command()
                duration = time.monotonic() - start
                print(
                    f"{name} duration={duration:.3f} s, "
                    f"position={robot.gripper_position():.1f}"
                )
        except Exception:
            robot.stop()
            raise


if __name__ == "__main__":
    main()
