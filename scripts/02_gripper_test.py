#!/usr/bin/env python3
"""Preview or explicitly run open→close→open on a confirmed gripper."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from xarm6_toss.config import load_robot_config
from xarm6_toss.xarm_adapter import XArm6Client


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="actually command the confirmed gripper; default is dry-run",
    )
    parser.add_argument("--pause-s", type=float, default=1.0)
    args = parser.parse_args()
    config = load_robot_config(args.config)
    sequence = [
        ("open", config.gripper_open_position),
        ("close", config.gripper_closed_position),
        ("open", config.gripper_open_position),
    ]
    print("sequence:", sequence)
    if not args.execute:
        print("dry-run only; pass --execute after hardware confirmation")
        return 0
    if not config.gripper_confirmed:
        raise SystemExit(
            "execution refused: hardware_confirmed and gripper settings "
            "must be completed"
        )
    with XArm6Client(config) as robot:
        robot.prepare_gripper_only()
        for label, position in sequence:
            print(f"gripper {label}: position={position}")
            robot.set_gripper(float(position), wait=True)
            time.sleep(args.pause_s)
        print("final snapshot:", robot.snapshot().as_dict())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
