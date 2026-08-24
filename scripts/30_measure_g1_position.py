#!/usr/bin/env python3
"""Move only the G1 to one candidate calibration position and record it.

The default is a dry-run. Real G1 motion requires both ``--execute`` and an
interactive ``MOVE G1`` confirmation. No arm motion method is called.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
REAL_DEMO = ROOT / "toss_project_sim_handoff" / "toss_project" / "real_cube_demo"
DEFAULT_HARDWARE_CONFIG = REAL_DEMO / "configs" / "hardware.json"
PURPOSES = ("held", "release", "preclose", "close")
OBJECTS = ("O0", "O1", "O2", "O3")


def g1_position(value: int) -> int:
    position = int(value)
    if not 0 <= position <= 850:
        raise ValueError("G1 position must be within 0..850")
    return position


def measure_position(robot, target: int) -> dict:
    """Use the gripper-only path; ``robot`` may be a fake in CPU tests."""
    robot.client.prepare_gripper_only()
    before = float(robot.gripper_position(check_baud=False))
    robot.set_gripper_position(float(target))
    after = float(robot.gripper_position(check_baud=False))
    return {
        "target_position": target,
        "reported_before": before,
        "reported_after": after,
        "absolute_target_error": abs(after - target),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--object", choices=OBJECTS, required=True)
    parser.add_argument("--purpose", choices=PURPOSES, required=True)
    parser.add_argument("--position", type=int, required=True)
    parser.add_argument("--hardware-config", type=Path, default=DEFAULT_HARDWARE_CONFIG)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    target = g1_position(args.position)
    preview = {
        "schema": "xarm6_g1_position_measurement_v1",
        "object_key": args.object,
        "purpose": args.purpose,
        "target_position": target,
        "hardware_config": str(args.hardware_config),
        "arm_motion_requested": False,
        "robot_connection_attempted": False,
    }
    print(json.dumps(preview, indent=2))
    if not args.execute:
        print("dry-run only; no robot connection or command was sent")
        return 0
    if args.output is not None and args.output.exists():
        raise FileExistsError(f"refusing to overwrite measurement: {args.output}")
    confirmation = input(
        f"G1 only: move {args.object} {args.purpose} candidate to {target}; "
        "fingers clear? Type MOVE G1: "
    )
    if confirmation.strip() != "MOVE G1":
        print("cancelled before robot connection")
        return 2

    demo_src = REAL_DEMO / "src"
    if str(demo_src) not in sys.path:
        sys.path.insert(0, str(demo_src))
    from real_cube_demo.config import load_hardware  # noqa: E402
    from real_cube_demo.robot import PickPlaceRobot  # noqa: E402

    hardware = load_hardware(args.hardware_config.resolve())
    if hardware.gripper_kind != "xarm_gripper_g1":
        raise RuntimeError("hardware config is not a UFACTORY G1")
    if not hardware.gripper_model_confirmed:
        raise RuntimeError("G1 model is not confirmed in hardware config")
    if float(hardware.gripper_speed) != 5000.0:
        raise RuntimeError("G1 speed must remain 5000 for current profiles")

    with PickPlaceRobot(hardware) as robot:
        measurement = measure_position(robot, target)
    result = {
        **preview,
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "robot_connection_attempted": True,
        "measurement": measurement,
        "operator_decision_required": (
            "inspect grip/release physically; this tool does not decide whether "
            "the candidate is suitable"
        ),
    }
    rendered = json.dumps(result, indent=2) + "\n"
    print(rendered, end="")
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
        print(f"saved G1 measurement: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
