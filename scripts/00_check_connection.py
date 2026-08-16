#!/usr/bin/env python3
"""Connect to xArm 6 and print state; this script never enables motion."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from xarm6_toss.config import load_robot_config
from xarm6_toss.xarm_adapter import XArm6Client


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        type=Path,
        default=ROOT / "configs" / "robot.example.json",
    )
    args = parser.parse_args()
    config = load_robot_config(args.config)
    if "XXX" in config.ip:
        raise SystemExit("replace robot.ip in a copied local config first")
    with XArm6Client(config) as robot:
        print(json.dumps(robot.snapshot().as_dict(), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
