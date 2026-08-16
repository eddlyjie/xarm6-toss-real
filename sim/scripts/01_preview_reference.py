#!/usr/bin/env python3
"""Preview the version-independent arm/gripper control contract."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np


XARM_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(XARM_ROOT / "src"))

from xarm6_toss.control_reference import (  # noqa: E402
    GripperEvent,
    QuinticJointSegment,
    generate_joint_reference,
)


DEFAULT_CONFIG = XARM_ROOT / "sim" / "configs" / "upward_throw_smoke.json"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args()
    config = json.loads(args.config.read_text(encoding="utf-8"))
    segments = tuple(
        QuinticJointSegment(**value)
        for value in config["reference_segments"]
    )
    events = tuple(GripperEvent(**value) for value in config["gripper_events"])
    samples = generate_joint_reference(
        segments, float(config["control_period_s"])
    )
    q = np.asarray([sample.joint_position_rad for sample in samples])
    dq = np.asarray([sample.joint_velocity_rad_s for sample in samples])
    ddq = np.asarray([sample.joint_acceleration_rad_s2 for sample in samples])
    print(f"scenario={config['name']}")
    print(
        f"samples={len(samples)}, duration={samples[-1].time_s:.3f} s, "
        f"physics/control={config['physics_dt_s']:.3f}/"
        f"{config['control_period_s']:.3f} s"
    )
    print(
        f"max joint speed={np.max(np.abs(dq)):.3f} rad/s, "
        f"max joint acceleration={np.max(np.abs(ddq)):.3f} rad/s^2, "
        f"joint path={np.sum(np.abs(np.diff(q, axis=0))):.3f} rad"
    )
    for event in events:
        print(
            f"gripper event t={event.time_s:.3f}s {event.name}: "
            f"real={event.real_position:.1f}, sim drive={event.drive_joint_rad:.3f} rad"
        )
    hidden = config["cube_physics"]
    print(
        "cube physics is simulator-only: "
        f"size={hidden['side_length_m_range']} m, "
        f"mass={hidden['mass_kg_range']} kg"
    )
    print("offline preview only; Isaac and the real robot were not connected")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
