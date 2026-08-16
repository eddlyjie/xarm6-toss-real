#!/usr/bin/env python3
"""Compare the calibrated arm TCP with the generated xArm6+G1 joint_tcp."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from yourdfpy import URDF


PROJECT_ROOT = Path(__file__).resolve().parents[3]
SIM_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = (
    PROJECT_ROOT
    / "RobotCamCalib"
    / "RobotCamCalib"
    / "assets"
    / "robots"
    / "xarm6"
    / "xarm6_wo_ee.urdf"
)
DEFAULT_GENERATED = SIM_ROOT / "assets" / "xarm6_g1" / "xarm6_g1.urdf"
DEFAULT_SCENARIO = SIM_ROOT / "configs" / "upward_throw_smoke.json"
TCP_OFFSET_M = 0.172


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--generated", type=Path, default=DEFAULT_GENERATED)
    parser.add_argument("--scenario", type=Path, default=DEFAULT_SCENARIO)
    args = parser.parse_args()

    source = URDF.load(args.source, load_meshes=False)
    generated = URDF.load(args.generated, load_meshes=False)
    config = json.loads(args.scenario.read_text(encoding="utf-8"))
    samples = []
    for segment in config["reference_segments"]:
        samples.append(segment["start_joint_rad"])
        samples.append(segment["end_joint_rad"])

    maximum_error = 0.0
    for joint in samples:
        source.update_cfg(np.asarray(joint, dtype=float))
        generated.update_cfg(np.asarray([*joint, 0.37], dtype=float))
        source_tcp = source.get_transform("link_eef", "link_base").copy()
        source_tcp[:3, 3] += source_tcp[:3, :3] @ np.asarray(
            [0.0, 0.0, TCP_OFFSET_M]
        )
        generated_tcp = generated.get_transform("link_tcp", "link_base")
        maximum_error = max(
            maximum_error,
            float(np.max(np.abs(source_tcp - generated_tcp))),
        )
    print(f"poses={len(samples)}, maximum transform error={maximum_error:.3e}")
    if maximum_error > 1e-9:
        raise RuntimeError("generated xArm6+G1 TCP does not match calibrated FK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
