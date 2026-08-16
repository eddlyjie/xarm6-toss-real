#!/usr/bin/env python3
"""Generate a throw trajectory CSV without importing the xArm SDK."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from xarm6_toss import generate_throw_samples, load_throw_plan


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    plan = load_throw_plan(args.plan)
    samples = generate_throw_samples(plan)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            ["time_s", *[f"joint_{i + 1}_rad" for i in range(plan.dof)],
             "release_gripper", "phase"]
        )
        for sample in samples:
            writer.writerow(
                [
                    f"{sample.time_s:.9f}",
                    *[f"{value:.9f}" for value in sample.joint_rad],
                    int(sample.release_gripper),
                    sample.phase,
                ]
            )
    release = next(sample for sample in samples if sample.release_gripper)
    print(f"plan={plan.name}")
    print(f"samples={len(samples)} duration_s={samples[-1].time_s:.3f}")
    print(
        f"release_time_s={release.time_s:.3f} "
        f"release_joint_rad={list(release.joint_rad)}"
    )
    print(f"output={args.output.resolve()}")
    print("preview only: no robot connection or motion occurred")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
