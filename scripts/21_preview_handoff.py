#!/usr/bin/env python3
"""Generate a speed-scaled handoff preview without connecting to a robot."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TIMELINE = ROOT / "real_handoff" / "nominal_timeline.json"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeline", type=Path, default=DEFAULT_TIMELINE)
    parser.add_argument(
        "--speed-scale", type=float, choices=(0.25, 0.5, 1.0), required=True
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    source = json.loads(args.timeline.read_text(encoding="utf-8"))
    scale = args.speed_scale
    samples = []
    for sample in source["samples"]:
        samples.append(
            {
                **sample,
                "time_s": sample["time_s"] / scale,
                "joint_velocity_rad_s": [
                    value * scale
                    for value in sample["joint_velocity_rad_s"]
                ],
                "joint_acceleration_rad_s2": [
                    value * scale**2
                    for value in sample["joint_acceleration_rad_s2"]
                ],
            }
        )
    events = []
    for event in source["g1_events"]:
        nominal_time_s = event.get("time_s", event.get("nominal_time_s"))
        if nominal_time_s is None:
            raise ValueError("G1 event requires time_s or nominal_time_s")
        scaled = {**event, "time_s": nominal_time_s / scale}
        if "time_from_observed_detach_s" in event:
            scaled["time_from_observed_detach_s"] = (
                event["time_from_observed_detach_s"] / scale
            )
            scaled["trigger"] = "observed_detach_relative"
        else:
            scaled["trigger"] = "absolute_timeline"
        events.append(scaled)
    qd = np.asarray([sample["joint_velocity_rad_s"] for sample in samples])
    qdd = np.asarray(
        [sample["joint_acceleration_rad_s2"] for sample in samples]
    )
    payload = {
        "schema": "xarm6_outward_toss_preview_v1",
        "robot_commands_sent": 0,
        "source_timeline": str(args.timeline),
        "speed_scale": scale,
        "duration_s": samples[-1]["time_s"],
        "peak_joint_speed_rad_s": float(np.max(np.abs(qd))),
        "peak_joint_acceleration_rad_s2": float(np.max(np.abs(qdd))),
        "g1_events": events,
        "samples": samples,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(payload, indent=2) + "\n", encoding="utf-8"
    )
    print(
        json.dumps(
            {
                key: payload[key]
                for key in (
                    "schema",
                    "robot_commands_sent",
                    "speed_scale",
                    "duration_s",
                    "peak_joint_speed_rad_s",
                    "peak_joint_acceleration_rad_s2",
                    "g1_events",
                )
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
