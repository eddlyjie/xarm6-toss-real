#!/usr/bin/env python3
"""Replay deployable observations through the frozen xArm6 catch controller."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from xarm6_toss.online_closed_loop import OnlineInterceptController  # noqa: E402


DEFAULT_OBSERVATIONS = ROOT / "configs" / "closed_loop_example.jsonl"
DEFAULT_MODEL = (
    ROOT / "sim" / "models" / "intercept_residual_native_outward_v1.json"
)


def read_events(path: str) -> list[dict]:
    if path == "-":
        lines = sys.stdin
    else:
        lines = Path(path).open(encoding="utf-8")
    try:
        return [json.loads(line) for line in lines if line.strip()]
    finally:
        if path != "-":
            lines.close()


def replay(
    events: list[dict],
    *,
    model: Path,
    release_time_s: float,
    horizon_s: float,
    intercept_time_s: float,
    minimum_camera_samples: int,
) -> list[dict]:
    controller = OnlineInterceptController.from_checkpoint(
        model,
        release_command_time_s=release_time_s,
        prediction_horizon_s=horizon_s,
        intercept_time_s=intercept_time_s,
        minimum_camera_samples=minimum_camera_samples,
    )
    commands = []
    for event in events:
        kind = event["type"]
        if kind == "encoder_detach_prior":
            controller.set_encoder_detach_prior(
                event["time_s"],
                event["position_base_m"],
                event["velocity_base_m_s"],
            )
        elif kind in {
            "global_camera_position",
            "third_view_camera_position",
            "wrist_camera_position",
        }:
            source_camera = (
                "wrist"
                if kind == "wrist_camera_position"
                else "third_view"
            )
            commands.append(
                controller.add_camera_position(
                    source_camera,
                    event["time_s"],
                    event["position_base_m"],
                ).as_dict()
            )
        else:
            raise ValueError(f"unsupported observation type: {kind}")
    return commands


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--observations", default=str(DEFAULT_OBSERVATIONS))
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--release-time-s", type=float, default=0.69)
    parser.add_argument("--prediction-horizon-s", type=float, default=0.075)
    parser.add_argument("--intercept-time-s", type=float, default=0.815)
    parser.add_argument("--minimum-camera-samples", type=int, default=3)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    commands = replay(
        read_events(args.observations),
        model=args.model,
        release_time_s=args.release_time_s,
        horizon_s=args.prediction_horizon_s,
        intercept_time_s=args.intercept_time_s,
        minimum_camera_samples=args.minimum_camera_samples,
    )
    payload = {
        "schema": "xarm6_closed_loop_dry_run_v1",
        "robot_commands_sent": 0,
        "observation_count": len(commands),
        "post_release_update_count": sum(
            item["time_since_release_s"] > 0.0 for item in commands
        ),
        "learned_update_count": sum(
            item["learned_residual_applied"] for item in commands
        ),
        "source_camera_counts": {
            source: sum(item["source_camera"] == source for item in commands)
            for source in ("third_view", "wrist")
        },
        "commands": commands,
    }
    rendered = json.dumps(payload, indent=2) + "\n"
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")


if __name__ == "__main__":
    main()
