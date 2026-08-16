#!/usr/bin/env python3
"""Measure the short G1 travel intended for release and catch."""

import argparse
from datetime import datetime
import json
from statistics import mean
import time

import _bootstrap  # noqa: F401
from real_cube_demo.config import DEMO_ROOT, load_hardware
from real_cube_demo.robot import PickPlaceRobot


def measure_move(
    robot: PickPlaceRobot,
    target: float,
    *,
    poll_period_s: float,
    tolerance: float,
    timeout_s: float,
) -> dict:
    start_position = robot.gripper_position(check_baud=False)
    samples = [{"time_s": 0.0, "position": start_position}]
    movement_threshold = 3.0
    first_observed_motion_s = None

    start = time.monotonic()
    robot.command_gripper_position(target)
    command_return_s = time.monotonic() - start
    next_sample = start

    while True:
        next_sample += poll_period_s
        time.sleep(max(0.0, next_sample - time.monotonic()))
        position = robot.gripper_position(check_baud=False)
        elapsed = time.monotonic() - start
        samples.append({"time_s": elapsed, "position": position})

        if (
            first_observed_motion_s is None
            and abs(position - start_position) >= movement_threshold
        ):
            first_observed_motion_s = elapsed
        if abs(position - target) <= tolerance:
            return {
                "start_position": start_position,
                "target_position": target,
                "final_position": position,
                "command_return_s": command_return_s,
                "first_observed_motion_s": first_observed_motion_s,
                "target_reached_s": elapsed,
                "samples": samples,
            }
        if elapsed >= timeout_s:
            raise RuntimeError(
                f"G1 did not reach {target:g} within {timeout_s:g} s; "
                f"last position={position:g}"
            )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--held-position", type=float, default=370.0)
    parser.add_argument("--release-position", type=float, default=520.0)
    parser.add_argument("--cycles", type=int, default=3)
    parser.add_argument("--poll-ms", type=float, default=10.0)
    parser.add_argument("--tolerance", type=float, default=5.0)
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    print(
        f"partial G1 timing: held={args.held_position:g}, "
        f"release/catch-open={args.release_position:g}, cycles={args.cycles}"
    )
    if not args.execute:
        print("dry-run only; pass --execute with an empty gripper")
        return

    trials = []
    with PickPlaceRobot(load_hardware()) as robot:
        robot.prepare_motion()
        try:
            robot.set_gripper_position(args.held_position)
            for cycle in range(1, args.cycles + 1):
                opening = measure_move(
                    robot,
                    args.release_position,
                    poll_period_s=args.poll_ms / 1000.0,
                    tolerance=args.tolerance,
                    timeout_s=args.timeout,
                )
                closing = measure_move(
                    robot,
                    args.held_position,
                    poll_period_s=args.poll_ms / 1000.0,
                    tolerance=args.tolerance,
                    timeout_s=args.timeout,
                )
                trials.append(
                    {
                        "cycle": cycle,
                        "opening": opening,
                        "closing": closing,
                    }
                )
                print(
                    f"cycle {cycle}: "
                    f"open command={opening['command_return_s']:.3f} s, "
                    f"first motion<={opening['first_observed_motion_s']:.3f} s, "
                    f"target={opening['target_reached_s']:.3f} s; "
                    f"close command={closing['command_return_s']:.3f} s, "
                    f"first motion<={closing['first_observed_motion_s']:.3f} s, "
                    f"target={closing['target_reached_s']:.3f} s"
                )
            robot.open_gripper()
        except Exception:
            robot.stop()
            raise

    result = {
        "held_position": args.held_position,
        "release_position": args.release_position,
        "speed": load_hardware().gripper_speed,
        "poll_period_s_requested": args.poll_ms / 1000.0,
        "target_tolerance": args.tolerance,
        "trials": trials,
        "mean_open_s": mean(
            value["opening"]["target_reached_s"] for value in trials
        ),
        "mean_close_s": mean(
            value["closing"]["target_reached_s"] for value in trials
        ),
    }
    output_dir = DEMO_ROOT / "outputs" / "gripper_timings"
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_path = output_dir / f"partial_{stamp}.json"
    output_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(
        f"mean partial open={result['mean_open_s']:.3f} s, "
        f"close={result['mean_close_s']:.3f} s"
    )
    print(f"saved timing: {output_path}")


if __name__ == "__main__":
    main()
