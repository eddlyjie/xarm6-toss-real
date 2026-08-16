#!/usr/bin/env python3
"""Offline demonstration of Probe selection and M2/M3 skill coordination."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from xarm6_toss.method import (
    CoordinatorCandidate,
    ProbeCandidate,
    select_coordinated,
    select_fixed_confidence,
    select_probe,
)


def load_candidates(config: dict, target_id: str) -> list[CoordinatorCandidate]:
    values = []
    for skill in config["skills"]:
        target = skill["targets"][target_id]
        values.append(
            CoordinatorCandidate(
                skill_id=skill["skill_id"],
                catch_probability=skill["catch_probability"],
                detach_flight_uncertainty=skill[
                    "detach_flight_uncertainty"
                ],
                collision_contact_risk=skill["collision_contact_risk"],
                **target,
            )
        )
    return values


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        type=Path,
        default=ROOT / "configs" / "method.example.json",
    )
    parser.add_argument(
        "--target",
        default="quarter_turn_forward",
    )
    args = parser.parse_args()
    config = json.loads(args.config.read_text(encoding="utf-8"))
    if args.target not in config["targets"]:
        raise SystemExit(f"unknown target {args.target!r}")

    probe_values = [ProbeCandidate(**value) for value in config["probe_candidates"]]
    probe, probe_ranking = select_probe(probe_values, task_conditioned=True)
    candidates = load_candidates(config, args.target)
    fixed = select_fixed_confidence(candidates)
    coordinated, coordinated_ranking = select_coordinated(candidates)

    print(f"target={args.target}")
    print(f"selected_probe={probe.name}")
    print("probe_ranking:")
    for name, score in probe_ranking:
        print(f"  {name}: {score:.4f}")
    print(f"M2_fixed_confidence={fixed.skill_id}")
    print(f"M3_target_coordinated={coordinated.skill_id}")
    print("M3_ranking:")
    for skill_id, eligible, cost in coordinated_ranking:
        print(f"  {skill_id}: eligible={eligible} J_coord={cost:.4f}")
    print("offline method demo only: no robot connection or motion occurred")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
