#!/usr/bin/env python3
"""Prepare all three onsite G1 commissioning stages for one cuboid.

The default is a local preview. ``--write`` creates profiles, schedules, and
one command bundle. This script never imports a robot SDK or opens a network
connection.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from xarm6_toss.open_loop_demo import prepare_deployment  # noqa: E402


CALIBRATION_SCRIPT = ROOT / "scripts" / "26_calibrate_open_loop_profile.py"
OBJECTS = {
    "O1": {
        "name": "cuboid30",
        "template": "configs/open_loop_flip/cuboid30/low_3deg.json",
        "object_id": "cuboid_44p5x46x30mm_20g",
    },
    "O2": {
        "name": "cuboid33",
        "template": "configs/open_loop_flip/cuboid33/low_5deg.json",
        "object_id": "cuboid_50p5x51x33p5mm_26p6g",
    },
    "O3": {
        "name": "cuboid38",
        "template": "configs/open_loop_flip/cuboid38/low_4p5deg.json",
        "object_id": "cuboid_57p5x58x38mm_37g",
    },
}
STAGES = ("empty_g1", "throw_only", "object")


def _load_calibration_module():
    spec = importlib.util.spec_from_file_location(
        "g1_calibration_for_bundle", CALIBRATION_SCRIPT
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load G1 calibration helper")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _label(value: str) -> str:
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]*", value):
        raise ValueError("label must contain only letters, numbers, '_' or '-'")
    return value


def build_bundle(
    *,
    object_key: str,
    label: str,
    held_position: int,
    release_position: int,
    preclose_position: int,
    close_position: int,
    root: Path = ROOT,
) -> dict:
    if object_key not in OBJECTS:
        raise ValueError(f"object must be one of: {', '.join(OBJECTS)}")
    label = _label(label)
    spec = OBJECTS[object_key]
    calibration = _load_calibration_module()
    template = calibration.load_json(root / spec["template"])

    profiles: dict[str, dict] = {}
    schedules: dict[str, dict] = {}
    files: list[dict] = []
    for stage in STAGES:
        profile_relative = (
            Path("configs/open_loop_flip/real_calibrated")
            / spec["name"]
            / label
            / f"{stage}.json"
        )
        schedule_relative = (
            Path("real_handoff")
            / spec["name"]
            / "low"
            / label
            / f"g1_schedule.{stage}.json"
        )
        profile, schedule = calibration.calibrate_profile(
            template,
            held_position=held_position,
            release_position=release_position,
            preclose_position=preclose_position,
            close_position=close_position,
            stage=stage,
            schedule_path=schedule_relative.as_posix(),
        )
        profiles[stage] = profile
        schedules[stage] = schedule
        files.extend(
            [
                {"path": profile_relative.as_posix(), "payload": profile},
                {"path": schedule_relative.as_posix(), "payload": schedule},
            ]
        )

    profile_paths = {
        stage: next(
            item["path"]
            for item in files
            if item["path"].endswith(f"/{stage}.json")
        )
        for stage in STAGES
    }
    commands = [
        {
            "step": "plan_only",
            "connects_robot": False,
            "command": (
                "python scripts/24_run_cube_open_loop_demo.py "
                f"--profile {profile_paths['empty_g1']}"
            ),
        },
        *[
            {
                "step": f"empty_arm_{speed:g}x",
                "connects_robot": True,
                "command": (
                    "python scripts/24_run_cube_open_loop_demo.py "
                    f"--profile {profile_paths['empty_g1']} "
                    f"--speed-scale {speed:g} --execute-empty-arm"
                ),
            }
            for speed in (0.25, 0.5, 1.0)
        ],
        {
            "step": "empty_g1",
            "connects_robot": True,
            "command": (
                "python scripts/24_run_cube_open_loop_demo.py "
                f"--profile {profile_paths['empty_g1']} --execute-empty-g1"
            ),
        },
        {
            "step": "soft_mat_throw_only",
            "connects_robot": True,
            "command": (
                "python scripts/24_run_cube_open_loop_demo.py "
                f"--profile {profile_paths['throw_only']} --execute-throw-only"
            ),
        },
        {
            "step": "guarded_object_recatch",
            "connects_robot": True,
            "command": (
                "python scripts/24_run_cube_open_loop_demo.py "
                f"--profile {profile_paths['object']} --execute-object"
            ),
        },
    ]
    bundle_path = (
        Path("real_handoff")
        / spec["name"]
        / "low"
        / label
        / "commissioning_bundle.json"
    )
    bundle = {
        "schema": "xarm6_object_commissioning_bundle_v1",
        "offline_generated": True,
        "robot_connection_attempted": False,
        "object_key": object_key,
        "object_id": spec["object_id"],
        "template_profile": spec["template"],
        "label": label,
        "g1": {
            "held_position": held_position,
            "release_position": release_position,
            "preclose_position": preclose_position,
            "close_position": close_position,
        },
        "profiles": profile_paths,
        "execution_order": commands,
    }
    files.append({"path": bundle_path.as_posix(), "payload": bundle})
    return {"bundle": bundle, "files": files}


def write_bundle(result: dict, root: Path = ROOT) -> list[Path]:
    paths = [root / item["path"] for item in result["files"]]
    existing = [path for path in paths if path.exists()]
    if existing:
        raise FileExistsError(f"refusing to overwrite existing file: {existing[0]}")
    for item, path in zip(result["files"], paths):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(item["payload"], indent=2) + "\n", encoding="utf-8")

    for stage in STAGES:
        profile_path = root / result["bundle"]["profiles"][stage]
        plan, _ = prepare_deployment(root, profile_path=profile_path, mode=stage)
        if plan["object_id"] != result["bundle"]["object_id"]:
            raise RuntimeError(f"{stage} profile object does not match bundle")
    return paths


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--object", choices=tuple(OBJECTS), required=True)
    parser.add_argument("--label", required=True, help="run label, e.g. 20260826")
    parser.add_argument("--held-position", type=int, required=True)
    parser.add_argument("--release-position", type=int, required=True)
    parser.add_argument("--preclose-position", type=int, required=True)
    parser.add_argument("--close-position", type=int, required=True)
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args()

    result = build_bundle(
        object_key=args.object,
        label=args.label,
        held_position=args.held_position,
        release_position=args.release_position,
        preclose_position=args.preclose_position,
        close_position=args.close_position,
    )
    print(json.dumps(result["bundle"], indent=2))
    if not args.write:
        print("preview only; no files, robot connection, or command were created")
        return 0
    paths = write_bundle(result)
    print(f"created {len(paths)} commissioning files")
    print("offline bundle complete; no robot connection or command was sent")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
