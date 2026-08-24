#!/usr/bin/env python3
"""Generate next/high real-G1 profiles from one completed low commissioning bundle.

The tool is offline. It reuses the measured G1 positions for the same object
and applies them to the existing Sim-validated next/high arm references.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
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
        "object_id": "cuboid_44p5x46x30mm_20g",
        "poses": {
            "next": "configs/open_loop_flip/cuboid30/pose_conditioned_5p5deg.json",
            "high": "configs/open_loop_flip/cuboid30/high_6p5deg.json",
        },
    },
    "O2": {
        "name": "cuboid33",
        "object_id": "cuboid_50p5x51x33p5mm_26p6g",
        "poses": {
            "next": "configs/open_loop_flip/cuboid33/pose_conditioned_5p5deg.json",
            "high": "configs/open_loop_flip/cuboid33/high_6p5deg.json",
        },
    },
    "O3": {
        "name": "cuboid38",
        "object_id": "cuboid_57p5x58x38mm_37g",
        "poses": {
            "next": "configs/open_loop_flip/cuboid38/pose_conditioned_5p5deg.json",
            "high": "configs/open_loop_flip/cuboid38/high_6p5deg.json",
        },
    },
}
STAGES = ("empty_g1", "throw_only", "object")


def _load_calibration_module():
    spec = importlib.util.spec_from_file_location(
        "g1_calibration_for_pose_ladder", CALIBRATION_SCRIPT
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load G1 calibration helper")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _read_bundle(path: Path) -> dict:
    if not path.is_file():
        raise FileNotFoundError(f"commissioning bundle does not exist: {path}")
    bundle = json.loads(path.read_text(encoding="utf-8"))
    if bundle.get("schema") != "xarm6_object_commissioning_bundle_v1":
        raise ValueError("unsupported commissioning bundle schema")
    object_key = bundle.get("object_key")
    if object_key not in OBJECTS:
        raise ValueError("pose ladder requires an O1, O2, or O3 commissioning bundle")
    if bundle.get("object_id") != OBJECTS[object_key]["object_id"]:
        raise ValueError("commissioning bundle object identity is inconsistent")
    return bundle


def _commands(profile_paths: dict[str, str]) -> list[dict]:
    runner = "python scripts/24_run_cube_open_loop_demo.py"
    return [
        {
            "step": "plan_only",
            "connects_robot": False,
            "command": f"{runner} --profile {profile_paths['empty_g1']}",
        },
        *[
            {
                "step": f"empty_arm_{speed:g}x",
                "connects_robot": True,
                "command": (
                    f"{runner} --profile {profile_paths['empty_g1']} "
                    f"--speed-scale {speed:g} --execute-empty-arm"
                ),
            }
            for speed in (0.25, 0.5, 1.0)
        ],
        {
            "step": "empty_g1",
            "connects_robot": True,
            "command": f"{runner} --profile {profile_paths['empty_g1']} --execute-empty-g1",
        },
        {
            "step": "soft_mat_throw_only",
            "connects_robot": True,
            "command": f"{runner} --profile {profile_paths['throw_only']} --execute-throw-only",
        },
        {
            "step": "guarded_object_recatch",
            "connects_robot": True,
            "command": f"{runner} --profile {profile_paths['object']} --execute-object",
        },
    ]


def build_pose_ladder(
    commissioning_bundle_path: Path,
    *,
    root: Path = ROOT,
) -> dict:
    bundle = _read_bundle(commissioning_bundle_path)
    object_key = bundle["object_key"]
    spec = OBJECTS[object_key]
    label = bundle["label"]
    g1 = bundle["g1"]
    calibration = _load_calibration_module()
    files: list[dict] = []
    poses: dict[str, dict] = {}

    for pose_name, template_relative in spec["poses"].items():
        template = calibration.load_json(root / template_relative)
        profile_paths = {
            stage: (
                Path("configs/open_loop_flip/real_calibrated")
                / spec["name"]
                / label
                / pose_name
                / f"{stage}.json"
            ).as_posix()
            for stage in STAGES
        }
        schedule_paths = {
            stage: (
                Path("real_handoff")
                / spec["name"]
                / pose_name
                / label
                / f"g1_schedule.{stage}.json"
            ).as_posix()
            for stage in STAGES
        }
        for stage in STAGES:
            profile, schedule = calibration.calibrate_profile(
                template,
                held_position=g1["held_position"],
                release_position=g1["release_position"],
                preclose_position=g1["preclose_position"],
                close_position=g1["close_position"],
                stage=stage,
                schedule_path=schedule_paths[stage],
            )
            files.extend(
                [
                    {"path": profile_paths[stage], "payload": profile},
                    {"path": schedule_paths[stage], "payload": schedule},
                ]
            )
        poses[pose_name] = {
            "template_profile": template_relative,
            "desired_angle_deg": float(template["desired_angle_deg"]),
            "profiles": profile_paths,
            "execution_order": _commands(profile_paths),
        }

    try:
        source_bundle = commissioning_bundle_path.resolve().relative_to(root.resolve()).as_posix()
    except ValueError:
        source_bundle = str(commissioning_bundle_path.resolve())
    ladder_path = (
        Path("real_handoff")
        / spec["name"]
        / "pose_ladder"
        / label
        / "pose_ladder_bundle.json"
    )
    ladder = {
        "schema": "xarm6_object_pose_ladder_bundle_v1",
        "offline_generated": True,
        "robot_connection_attempted": False,
        "object_key": object_key,
        "object_id": spec["object_id"],
        "label": label,
        "source_low_commissioning_bundle": source_bundle,
        "g1": g1,
        "poses": poses,
    }
    files.append({"path": ladder_path.as_posix(), "payload": ladder})
    return {"bundle": ladder, "files": files}


def write_pose_ladder(result: dict, *, root: Path = ROOT) -> list[Path]:
    paths = [root / item["path"] for item in result["files"]]
    existing = [path for path in paths if path.exists()]
    if existing:
        raise FileExistsError(f"refusing to overwrite existing file: {existing[0]}")
    for item, path in zip(result["files"], paths):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(item["payload"], indent=2) + "\n", encoding="utf-8")

    for pose in result["bundle"]["poses"].values():
        for stage in STAGES:
            profile_path = root / pose["profiles"][stage]
            plan, _ = prepare_deployment(root, profile_path=profile_path, mode=stage)
            if plan["object_id"] != result["bundle"]["object_id"]:
                raise RuntimeError(f"{stage} profile object does not match pose ladder")
    return paths


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--commissioning-bundle", type=Path, required=True)
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args()

    result = build_pose_ladder(args.commissioning_bundle)
    print(json.dumps(result["bundle"], indent=2))
    if not args.write:
        print("preview only; no files, robot connection, or command were created")
        return 0
    paths = write_pose_ladder(result)
    print(f"created {len(paths)} pose-ladder files")
    print("offline pose ladder complete; no robot connection or command was sent")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
