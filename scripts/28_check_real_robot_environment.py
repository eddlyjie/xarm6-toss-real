#!/usr/bin/env python3
"""Offline preflight for the four-object xArm6 handoff.

This script reads local files and installed-package metadata only. It never
imports the xArm SDK, opens a socket, or constructs a robot client.
"""

from __future__ import annotations

import argparse
from importlib import metadata
import importlib.util
import json
from pathlib import Path
import re
import sys
from typing import Callable


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_HARDWARE = (
    ROOT
    / "toss_project_sim_handoff"
    / "toss_project"
    / "real_cube_demo"
    / "configs"
    / "hardware.json"
)
HANDOFF_CHECKER = ROOT / "scripts" / "27_check_four_object_handoff.py"


def _load_handoff_checker():
    spec = importlib.util.spec_from_file_location(
        "four_object_handoff_for_preflight", HANDOFF_CHECKER
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load offline four-object handoff checker")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _release(version: str) -> tuple[int, ...]:
    match = re.match(r"^(\d+(?:\.\d+)*)", version)
    return tuple(int(value) for value in match.group(1).split(".")) if match else ()


def _package_check(
    name: str,
    resolver: Callable[[str], str],
    compatible: Callable[[tuple[int, ...]], bool],
    requirement: str,
) -> dict:
    try:
        installed = resolver(name)
    except metadata.PackageNotFoundError:
        return {
            "name": name,
            "installed": None,
            "requirement": requirement,
            "ok": False,
            "detail": "package is not installed",
        }
    release = _release(installed)
    ok = bool(release) and compatible(release)
    return {
        "name": name,
        "installed": installed,
        "requirement": requirement,
        "ok": ok,
        "detail": "compatible" if ok else "installed version is outside the supported range",
    }


def check_packages(
    resolver: Callable[[str], str] = metadata.version,
) -> list[dict]:
    def at_least(release: tuple[int, ...], minimum: tuple[int, ...]) -> bool:
        padded = release + (0,) * (len(minimum) - len(release))
        return padded[: len(minimum)] >= minimum

    return [
        _package_check(
            "xarm-python-sdk",
            resolver,
            lambda value: value[0] == 1 and at_least(value, (1, 16)),
            ">=1.16,<2",
        ),
        _package_check(
            "numpy",
            resolver,
            lambda value: value[0] < 3 and at_least(value, (1, 24)),
            ">=1.24,<3",
        ),
        _package_check(
            "scipy",
            resolver,
            lambda value: value[0] == 1 and at_least(value, (1, 10)),
            ">=1.10,<2",
        ),
    ]


def check_hardware_config(path: Path) -> dict:
    problems: list[str] = []
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return {
            "path": str(path),
            "configuration_valid": False,
            "motion_confirmed": False,
            "problems": [str(exc)],
        }

    robot = raw.get("robot", {})
    gripper = raw.get("gripper", {})
    if raw.get("schema") != "real_cube_hardware_v1":
        problems.append("schema must be real_cube_hardware_v1")
    if not isinstance(robot.get("ip"), str) or not robot.get("ip"):
        problems.append("robot.ip is missing")
    if robot.get("dof") != 6:
        problems.append("robot.dof must be 6")
    if float(robot.get("control_period_s", -1.0)) != 0.02:
        problems.append("robot.control_period_s must be 0.02")
    if gripper.get("kind") != "xarm_gripper_g1":
        problems.append("gripper.kind must be xarm_gripper_g1")
    if gripper.get("model_confirmed") is not True:
        problems.append("G1 model has not been confirmed")
    if float(gripper.get("speed", -1.0)) != 5000.0:
        problems.append("G1 speed must be 5000 for the current profiles")
    for name in ("open_position", "closed_position"):
        value = gripper.get(name)
        if not isinstance(value, (int, float)) or not 0 <= float(value) <= 850:
            problems.append(f"gripper.{name} must be within 0..850")

    return {
        "path": str(path),
        "configuration_valid": not problems,
        "motion_confirmed": raw.get("motion_confirmed") is True,
        "robot_ip_configured": bool(robot.get("ip")),
        "control_period_s": robot.get("control_period_s"),
        "g1_speed": gripper.get("speed"),
        "problems": problems,
    }


def discover_commissioning_bundles(root: Path) -> dict:
    object_dirs = {"O1": "cuboid30", "O2": "cuboid33", "O3": "cuboid38"}
    result = {}
    for object_key, directory in object_dirs.items():
        records = []
        base = root / "real_handoff" / directory / "low"
        for path in sorted(base.glob("*/commissioning_bundle.json")):
            try:
                bundle = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError):
                continue
            profiles = bundle.get("profiles", {})
            expected_stages = ("empty_g1", "throw_only", "object")
            if (
                bundle.get("schema") != "xarm6_object_commissioning_bundle_v1"
                or bundle.get("object_key") != object_key
                or tuple(profiles) != expected_stages
            ):
                continue
            profile_paths = [root / profiles[stage] for stage in expected_stages]
            if not all(profile_path.is_file() for profile_path in profile_paths):
                continue
            records.append(
                {
                    "label": bundle.get("label", path.parent.name),
                    "bundle": str(path.relative_to(root)),
                    "profiles": profiles,
                }
            )
        result[object_key] = {
            "ready": bool(records),
            "latest": records[-1] if records else None,
            "bundles": records,
        }
    return result


def build_report(
    root: Path = ROOT,
    hardware_path: Path = DEFAULT_HARDWARE,
    package_resolver: Callable[[str], str] = metadata.version,
) -> dict:
    packages = check_packages(package_resolver)
    hardware = check_hardware_config(hardware_path)
    handoff_module = _load_handoff_checker()
    handoff = handoff_module.build_report(root)

    calibration = {
        row["key"]: row["baseline"]["g1_calibration"]["complete"]
        for row in handoff["objects"]
    }
    commissioning = discover_commissioning_bundles(root)
    staged_ready = {
        key: complete or commissioning.get(key, {}).get("ready", False)
        for key, complete in calibration.items()
    }
    files_ready = all(
        profile["plan_only_verified"] and profile["joint_envelope_pass"]
        for row in handoff["objects"]
        for profile in (row["baseline"], row["next_pose"])
    )
    software_ready = sys.version_info >= (3, 10) and all(
        package["ok"] for package in packages
    )
    configuration_ready = bool(hardware["configuration_valid"])
    environment_ready = software_ready and configuration_ready
    return {
        "schema": "xarm6_four_object_environment_preflight_v1",
        "offline_only": True,
        "robot_connection_attempted": False,
        "python": {
            "version": ".".join(str(value) for value in sys.version_info[:3]),
            "requirement": ">=3.10",
            "ok": sys.version_info >= (3, 10),
        },
        "packages": packages,
        "hardware": hardware,
        "handoff": {
            "files_and_joint_envelopes_ready": files_ready,
            "g1_calibration_complete": calibration,
            "commissioning_bundles": commissioning,
            "staged_execution_ready": staged_ready,
            "o0_staged_profile_ready": calibration.get("O0", False),
            "objects_awaiting_g1_calibration": [
                key for key, ready in staged_ready.items() if not ready
            ],
        },
        "environment_ready": environment_ready,
        "ready_for_o0_staged_execution": (
            environment_ready
            and hardware["motion_confirmed"]
            and files_ready
            and calibration.get("O0", False)
        ),
        "next_action": (
            "run the staged O0 baseline with the onsite operator"
            if environment_ready and calibration.get("O0", False)
            else "fix failed environment checks before any hardware command"
        ),
    }


def render_summary(report: dict) -> str:
    lines = [
        "xArm6 four-object offline environment preflight",
        "No robot connection or command was attempted.",
    ]
    python = report["python"]
    lines.append(
        f"[{'PASS' if python['ok'] else 'FAIL'}] Python {python['version']} "
        f"(required {python['requirement']})"
    )
    for package in report["packages"]:
        installed = package["installed"] or "MISSING"
        lines.append(
            f"[{'PASS' if package['ok'] else 'FAIL'}] {package['name']} "
            f"{installed} (required {package['requirement']})"
        )
    hardware = report["hardware"]
    lines.append(
        f"[{'PASS' if hardware['configuration_valid'] else 'FAIL'}] "
        f"hardware config: {hardware['path']}"
    )
    for problem in hardware["problems"]:
        lines.append(f"  - {problem}")
    handoff = report["handoff"]
    lines.append(
        f"[{'PASS' if handoff['files_and_joint_envelopes_ready'] else 'FAIL'}] "
        "four-object profiles, timelines, and joint envelopes"
    )
    for key, ready in handoff["staged_execution_ready"].items():
        canonical = handoff["g1_calibration_complete"][key]
        latest = handoff["commissioning_bundles"].get(key, {}).get("latest")
        if canonical:
            state = "ready"
        elif latest is not None:
            state = f"ready via commissioning bundle {latest['label']}"
        else:
            state = "onsite calibration required"
        lines.append(f"[{'PASS' if ready else 'WAIT'}] {key} G1: {state}")
    lines.append(
        "RESULT: "
        + ("environment ready" if report["environment_ready"] else "environment not ready")
    )
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--hardware-config", type=Path, default=DEFAULT_HARDWARE)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    report = build_report(hardware_path=args.hardware_config.resolve())
    print(json.dumps(report, indent=2) if args.json else render_summary(report), end="")
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(f"saved offline preflight report: {args.output}")
    return 0 if report["environment_ready"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
