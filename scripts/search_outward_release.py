#!/usr/bin/env python3
"""Search an outward xArm6 release state under the current real limits."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import least_squares, minimize


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from xarm6_toss_sim import URDFKinematics, load_real_setup  # noqa: E402


def solve_pose(kinematics, seed, target_position, target_axis, lower, upper):
    def residual(joint):
        transform = kinematics.forward(joint)
        position_error = (transform[:3, 3] - target_position) / 0.025
        axis_error = (transform[:3, 2] - target_axis) / 0.12
        regularization = 0.015 * (joint - seed)
        return np.concatenate((position_error, axis_error, regularization))

    result = least_squares(residual, seed, bounds=(lower, upper), max_nfev=300)
    transform = kinematics.forward(result.x)
    position_error = float(np.linalg.norm(transform[:3, 3] - target_position))
    axis_error = float(np.linalg.norm(transform[:3, 2] - target_axis))
    if position_error > 0.015 or axis_error > 0.15:
        return None
    return result.x


def release_velocity(kinematics, joint, speed_limit):
    jacobian = kinematics.jacobian(joint)

    def objective(velocity):
        linear = jacobian[:3] @ velocity
        angular = jacobian[3:] @ velocity
        return (
            -8.0 * linear[2]
            + 5.0 * linear[1] ** 2
            + 1.5 * linear[0] ** 2
            + 0.05 * float(np.dot(angular, angular))
            + 0.01 * float(np.dot(velocity, velocity))
        )

    result = minimize(
        objective,
        np.zeros(6),
        method="SLSQP",
        bounds=[(-speed_limit, speed_limit)] * 6,
        options={"maxiter": 500, "ftol": 1.0e-12},
    )
    return result.x, jacobian @ result.x


def project_release(setup, kinematics, joint):
    tcp = kinematics.forward(joint, "link_tcp")[:3, 3]
    base_from_eef = kinematics.forward(joint, "link_eef")
    base_from_wrist = base_from_eef @ setup.wrist.mount_from_camera
    wrist_point = np.linalg.inv(base_from_wrist) @ np.append(tcp, 1.0)
    wrist_projection = None
    if wrist_point[2] > 0.0:
        pixel = setup.wrist.intrinsic @ wrist_point[:3]
        u = float(pixel[0] / pixel[2])
        v = float(pixel[1] / pixel[2])
        if 0.0 <= u < setup.wrist.width and 0.0 <= v < setup.wrist.height:
            wrist_projection = [u, v, float(wrist_point[2])]
    third_projection = setup.third_view.project(tcp)
    return {
        "wrist": wrist_projection,
        "third_view": None if third_projection is None else list(third_projection),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=ROOT / "outputs" / "planning" / "outward_release.json")
    parser.add_argument("--seed", type=int, default=20260816)
    args = parser.parse_args()

    setup = load_real_setup()
    kinematics = URDFKinematics(setup.urdf_path)
    lower, upper = kinematics.arm_limits
    handoff = json.loads(
        (setup.source_root / "real_cube_demo/configs/handoff_place.json").read_text(encoding="utf-8")
    )
    recorded_seed = np.asarray(handoff["preplace_joint_rad"], dtype=float)
    rng = np.random.default_rng(args.seed)
    seeds = [recorded_seed]
    for _ in range(14):
        candidate = recorded_seed + rng.normal(0.0, [0.2, 0.3, 0.3, 0.3, 0.35, 0.3])
        seeds.append(np.clip(candidate, lower + 0.02, upper - 0.02))

    targets = []
    for x in (0.50, 0.56, 0.59):
        for y in (-0.04, 0.0):
            for z in (0.28, 0.34, 0.40):
                for elevation_deg in (-85.0, -60.0, -30.0, 10.0):
                    radial = np.asarray([x, y], dtype=float)
                    radial /= np.linalg.norm(radial)
                    elevation = np.deg2rad(elevation_deg)
                    axis = np.asarray(
                        [radial[0] * np.cos(elevation), radial[1] * np.cos(elevation), np.sin(elevation)]
                    )
                    targets.append((np.asarray([x, y, z]), axis))

    records = []
    for target_position, target_axis in targets:
        for seed in seeds:
            joint = solve_pose(kinematics, seed, target_position, target_axis, lower, upper)
            if joint is None:
                continue
            metrics = kinematics.radial_metrics(joint)
            if metrics["tcp_horizontal_radius_m"] < setup.acceptance["minimum_tcp_radius_m"]:
                continue
            if metrics["outward_dot_m"] <= 0.0:
                continue
            qd, twist = release_velocity(kinematics, joint, setup.max_joint_speed_rad_s)
            visibility = project_release(setup, kinematics, joint)
            record = {
                "joint_rad": joint.tolist(),
                "release_joint_velocity_rad_s": qd.tolist(),
                "release_twist": twist.tolist(),
                "metrics": metrics,
                "visibility": visibility,
                "joint_margin_rad": np.minimum(joint - lower, upper - joint).tolist(),
            }
            record["score"] = float(
                5.0 * twist[2]
                + (0.25 if visibility["third_view"] is not None else 0.0)
                + 0.2 * metrics["outward_dot_m"]
                - 0.03 * np.linalg.norm(joint - recorded_seed)
            )
            records.append(record)
            break

    records.sort(key=lambda record: record["score"], reverse=True)
    unique = []
    for record in records:
        joint = np.asarray(record["joint_rad"])
        if any(np.linalg.norm(joint - np.asarray(existing["joint_rad"])) < 0.05 for existing in unique):
            continue
        unique.append(record)
        if len(unique) == 20:
            break
    if not unique:
        raise RuntimeError("no outward release state satisfies geometry and wrist visibility")

    payload = {
        "schema": "xarm6_outward_release_search_v1",
        "real_limits": {
            "control_period_s": setup.control_period_s,
            "max_joint_speed_rad_s": setup.max_joint_speed_rad_s,
            "max_joint_acceleration_rad_s2": setup.max_joint_acceleration_rad_s2,
        },
        "candidate_count": len(unique),
        "candidates": unique,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    best = unique[0]
    print(json.dumps({"output": str(args.output), "best": best}, indent=2))


if __name__ == "__main__":
    main()
