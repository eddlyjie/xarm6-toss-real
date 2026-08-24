#!/usr/bin/env python3
"""Generate and rank angle-conditioned J2/J3/J5 release candidates."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import build_pose_rotation_ladder as ladder  # noqa: E402
from xarm6_toss_sim.kinematics import URDFKinematics  # noqa: E402

GRAVITY_M_S2 = 9.81
DYNAMIC_INDICES = (1, 2, 4)
FIXED_INDICES = (0, 3, 5)


def load_retention_calibration(paths: list[Path]) -> dict:
    observations = []
    for path in paths:
        summary = json.loads(path.read_text(encoding="utf-8"))
        transfer = summary["release_transfer_evidence"]
        hand = abs(float(transfer["hand_detach_axis_omega_rad_s"]))
        cube = abs(float(transfer["cube_postdetach_axis_omega_rad_s"]))
        if hand <= 1.0e-6 or cube <= 0.0:
            raise ValueError(f"invalid release transfer evidence: {path}")
        observations.append(
            {
                "path": str(path),
                "hand_axis_omega_rad_s": hand,
                "cube_postdetach_axis_omega_rad_s": cube,
                "retention": cube / hand,
                "measured_rotation_deg": float(summary["free_flight_rotation_deg"]),
                "catch_stable": bool(summary["catch_stable"]),
            }
        )
    if not observations:
        raise ValueError("at least one calibration summary is required")
    retention = float(np.median([row["retention"] for row in observations]))
    return {"angular_retention": retention, "observations": observations}


def candidate_from_state(
    kinematics: URDFKinematics,
    pose_offset_rad: np.ndarray,
    dynamic_velocity_rad_s: np.ndarray,
    desired_angle_deg: float,
    angular_retention: float,
) -> dict | None:
    q = ladder.MEASURED_V62_DETACH_Q_RAD + pose_offset_rad
    dq = np.zeros(6, dtype=float)
    dq[list(DYNAMIC_INDICES)] = dynamic_velocity_rad_s
    if not (
        dq[1] <= 0.0 and dq[2] <= 0.0 and dq[4] >= 0.0
        and np.all(dq[list(FIXED_INDICES)] == 0.0)
    ):
        return None
    transform = kinematics.forward(q)
    release_height_m = float(transform[2, 3])
    if release_height_m < 0.40:
        return None
    jacobian = kinematics.jacobian(q)
    twist = jacobian @ dq
    tumble_axis = ladder.base.tumble_axis(transform)
    angular = twist[3:]
    axis_omega = float(np.dot(angular, tumble_axis))
    angular_norm = float(np.linalg.norm(angular))
    if axis_omega <= 0.0 or angular_norm <= 1.0e-9:
        return None
    axis_alignment = abs(axis_omega) / angular_norm
    cube_com_velocity = twist[:3] + np.cross(
        angular, ladder.MEASURED_CUBE_OFFSET_WORLD_M
    )
    lateral_speed = float(np.linalg.norm(cube_com_velocity[:2]))
    tcp_speed = float(np.linalg.norm(twist[:3]))
    vertical_speed = float(cube_com_velocity[2])
    if not (
        axis_alignment >= 0.92
        and 0.90 <= vertical_speed <= 2.20
        and lateral_speed <= 0.45
        and tcp_speed <= 1.60
    ):
        return None
    predicted_flight_s = 2.0 * vertical_speed / GRAVITY_M_S2
    predicted_angle_deg = math.degrees(
        angular_retention * axis_omega * predicted_flight_s
    )
    score_terms = {
        "angle_error_deg": abs(predicted_angle_deg - desired_angle_deg),
        "lateral_penalty": 4.0 * lateral_speed,
        "height_penalty": 2.0 * abs(release_height_m - 0.42),
        "motion_penalty": 0.05 * float(np.linalg.norm(dynamic_velocity_rad_s)),
    }
    score = float(sum(score_terms.values()))
    return {
        "desired_angle_deg": desired_angle_deg,
        "predicted_rotation_deg": predicted_angle_deg,
        "predicted_free_flight_s": predicted_flight_s,
        "release_pose_offset_rad": pose_offset_rad.tolist(),
        "release_joint_velocity_rad_s": dq.tolist(),
        "release_height_m": release_height_m,
        "predicted_tumble_axis_world": tumble_axis.tolist(),
        "predicted_hand_axis_omega_rad_s": axis_omega,
        "predicted_axis_alignment": axis_alignment,
        "predicted_tcp_speed_m_s": tcp_speed,
        "predicted_cube_com_velocity_m_s": cube_com_velocity.tolist(),
        "j_score": score,
        "j_breakdown": score_terms,
    }


def search_angle(
    kinematics: URDFKinematics,
    desired_angle_deg: float,
    angular_retention: float,
    *,
    pose_offsets: np.ndarray,
    velocity_values: np.ndarray,
    top_k: int,
) -> dict:
    candidates = []
    negative = velocity_values[velocity_values <= 0.0]
    positive = velocity_values[velocity_values >= 0.0]
    for offset_j2 in pose_offsets:
        for offset_j3 in pose_offsets:
            pose_offset = np.zeros(6, dtype=float)
            pose_offset[1] = offset_j2
            pose_offset[2] = offset_j3
            for velocity_j2 in negative:
                for velocity_j3 in negative:
                    for velocity_j5 in positive:
                        candidate = candidate_from_state(
                            kinematics,
                            pose_offset,
                            np.asarray([velocity_j2, velocity_j3, velocity_j5]),
                            desired_angle_deg,
                            angular_retention,
                        )
                        if candidate is not None:
                            candidates.append(candidate)
    if not candidates:
        raise RuntimeError(f"no mechanically admitted candidate for {desired_angle_deg:g} deg")
    candidates.sort(
        key=lambda row: (
            row["j_score"],
            row["j_breakdown"]["angle_error_deg"],
            row["release_pose_offset_rad"],
            row["release_joint_velocity_rad_s"],
        )
    )
    maximum_predicted_angle = max(row["predicted_rotation_deg"] for row in candidates)
    return {
        "desired_angle_deg": desired_angle_deg,
        "target_reachable_in_search": maximum_predicted_angle >= desired_angle_deg - 1.0,
        "maximum_predicted_rotation_deg": maximum_predicted_angle,
        "candidate_count": len(candidates),
        "selected": candidates[0],
        "top_candidates": candidates[:top_k],
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--desired-angle-deg", type=float, nargs="+", required=True)
    parser.add_argument(
        "--calibration-summary", type=Path, action="append", required=True
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--top-k", type=int, default=8)
    parser.add_argument("--pose-grid-count", type=int, default=21)
    parser.add_argument("--velocity-grid-count", type=int, default=19)
    args = parser.parse_args()
    if any(angle <= 0.0 for angle in args.desired_angle_deg):
        parser.error("desired angles must be positive")
    if args.top_k < 1 or args.pose_grid_count < 3 or args.velocity_grid_count < 3:
        parser.error("top-k and grid counts are too small")

    calibration = load_retention_calibration(args.calibration_summary)
    kinematics = URDFKinematics(ladder.base.URDF)
    pose_offsets = np.linspace(-0.35, 0.05, args.pose_grid_count)
    velocity_values = np.linspace(-1.7448, 1.7448, args.velocity_grid_count)
    searches = [
        search_angle(
            kinematics,
            angle,
            calibration["angular_retention"],
            pose_offsets=pose_offsets,
            velocity_values=velocity_values,
            top_k=args.top_k,
        )
        for angle in args.desired_angle_deg
    ]
    report = {
        "schema": "xarm6_pose_conditioned_j235_search_v1",
        "dynamic_joint_indices_zero_based": list(DYNAMIC_INDICES),
        "fixed_joint_indices_zero_based": list(FIXED_INDICES),
        "desired_angles_deg": args.desired_angle_deg,
        "calibrated_detach_model": calibration,
        "search_space": {
            "pose_offset_j2_j3_rad": [-0.35, 0.05],
            "pose_grid_count": args.pose_grid_count,
            "joint_velocity_rad_s": [-1.7448, 1.7448],
            "velocity_grid_count": args.velocity_grid_count,
            "minimum_release_height_m": 0.40,
            "maximum_tcp_speed_m_s": 1.60,
            "maximum_cube_com_lateral_speed_m_s": 0.45,
            "minimum_axis_alignment": 0.92,
        },
        "ranking": {
            "name": "J",
            "terms": [
                "angle_error_deg",
                "lateral_penalty",
                "height_penalty",
                "motion_penalty",
            ],
        },
        "results": searches,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
