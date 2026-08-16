#!/usr/bin/env python3
"""Read-only search for a J2/J3/J5 throw whose tool-z turns upward."""

import math

import numpy as np
from scipy.optimize import least_squares

import _bootstrap  # noqa: F401
from real_cube_demo.config import load_probe_plan
from real_cube_demo.local_kinematics import LocalXArmFK
from real_cube_demo.spin_toss import (
    _evaluate,
    _quintic_coefficients,
    numerical_jacobian,
    pose_matrix,
)


ACTIVE = np.asarray([1, 2, 4])  # J2, J3, J5
CONTROL_PERIOD_S = 0.02
FOLLOWTHROUGH_S = 0.16


def features(fk, joint):
    transform = pose_matrix(fk.forward_kinematics(tuple(joint)))
    xyz = transform[:3, 3]
    tool_z = transform[:3, 2]
    elevation = math.asin(float(tool_z[2]))
    return xyz, tool_z, elevation


def solve_pose(fk, fixed_q, target, seeds, selection_reference):
    target_x, target_z, target_elevation = target

    def residual(active_q):
        joint = fixed_q.copy()
        joint[ACTIVE] = active_q
        xyz, _, elevation = features(fk, joint)
        return np.asarray(
            [
                (xyz[0] - target_x) / 0.10,
                (xyz[2] - target_z) / 0.10,
                (elevation - target_elevation) / 0.50,
            ]
        )

    solutions = []
    for seed in seeds:
        result = least_squares(
            residual,
            np.asarray(seed)[ACTIVE],
            bounds=(
                np.asarray([-1.90, -3.90, -1.69]),
                np.asarray([2.09, 0.19, 3.05]),
            ),
        )
        joint = fixed_q.copy()
        joint[ACTIVE] = result.x
        error = float(np.linalg.norm(residual(result.x)))
        if error <= 0.02:
            solutions.append(joint)
    if not solutions:
        return None
    return min(
        solutions,
        key=lambda joint: float(np.sum(np.abs(joint - selection_reference))),
    )


def elevation_gradient(fk, joint, epsilon=1e-4):
    _, _, base = features(fk, joint)
    gradient = np.zeros(6)
    for index in ACTIVE:
        perturbed = joint.copy()
        perturbed[index] += epsilon
        _, _, moved = features(fk, perturbed)
        gradient[index] = (moved - base) / epsilon
    return gradient


def release_velocity(fk, release_q, elevation_rate_rad_s):
    jacobian = numerical_jacobian(tuple(release_q), fk.forward_kinematics)
    elevation_jacobian = elevation_gradient(fk, release_q)
    task = np.vstack(
        (
            jacobian[0, ACTIVE],
            jacobian[2, ACTIVE],
            elevation_jacobian[ACTIVE],
        )
    )
    active_qd = np.linalg.lstsq(
        task,
        np.asarray([0.18, 0.40, elevation_rate_rad_s]),
        rcond=None,
    )[0]
    qd = np.zeros(6)
    qd[ACTIVE] = active_qd
    return qd, jacobian @ qd, float(elevation_jacobian @ qd)


def build_motion(fk, start_q, release_q, release_qd, duration_s):
    zeros = np.zeros(6)
    throw_coefficients = _quintic_coefficients(
        start_q,
        zeros,
        zeros,
        release_q,
        release_qd,
        zeros,
        duration_s,
    )
    follow_q = release_q + 0.5 * FOLLOWTHROUGH_S * release_qd
    follow_coefficients = _quintic_coefficients(
        release_q,
        release_qd,
        zeros,
        follow_q,
        zeros,
        zeros,
        FOLLOWTHROUGH_S,
    )

    states = []
    for time_s in np.linspace(0.0, duration_s, round(duration_s / 0.02) + 1):
        states.append(_evaluate(throw_coefficients, time_s))
    for local_time in np.linspace(
        CONTROL_PERIOD_S,
        FOLLOWTHROUGH_S,
        round(FOLLOWTHROUGH_S / CONTROL_PERIOD_S),
    ):
        states.append(_evaluate(follow_coefficients, local_time))

    joints = np.asarray([state[0] for state in states])
    velocities = np.asarray([state[1] for state in states])
    accelerations = np.asarray([state[2] for state in states])
    xyz = []
    elevations = []
    tool_z = []
    for joint in joints:
        point, direction, elevation = features(fk, joint)
        xyz.append(point)
        tool_z.append(direction)
        elevations.append(elevation)
    return {
        "follow_q": follow_q,
        "joints": joints,
        "xyz": np.asarray(xyz),
        "tool_z": np.asarray(tool_z),
        "elevations": np.asarray(elevations),
        "max_qdot": float(np.max(np.abs(velocities))),
        "max_qdd": float(np.max(np.abs(accelerations))),
        "joint_path": float(np.sum(np.abs(np.diff(joints, axis=0)))),
        "max_swing": float(np.max(np.ptp(joints, axis=0))),
    }


def main() -> None:
    fk = LocalXArmFK()
    handoff_q = np.asarray(load_probe_plan().center_joint_rad, dtype=float)
    alternative_seeds = (
        handoff_q,
        np.asarray([0.061, -0.25, -1.55, 0.023, 2.5, 0.332]),
        np.asarray([0.061, -0.10, -1.60, 0.023, 2.65, 0.332]),
        np.asarray([0.061, 0.0, -1.5, 0.023, 2.7, 0.332]),
    )
    start_targets = (
        (0.14, 0.30, math.radians(-30.0)),
        (0.16, 0.33, math.radians(-25.0)),
        (0.18, 0.34, math.radians(-20.0)),
        (0.22, 0.36, math.radians(-15.0)),
    )
    release_offsets = (
        (0.04, 0.10, math.radians(15.0)),
        (0.06, 0.12, math.radians(10.0)),
        (0.08, 0.12, math.radians(5.0)),
    )
    candidates = []
    pose_records = []
    dynamic_attempts = []
    pose_pairs = 0
    positive_j5_pairs = 0
    positive_j5_velocity = 0

    for start_target in start_targets:
        start_q = solve_pose(
            fk,
            handoff_q,
            start_target,
            alternative_seeds,
            handoff_q,
        )
        if start_q is None:
            continue
        for delta_x, delta_z, release_elevation in release_offsets:
            release_target = (
                start_target[0] + delta_x,
                start_target[1] + delta_z,
                release_elevation,
            )
            release_q = solve_pose(
                fk,
                handoff_q,
                release_target,
                (start_q,) + alternative_seeds,
                start_q,
            )
            if release_q is None:
                continue
            pose_pairs += 1
            _, _, start_elevation = features(fk, start_q)
            _, _, release_elevation_actual = features(fk, release_q)
            pose_records.append(
                {
                    "start_q": start_q.copy(),
                    "release_q": release_q.copy(),
                    "start_elevation": start_elevation,
                    "release_elevation": release_elevation_actual,
                }
            )
            if release_q[4] - start_q[4] < 0.10:
                continue
            positive_j5_pairs += 1
            for elevation_rate in (0.35, 0.5, 0.8):
                release_qd, release_twist, actual_elevation_rate = release_velocity(
                    fk, release_q, elevation_rate
                )
                if release_qd[4] <= 0.10:
                    continue
                positive_j5_velocity += 1
                for duration_s in (0.50, 0.60, 0.70):
                    motion = build_motion(
                        fk, start_q, release_q, release_qd, duration_s
                    )
                    xyz = motion["xyz"]
                    elevations = motion["elevations"]
                    release_index = round(duration_s / CONTROL_PERIOD_S)
                    dynamic_attempts.append(
                        {
                            "start_elevation": elevations[0],
                            "release_elevation": elevations[release_index],
                            "follow_elevation": elevations[-1],
                            "max_qdot": motion["max_qdot"],
                            "max_qdd": motion["max_qdd"],
                            "min_z": float(np.min(xyz[:, 2])),
                            "z_monotonic": bool(
                                np.all(np.diff(xyz[:, 2]) >= -0.002)
                            ),
                            "elevation_monotonic": bool(
                                np.all(
                                    np.diff(elevations) >= -math.radians(1.0)
                                )
                            ),
                            "start_q": start_q.copy(),
                            "release_q": release_q.copy(),
                            "release_qd": release_qd.copy(),
                        }
                    )
                    if not (
                        motion["max_qdot"] <= 3.10
                        and motion["max_qdd"] <= 20.0
                        and np.min(xyz[:, 2]) >= 0.25
                        and np.all(np.diff(xyz[:, 2]) >= -0.002)
                        and np.all(np.diff(elevations) >= -math.radians(1.0))
                        and elevations[release_index] >= math.radians(10.0)
                        and elevations[-1] > 0.0
                        and np.max(motion["joints"][:, 4]) <= 3.05
                    ):
                        continue
                    candidates.append(
                        {
                            "start_q": start_q.copy(),
                            "release_q": release_q.copy(),
                            "release_qd": release_qd.copy(),
                            "release_twist": release_twist.copy(),
                            "elevation_rate": actual_elevation_rate,
                            "duration_s": duration_s,
                            "setup_path": float(np.sum(np.abs(start_q - handoff_q))),
                            **motion,
                        }
                    )

    candidates.sort(
        key=lambda item: (
            item["setup_path"] + item["joint_path"],
            item["release_q"][4],
        )
    )
    print(
        "read-only upward-EE search; active joints=J2/J3/J5; "
        "release tool-z elevation >= 10 deg"
    )
    print(
        f"pose pairs={pose_pairs}, positive J5 position={positive_j5_pairs}, "
        f"positive J5 velocity={positive_j5_velocity}, "
        f"feasible trajectories={len(candidates)}"
    )
    if not positive_j5_pairs:
        print("pose solutions:")
        for record in pose_records:
            print(
                "  elevation "
                f"{math.degrees(record['start_elevation']):.1f} -> "
                f"{math.degrees(record['release_elevation']):.1f} deg, "
                f"J5 {record['start_q'][4]:.3f} -> "
                f"{record['release_q'][4]:.3f}"
            )
    elif not candidates:
        dynamic_attempts.sort(
            key=lambda item: max(
                item["max_qdot"] / 3.10, item["max_qdd"] / 20.0
            )
        )
        print("closest dynamic attempts:")
        for attempt in dynamic_attempts[:8]:
            print(
                "  elevation "
                f"{math.degrees(attempt['start_elevation']):.1f} -> "
                f"{math.degrees(attempt['release_elevation']):.1f} -> "
                f"{math.degrees(attempt['follow_elevation']):.1f} deg, "
                f"qdot={attempt['max_qdot']:.2f}, "
                f"qdd={attempt['max_qdd']:.2f}, "
                f"min_z={1000 * attempt['min_z']:.0f} mm, "
                f"monotonic={attempt['z_monotonic']}/"
                f"{attempt['elevation_monotonic']}"
            )
    for index, candidate in enumerate(candidates[:12], start=1):
        release_index = round(candidate["duration_s"] / CONTROL_PERIOD_S)
        print(f"\n#{index}")
        print(
            f"  throw/follow duration={candidate['duration_s']:.2f}/"
            f"{FOLLOWTHROUGH_S:.2f} s"
        )
        print(
            f"  start q={[round(float(v), 4) for v in candidate['start_q']]}"
        )
        print(
            f"  release q={[round(float(v), 4) for v in candidate['release_q']]}"
        )
        print(
            "  TCP start -> release -> follow mm: "
            f"{[round(1000 * float(v), 1) for v in candidate['xyz'][0]]} -> "
            f"{[round(1000 * float(v), 1) for v in candidate['xyz'][release_index]]} -> "
            f"{[round(1000 * float(v), 1) for v in candidate['xyz'][-1]]}"
        )
        print(
            "  tool-z elevation start -> release -> follow: "
            f"{math.degrees(candidate['elevations'][0]):.1f} -> "
            f"{math.degrees(candidate['elevations'][release_index]):.1f} -> "
            f"{math.degrees(candidate['elevations'][-1]):.1f} deg"
        )
        print(
            "  tool-z release="
            f"{[round(float(v), 3) for v in candidate['tool_z'][release_index]]}"
        )
        print(
            "  release linear="
            f"{[round(float(v), 3) for v in candidate['release_twist'][:3]]} m/s, "
            f"angular={np.linalg.norm(candidate['release_twist'][3:]):.3f} rad/s, "
            f"elevation rate={candidate['elevation_rate']:.3f} rad/s"
        )
        print(
            f"  J5 start/release/velocity={candidate['start_q'][4]:.3f}/"
            f"{candidate['release_q'][4]:.3f}/"
            f"{candidate['release_qd'][4]:.3f}; "
            f"qdot={candidate['max_qdot']:.3f}, qdd={candidate['max_qdd']:.3f}"
        )
        print(
            f"  setup path={candidate['setup_path']:.3f}, "
            f"throw+follow path={candidate['joint_path']:.3f}, "
            f"max swing={candidate['max_swing']:.3f} rad"
        )


if __name__ == "__main__":
    main()
