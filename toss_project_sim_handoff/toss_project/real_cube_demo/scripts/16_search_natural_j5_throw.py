#!/usr/bin/env python3
"""Read-only search for a compact J5-led throw segment.

The search keeps J1/J4/J6 at the measured handoff posture.  J5 supplies the
main wrist rotation while J2/J3 assist the upward TCP velocity.  It sends no
motion or gripper command.
"""

import argparse
import math

import numpy as np
from scipy.optimize import lsq_linear

import _bootstrap  # noqa: F401
from real_cube_demo.config import load_probe_plan
from real_cube_demo.local_kinematics import LocalXArmFK
from real_cube_demo.spin_toss import (
    _evaluate,
    _quintic_coefficients,
    numerical_jacobian,
    pose_matrix,
)


ACTIVE_JOINTS = np.asarray([1, 2, 4])  # J2, J3, J5 (zero-based)
CONTROL_PERIOD_S = 0.02
TARGET_UPWARD_M_S = 0.55
TARGET_J5_RELEASE_SPEED_RAD_S = 1.20
MAX_JOINT_SPEED_RAD_S = 3.10
MAX_JOINT_ACCELERATION_RAD_S2 = 20.0
def restricted_release_velocity(
    robot, release_q, target_upward_m_s, target_j5_speed_rad_s
):
    """Use J5 for rotation and J2/J3 to assist upward translation."""
    jacobian = numerical_jacobian(release_q, robot.forward_kinematics)
    release_qd = np.zeros(6, dtype=float)
    release_qd[4] = target_j5_speed_rad_s
    joint_5_axis_world = jacobian[3:, 4]
    joint_5_axis_world /= np.linalg.norm(joint_5_axis_world)
    target_linear_velocity = np.asarray([0.0, 0.0, target_upward_m_s])
    target_twist = np.concatenate(
        (target_linear_velocity, joint_5_axis_world * target_j5_speed_rad_s)
    )
    residual_twist = target_twist - jacobian[:, 4] * target_j5_speed_rad_s
    weights = np.diag([3.0, 3.0, 3.0, 1.0, 1.0, 1.0])
    result = lsq_linear(
        weights @ jacobian[:, [1, 2]],
        weights @ residual_twist,
        bounds=(np.asarray([-2.8, -2.8]), np.asarray([2.8, 2.8])),
    )
    release_qd[[1, 2]] = result.x
    return release_qd, jacobian @ release_qd, joint_5_axis_world


def throw_segment(robot, release_q, release_qd, duration_s, displacement_scale):
    """Build a rest-to-release segment whose motion uses only J2/J3/J5."""
    start_q = release_q - displacement_scale * duration_s * release_qd
    coefficients = _quintic_coefficients(
        start_q,
        np.zeros(6),
        np.zeros(6),
        release_q,
        release_qd,
        np.zeros(6),
        duration_s,
    )
    steps = round(duration_s / CONTROL_PERIOD_S)
    states = [
        _evaluate(coefficients, step * duration_s / steps)
        for step in range(steps + 1)
    ]
    joints = np.asarray([state[0] for state in states])
    velocities = np.asarray([state[1] for state in states])
    accelerations = np.asarray([state[2] for state in states])
    tcp_xyz_m = np.asarray(
        [
            robot.forward_kinematics(tuple(float(value) for value in joint))[:3]
            for joint in joints
        ]
    ) / 1000.0
    tcp_speed = np.linalg.norm(
        np.diff(tcp_xyz_m, axis=0) / (duration_s / steps), axis=1
    )
    joint_path = float(np.sum(np.abs(np.diff(joints, axis=0))))
    joint_swing = np.max(joints, axis=0) - np.min(joints, axis=0)
    return {
        "start_q": start_q,
        "min_tcp_z_m": float(np.min(tcp_xyz_m[:, 2])),
        "start_tcp_xyz_m": tcp_xyz_m[0],
        "joint_path_rad": joint_path,
        "maximum_joint_swing_rad": float(np.max(joint_swing)),
        "max_joint_speed_rad_s": float(np.max(np.abs(velocities))),
        "max_joint_acceleration_rad_s2": float(np.max(np.abs(accelerations))),
        "max_tcp_speed_m_s": float(np.max(tcp_speed)),
        "j5_increase_rad": float(release_q[4] - start_q[4]),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--max-j5", type=float, default=2.80)
    parser.add_argument("--min-start-height-mm", type=float, default=380.0)
    parser.add_argument(
        "--upward-velocity", type=float, default=TARGET_UPWARD_M_S
    )
    parser.add_argument(
        "--j5-release-speed",
        type=float,
        default=TARGET_J5_RELEASE_SPEED_RAD_S,
    )
    parser.add_argument(
        "--sort-by", choices=("motion", "horizon"), default="motion"
    )
    args = parser.parse_args()
    handoff_q = np.asarray(load_probe_plan().center_joint_rad, dtype=float)
    wrist_camera_axis_in_tool = np.asarray(
        [-0.030640567026413445, -0.007355433409032735, 0.999503403321702]
    )
    candidates = []
    high_postures = []
    horizon_postures = []
    pose_candidates = 0
    segment_candidates = 0
    robot = LocalXArmFK()

    joint_5_count = round((args.max_j5 - 1.55) / 0.05) + 1
    for joint_2 in np.linspace(-1.40, 0.30, 35):
        for joint_3 in np.linspace(-1.80, 0.15, 40):
            for joint_5 in np.linspace(1.55, args.max_j5, joint_5_count):
                release_q = handoff_q.copy()
                release_q[1] = joint_2
                release_q[2] = joint_3
                release_q[4] = joint_5
                release_tuple = tuple(float(value) for value in release_q)
                release_pose = robot.forward_kinematics(release_tuple)
                release_transform = pose_matrix(release_pose)
                tool_z = release_transform[:3, 2]
                elevation_deg = math.degrees(math.asin(tool_z[2]))
                xyz = release_transform[:3, 3]
                if (
                    0.12 <= xyz[0] <= 0.50
                    and abs(xyz[1]) <= 0.10
                    and 0.35 <= xyz[2] <= 0.60
                ):
                    high_postures.append(
                        (abs(elevation_deg), release_q.copy(), xyz.copy(), elevation_deg)
                    )
                if (
                    0.12 <= xyz[0] <= 0.50
                    and abs(xyz[1]) <= 0.10
                    and -15.0 <= elevation_deg <= 35.0
                ):
                    horizon_postures.append(
                        (abs(xyz[2] - 0.44), release_q.copy(), xyz.copy(), elevation_deg)
                    )
                if not (
                    0.18 <= xyz[0] <= 0.45
                    and abs(xyz[1]) <= 0.10
                    and 0.40 <= xyz[2] <= 0.62
                    and -50.0 <= elevation_deg <= 15.0
                ):
                    continue
                pose_candidates += 1

                release_qd, actual_twist, spin_axis = restricted_release_velocity(
                    robot,
                    release_tuple,
                    args.upward_velocity,
                    args.j5_release_speed,
                )
                actual_spin = float(np.dot(actual_twist[3:], spin_axis))
                off_axis_spin = float(
                    np.linalg.norm(actual_twist[3:] - actual_spin * spin_axis)
                )
                camera_axis = release_transform[:3, :3] @ wrist_camera_axis_in_tool

                for duration_s in (0.44, 0.50, 0.56, 0.62):
                    for displacement_scale in (0.45, 0.55):
                        segment = throw_segment(
                            robot,
                            release_q,
                            release_qd,
                            duration_s,
                            displacement_scale,
                        )
                        segment_candidates += 1
                        if not (
                            segment["min_tcp_z_m"]
                            >= args.min_start_height_mm / 1000.0
                            and segment["max_joint_speed_rad_s"]
                            <= MAX_JOINT_SPEED_RAD_S
                            and segment["max_joint_acceleration_rad_s2"]
                            <= MAX_JOINT_ACCELERATION_RAD_S2
                            and 0.12 <= segment["j5_increase_rad"] <= 0.55
                        ):
                            continue
                        candidates.append(
                            {
                                "release_q": release_q.copy(),
                                "release_xyz_m": xyz.copy(),
                                "release_elevation_deg": elevation_deg,
                                "camera_elevation_deg": math.degrees(
                                    math.asin(camera_axis[2])
                                ),
                                "release_qd": release_qd.copy(),
                                "actual_twist": actual_twist.copy(),
                                "actual_spin_rad_s": actual_spin,
                                "off_axis_spin_rad_s": off_axis_spin,
                                "duration_s": duration_s,
                                "handoff_to_start_path_rad": float(
                                    np.sum(np.abs(segment["start_q"] - handoff_q))
                                ),
                                **segment,
                            }
                        )

    if args.sort_by == "horizon":
        candidates.sort(
            key=lambda item: (
                abs(item["release_elevation_deg"]),
                item["handoff_to_start_path_rad"] + item["joint_path_rad"],
            )
        )
    else:
        candidates.sort(
            key=lambda item: (
                item["handoff_to_start_path_rad"] + item["joint_path_rad"],
                abs(item["release_elevation_deg"]),
            )
        )
    print(
        "read-only natural throw search; active joints=J2/J3/J5, "
        "J1/J4/J6 fixed"
    )
    print(
        f"constraints: J5 <= {args.max_j5:.2f} rad, "
        f"start TCP z >= {args.min_start_height_mm:.0f} mm, "
        f"upward={args.upward_velocity:.2f} m/s, "
        f"J5 release speed={args.j5_release_speed:.2f} rad/s"
    )
    print(
        f"pose candidates={pose_candidates}, "
        f"throw segments={segment_candidates}, feasible={len(candidates)}"
    )
    if not pose_candidates:
        high_postures.sort(key=lambda item: item[0])
        horizon_postures.sort(key=lambda item: item[0])
        print("closest high postures (showing why the intersection is empty):")
        for _, joint, xyz, elevation in high_postures[:4]:
            print(
                f"  q={[round(value, 3) for value in joint]}, "
                f"TCP={[round(1000.0 * value, 1) for value in xyz]} mm, "
                f"elevation={elevation:.1f} deg"
            )
        print("closest near-horizontal postures:")
        for _, joint, xyz, elevation in horizon_postures[:4]:
            print(
                f"  q={[round(value, 3) for value in joint]}, "
                f"TCP={[round(1000.0 * value, 1) for value in xyz]} mm, "
                f"elevation={elevation:.1f} deg"
            )
    for index, candidate in enumerate(candidates[:12], start=1):
        print(
            f"\n#{index} release q="
            f"{[round(float(value), 4) for value in candidate['release_q']]}"
        )
        print(
            "  start q="
            f"{[round(float(value), 4) for value in candidate['start_q']]}"
        )
        print(
            "  TCP start -> release mm: "
            f"{[round(1000.0 * float(value), 1) for value in candidate['start_tcp_xyz_m']]}"
            " -> "
            f"{[round(1000.0 * float(value), 1) for value in candidate['release_xyz_m']]}"
        )
        print(
            f"  tool/camera elevation: {candidate['release_elevation_deg']:.1f}/"
            f"{candidate['camera_elevation_deg']:.1f} deg; "
            f"J5 increase={candidate['j5_increase_rad']:.3f} rad"
        )
        twist = candidate["actual_twist"]
        print(
            "  actual release linear="
            f"{[round(float(value), 3) for value in twist[:3]]} m/s, "
            f"spin={candidate['actual_spin_rad_s']:.3f} rad/s, "
            f"off-axis={candidate['off_axis_spin_rad_s']:.3f} rad/s"
        )
        print(
            "  release qd="
            f"{[round(float(value), 3) for value in candidate['release_qd']]} rad/s; "
            f"duration={candidate['duration_s']:.2f} s"
        )
        print(
            f"  motion: path={candidate['joint_path_rad']:.3f} rad, "
            f"handoff-to-start={candidate['handoff_to_start_path_rad']:.3f} rad, "
            f"max swing={candidate['maximum_joint_swing_rad']:.3f} rad, "
            f"qdot={candidate['max_joint_speed_rad_s']:.3f}, "
            f"qdd={candidate['max_joint_acceleration_rad_s2']:.3f}, "
            f"TCP={candidate['max_tcp_speed_m_s']:.3f} m/s, "
            f"min z={1000.0 * candidate['min_tcp_z_m']:.1f} mm"
        )


if __name__ == "__main__":
    main()
