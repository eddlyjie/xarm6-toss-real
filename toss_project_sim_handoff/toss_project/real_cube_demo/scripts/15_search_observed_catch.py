#!/usr/bin/env python3
"""Read-only upward-facing catch search using an observed grasp offset."""

import argparse
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

import _bootstrap  # noqa: F401
from real_cube_demo.config import DEMO_ROOT, load_hardware
from real_cube_demo.robot import PickPlaceRobot
from real_cube_demo.spin_toss import (
    _cubic_coefficients,
    _evaluate,
    load_spin_toss_spec,
    matrix_pose,
    numerical_jacobian,
    pose_matrix,
)


def _trajectory_metrics(robot, spec, coefficients, flight_time_s):
    steps = round(flight_time_s / spec.control_period_s)
    samples = [
        _evaluate(coefficients, step * spec.control_period_s)
        for step in range(steps + 1)
    ]
    velocities = np.asarray([sample[1] for sample in samples])
    accelerations = np.asarray([sample[2] for sample in samples])
    tcp_positions = np.asarray(
        [
            robot.forward_kinematics(
                tuple(float(value) for value in sample[0])
            )[:3]
            for sample in samples
        ]
    ) / 1000.0
    return (
        float(np.max(np.abs(velocities))),
        float(np.max(np.abs(accelerations))),
        float(
            np.max(
                np.linalg.norm(
                    np.diff(tcp_positions, axis=0) / spec.control_period_s,
                    axis=1,
                )
            )
        ),
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", type=Path)
    parser.add_argument("--broad-pose-search", action="store_true")
    parser.add_argument("--fine-pose-search", action="store_true")
    parser.add_argument("--final-search", action="store_true")
    args = parser.parse_args()
    run_dir = args.run or sorted(
        (DEMO_ROOT / "outputs" / "spin_toss").glob("*_cube")
    )[-1]
    analysis = json.loads(
        (run_dir / "cube_motion_analysis.json").read_text(encoding="utf-8")
    )
    run_summary = json.loads(
        (run_dir / "summary.json").read_text(encoding="utf-8")
    )
    observed_release_joint_rad = tuple(
        float(value) for value in run_summary["release_joint_rad"]
    )
    grasp_offset_tool = np.asarray(
        analysis["baseline_relative_tool_m"], dtype=float
    )

    hardware = load_hardware()
    spec = load_spin_toss_spec()
    gravity = np.asarray([0.0, 0.0, -9.81])
    candidates = []
    release_ik_failures = 0
    catch_ik_failures = 0

    if args.final_search:
        release_tilts_deg = (145.0, 150.0, 155.0)
        release_x_positions_m = (0.58,)
        release_z_positions_m = (0.44,)
        upward_velocities_m_s = (0.50, 0.55, 0.60, 0.65)
        spin_rates_rad_s = (1.2, 1.4, 1.6, 1.8)
        flight_times_s = (0.16, 0.18, 0.20)
        orientation_follows = (0.0, 0.25, 0.50)
        linear_velocity_matches = (0.40, 0.50, 0.60, 0.70)
        angular_velocity_matches = (0.0, 0.25, 0.50)
    elif args.fine_pose_search:
        release_tilts_deg = (
            125.0,
            130.0,
            135.0,
            140.0,
            145.0,
            150.0,
            155.0,
            160.0,
            165.0,
        )
        release_x_positions_m = (0.46, 0.50, 0.54, 0.58)
        release_z_positions_m = (0.44, 0.48, 0.52)
        upward_velocities_m_s = (0.55,)
        spin_rates_rad_s = (1.2,)
        flight_times_s = (0.16,)
        orientation_follows = (0.0,)
        linear_velocity_matches = (0.50,)
        angular_velocity_matches = (0.0,)
    elif args.broad_pose_search:
        release_tilts_deg = (135.0, 150.0, 165.0)
        release_x_positions_m = (0.24, 0.30, 0.36, 0.42, 0.48)
        release_z_positions_m = (0.30, 0.36, 0.42, 0.48)
        upward_velocities_m_s = (0.55,)
        spin_rates_rad_s = (1.4, 1.8)
        flight_times_s = (0.16,)
        orientation_follows = (0.0,)
        linear_velocity_matches = (0.50, 0.75)
        angular_velocity_matches = (0.0,)
    else:
        release_tilts_deg = (150.0, 165.0)
        release_x_positions_m = (0.24, 0.30)
        release_z_positions_m = (0.34, 0.40)
        upward_velocities_m_s = (0.45, 0.55, 0.65)
        spin_rates_rad_s = (1.4, 1.8, 2.2)
        flight_times_s = (0.12, 0.14, 0.16, 0.18)
        orientation_follows = (0.0, 0.5)
        linear_velocity_matches = (0.25, 0.50, 0.75)
        angular_velocity_matches = (0.0, 0.25)

    with PickPlaceRobot(hardware) as robot:
        observed_release = pose_matrix(
            robot.forward_kinematics(observed_release_joint_rad)
        )
        reference_release_q = observed_release_joint_rad

        for release_tilt_deg in release_tilts_deg:
            release_rotation = (
                observed_release[:3, :3]
                @ Rotation.from_euler("y", release_tilt_deg, degrees=True).as_matrix()
            )
            for release_x_m in release_x_positions_m:
                for release_z_m in release_z_positions_m:
                    requested_release = np.eye(4)
                    requested_release[:3, :3] = release_rotation
                    requested_release[:3, 3] = (
                        release_x_m,
                        observed_release[1, 3],
                        release_z_m,
                    )
                    try:
                        release_q = np.asarray(
                            robot.inverse_kinematics(
                                matrix_pose(requested_release),
                                ref_joint_rad=reference_release_q,
                            )
                        )
                    except RuntimeError:
                        release_ik_failures += 1
                        continue

                    release_q_tuple = tuple(float(value) for value in release_q)
                    release_transform = pose_matrix(
                        robot.forward_kinematics(release_q_tuple)
                    )
                    release_jacobian = numerical_jacobian(
                        release_q_tuple, robot.forward_kinematics
                    )
                    spin_axis = release_transform[:3, 1]
                    tool_z_world = release_transform[:3, 2]
                    object_position_release = (
                        release_transform[:3, 3]
                        + release_transform[:3, :3] @ grasp_offset_tool
                    )
                    gripper_base_position = (
                        release_transform[:3, 3] - 0.172 * tool_z_world
                    )
                    base_below_cube_m = float(
                        object_position_release[2] - gripper_base_position[2]
                    )

                    for upward_velocity_m_s in upward_velocities_m_s:
                        object_linear_velocity = np.asarray(
                            [0.0, 0.0, upward_velocity_m_s]
                        )
                        for spin_rate_rad_s in spin_rates_rad_s:
                            object_angular_velocity = spin_axis * spin_rate_rad_s
                            release_twist = np.concatenate(
                                (object_linear_velocity, object_angular_velocity)
                            )
                            release_qd = (
                                np.linalg.pinv(release_jacobian, rcond=1e-3)
                                @ release_twist
                            )

                            for flight_time_s in flight_times_s:
                                object_position = (
                                    object_position_release
                                    + object_linear_velocity * flight_time_s
                                    + 0.5 * gravity * flight_time_s**2
                                )
                                object_velocity = (
                                    object_linear_velocity
                                    + gravity * flight_time_s
                                )
                                for orientation_follow in orientation_follows:
                                    catch_rotation = (
                                        Rotation.from_rotvec(
                                            object_angular_velocity
                                            * flight_time_s
                                            * orientation_follow
                                        ).as_matrix()
                                        @ release_transform[:3, :3]
                                    )
                                    catch_transform = np.eye(4)
                                    catch_transform[:3, :3] = catch_rotation
                                    catch_transform[:3, 3] = (
                                        object_position
                                        - catch_rotation @ grasp_offset_tool
                                    )
                                    try:
                                        catch_q = np.asarray(
                                            robot.inverse_kinematics(
                                                matrix_pose(catch_transform),
                                                ref_joint_rad=release_q_tuple,
                                            )
                                        )
                                    except RuntimeError:
                                        catch_ik_failures += 1
                                        continue
                                    catch_q_tuple = tuple(
                                        float(value) for value in catch_q
                                    )
                                    catch_jacobian = numerical_jacobian(
                                        catch_q_tuple,
                                        robot.forward_kinematics,
                                    )
                                    center_offset_world = (
                                        catch_rotation @ grasp_offset_tool
                                    )

                                    for linear_velocity_match in (
                                        linear_velocity_matches
                                    ):
                                        for angular_velocity_match in (
                                            angular_velocity_matches
                                        ):
                                            catch_tcp_angular_velocity = (
                                                object_angular_velocity
                                                * angular_velocity_match
                                            )
                                            catch_tcp_linear_velocity = (
                                                object_velocity
                                                * linear_velocity_match
                                                - np.cross(
                                                    catch_tcp_angular_velocity,
                                                    center_offset_world,
                                                )
                                            )
                                            catch_twist = np.concatenate(
                                                (
                                                    catch_tcp_linear_velocity,
                                                    catch_tcp_angular_velocity,
                                                )
                                            )
                                            catch_qd = (
                                                np.linalg.pinv(
                                                    catch_jacobian, rcond=1e-3
                                                )
                                                @ catch_twist
                                            )
                                            coefficients = _cubic_coefficients(
                                                release_q,
                                                release_qd,
                                                catch_q,
                                                catch_qd,
                                                flight_time_s,
                                            )
                                            (
                                                max_joint_speed,
                                                max_joint_acceleration,
                                                max_tcp_speed,
                                            ) = _trajectory_metrics(
                                                robot,
                                                spec,
                                                coefficients,
                                                flight_time_s,
                                            )
                                            gripper_center_velocity = (
                                                catch_tcp_linear_velocity
                                                + np.cross(
                                                    catch_tcp_angular_velocity,
                                                    center_offset_world,
                                                )
                                            )
                                            relative_linear_speed = float(
                                                np.linalg.norm(
                                                    object_velocity
                                                    - gripper_center_velocity
                                                )
                                            )
                                            arm_feasible = (
                                                max_joint_speed
                                                <= spec.max_joint_speed_rad_s
                                                and max_joint_acceleration
                                                <= spec.max_joint_acceleration_rad_s2
                                                and max_tcp_speed
                                                <= spec.max_tcp_speed_m_s
                                            )
                                            g1_520_timing_feasible = (
                                                flight_time_s >= 0.16
                                            )
                                            score = (
                                                max_joint_acceleration
                                                / spec.max_joint_acceleration_rad_s2
                                                + 0.4
                                                * max_joint_speed
                                                / spec.max_joint_speed_rad_s
                                                + 0.5 * relative_linear_speed
                                                + 0.15 * (1.0 - tool_z_world[2])
                                                - 0.1 * orientation_follow
                                            )
                                            candidates.append(
                                                {
                                                    "feasible": (
                                                        arm_feasible
                                                        and g1_520_timing_feasible
                                                    ),
                                                    "arm_feasible": arm_feasible,
                                                    "g1_520_timing_feasible": (
                                                        g1_520_timing_feasible
                                                    ),
                                                    "score": score,
                                                    "release_tilt_deg": (
                                                        release_tilt_deg
                                                    ),
                                                    "release_tcp_pose": list(
                                                        matrix_pose(
                                                            release_transform
                                                        )
                                                    ),
                                                    "release_joint_rad": (
                                                        release_q.tolist()
                                                    ),
                                                    "release_joint_velocity_rad_s": (
                                                        release_qd.tolist()
                                                    ),
                                                    "release_tool_z_world": (
                                                        tool_z_world.tolist()
                                                    ),
                                                    "base_below_cube_m": (
                                                        base_below_cube_m
                                                    ),
                                                    "upward_velocity_m_s": (
                                                        upward_velocity_m_s
                                                    ),
                                                    "spin_rate_rad_s": (
                                                        spin_rate_rad_s
                                                    ),
                                                    "flight_time_s": (
                                                        flight_time_s
                                                    ),
                                                    "orientation_follow": (
                                                        orientation_follow
                                                    ),
                                                    "linear_velocity_match": (
                                                        linear_velocity_match
                                                    ),
                                                    "angular_velocity_match": (
                                                        angular_velocity_match
                                                    ),
                                                    "rotation_rad": (
                                                        spin_rate_rad_s
                                                        * flight_time_s
                                                    ),
                                                    "catch_tcp_pose": list(
                                                        matrix_pose(
                                                            catch_transform
                                                        )
                                                    ),
                                                    "catch_joint_rad": (
                                                        catch_q.tolist()
                                                    ),
                                                    "catch_joint_velocity_rad_s": (
                                                        catch_qd.tolist()
                                                    ),
                                                    "max_joint_speed_rad_s": (
                                                        max_joint_speed
                                                    ),
                                                    "max_joint_acceleration_rad_s2": (
                                                        max_joint_acceleration
                                                    ),
                                                    "max_tcp_speed_m_s": (
                                                        max_tcp_speed
                                                    ),
                                                    "relative_linear_speed_m_s": (
                                                        relative_linear_speed
                                                    ),
                                                }
                                            )

    candidates.sort(
        key=lambda value: (
            not value["feasible"],
            not value["arm_feasible"],
            value["score"],
        )
    )
    feasible_count = sum(value["feasible"] for value in candidates)
    arm_feasible_count = sum(value["arm_feasible"] for value in candidates)
    print(
        "observed grasp offset tool: "
        f"{[round(1000.0 * value, 1) for value in grasp_offset_tool]} mm"
    )
    print(
        f"searched={len(candidates)}, release IK failures={release_ik_failures}, "
        f"catch IK failures={catch_ik_failures}, arm feasible={arm_feasible_count}, "
        f"arm plus G1-520 feasible={feasible_count}"
    )
    for value in candidates[:12]:
        print(
            f"{'OK' if value['feasible'] else ('ARM' if value['arm_feasible'] else 'NO')} "
            f"tilt={value['release_tilt_deg']:.0f}deg "
            f"xyz={[round(v, 1) for v in value['release_tcp_pose'][:3]]}mm "
            f"z-up={value['release_tool_z_world'][2]:.2f} "
            f"base-below={1000.0 * value['base_below_cube_m']:.0f}mm "
            f"up={value['upward_velocity_m_s']:.2f} "
            f"spin={value['spin_rate_rad_s']:.2f} "
            f"T={value['flight_time_s']:.2f} "
            f"orient={value['orientation_follow']:.2f} "
            f"linear={value['linear_velocity_match']:.2f} "
            f"angular={value['angular_velocity_match']:.2f} "
            f"rot={math.degrees(value['rotation_rad']):.1f}deg "
            f"qdot={value['max_joint_speed_rad_s']:.2f} "
            f"qdd={value['max_joint_acceleration_rad_s2']:.2f} "
            f"tcp={value['max_tcp_speed_m_s']:.2f} "
            f"relative={value['relative_linear_speed_m_s']:.2f}m/s"
        )
    output_name = (
        "upward_final_candidates.json"
        if args.final_search
        else (
            "upward_fine_pose_candidates.json"
            if args.fine_pose_search
            else (
                "upward_pose_candidates.json"
                if args.broad_pose_search
                else "upward_catch_candidates.json"
            )
        )
    )
    output_path = run_dir / output_name
    output_path.write_text(json.dumps(candidates, indent=2) + "\n", encoding="utf-8")
    print(f"saved candidates: {output_path}")


if __name__ == "__main__":
    main()
