#!/usr/bin/env python3
"""Read-only search for a forward/upward throw visible to the global camera."""

import cv2
import numpy as np
from scipy.optimize import least_squares
import yaml

import _bootstrap  # noqa: F401
from real_cube_demo.config import load_hardware, load_probe_plan
from real_cube_demo.local_kinematics import LocalXArmFK
from real_cube_demo.spin_toss import (
    _evaluate,
    _quintic_coefficients,
    numerical_jacobian,
    pose_matrix,
)


CONTROL_PERIOD_S = 0.02
FLIGHT_TIME_S = 0.20
GRAVITY = np.asarray([0.0, 0.0, -9.81])
GRASP_OFFSET_TOOL_M = np.asarray([0.019914357, -0.008274612, -0.026013599])


class GlobalCameraProjection:
    def __init__(self, intrinsics_path, extrinsics_path):
        intrinsics = yaml.safe_load(intrinsics_path.read_text(encoding="utf-8"))
        extrinsics = yaml.safe_load(extrinsics_path.read_text(encoding="utf-8"))
        self.camera_pose_world = np.asarray(
            extrinsics["X_CammountCam"], dtype=float
        )
        self.world_to_camera = np.linalg.inv(self.camera_pose_world)
        self.camera_matrix = np.asarray(intrinsics["K"], dtype=float)
        self.distortion = np.asarray(intrinsics["dist"], dtype=float)

    def project(self, world_points_m: np.ndarray) -> np.ndarray:
        points = np.asarray(world_points_m, dtype=float).reshape(-1, 3)
        camera_points = (
            self.world_to_camera
            @ np.column_stack((points, np.ones(len(points)))).T
        ).T[:, :3]
        pixels, _ = cv2.projectPoints(
            camera_points,
            np.zeros(3),
            np.zeros(3),
            self.camera_matrix,
            self.distortion,
        )
        return pixels.reshape(-1, 2)


def solve_position(fk, fixed_q, q5, target_xz_m, seed_q23):
    def residual(q23):
        joint = fixed_q.copy()
        joint[1:3] = q23
        joint[4] = q5
        xyz_m = np.asarray(fk.forward_kinematics(tuple(joint))[:3]) / 1000.0
        return xyz_m[[0, 2]] - target_xz_m

    result = least_squares(
        residual,
        seed_q23,
        bounds=(np.asarray([-1.90, -3.90]), np.asarray([2.09, 0.19])),
    )
    joint = fixed_q.copy()
    joint[1:3] = result.x
    joint[4] = q5
    error_m = float(np.linalg.norm(residual(result.x)))
    return joint, error_m


def release_velocity(fk, release_q, target_spin_rad_s):
    jacobian = numerical_jacobian(tuple(release_q), fk.forward_kinematics)
    joint_5_axis = jacobian[3:, 4]
    joint_5_axis /= np.linalg.norm(joint_5_axis)
    active_jacobian = jacobian[:, [1, 2, 4]]
    task_matrix = np.vstack(
        (
            active_jacobian[0],
            active_jacobian[2],
            joint_5_axis @ active_jacobian[3:],
        )
    )
    active_qd = np.linalg.lstsq(
        task_matrix,
        np.asarray([0.23, 0.42, target_spin_rad_s]),
        rcond=None,
    )[0]
    release_qd = np.zeros(6)
    release_qd[[1, 2, 4]] = active_qd
    return release_qd, jacobian @ release_qd


def trajectory(fk, start_q, release_q, release_qd, duration_s):
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
        [fk.forward_kinematics(tuple(joint))[:3] for joint in joints]
    ) / 1000.0
    release_jacobian = numerical_jacobian(tuple(release_q), fk.forward_kinematics)
    release_twist = release_jacobian @ release_qd
    return {
        "release_qd": release_qd,
        "release_twist": release_twist,
        "joints": joints,
        "tcp_xyz_m": tcp_xyz_m,
        "joint_path_rad": float(np.sum(np.abs(np.diff(joints, axis=0)))),
        "maximum_joint_swing_rad": float(
            np.max(np.ptp(joints, axis=0))
        ),
        "max_joint_speed_rad_s": float(np.max(np.abs(velocities))),
        "max_joint_acceleration_rad_s2": float(np.max(np.abs(accelerations))),
    }


def visible(pixels):
    pixels = np.asarray(pixels)
    return bool(
        np.all((30.0 <= pixels[:, 0]) & (pixels[:, 0] <= 610.0))
        and np.all((30.0 <= pixels[:, 1]) & (pixels[:, 1] <= 450.0))
    )


def main() -> None:
    hardware = load_hardware()
    global_camera = next(
        camera for camera in hardware.cameras if camera.role == "global"
    )
    projection = GlobalCameraProjection(
        global_camera.intrinsics_path, global_camera.extrinsics_path
    )
    fk = LocalXArmFK()
    handoff_q = np.asarray(load_probe_plan().center_joint_rad, dtype=float)
    candidates = []
    dynamic_attempts = []
    camera_attempts = []
    start_solutions = 0
    release_solutions = 0
    dynamic_solutions = 0
    camera_solutions = 0

    start_targets = ((0.30, 0.36), (0.34, 0.38), (0.38, 0.40))
    release_offsets = (
        (0.12, 0.08),
        (0.14, 0.08),
        (0.15, 0.08),
        (0.16, 0.08),
        (0.16, 0.12),
    )
    for start_x, start_z in start_targets:
        for start_q5 in (1.15, 1.35, 1.55):
            start_q, start_error = solve_position(
                fk,
                handoff_q,
                start_q5,
                np.asarray([start_x, start_z]),
                handoff_q[1:3],
            )
            if start_error > 0.003:
                continue
            start_solutions += 1
            for delta_x, delta_z in release_offsets:
                for delta_q5 in (0.18, 0.28):
                    release_q, release_error = solve_position(
                        fk,
                        handoff_q,
                        start_q5 + delta_q5,
                        np.asarray([start_x + delta_x, start_z + delta_z]),
                        start_q[1:3],
                    )
                    if release_error > 0.003:
                        continue
                    release_solutions += 1
                    for target_spin_rad_s in (0.55, 0.75):
                        release_qd, _ = release_velocity(
                            fk, release_q, target_spin_rad_s
                        )
                        if not 0.10 <= release_qd[4] <= 3.10:
                            continue
                        for duration_s in (0.44, 0.52, 0.60):
                            motion = trajectory(
                                fk,
                                start_q,
                                release_q,
                                release_qd,
                                duration_s,
                            )
                            twist = motion["release_twist"]
                            forward_monotonic = bool(
                                np.all(np.diff(motion["tcp_xyz_m"][:, 0]) >= -0.002)
                            )
                            upward_monotonic = bool(
                                np.all(np.diff(motion["tcp_xyz_m"][:, 2]) >= -0.002)
                            )
                            dynamic_attempts.append(
                                {
                                    "start": motion["tcp_xyz_m"][0],
                                    "release": motion["tcp_xyz_m"][-1],
                                    "twist": twist,
                                    "qdot": motion["max_joint_speed_rad_s"],
                                    "qdd": motion["max_joint_acceleration_rad_s2"],
                                    "forward_monotonic": forward_monotonic,
                                    "upward_monotonic": upward_monotonic,
                                }
                            )
                            if not (
                                motion["max_joint_speed_rad_s"] <= 3.10
                                and motion["max_joint_acceleration_rad_s2"] <= 20.0
                                and twist[0] >= 0.15
                                and twist[2] >= 0.30
                                and np.linalg.norm(twist[3:]) >= 0.40
                                and forward_monotonic
                                and upward_monotonic
                            ):
                                continue
                            dynamic_solutions += 1

                            release_transform = pose_matrix(
                                fk.forward_kinematics(tuple(release_q))
                            )
                            object_release = (
                                release_transform[:3, 3]
                                + release_transform[:3, :3]
                                @ GRASP_OFFSET_TOOL_M
                            )
                            flight_times = np.linspace(0.0, FLIGHT_TIME_S, 21)
                            object_flight = np.asarray(
                                [
                                    object_release
                                    + twist[:3] * time_s
                                    + 0.5 * GRAVITY * time_s**2
                                    for time_s in flight_times
                                ]
                            )
                            tcp_pixels = projection.project(motion["tcp_xyz_m"])
                            object_pixels = projection.project(object_flight)
                            camera_attempts.append(
                                {
                                    "tcp_min": np.min(tcp_pixels, axis=0),
                                    "tcp_max": np.max(tcp_pixels, axis=0),
                                    "object_min": np.min(object_pixels, axis=0),
                                    "object_max": np.max(object_pixels, axis=0),
                                }
                            )
                            if not visible(tcp_pixels) or not visible(object_pixels):
                                continue
                            camera_solutions += 1
                            if object_flight[-1, 2] < 0.30:
                                continue

                            center = np.asarray([320.0, 260.0])
                            image_center_distance = float(
                                np.mean(np.linalg.norm(object_pixels - center, axis=1))
                            )
                            candidates.append(
                                {
                                    "start_q": start_q.copy(),
                                    "release_q": release_q.copy(),
                                    "duration_s": duration_s,
                                    "target_spin_rad_s": target_spin_rad_s,
                                    "setup_path_rad": float(
                                        np.sum(np.abs(start_q - handoff_q))
                                    ),
                                    "tcp_start_m": motion["tcp_xyz_m"][0],
                                    "tcp_release_m": motion["tcp_xyz_m"][-1],
                                    "tcp_pixels_start": tcp_pixels[0],
                                    "tcp_pixels_release": tcp_pixels[-1],
                                    "object_pixels_release": object_pixels[0],
                                    "object_pixels_catch": object_pixels[-1],
                                    "object_catch_m": object_flight[-1],
                                    "image_center_distance": image_center_distance,
                                    **motion,
                                }
                            )

    candidates.sort(
        key=lambda item: (
            item["setup_path_rad"] + item["joint_path_rad"],
            item["image_center_distance"],
        )
    )
    print(
        "read-only global-camera-visible search; forward+upward throw, "
        "active joints=J2/J3/J5"
    )
    print(
        f"start IK={start_solutions}, release IK={release_solutions}, "
        f"dynamic={dynamic_solutions}, camera-visible={camera_solutions}, "
        f"feasible={len(candidates)}"
    )
    if not dynamic_solutions:
        dynamic_attempts.sort(
            key=lambda item: (
                max(item["qdot"] / 3.10, item["qdd"] / 20.0),
                -item["twist"][0] - item["twist"][2],
            )
        )
        print("closest dynamic attempts:")
        for attempt in dynamic_attempts[:6]:
            print(
                "  TCP "
                f"{[round(1000 * float(v), 0) for v in attempt['start']]} -> "
                f"{[round(1000 * float(v), 0) for v in attempt['release']]}, "
                f"linear={[round(float(v), 3) for v in attempt['twist'][:3]]}, "
                f"angular={np.linalg.norm(attempt['twist'][3:]):.3f}, "
                f"qdot={attempt['qdot']:.2f}, qdd={attempt['qdd']:.2f}, "
                f"monotonic={attempt['forward_monotonic']}/"
                f"{attempt['upward_monotonic']}"
            )
    elif not camera_solutions:
        print("camera bounds of dynamically feasible attempts:")
        camera_attempts.sort(key=lambda item: item["object_max"][1])
        for attempt in camera_attempts[:12]:
            print(
                "  TCP uv min/max="
                f"{np.round(attempt['tcp_min'], 1).tolist()}/"
                f"{np.round(attempt['tcp_max'], 1).tolist()}, object="
                f"{np.round(attempt['object_min'], 1).tolist()}/"
                f"{np.round(attempt['object_max'], 1).tolist()}"
            )
    for index, candidate in enumerate(candidates[:12], start=1):
        print(f"\n#{index}")
        print(
            f"  start q={[round(float(v), 4) for v in candidate['start_q']]}"
        )
        print(
            f"  release q={[round(float(v), 4) for v in candidate['release_q']]}"
        )
        print(
            "  TCP start -> release mm: "
            f"{[round(1000 * float(v), 1) for v in candidate['tcp_start_m']]} -> "
            f"{[round(1000 * float(v), 1) for v in candidate['tcp_release_m']]}"
        )
        print(
            "  global pixels TCP start -> release: "
            f"{[round(float(v), 1) for v in candidate['tcp_pixels_start']]} -> "
            f"{[round(float(v), 1) for v in candidate['tcp_pixels_release']]}"
        )
        print(
            "  object pixels release -> catch: "
            f"{[round(float(v), 1) for v in candidate['object_pixels_release']]} -> "
            f"{[round(float(v), 1) for v in candidate['object_pixels_catch']]}"
        )
        twist = candidate["release_twist"]
        print(
            "  release linear="
            f"{[round(float(v), 3) for v in twist[:3]]} m/s, "
            f"angular={np.linalg.norm(twist[3:]):.3f} rad/s"
        )
        print(
            f"  duration={candidate['duration_s']:.2f} s, "
            f"setup path={candidate['setup_path_rad']:.3f} rad, "
            f"throw path={candidate['joint_path_rad']:.3f} rad, "
            f"max swing={candidate['maximum_joint_swing_rad']:.3f} rad, "
            f"qdot={candidate['max_joint_speed_rad_s']:.3f}, "
            f"qdd={candidate['max_joint_acceleration_rad_s2']:.3f}"
        )


if __name__ == "__main__":
    main()
