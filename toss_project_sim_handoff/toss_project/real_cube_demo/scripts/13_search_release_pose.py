#!/usr/bin/env python3
"""Read-only search for a nearby release posture with better toss dynamics."""

from dataclasses import replace
import math

import numpy as np

import _bootstrap  # noqa: F401
from real_cube_demo.config import load_hardware
from real_cube_demo.robot import PickPlaceRobot
from real_cube_demo.spin_toss import (
    build_spin_toss_plan,
    load_spin_toss_spec,
)


def main() -> None:
    base = load_spin_toss_spec()
    search_spec = replace(
        base,
        nominal_spin_rate_rad_s=2.2,
    )
    hardware = load_hardware()
    candidates = []
    failures = 0

    with PickPlaceRobot(hardware) as robot:
        base_pose = np.asarray(robot.forward_kinematics(base.release_joint_rad))
        for joint_2 in np.linspace(-2.0, -0.30, 12):
            for joint_3 in np.linspace(-1.20, 0.85, 13):
                release = list(base.release_joint_rad)
                release[1] = float(joint_2)
                release[2] = float(joint_3)
                release[4] = float(
                    base.release_joint_rad[1]
                    + base.release_joint_rad[2]
                    + base.release_joint_rad[4]
                    - joint_2
                    - joint_3
                )
                if not -2.0 <= release[4] <= 2.0:
                    continue
                pose = np.asarray(robot.forward_kinematics(tuple(release)))
                displacement_m = float(
                    np.linalg.norm(pose[:3] - base_pose[:3]) / 1000.0
                )
                if not (
                    120.0 <= pose[0] <= 450.0
                    and -180.0 <= pose[1] <= 180.0
                    and 140.0 <= pose[2] <= 420.0
                ):
                    continue
                spec = replace(search_spec, release_joint_rad=tuple(release))
                try:
                    plan = build_spin_toss_plan(
                        spec,
                        robot.forward_kinematics,
                        robot.inverse_kinematics,
                        validate=False,
                    )
                except RuntimeError:
                    failures += 1
                    continue
                feasible = (
                    plan.max_joint_speed_rad_s <= spec.max_joint_speed_rad_s
                    and plan.max_joint_acceleration_rad_s2
                    <= spec.max_joint_acceleration_rad_s2
                    and plan.max_tcp_speed_m_s <= spec.max_tcp_speed_m_s
                )
                score = (
                    plan.max_joint_acceleration_rad_s2
                    / spec.max_joint_acceleration_rad_s2
                    + 0.3
                    * plan.max_joint_speed_rad_s
                    / spec.max_joint_speed_rad_s
                    + 1.5 * displacement_m
                )
                candidates.append(
                    (
                        not feasible,
                        score,
                        displacement_m,
                        pose,
                        plan,
                    )
                )

    candidates.sort(key=lambda item: (item[0], item[1]))
    feasible_count = sum(not item[0] for item in candidates)
    print(f"base TCP pose: {[round(value, 3) for value in base_pose]}")
    print(
        f"evaluated={len(candidates)}, IK failures={failures}, feasible={feasible_count}"
    )
    for infeasible, _, displacement, pose, plan in candidates[:12]:
        print(
            f"{'NO' if infeasible else 'OK'} "
            f"q={[round(value, 4) for value in plan.release_joint_rad]} | "
            f"TCP={[round(value, 1) for value in pose[:3]]}mm "
            f"shift={displacement * 1000.0:.1f}mm "
            f"rot={math.degrees(plan.predicted_object_rotation_rad):.1f}deg "
            f"qdot={plan.max_joint_speed_rad_s:.2f} "
            f"qdd={plan.max_joint_acceleration_rad_s2:.2f} "
            f"tcp={plan.max_tcp_speed_m_s:.2f}"
        )


if __name__ == "__main__":
    main()
