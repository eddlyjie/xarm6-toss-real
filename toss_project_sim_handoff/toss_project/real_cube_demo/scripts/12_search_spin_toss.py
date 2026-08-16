#!/usr/bin/env python3
"""Read-only search for a feasible first spin-toss boundary trajectory."""

from dataclasses import replace
import math

import _bootstrap  # noqa: F401
from real_cube_demo.config import load_hardware
from real_cube_demo.robot import PickPlaceRobot
from real_cube_demo.spin_toss import build_spin_toss_plan, load_spin_toss_spec


def main() -> None:
    base = load_spin_toss_spec()
    hardware = load_hardware()
    candidates = []
    with PickPlaceRobot(hardware) as robot:
        for upward_velocity in (0.60, 0.70, 0.80):
            for spin_rate in (2.0, 2.5, 3.0):
                for flight_time in (0.10, 0.12, 0.14, 0.16):
                    for translation_stop in (0.14, 0.16):
                        spec = replace(
                            base,
                            nominal_upward_velocity_m_s=upward_velocity,
                            nominal_spin_rate_rad_s=spin_rate,
                            free_flight_duration_s=flight_time,
                            translation_stop_duration_s=translation_stop,
                        )
                        plan = build_spin_toss_plan(
                            spec,
                            robot.forward_kinematics,
                            robot.inverse_kinematics,
                            validate=False,
                        )
                        acceleration_peak = max(
                            plan.samples,
                            key=lambda sample: max(
                                abs(value)
                                for value in sample.joint_acceleration_rad_s2
                            ),
                        )
                        relative_distance = math.sqrt(
                            sum(
                                value**2
                                for value in (
                                    plan.object_relative_displacement_at_catch_m
                                )
                            )
                        )
                        feasible = (
                            plan.max_joint_speed_rad_s
                            <= spec.max_joint_speed_rad_s
                            and plan.max_joint_acceleration_rad_s2
                            <= spec.max_joint_acceleration_rad_s2
                            and plan.max_tcp_speed_m_s <= spec.max_tcp_speed_m_s
                            and plan.predicted_object_rotation_rad >= 0.35
                        )
                        score = (
                            plan.max_joint_acceleration_rad_s2
                            / spec.max_joint_acceleration_rad_s2
                            + 0.35
                            * plan.max_joint_speed_rad_s
                            / spec.max_joint_speed_rad_s
                            + 4.0 * relative_distance
                            - 0.15 * plan.predicted_object_rotation_rad
                        )
                        candidates.append(
                            (
                                not feasible,
                                score,
                                upward_velocity,
                                spin_rate,
                                flight_time,
                                translation_stop,
                                plan,
                                relative_distance,
                                acceleration_peak,
                            )
                        )

    candidates.sort(key=lambda item: (item[0], item[1]))
    feasible_count = sum(not item[0] for item in candidates)
    print(f"searched {len(candidates)} candidates; feasible={feasible_count}")
    for item in candidates[:10]:
        (
            infeasible,
            _,
            upward,
            spin,
            flight_time,
            translation_stop,
            plan,
            relative_distance,
            acceleration_peak,
        ) = item
        print(
            f"{'NO' if infeasible else 'OK'} "
            f"up={upward:.2f} spin={spin:.1f} "
            f"flight={flight_time:.2f} stop={translation_stop:.2f} | "
            f"T={plan.catch_time_s - plan.release_time_s:.2f} "
            f"rot={plan.predicted_object_rotation_rad:.2f} "
            f"qdot={plan.max_joint_speed_rad_s:.2f} "
            f"qdd={plan.max_joint_acceleration_rad_s2:.2f} "
            f"qdd_at={acceleration_peak.phase}:{acceleration_peak.time_s:.2f} "
            f"tcp={plan.max_tcp_speed_m_s:.2f} "
            f"relative={relative_distance * 1000.0:.1f}mm"
        )


if __name__ == "__main__":
    main()
