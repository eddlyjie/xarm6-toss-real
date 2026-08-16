#!/usr/bin/env python3
"""Readable, hardware-free reference for the Panda toss/catch control flow.

This is not a Panda trajectory to send to xArm.  It demonstrates the timing,
paired q/dq interpolation, gripper events, and pose-conditioned skill choice
that the real-robot implementation must preserve.
"""

from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass
import math
from typing import Sequence


@dataclass(frozen=True)
class PhaseWindow:
    name: str
    start_s: float
    end_s: float


@dataclass(frozen=True)
class JointReference:
    time_s: tuple[float, ...]
    joint_position_rad: tuple[tuple[float, ...], ...]
    joint_velocity_rad_s: tuple[tuple[float, ...], ...]

    def __post_init__(self) -> None:
        if len(self.time_s) < 2:
            raise ValueError("a joint reference needs at least two samples")
        if any(b <= a for a, b in zip(self.time_s, self.time_s[1:])):
            raise ValueError("time_s must be strictly increasing")
        if not (
            len(self.time_s)
            == len(self.joint_position_rad)
            == len(self.joint_velocity_rad_s)
        ):
            raise ValueError("time, q, and dq sample counts must match")
        dof = len(self.joint_position_rad[0])
        if dof == 0 or any(len(row) != dof for row in self.joint_position_rad):
            raise ValueError("joint_position rows must have one fixed DoF")
        if any(len(row) != dof for row in self.joint_velocity_rad_s):
            raise ValueError("joint_velocity rows must match joint_position")

    def sample(self, time_s: float) -> tuple[tuple[float, ...], tuple[float, ...]]:
        """Linearly sample paired q/dq from the same admitted plan."""

        if time_s <= self.time_s[0]:
            return self.joint_position_rad[0], self.joint_velocity_rad_s[0]
        if time_s >= self.time_s[-1]:
            return self.joint_position_rad[-1], self.joint_velocity_rad_s[-1]
        right = bisect_right(self.time_s, time_s)
        left = right - 1
        alpha = (time_s - self.time_s[left]) / (
            self.time_s[right] - self.time_s[left]
        )
        q = tuple(
            a + alpha * (b - a)
            for a, b in zip(
                self.joint_position_rad[left], self.joint_position_rad[right]
            )
        )
        dq = tuple(
            a + alpha * (b - a)
            for a, b in zip(
                self.joint_velocity_rad_s[left],
                self.joint_velocity_rad_s[right],
            )
        )
        return q, dq


@dataclass(frozen=True)
class GripperSchedule:
    release_time_s: float
    catch_time_s: float
    close_lead_s: float
    grasp_width_m: float
    open_width_m: float
    catch_width_m: float

    @property
    def catch_close_start_s(self) -> float:
        return self.catch_time_s - self.close_lead_s

    def width_at(self, time_s: float) -> float:
        if time_s < self.release_time_s:
            return self.grasp_width_m
        if time_s < self.catch_close_start_s:
            return self.open_width_m
        return self.catch_width_m


@dataclass(frozen=True)
class PoseConditionedSkill:
    skill_id: str
    expected_rotation_deg: float
    catch_probability: float
    motion_cost: float


def _wrapped_angle_error_deg(a: float, b: float) -> float:
    return abs((a - b + 180.0) % 360.0 - 180.0)


def select_pose_conditioned_skill(
    target_rotation_deg: float,
    skills: Sequence[PoseConditionedSkill],
) -> tuple[PoseConditionedSkill, list[tuple[str, float]]]:
    """Small 1-D illustration of the real SE(3) M3 coordinator.

    The production method uses full pose, IK, uncertainty, collision and motion
    terms.  This compact version makes the key behavior visible: changing the
    target can change the selected toss/regrasp skill.
    """

    if not skills:
        raise ValueError("at least one skill is required")
    ranked = []
    for skill in skills:
        if not 0.0 < skill.catch_probability <= 1.0:
            raise ValueError("catch_probability must lie in (0, 1]")
        pose_cost = math.radians(
            _wrapped_angle_error_deg(
                target_rotation_deg, skill.expected_rotation_deg
            )
        )
        risk_cost = -math.log(skill.catch_probability)
        total = 2.0 * pose_cost + 1.5 * risk_cost + 0.35 * skill.motion_cost
        ranked.append((skill, total))
    ranked.sort(key=lambda row: (row[1], row[0].skill_id))
    return ranked[0][0], [(skill.skill_id, cost) for skill, cost in ranked]


def reference_phases() -> tuple[PhaseWindow, ...]:
    """Compressed explanatory timeline, not measured Panda timestamps."""

    return (
        PhaseWindow("table_pick_and_lift", 0.0, 1.0),
        PhaseWindow("active_probe", 1.0, 2.8),
        PhaseWindow("prethrow", 2.8, 3.2),
        PhaseWindow("throw_and_release", 3.2, 3.8),
        PhaseWindow("free_flight_and_catch", 3.8, 4.2),
        PhaseWindow("stable_hold", 4.2, 6.2),
        PhaseWindow("postcatch_multiview", 6.2, 7.0),
        PhaseWindow("target_transport", 7.0, 8.0),
    )


def phase_at(time_s: float, phases: Sequence[PhaseWindow]) -> str:
    for phase in phases:
        if phase.start_s <= time_s < phase.end_s:
            return phase.name
    return "done" if phases and time_s >= phases[-1].end_s else "not_started"


def _demo_plan() -> JointReference:
    # Seven numbers make the Panda dimensionality visible.  These are synthetic
    # teaching values and must never be copied to xArm 6.
    return JointReference(
        time_s=(3.2, 3.5, 3.8, 4.1),
        joint_position_rad=(
            (0.00, -0.55, 0.00, -1.90, 0.00, 1.40, 0.70),
            (-0.10, -0.42, 0.05, -1.72, 0.16, 1.52, 0.82),
            (0.18, -0.30, 0.14, -1.58, 0.28, 1.61, 0.96),
            (0.28, -0.36, 0.20, -1.66, 0.34, 1.55, 1.04),
        ),
        joint_velocity_rad_s=(
            (-0.20, 0.15, 0.05, 0.18, 0.18, 0.10, 0.14),
            (0.65, 0.45, 0.20, 0.42, 0.38, 0.22, 0.30),
            (0.48, -0.08, 0.18, -0.20, 0.22, -0.12, 0.22),
            (0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00),
        ),
    )


def main() -> int:
    phases = reference_phases()
    plan = _demo_plan()
    gripper = GripperSchedule(
        release_time_s=3.65,
        catch_time_s=4.02,
        close_lead_s=0.10,
        grasp_width_m=0.025,
        open_width_m=0.080,
        catch_width_m=0.030,
    )
    skills = (
        PoseConditionedSkill("low_spin", 0.0, 0.93, 0.25),
        PoseConditionedSkill("quarter_turn_regrasp", 45.0, 0.82, 0.42),
        PoseConditionedSkill("larger_turn", 90.0, 0.70, 0.58),
    )

    print("Compressed Panda phase timeline:")
    for phase in phases:
        print(f"  {phase.start_s:4.1f}-{phase.end_s:4.1f}s  {phase.name}")
    print("\nPaired q/dq and gripper schedule around release/catch:")
    for time_s in (3.50, 3.65, 3.85, 3.92, 4.02, 4.10):
        q, dq = plan.sample(time_s)
        print(
            f"  t={time_s:4.2f} phase={phase_at(time_s, phases):24s} "
            f"q0={q[0]:+.3f} dq0={dq[0]:+.3f} "
            f"gripper={gripper.width_at(time_s):.3f}m"
        )
    print("\nPose-conditioned choice:")
    for target_deg in (0.0, 45.0, 90.0):
        selected, ranking = select_pose_conditioned_skill(target_deg, skills)
        costs = ", ".join(f"{name}:{cost:.3f}" for name, cost in ranking)
        print(f"  target={target_deg:4.0f}deg -> {selected.skill_id} ({costs})")
    print("\nReference only: no simulator or robot was connected.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
