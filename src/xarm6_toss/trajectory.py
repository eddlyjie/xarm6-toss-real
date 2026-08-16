"""Pure-Python throw trajectory generation; this module never moves a robot."""

from __future__ import annotations

from dataclasses import dataclass
import math

from .config import ThrowPlan


@dataclass(frozen=True)
class ThrowSample:
    time_s: float
    joint_rad: tuple[float, ...]
    release_gripper: bool
    phase: str


def _quintic(progress: float) -> float:
    value = min(1.0, max(0.0, float(progress)))
    return value**3 * (10.0 + value * (-15.0 + 6.0 * value))


def _interpolate(
    start: tuple[float, ...],
    end: tuple[float, ...],
    progress: float,
) -> tuple[float, ...]:
    blend = _quintic(progress)
    return tuple(a + blend * (b - a) for a, b in zip(start, end))


def _segment_steps(duration_s: float, period_s: float) -> int:
    return max(1, int(math.ceil(duration_s / period_s)))


def generate_throw_samples(plan: ThrowPlan) -> list[ThrowSample]:
    """Generate start→release→follow-through samples with one release event."""

    first_steps = _segment_steps(
        plan.duration_to_release_s, plan.control_period_s
    )
    second_steps = _segment_steps(
        plan.duration_followthrough_s, plan.control_period_s
    )
    samples: list[ThrowSample] = []
    for index in range(first_steps + 1):
        progress = index / first_steps
        samples.append(
            ThrowSample(
                time_s=progress * plan.duration_to_release_s,
                joint_rad=_interpolate(
                    plan.start_joint_rad, plan.release_joint_rad, progress
                ),
                release_gripper=index == first_steps,
                phase="to_release",
            )
        )
    for index in range(1, second_steps + 1):
        progress = index / second_steps
        samples.append(
            ThrowSample(
                time_s=(
                    plan.duration_to_release_s
                    + progress * plan.duration_followthrough_s
                ),
                joint_rad=_interpolate(
                    plan.release_joint_rad,
                    plan.followthrough_joint_rad,
                    progress,
                ),
                release_gripper=False,
                phase="followthrough",
            )
        )
    if sum(sample.release_gripper for sample in samples) != 1:
        raise RuntimeError("throw trajectory must contain exactly one release")
    return samples
