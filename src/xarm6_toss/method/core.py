"""Small, hardware-independent pieces of the full sim method.

These functions are deliberately useful before cameras, Torch checkpoints, or
the robot are available.  They preserve the project distinction between
model-based Probe selection, candidate-level catch objective J, and the M2/M3
target-conditioned coordinator.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable


DETACH_RESIDUAL_NAMES = (
    "detach_time_s",
    "position_x_m",
    "position_y_m",
    "position_z_m",
    "rotation_x_rad",
    "rotation_y_rad",
    "rotation_z_rad",
    "linear_velocity_x_m_s",
    "linear_velocity_y_m_s",
    "linear_velocity_z_m_s",
    "angular_velocity_x_rad_s",
    "angular_velocity_y_rad_s",
    "angular_velocity_z_rad_s",
)

WHOLE_ARM_ACTION_NAMES = (
    "catch_time_s",
    "retreat_knot_x",
    "retreat_knot_y",
    "retreat_knot_z",
    "transfer_knot_x",
    "transfer_knot_y",
    "transfer_knot_z",
    "precatch_knot_x",
    "precatch_knot_y",
    "precatch_knot_z",
    "catch_rotation_6d_0",
    "catch_rotation_6d_1",
    "catch_rotation_6d_2",
    "catch_rotation_6d_3",
    "catch_rotation_6d_4",
    "catch_rotation_6d_5",
    "match_velocity_x",
    "match_velocity_y",
    "match_velocity_z",
    "close_lead_s",
    "compliance_scale",
    "execute_probability",
)


@dataclass(frozen=True)
class ProbeCandidate:
    name: str
    information_gain: float
    task_uncertainty_reduction: float
    slip_probability: float
    duration_s: float
    energy_cost: float
    peak_force_ratio: float

    def __post_init__(self) -> None:
        values = (
            self.information_gain,
            self.task_uncertainty_reduction,
            self.slip_probability,
            self.duration_s,
            self.energy_cost,
            self.peak_force_ratio,
        )
        if not self.name or any(not math.isfinite(value) for value in values):
            raise ValueError("probe candidate fields must be finite")
        if min(values) < 0.0:
            raise ValueError("probe candidate costs/gains must be non-negative")
        if self.slip_probability > 1.0 or self.peak_force_ratio > 1.0:
            raise ValueError("probe probabilities/ratios must lie in [0, 1]")


@dataclass(frozen=True)
class ProbeSelectionWeights:
    slip: float = 1.0
    duration: float = 0.02
    energy: float = 0.02
    peak_force: float = 0.10


def probe_score(
    candidate: ProbeCandidate,
    *,
    task_conditioned: bool,
    weights: ProbeSelectionWeights = ProbeSelectionWeights(),
) -> float:
    gain = (
        candidate.task_uncertainty_reduction
        if task_conditioned
        else candidate.information_gain
    )
    penalty = (
        weights.slip * candidate.slip_probability
        + weights.duration * candidate.duration_s
        + weights.energy * candidate.energy_cost
        + weights.peak_force * candidate.peak_force_ratio
    )
    return gain - penalty


def select_probe(
    candidates: Iterable[ProbeCandidate],
    *,
    task_conditioned: bool = True,
    weights: ProbeSelectionWeights = ProbeSelectionWeights(),
) -> tuple[ProbeCandidate, list[tuple[str, float]]]:
    values = list(candidates)
    if not values:
        raise ValueError("at least one Probe candidate is required")
    ranked = sorted(
        (
            (
                candidate,
                probe_score(
                    candidate,
                    task_conditioned=task_conditioned,
                    weights=weights,
                ),
            )
            for candidate in values
        ),
        key=lambda item: (-item[1], item[0].name),
    )
    return ranked[0][0], [(item.name, score) for item, score in ranked]


@dataclass(frozen=True)
class CatchObjectiveWeights:
    task: float = 2.0
    relative: float = 1.5
    impact: float = 1.0
    slip: float = 1.2
    motion: float = 0.35
    uncertainty: float = 1.0
    success: float = 1.5


@dataclass(frozen=True)
class CatchObjectiveTerms:
    task_grasp_error: float
    relative_contact_velocity: float
    impact_energy: float
    slip_risk: float
    arm_motion_cost: float
    cvar_failure: float
    catch_probability: float

    def __post_init__(self) -> None:
        costs = (
            self.task_grasp_error,
            self.relative_contact_velocity,
            self.impact_energy,
            self.slip_risk,
            self.arm_motion_cost,
            self.cvar_failure,
        )
        if any(not math.isfinite(value) or value < 0.0 for value in costs):
            raise ValueError("catch objective costs must be non-negative")
        if not 0.0 < self.catch_probability <= 1.0:
            raise ValueError("catch_probability must lie in (0, 1]")


def catch_objective(
    terms: CatchObjectiveTerms,
    weights: CatchObjectiveWeights = CatchObjectiveWeights(),
) -> float:
    """Return the sim J_catch cost; lower is better."""

    return (
        weights.task * terms.task_grasp_error
        + weights.relative * terms.relative_contact_velocity
        + weights.impact * terms.impact_energy
        + weights.slip * terms.slip_risk
        + weights.motion * terms.arm_motion_cost
        + weights.uncertainty * terms.cvar_failure
        - weights.success * math.log(max(terms.catch_probability, 1e-8))
    )


@dataclass(frozen=True)
class CoordinatorCandidate:
    """One toss/regrasp skill evaluated for one requested target pose."""

    skill_id: str
    catch_probability: float
    robust_ik_fraction: float
    position_error_m: float
    rotation_error_rad: float
    detach_flight_uncertainty: float
    collision_contact_risk: float
    weighted_joint_path_rad: float
    maximum_joint_swing_rad: float
    postcatch_ee_rotation_rad: float
    execution_duration_s: float

    def __post_init__(self) -> None:
        if not self.skill_id:
            raise ValueError("skill_id is required")
        values = tuple(
            float(getattr(self, field))
            for field in self.__dataclass_fields__
            if field != "skill_id"
        )
        if any(not math.isfinite(value) or value < 0.0 for value in values):
            raise ValueError("coordinator values must be finite and non-negative")
        if self.catch_probability > 1.0 or self.robust_ik_fraction > 1.0:
            raise ValueError("probabilities/fractions must lie in [0, 1]")


@dataclass(frozen=True)
class CoordinatorConstraints:
    position_tolerance_m: float = 0.030
    orientation_tolerance_rad: float = math.radians(20.0)
    minimum_catch_probability: float = 0.20
    minimum_robust_ik_fraction: float = 0.80
    maximum_collision_contact_risk: float = 0.25
    rotation_normalizer_rad: float = math.radians(45.0)


@dataclass(frozen=True)
class CoordinatorWeights:
    toss_failure: float = 2.0
    robust_ik_failure: float = 5.0
    pose_residual: float = 0.25
    detach_flight_uncertainty: float = 0.50
    collision_contact_risk: float = 2.0
    weighted_joint_path: float = 0.20
    maximum_joint_swing: float = 0.30
    postcatch_ee_rotation: float = 0.35
    execution_duration: float = 0.05


def coordinator_eligible(
    candidate: CoordinatorCandidate,
    constraints: CoordinatorConstraints = CoordinatorConstraints(),
) -> bool:
    return (
        candidate.catch_probability >= constraints.minimum_catch_probability
        and candidate.robust_ik_fraction
        >= constraints.minimum_robust_ik_fraction
        and candidate.collision_contact_risk
        <= constraints.maximum_collision_contact_risk
    )


def coordinated_cost(
    candidate: CoordinatorCandidate,
    constraints: CoordinatorConstraints = CoordinatorConstraints(),
    weights: CoordinatorWeights = CoordinatorWeights(),
) -> float:
    """Return the target-specific M3 bridge cost; lower is better."""

    pose_residual = (
        candidate.position_error_m / constraints.position_tolerance_m
        + candidate.rotation_error_rad / constraints.orientation_tolerance_rad
    )
    return (
        weights.toss_failure
        * -math.log(max(candidate.catch_probability, 1e-6))
        + weights.robust_ik_failure * (1.0 - candidate.robust_ik_fraction)
        + weights.pose_residual * pose_residual
        + weights.detach_flight_uncertainty
        * candidate.detach_flight_uncertainty
        + weights.collision_contact_risk * candidate.collision_contact_risk
        + weights.weighted_joint_path
        * candidate.weighted_joint_path_rad
        / 5.0
        + weights.maximum_joint_swing
        * candidate.maximum_joint_swing_rad
        / 1.5
        + weights.postcatch_ee_rotation
        * candidate.postcatch_ee_rotation_rad
        / constraints.rotation_normalizer_rad
        + weights.execution_duration * candidate.execution_duration_s / 2.0
    )


def select_fixed_confidence(
    candidates: Iterable[CoordinatorCandidate],
) -> CoordinatorCandidate:
    """M2: select without using target-specific IK terms."""

    values = list(candidates)
    if not values:
        raise ValueError("at least one skill candidate is required")
    return min(
        values,
        key=lambda value: (
            -value.catch_probability,
            value.collision_contact_risk,
            value.detach_flight_uncertainty,
            value.skill_id,
        ),
    )


def select_coordinated(
    candidates: Iterable[CoordinatorCandidate],
    constraints: CoordinatorConstraints = CoordinatorConstraints(),
    weights: CoordinatorWeights = CoordinatorWeights(),
) -> tuple[CoordinatorCandidate, list[tuple[str, bool, float]]]:
    """M3: select the lowest target-specific cost among eligible skills."""

    values = list(candidates)
    if not values:
        raise ValueError("at least one skill candidate is required")
    ranking = sorted(
        (
            (
                value,
                coordinator_eligible(value, constraints),
                coordinated_cost(value, constraints, weights),
            )
            for value in values
        ),
        key=lambda item: (not item[1], item[2], item[0].skill_id),
    )
    if not ranking[0][1]:
        raise RuntimeError("no target-coordinated skill is eligible")
    return ranking[0][0], [
        (item.skill_id, eligible, cost)
        for item, eligible, cost in ranking
    ]
