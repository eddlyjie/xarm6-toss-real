"""Minimal supervised, behavior-cloning, and PPO objectives."""

from __future__ import annotations

try:
    import torch
    from torch.nn import functional as F
except ImportError:
    torch = None
    F = None


def _torch():
    if torch is None:
        raise RuntimeError("PyTorch is required for learning losses")
    return torch


def detach_heteroscedastic_loss(
    prediction: dict,
    residual_target,
    valid_target,
    *,
    valid_weight: float = 0.25,
):
    """Diagonal xArm starter version of sim Detach NLL + validity loss."""

    t = _torch()
    mean = prediction["residual_mean"]
    log_scale = prediction["residual_log_scale"]
    scale = t.exp(log_scale).clamp_min(1e-4)
    per_dimension = (
        0.5 * ((residual_target - mean) / scale).square()
        + log_scale
        + 0.5 * 1.8378770664093453
    )
    valid = valid_target > 0.5
    regression = (
        per_dimension[valid].sum(dim=-1).mean()
        if bool(valid.any())
        else per_dimension.mean() * 0.0
    )
    classification = F.binary_cross_entropy_with_logits(
        prediction["valid_logit"], valid_target
    )
    return regression + valid_weight * classification


def behavior_cloning_loss(predicted_action, expert_action, weights=None):
    t = _torch()
    error = (predicted_action - expert_action).square()
    if weights is not None:
        error = error * t.as_tensor(
            weights, dtype=error.dtype, device=error.device
        )
    return error.mean()


def ppo_clipped_actor_loss(
    new_log_probability,
    old_log_probability,
    advantage,
    *,
    clip_ratio: float = 0.2,
    bc_anchor=None,
    bc_coefficient: float = 0.08,
):
    """PPO actor objective with the project's BC anchor near safe experts."""

    t = _torch()
    ratio = t.exp(new_log_probability - old_log_probability)
    unclipped = ratio * advantage
    clipped = t.clamp(ratio, 1.0 - clip_ratio, 1.0 + clip_ratio) * advantage
    loss = -t.minimum(unclipped, clipped).mean()
    if bc_anchor is not None:
        loss = loss + bc_coefficient * bc_anchor
    return loss


def critic_value_loss(predicted_value, reward):
    _torch()
    return F.mse_loss(predicted_value, reward)
