"""Compact PyTorch skeletons mirroring the simulator learning architecture.

Nothing in this module is a trained xArm policy.  It gives the real-robot
computer the correct module boundaries and tensor contracts so collected data
can replace placeholders without redesigning the project.
"""

from __future__ import annotations

try:
    import torch
    from torch import nn
    from torch.nn import functional as F
except ImportError:  # The basic robot starter kit does not require PyTorch.
    torch = None
    nn = None
    F = None


DETACH_RESIDUAL_DIM = 13
WHOLE_ARM_ACTION_DIM = 22


def require_torch() -> None:
    if torch is None:
        raise RuntimeError(
            "PyTorch is required for learning models; install the optional "
            "requirements-learning.txt environment on the training computer"
        )


if nn is not None:

    def _mlp(input_dim: int, hidden_dim: int, output_dim: int) -> nn.Sequential:
        return nn.Sequential(
            nn.Linear(input_dim, hidden_dim),
            nn.LayerNorm(hidden_dim),
            nn.SiLU(),
            nn.Linear(hidden_dim, output_dim),
            nn.SiLU(),
        )


    class DetachResidualNet(nn.Module):
        """Probe-conditioned supervised Detach predictor.

        Defaults follow the current sim schemas: 28 action features, 76
        posterior/physics features, 36 grasp-geometry features, and a temporal
        sequence of 18 bilateral tactile features.  The production sim uses a
        richer local point-patch encoder and full covariance; this compact
        xArm skeleton starts with a diagonal heteroscedastic head.
        """

        def __init__(
            self,
            action_dim: int = 28,
            physics_dim: int = 76,
            geometry_dim: int = 36,
            tactile_dim: int = 18,
            hidden_dim: int = 128,
        ) -> None:
            super().__init__()
            branch = hidden_dim // 2
            self.action_encoder = _mlp(action_dim, branch, branch)
            self.physics_encoder = _mlp(physics_dim, branch, branch)
            self.geometry_encoder = _mlp(geometry_dim, branch, branch)
            self.tactile_frame = _mlp(tactile_dim, branch, branch)
            self.tactile_gru = nn.GRU(branch, branch, batch_first=True)
            self.trunk = nn.Sequential(
                nn.Linear(branch * 4, hidden_dim),
                nn.LayerNorm(hidden_dim),
                nn.SiLU(),
                nn.Linear(hidden_dim, hidden_dim),
                nn.SiLU(),
            )
            self.residual_mean = nn.Linear(hidden_dim, DETACH_RESIDUAL_DIM)
            self.residual_log_scale = nn.Linear(hidden_dim, DETACH_RESIDUAL_DIM)
            self.valid_logit = nn.Linear(hidden_dim, 1)

        def forward(
            self,
            action: torch.Tensor,
            physics_posterior: torch.Tensor,
            grasp_geometry: torch.Tensor,
            tactile_sequence: torch.Tensor,
        ) -> dict[str, torch.Tensor]:
            tactile_frame = self.tactile_frame(tactile_sequence)
            _, tactile_state = self.tactile_gru(tactile_frame)
            hidden = self.trunk(
                torch.cat(
                    (
                        self.action_encoder(action),
                        self.physics_encoder(physics_posterior),
                        self.geometry_encoder(grasp_geometry),
                        tactile_state[-1],
                    ),
                    dim=-1,
                )
            )
            return {
                "residual_mean": self.residual_mean(hidden),
                "residual_log_scale": torch.clamp(
                    self.residual_log_scale(hidden), -7.0, 3.0
                ),
                "valid_logit": self.valid_logit(hidden).squeeze(-1),
            }


    class PointCloudEncoder(nn.Module):
        def __init__(self, latent_dim: int = 96) -> None:
            super().__init__()
            self.point = nn.Sequential(
                nn.Linear(3, 64),
                nn.SiLU(),
                nn.Linear(64, latent_dim),
                nn.SiLU(),
            )

        def forward(self, points_o: torch.Tensor) -> torch.Tensor:
            if points_o.ndim != 3 or points_o.shape[-1] != 3:
                raise ValueError("points_o must have shape (batch, points, 3)")
            return self.point(points_o).amax(dim=1)


    class CandidateConditionedCatchActor(nn.Module):
        """Deployable 22-D whole-arm residual actor.

        `state_dim` should contain only real observations: Probe posterior,
        Detach/flight belief, robot q/dq, one dynamic contact candidate and
        camera latency.  Final-pose coordination may happen outside this actor
        over a skill library, as in sim M3.
        """

        def __init__(
            self,
            state_dim: int,
            hidden_dim: int = 256,
            point_latent_dim: int = 96,
        ) -> None:
            super().__init__()
            self.point_encoder = PointCloudEncoder(point_latent_dim)
            self.actor = nn.Sequential(
                nn.Linear(state_dim + point_latent_dim, hidden_dim),
                nn.LayerNorm(hidden_dim),
                nn.SiLU(),
                nn.Linear(hidden_dim, hidden_dim),
                nn.SiLU(),
                nn.Linear(hidden_dim, WHOLE_ARM_ACTION_DIM),
            )
            self.log_std = nn.Parameter(
                torch.full((WHOLE_ARM_ACTION_DIM,), -1.8)
            )

        def forward(
            self,
            points_o: torch.Tensor,
            deployable_state: torch.Tensor,
        ) -> tuple[torch.Tensor, torch.Tensor]:
            latent = self.point_encoder(points_o)
            mean = self.actor(torch.cat((latent, deployable_state), dim=-1))
            return mean, torch.clamp(self.log_std, -4.5, 0.5).expand_as(mean)


    class PoseConditionedSkillPolicy(nn.Module):
        """Small real-robot policy for selecting among learned throw skills."""

        def __init__(
            self,
            context_dim: int,
            skill_count: int,
            residual_dim: int = 4,
            hidden_dim: int = 128,
        ) -> None:
            super().__init__()
            self.encoder = nn.Sequential(
                nn.Linear(context_dim, hidden_dim),
                nn.LayerNorm(hidden_dim),
                nn.SiLU(),
                nn.Linear(hidden_dim, hidden_dim),
                nn.SiLU(),
            )
            self.skill_logits = nn.Linear(hidden_dim, skill_count)
            self.release_residual = nn.Linear(hidden_dim, residual_dim)

        def forward(self, context: torch.Tensor) -> dict[str, torch.Tensor]:
            hidden = self.encoder(context)
            return {
                "skill_logits": self.skill_logits(hidden),
                "release_residual": self.release_residual(hidden),
            }


    class AsymmetricCritic(nn.Module):
        """Training-only value function; privileged inputs never reach actor."""

        def __init__(
            self,
            deployable_dim: int,
            privileged_dim: int,
            hidden_dim: int = 256,
        ) -> None:
            super().__init__()
            self.value = nn.Sequential(
                nn.Linear(deployable_dim + privileged_dim, hidden_dim),
                nn.SiLU(),
                nn.Linear(hidden_dim, hidden_dim),
                nn.SiLU(),
                nn.Linear(hidden_dim, 1),
            )

        def forward(
            self,
            deployable_state: torch.Tensor,
            privileged_training_state: torch.Tensor,
        ) -> torch.Tensor:
            return self.value(
                torch.cat(
                    (deployable_state, privileged_training_state), dim=-1
                )
            ).squeeze(-1)

else:

    class _TorchMissing:
        def __init__(self, *_args, **_kwargs) -> None:
            require_torch()

    DetachResidualNet = _TorchMissing
    CandidateConditionedCatchActor = _TorchMissing
    PoseConditionedSkillPolicy = _TorchMissing
    AsymmetricCritic = _TorchMissing
