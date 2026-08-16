import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from xarm6_toss.intercept_residual import InterceptResidualPolicy
from xarm6_toss.online_closed_loop import OnlineInterceptController


class OnlineInterceptControllerTest(unittest.TestCase):
    def policy(self):
        return InterceptResidualPolicy(
            feature_mean=np.zeros(9),
            feature_scale=np.ones(9),
            weight=np.zeros((3, 9)),
            bias=np.asarray([0.004, -0.003, 0.002]),
            action_norm_limit_m=0.02,
        )

    def test_camera_measurement_changes_deployable_command(self):
        controller = OnlineInterceptController(
            release_command_time_s=0.30,
            prediction_horizon_s=0.09,
            policy=self.policy(),
        )
        controller.set_encoder_detach_prior(
            0.325,
            [0.20, 0.01, 0.38],
            [0.28, 0.03, 0.24],
        )
        command = controller.add_global_camera_position(
            0.34, [0.205, 0.012, 0.385]
        )
        nominal = np.asarray(command.nominal_intercept_base_m)
        corrected = np.asarray(command.corrected_intercept_base_m)
        np.testing.assert_allclose(
            corrected - nominal, [0.004, -0.003, 0.002]
        )
        self.assertEqual(command.camera_sample_count, 1)
        self.assertGreater(command.time_since_release_s, 0.0)

    def test_two_camera_frames_fit_velocity(self):
        controller = OnlineInterceptController(
            release_command_time_s=0.30,
            prediction_horizon_s=0.09,
            policy=self.policy(),
        )
        controller.set_encoder_detach_prior(
            0.32, [0.2, 0.0, 0.4], [0.3, 0.0, 0.3]
        )
        controller.add_global_camera_position(0.34, [0.206, 0.0, 0.404])
        command = controller.add_global_camera_position(
            0.36, [0.212, 0.0, 0.404076]
        )
        self.assertEqual(command.camera_sample_count, 2)
        self.assertIsNotNone(command.fit_rms_m)
        self.assertEqual(len(command.corrected_intercept_base_m), 3)

    def test_checkpoint_loader_uses_frozen_schema(self):
        payload = {
            "schema": "xarm6_intercept_residual_v1",
            "feature_names": [
                "time_since_release_s", "camera_sample_count", "fit_rms_m",
                "position_innovation_x_m", "position_innovation_y_m",
                "position_innovation_z_m", "velocity_innovation_x_m_s",
                "velocity_innovation_y_m_s", "velocity_innovation_z_m_s"
            ],
            "feature_mean": [0.0] * 9,
            "feature_scale": [1.0] * 9,
            "weight": [[0.0] * 9 for _ in range(3)],
            "bias": [0.0, 0.0, 0.0],
            "action_norm_limit_m": 0.02,
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "model.json"
            path.write_text(json.dumps(payload))
            controller = OnlineInterceptController.from_checkpoint(
                path, release_command_time_s=0.3, prediction_horizon_s=0.09
            )
        self.assertEqual(controller.policy.action_norm_limit_m, 0.02)


if __name__ == "__main__":
    unittest.main()
