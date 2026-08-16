from pathlib import Path
import sys
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from xarm6_toss.intercept_residual import (
    FEATURE_NAMES,
    InterceptResidualPolicy,
    residual_features,
    ridge_fit,
)


class InterceptResidualTests(unittest.TestCase):
    def test_ridge_fit_recovers_affine_teacher(self):
        rng = np.random.default_rng(3)
        features = rng.normal(size=(300, len(FEATURE_NAMES)))
        teacher_weight = rng.normal(scale=0.003, size=(3, len(FEATURE_NAMES)))
        teacher_bias = np.asarray([0.003, -0.002, 0.001])
        targets = features @ teacher_weight.T + teacher_bias
        mean, scale, weight, bias = ridge_fit(features, targets, 1.0e-10)
        prediction = ((features - mean) / scale) @ weight.T + bias
        np.testing.assert_allclose(prediction, targets, atol=1.0e-9)

    def test_runtime_action_is_norm_bounded(self):
        policy = InterceptResidualPolicy.from_dict(
            {
                "schema": "xarm6_intercept_residual_v1",
                "feature_names": list(FEATURE_NAMES),
                "feature_mean": [0.0] * len(FEATURE_NAMES),
                "feature_scale": [1.0] * len(FEATURE_NAMES),
                "weight": [[1.0] * len(FEATURE_NAMES)] * 3,
                "bias": [0.0, 0.0, 0.0],
                "action_norm_limit_m": 0.02,
            }
        )
        action = policy.predict(np.ones(len(FEATURE_NAMES)))
        self.assertAlmostEqual(float(np.linalg.norm(action)), 0.02)

    def test_feature_contract_contains_no_truth_or_hidden_physics(self):
        joined = " ".join(FEATURE_NAMES)
        self.assertNotIn("truth", joined)
        self.assertNotIn("mass", joined)
        self.assertNotIn("friction", joined)
        feature = residual_features(
            time_since_release_s=0.06,
            camera_sample_count=3,
            fit_rms_m=0.001,
            position_innovation_m=[0.01, 0.0, -0.01],
            velocity_innovation_m_s=[0.1, 0.0, -0.2],
        )
        self.assertEqual(feature.shape, (len(FEATURE_NAMES),))


if __name__ == "__main__":
    unittest.main()
