#!/usr/bin/env bash

set -u
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
PYTHON=/home/ubuntu/IsaacLab-3.0.0-beta2/env_isaaclab/bin/python
OUTPUT_ROOT="$ROOT/sim/outputs/nominal_learned_3_v2"
mkdir -p "$OUTPUT_ROOT"

for trial in 1 2 3; do
  output="$OUTPUT_ROOT/trial_${trial}"
  log="$OUTPUT_ROOT/trial_${trial}.log"
  OMNI_KIT_ACCEPT_EULA=Y ACCEPT_EULA=Y PRIVACY_CONSENT=Y \
    "$PYTHON" "$ROOT/sim/scripts/04_native_release_smoke.py" \
    --enable_cameras \
    --headless \
    --config "$ROOT/sim/configs/ballistic_throw_v1.json" \
    --observation-mode global_camera \
    --held-drive-rad 0.58 \
    --partial-open-drive-rad 0.40 \
    --cube-offset-hand-m 0 0 0.029 \
    --cube-mass-kg 0.035 \
    --release-time-s 0.30 \
    --detach-delay-prior-s 0.025 \
    --catch-servo-start-time-s 0.32 \
    --vision-control-end-time-s 0.40 \
    --catch-prediction-horizon-s 0.09 \
    --intercept-residual-model "$ROOT/sim/models/intercept_residual_v1.json" \
    --catch-servo-gain 1.0 \
    --catch-max-joint-step-rad 0.06 \
    --catch-close-time-s 0.42 \
    --output "$output" > "$log" 2>&1
done

"$PYTHON" - "$OUTPUT_ROOT" <<'PY'
import json
from pathlib import Path
import sys

root = Path(sys.argv[1])
summaries = [
    json.loads((root / f"trial_{index}" / "summary.json").read_text())
    for index in range(1, 4)
]
result = {
    "schema": "xarm6_nominal_learned_3_v1",
    "trial_count": len(summaries),
    "success_count": sum(item["catch_stable"] is True for item in summaries),
    "all_bilateral": all(
        item["bilateral_contact_fraction"] >= 0.9 for item in summaries
    ),
    "all_postdetach_learned": all(
        item["learned_control_updates_after_detach"] >= 1 for item in summaries
    ),
    "mean_intercept_error_before_m": sum(
        item["intercept_mean_error_before_residual_m"] for item in summaries
    ) / len(summaries),
    "mean_intercept_error_after_m": sum(
        item["intercept_mean_error_after_residual_m"] for item in summaries
    ) / len(summaries),
}
(root / "summary.json").write_text(json.dumps(result, indent=2) + "\n")
print(json.dumps(result, indent=2))
PY
