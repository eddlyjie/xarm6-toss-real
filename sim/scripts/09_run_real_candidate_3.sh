#!/usr/bin/env bash

set -u
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
PYTHON=/home/ubuntu/IsaacLab-3.0.0-beta2/env_isaaclab/bin/python
OUTPUT_ROOT="$ROOT/sim/outputs/real_candidate_learned_3"
mkdir -p "$OUTPUT_ROOT"

TRIALS=$(cat <<'EOF'
01 0.035  0.000  0.000 0.029 0.075
02 0.025 -0.001  0.000 0.028 0.070
03 0.045  0.000 -0.001 0.030 0.080
EOF
)

while read -r trial mass x y z prior; do
  output="$OUTPUT_ROOT/trial_$trial"
  log="$OUTPUT_ROOT/trial_$trial.log"
  video="$OUTPUT_ROOT/trial_$trial.mp4"
  OMNI_KIT_ACCEPT_EULA=Y ACCEPT_EULA=Y PRIVACY_CONSENT=Y \
    "$PYTHON" "$ROOT/sim/scripts/04_native_release_smoke.py" \
    --enable_cameras \
    --headless \
    --config "$ROOT/sim/configs/ballistic_throw_real_v1.json" \
    --observation-mode global_camera \
    --held-drive-rad 0.56 \
    --partial-open-drive-rad 0.40 \
    --cube-offset-hand-m "$x" "$y" "$z" \
    --cube-mass-kg "$mass" \
    --release-time-s 0.50 \
    --detach-delay-prior-s "$prior" \
    --catch-servo-start-time-s 0.52 \
    --vision-control-end-time-s 0.58 \
    --catch-prediction-horizon-s 0.05 \
    --intercept-residual-model \
    "$ROOT/sim/models/intercept_residual_real_v1.json" \
    --catch-servo-gain 1.0 \
    --catch-max-joint-step-rad 0.06 \
    --catch-close-time-s 0.58 \
    --video-path "$video" \
    --output "$output" > "$log" 2>&1
done <<< "$TRIALS"

"$PYTHON" - "$OUTPUT_ROOT" <<'PY'
import json
from pathlib import Path
import sys

root = Path(sys.argv[1])
summaries = [
    json.loads((root / f"trial_{index:02d}" / "summary.json").read_text())
    for index in range(1, 4)
]
result = {
    "schema": "xarm6_real_candidate_learned_3_v1",
    "trial_count": len(summaries),
    "success_count": sum(item["catch_stable"] is True for item in summaries),
    "all_bilateral": all(
        item["bilateral_contact_fraction"] >= 0.9 for item in summaries
    ),
    "all_postdetach_camera": all(
        item["camera_control_updates_after_detach"] >= 1 for item in summaries
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
    "video_paths": [str(root / f"trial_{index:02d}.mp4") for index in range(1, 4)],
}
(root / "summary.json").write_text(json.dumps(result, indent=2) + "\n")
print(json.dumps(result, indent=2))
PY
