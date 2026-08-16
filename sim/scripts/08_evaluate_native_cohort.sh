#!/usr/bin/env bash

set -u
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
PYTHON=/home/ubuntu/IsaacLab-3.0.0-beta2/env_isaaclab/bin/python
OUTPUT_ROOT="$ROOT/sim/outputs/native_cohort_v3"
mkdir -p "$OUTPUT_ROOT/fixed" "$OUTPUT_ROOT/learned"

TRIALS=$(cat <<'EOF'
01 0.300 0.035 0.560 0.400  0.000  0.000 0.029 0.025
02 0.295 0.025 0.560 0.400 -0.001  0.000 0.028 0.020
03 0.305 0.045 0.560 0.400  0.000  0.000 0.029 0.030
04 0.300 0.030 0.555 0.400  0.000  0.001 0.029 0.025
05 0.300 0.040 0.560 0.400 -0.001 -0.001 0.029 0.025
06 0.295 0.050 0.560 0.390 -0.001  0.000 0.028 0.020
07 0.305 0.020 0.565 0.410 -0.001 -0.001 0.029 0.035
08 0.300 0.028 0.560 0.400  0.001 -0.001 0.030 0.030
09 0.300 0.048 0.560 0.410  0.000  0.000 0.030 0.035
10 0.300 0.035 0.555 0.390  0.000  0.001 0.028 0.020
EOF
)

for mode in fixed learned; do
  while read -r trial release mass held partial x y z prior; do
    output="$OUTPUT_ROOT/$mode/trial_$trial"
    log="$OUTPUT_ROOT/$mode/trial_$trial.log"
    residual_args=()
    if [[ "$mode" == "learned" ]]; then
      residual_args=(
        --intercept-residual-model
        "$ROOT/sim/models/intercept_residual_v1.json"
      )
    fi
    OMNI_KIT_ACCEPT_EULA=Y ACCEPT_EULA=Y PRIVACY_CONSENT=Y \
      "$PYTHON" "$ROOT/sim/scripts/04_native_release_smoke.py" \
      --enable_cameras \
      --headless \
      --config "$ROOT/sim/configs/ballistic_throw_v1.json" \
      --observation-mode global_camera \
      --held-drive-rad "$held" \
      --partial-open-drive-rad "$partial" \
      --cube-offset-hand-m "$x" "$y" "$z" \
      --cube-mass-kg "$mass" \
      --release-time-s "$release" \
      --detach-delay-prior-s "$prior" \
      --catch-servo-start-time-s 0.32 \
      --vision-control-end-time-s 0.40 \
      --catch-prediction-horizon-s 0.09 \
      "${residual_args[@]}" \
      --catch-servo-gain 1.0 \
      --catch-max-joint-step-rad 0.06 \
      --catch-close-time-s 0.40 \
      --output "$output" > "$log" 2>&1
  done <<< "$TRIALS"
done

"$PYTHON" - "$OUTPUT_ROOT" <<'PY'
import json
from pathlib import Path
import sys

root = Path(sys.argv[1])
result = {"schema": "xarm6_native_cohort_v3", "trial_count_per_mode": 10}
mode_summaries = {}
for mode in ("fixed", "learned"):
    summaries = []
    missing = []
    for index in range(1, 11):
        path = root / mode / f"trial_{index:02d}" / "summary.json"
        if path.is_file():
            summaries.append(json.loads(path.read_text()))
        else:
            missing.append(index)
    mode_summaries[mode] = summaries
    success = [item for item in summaries if item["catch_stable"] is True]
    result[mode] = {
        "completed_count": len(summaries),
        "missing_trials": missing,
        "success_count": len(success),
        "bilateral_success_count": sum(
            item["catch_stable"] is True
            and item["bilateral_contact_fraction"] >= 0.9
            for item in summaries
        ),
        "postdetach_camera_count": sum(
            item["camera_control_updates_after_detach"] >= 1
            for item in summaries
        ),
        "postdetach_learned_count": sum(
            item["learned_control_updates_after_detach"] >= 1
            for item in summaries
        ),
        "mean_intercept_error_before_m": (
            None if not summaries else sum(
                item["intercept_mean_error_before_residual_m"]
                for item in summaries
            ) / len(summaries)
        ),
        "mean_intercept_error_after_m": (
            None if not summaries else sum(
                item["intercept_mean_error_after_residual_m"]
                for item in summaries
            ) / len(summaries)
        ),
    }
fixed = result["fixed"]["mean_intercept_error_after_m"]
learned = result["learned"]["mean_intercept_error_after_m"]
result["comparison"] = {
    "learned_minus_fixed_successes": (
        result["learned"]["success_count"]
        - result["fixed"]["success_count"]
    ),
    "learned_intercept_error_reduction_fraction": (
        None if fixed is None or learned is None or fixed == 0.0
        else (fixed - learned) / fixed
    ),
}
(root / "summary.json").write_text(json.dumps(result, indent=2) + "\n")
print(json.dumps(result, indent=2))
PY
