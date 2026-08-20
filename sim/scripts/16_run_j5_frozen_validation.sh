#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
OUTPUT_ROOT="${1:-$ROOT/outputs/j5_forward_rotation/v47_frozen_validation_20260821}"
SEEDS=(202608211 202608212 202608213 202608214 202608215)

mkdir -p "$OUTPUT_ROOT"
for seed in "${SEEDS[@]}"; do
  output="$OUTPUT_ROOT/repeat_$seed"
  if [[ -e "$output" ]]; then
    echo "refusing to overwrite existing validation output: $output" >&2
    exit 2
  fi
  bash "$ROOT/sim/scripts/15_run_j5_probe_j_regrasp.sh" \
    "$output" \
    --camera-seed "$seed"
done

echo "frozen_config=sim/configs/j5_forward_rotation_throwonly_1p6.json"
echo "probe_j_config=sim/configs/probe_j_j5_dynamic_regrasp_v2.json"
echo "observation_mode=proprioceptive"
echo "camera_seed_affects_control=false"
echo "output_root=$OUTPUT_ROOT"
for seed in "${SEEDS[@]}"; do
  echo "summary=$OUTPUT_ROOT/repeat_$seed/summary.json"
  echo "spectator=$OUTPUT_ROOT/repeat_$seed/spectator.mp4"
  echo "third_view=$OUTPUT_ROOT/repeat_$seed/spectator_third_view.mp4"
done
