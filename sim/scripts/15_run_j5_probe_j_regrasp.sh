#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
OUTPUT="${1:-$ROOT/outputs/j5_forward_rotation/probe_j_dynamic_regrasp}"
EXTRA_ARGS=("${@:2}")

bash "$ROOT/sim/scripts/14_run_j5_rotation_ladder.sh" \
  1p6 \
  "$OUTPUT" \
  --wrist-camera-hardware-removed \
  --observation-mode proprioceptive \
  --detach-observer-drive-delta-rad 0.0001 \
  --detach-observer-opening-effort-nm 0.05 \
  --probe-j-config \
  "$ROOT/sim/configs/probe_j_j5_dynamic_regrasp_v2.json" \
  --catch-gripper-effort-limit-n 4.0 \
  --catch-gripper-stiffness 60.0 \
  --catch-evidence-window-s 0.50 \
  "${EXTRA_ARGS[@]}"

echo "probe_j=$OUTPUT/probe_j.json"
echo "summary=$OUTPUT/summary.json"
echo "spectator=$OUTPUT/spectator.mp4"
echo "third_view=$OUTPUT/spectator_third_view.mp4"
