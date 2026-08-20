#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
OUTPUT="${1:-$ROOT/outputs/j5_forward_rotation/probe_j_regrasp}"

bash "$ROOT/sim/scripts/14_run_j5_rotation_ladder.sh" \
  1p6 \
  "$OUTPUT" \
  --probe-j-config \
  "$ROOT/sim/configs/probe_j_j5_forward_rotation_v1.json" \
  --catch-lock-wrist \
  --catch-gripper-effort-limit-n 4.0 \
  --catch-gripper-stiffness 60.0 \
  --catch-evidence-window-s 0.25

echo "probe_j=$OUTPUT/probe_j.json"
echo "spectator=$OUTPUT/spectator.mp4"
echo "third_view=$OUTPUT/spectator_third_view.mp4"
echo "wrist=$OUTPUT/spectator_wrist.mp4"
