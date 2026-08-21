#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
OUTPUT="${1:-$ROOT/outputs/stock_g1_10deg_v86}"
EXTRA_ARGS=("${@:2}")

bash "$ROOT/sim/scripts/14_run_j5_rotation_ladder.sh" \
  r10cfh "$OUTPUT" \
  --release-time-s 0.66 \
  --gripper-open-command-time-s 0.62 \
  --gripper-preopen-command-time-s 0.36 \
  --gripper-preopen-drive-rad 0.55 \
  --gripper-preopen-transition-s 0.10279 \
  --detach-observer-drive-delta-rad 0.0001 \
  --detach-observer-opening-effort-nm 0.20 \
  --wrist-camera-hardware-removed \
  --catch-servo-start-time-s 0.68 \
  --catch-intercept-time-s 1.00 \
  --catch-preclose-time-s 0.84 \
  --catch-preclose-drive-rad 0.48 \
  --catch-close-time-s 0.92 \
  --catch-drive-rad 0.65 \
  --catch-gripper-effort-limit-n 4.0 \
  --catch-gripper-stiffness 60.0 \
  --catch-lock-wrist \
  --vision-control-end-time-s 1.04 \
  --catch-position-bias-m 0 0 0 \
  --catch-preposition-bias-m 0.0028 -0.0095 0.0017 \
  --catch-preposition-start-time-s 0.72 \
  --catch-preposition-end-time-s 0.96 \
  "${EXTRA_ARGS[@]}"

echo "stock_g1_regrasp_target_deg=10"
echo "output=$OUTPUT"
echo "summary=$OUTPUT/summary.json"
echo "trajectory=$OUTPUT/trajectory.json"
echo "spectator=$OUTPUT/spectator.mp4"
echo "third_view=$OUTPUT/spectator_third_view.mp4"
echo "wrist=$OUTPUT/spectator_wrist.mp4"
