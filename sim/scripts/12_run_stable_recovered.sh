#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
OUTPUT="${1:-$ROOT/outputs/stable_recovered_probe_j}"
mkdir -p "$OUTPUT"

OMNI_KIT_ACCEPT_EULA=Y ACCEPT_EULA=Y PRIVACY_CONSENT=Y \
  env -u VIRTUAL_ENV -u CONDA_PREFIX \
  /home/ubuntu/IsaacLab-3.0.0-beta2/isaaclab.sh -p \
  "$ROOT/sim/scripts/04_native_release_smoke.py" \
  --headless \
  --enable_cameras \
  --config "$ROOT/sim/configs/camera_under_tumble_stable_recovered.json" \
  --probe-j-config "$ROOT/sim/configs/probe_j_stable_recovered_v1.json" \
  --output "$OUTPUT" \
  --video-path "$OUTPUT/spectator.mp4" \
  --record-policy-cameras \
  --observation-mode proprioceptive \
  --cube-size-m 0.035 \
  --cube-mass-kg 0.025 \
  --cube-offset-hand-m 0.0 0.0 0.006 \
  --held-drive-rad 0.56 \
  --held-gripper-effort-limit-n 4.0 \
  --partial-open-drive-rad 0.39 \
  --release-gripper-effort-limit-n 20.0 \
  --release-gripper-stiffness 200.0 \
  --catch-drive-rad 0.56 \
  --catch-gripper-effort-limit-n 20.0 \
  --catch-gripper-stiffness 200.0 \
  --settle-s 0.50 \
  --post-release-s 0.80 \
  --release-time-s 0.62 \
  --gripper-open-command-time-s 0.60 \
  --release-drive-transition-s 0.01 \
  --detach-delay-prior-s 0.035 \
  --catch-servo-start-time-s 0.70 \
  --catch-intercept-time-s 0.74 \
  --catch-close-time-s 0.74 \
  --vision-control-end-time-s 0.74 \
  --catch-lock-wrist \
  --arm-tracking-delay-s 0.08 \
  --arm-drive-interpolation linear \
  --arm-sim-effort-scale 2.0 \
  --arm-sim-stiffness-scale 2.5 \
  > "$OUTPUT/run.log" 2>&1

echo "stable_recovered_output=$OUTPUT"
echo "summary=$OUTPUT/summary.json"
echo "probe_j=$OUTPUT/probe_j.json"
echo "spectator=$OUTPUT/spectator.mp4"
echo "third_view=$OUTPUT/spectator_third_view.mp4"
echo "wrist=$OUTPUT/spectator_wrist.mp4"
