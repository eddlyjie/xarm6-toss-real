#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
OUTPUT="${1:-$ROOT/outputs/visible_spin_natural_proprio_v1}"
mkdir -p "$OUTPUT"

OMNI_KIT_ACCEPT_EULA=Y ACCEPT_EULA=Y PRIVACY_CONSENT=Y \
  env -u VIRTUAL_ENV -u CONDA_PREFIX \
  /home/ubuntu/IsaacLab-3.0.0-beta2/isaaclab.sh -p \
  "$ROOT/sim/scripts/04_native_release_smoke.py" \
  --headless \
  --enable_cameras \
  --config "$ROOT/sim/configs/outward_vertical_real_detach_v7.json" \
  --output "$OUTPUT" \
  --video-path "$OUTPUT/spectator.mp4" \
  --observation-mode proprioceptive \
  --record-policy-cameras \
  --held-drive-rad 0.56 \
  --partial-open-drive-rad 0.30 \
  --catch-drive-rad 0.60 \
  --catch-gripper-stiffness 40.0 \
  --cube-size-m 0.038 \
  --cube-mass-kg 0.035 \
  --cube-offset-hand-m 0.0 -0.002 0.0 \
  --release-time-s 0.690 \
  --gripper-open-command-time-s 0.655 \
  --release-drive-start-delay-s 0.025 \
  --release-drive-transition-s 0.035 \
  --detach-delay-prior-s 0.030 \
  --catch-servo-start-time-s 0.700 \
  --catch-close-time-s 0.800 \
  --vision-control-end-time-s 1.050 \
  --catch-intercept-time-s 0.840 \
  --catch-position-bias-m 0.0 0.0135 0.0 \
  --catch-lateral-only \
  --catch-hold-throw-joints \
  --catch-max-joint-speed-rad-s 3.10 \
  --catch-max-joint-acceleration-rad-s2 20.0 \
  --catch-evidence-window-s 0.25 \
  --arm-tracking-delay-s 0.0 \
  --arm-drive-interpolation hold \
  --arm-sim-effort-scale 2.0 \
  --arm-sim-stiffness-scale 2.5 \
  --post-release-s 0.50 \
  > "$OUTPUT/run.log" 2>&1

echo "visible_spin_output=$OUTPUT"
