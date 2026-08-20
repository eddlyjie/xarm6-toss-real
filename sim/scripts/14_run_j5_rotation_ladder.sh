#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
TARGET="${1:-0p8}"
case "$TARGET" in
  0p8|1p2|1p6|2p0|2p4h) ;;
  *)
    echo "target must be one of: 0p8, 1p2, 1p6, 2p0, 2p4h" >&2
    exit 2
    ;;
esac
OUTPUT="${2:-$ROOT/outputs/j5_forward_rotation/$TARGET}"
EXTRA_ARGS=("${@:3}")
CONFIG="$ROOT/sim/configs/j5_forward_rotation_throwonly_$TARGET.json"
mkdir -p "$OUTPUT"

OMNI_KIT_ACCEPT_EULA=Y ACCEPT_EULA=Y PRIVACY_CONSENT=Y \
  env -u VIRTUAL_ENV -u CONDA_PREFIX \
  /home/ubuntu/IsaacLab-3.0.0-beta2/isaaclab.sh -p \
  "$ROOT/sim/scripts/04_native_release_smoke.py" \
  --headless \
  --enable_cameras \
  --config "$CONFIG" \
  --output "$OUTPUT" \
  --video-path "$OUTPUT/spectator.mp4" \
  --record-policy-cameras \
  --observation-mode proprioceptive \
  --cube-size-m 0.035 \
  --cube-mass-kg 0.025 \
  --cube-offset-hand-m 0.004 0.0 0.024 \
  --held-drive-rad 0.56 \
  --held-gripper-effort-limit-n 0.25 \
  --partial-open-drive-rad 0.39 \
  --release-gripper-effort-limit-n 0.0 \
  --release-gripper-stiffness 0.0 \
  --settle-s 0.50 \
  --post-release-s 0.65 \
  --release-time-s 0.612 \
  --gripper-open-command-time-s 0.612 \
  --release-drive-transition-s 0.01 \
  --release-dynamics-after-transition \
  --detach-delay-prior-s 0.035 \
  --arm-tracking-delay-s 0.08 \
  --arm-drive-interpolation linear \
  --arm-sim-effort-scale 2.0 \
  --arm-sim-stiffness-scale 5.0 \
  "${EXTRA_ARGS[@]}" \
  > "$OUTPUT/run.log" 2>&1

echo "j5_rotation_target=$TARGET"
echo "output=$OUTPUT"
echo "summary=$OUTPUT/summary.json"
echo "trajectory=$OUTPUT/trajectory.json"
echo "spectator=$OUTPUT/spectator.mp4"
echo "third_view=$OUTPUT/spectator_third_view.mp4"
echo "wrist=$OUTPUT/spectator_wrist.mp4"
