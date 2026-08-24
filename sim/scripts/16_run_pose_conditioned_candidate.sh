#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
if [[ $# -ne 4 ]]; then
  echo "usage: $0 OBJECT_PROFILE CANDIDATE_CONFIG CANDIDATE_REPORT OUTPUT_DIR" >&2
  exit 2
fi
OBJECT_PROFILE="$1"
CONFIG="$2"
REPORT="$3"
OUTPUT="$4"
for path in "$OBJECT_PROFILE" "$CONFIG" "$REPORT"; do
  [[ -f "$path" ]] || { echo "missing input: $path" >&2; exit 2; }
done
mkdir -p "$OUTPUT"

mapfile -t PARAMS < <("$ROOT/../.venv/bin/python" - "$OBJECT_PROFILE" "$REPORT" <<'PY'
import json
from pathlib import Path
import sys
profile = json.loads(Path(sys.argv[1]).read_text())
report = json.loads(Path(sys.argv[2]).read_text())
if report["object_id"] != profile["object_id"]:
    raise SystemExit("candidate report and object profile disagree")
cal = profile["sim_open_loop_calibration"]
action = report["selected_action"]
values = [
    *cal["cube_offset_hand_m"],
    cal["held_drive_rad"],
    cal["release_preopen_drive_rad"],
    cal["release_time_s"],
    cal["gripper_open_command_time_s"],
    cal["gripper_preopen_command_time_s"],
    cal["gripper_transition_s"],
    action["catch_intercept_time_s"],
    action["catch_preclose_time_s"],
    action["catch_close_time_s"],
    cal["catch_preclose_drive_rad"],
    cal["catch_drive_rad"],
]
for value in values:
    print(float(value))
PY
)
if [[ ${#PARAMS[@]} -ne 14 ]]; then
  echo "candidate parameter extraction failed" >&2
  exit 2
fi

OMNI_KIT_ACCEPT_EULA=Y ACCEPT_EULA=Y PRIVACY_CONSENT=Y \
  env -u VIRTUAL_ENV -u CONDA_PREFIX \
  /home/ubuntu/IsaacLab-3.0.0-beta2/isaaclab.sh -p \
  "$ROOT/sim/scripts/04_native_release_smoke.py" \
  --headless \
  --enable_cameras \
  --config "$CONFIG" \
  --object-profile "$OBJECT_PROFILE" \
  --output "$OUTPUT" \
  --video-path "$OUTPUT/spectator.mp4" \
  --record-policy-cameras \
  --wrist-camera-hardware-removed \
  --observation-mode proprioceptive \
  --cube-offset-hand-m "${PARAMS[0]}" "${PARAMS[1]}" "${PARAMS[2]}" \
  --held-drive-rad "${PARAMS[3]}" \
  --held-gripper-effort-limit-n 1.0 \
  --partial-open-drive-rad 0.20 \
  --release-gripper-effort-limit-n 0.0 \
  --release-gripper-stiffness 0.0 \
  --settle-s 0.50 \
  --post-release-s 0.85 \
  --release-time-s "${PARAMS[5]}" \
  --gripper-open-command-time-s "${PARAMS[6]}" \
  --gripper-preopen-command-time-s "${PARAMS[7]}" \
  --gripper-preopen-drive-rad "${PARAMS[4]}" \
  --gripper-preopen-transition-s "${PARAMS[8]}" \
  --release-drive-transition-s "${PARAMS[8]}" \
  --release-drive-start-delay-s 0.02264 \
  --release-dynamics-after-transition \
  --detach-delay-prior-s 0.035 \
  --arm-tracking-delay-s 0.08 \
  --arm-drive-interpolation linear \
  --arm-sim-effort-scale 2.0 \
  --arm-sim-stiffness-scale 5.0 \
  --catch-servo-start-time-s 0.68 \
  --catch-intercept-time-s "${PARAMS[9]}" \
  --catch-preclose-time-s "${PARAMS[10]}" \
  --catch-close-time-s "${PARAMS[11]}" \
  --catch-preclose-drive-rad "${PARAMS[12]}" \
  --catch-drive-rad "${PARAMS[13]}" \
  --catch-gripper-effort-limit-n 4.0 \
  --catch-gripper-stiffness 60.0 \
  --catch-j235-only \
  > "$OUTPUT/run.log" 2>&1
