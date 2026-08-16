#!/usr/bin/env bash
set -u

XARM_ROOT=/home/ubuntu/toss_project/xarm_6
ISAACLAB_ROOT=/home/ubuntu/IsaacLab-3.0.0-beta2
URDF_PATH="$XARM_ROOT/toss_project_sim_handoff/toss_project/real_cube_demo/urdf/xarm6_with_gripper_g1.urdf"
OUTPUT_DIR="$XARM_ROOT/outputs/assets/xarm6_g1"
LOG_PATH="$XARM_ROOT/outputs/logs/urdf_convert.log"

mkdir -p "$OUTPUT_DIR" "$(dirname "$LOG_PATH")"
export OMNI_KIT_ACCEPT_EULA=yes
cd "$ISAACLAB_ROOT" || exit 1

env -u VIRTUAL_ENV -u CONDA_PREFIX ./isaaclab.sh -p scripts/tools/convert_urdf.py \
  "$URDF_PATH" \
  "$OUTPUT_DIR" \
  --fix-base \
  --joint-stiffness 80 \
  --joint-damping 8 \
  --headless \
  >"$LOG_PATH" 2>&1
status=$?

echo "EXIT:$status" >>"$LOG_PATH"
exit "$status"
