#!/usr/bin/env bash
set -u

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
OUTPUT_ROOT="$ROOT/outputs/probe_roll"
mkdir -p "$OUTPUT_ROOT"

for item in m1570:-1.5708 m0785:-0.7854 zero:0 p0785:0.7854 p1570:1.5708; do
  label="${item%%:*}"
  roll="${item#*:}"
  OMNI_KIT_ACCEPT_EULA=Y ACCEPT_EULA=Y PRIVACY_CONSENT=Y \
    env -u VIRTUAL_ENV -u CONDA_PREFIX \
    /home/ubuntu/IsaacLab-3.0.0-beta2/isaaclab.sh -p \
    "$ROOT/sim/scripts/04_native_release_smoke.py" \
    --headless \
    --config "$ROOT/sim/configs/outward_minimal_v1.json" \
    --observation-mode physics \
    --held-drive-rad 0.56 \
    --partial-open-drive-rad 0.40 \
    --cube-mass-kg 0.035 \
    --release-time-s 0.69 \
    --detach-delay-prior-s 0.035 \
    --post-release-s 0.30 \
    --joint6-roll-offset-rad "$roll" \
    --output "$OUTPUT_ROOT/$label" \
    > "$OUTPUT_ROOT/$label.log" 2>&1 || true
done

env -u VIRTUAL_ENV -u CONDA_PREFIX \
  /home/ubuntu/IsaacLab-3.0.0-beta2/isaaclab.sh -p \
  - "$OUTPUT_ROOT" <<'PY'
import json
from pathlib import Path
import sys
root = Path(sys.argv[1])
rows = []
for path in sorted(root.glob("*/summary.json")):
    summary = json.loads(path.read_text())
    trajectory = json.loads((path.parent / "trajectory.json").read_text())
    detach = min(trajectory, key=lambda row: abs(row["time_s"] - summary["detach_time_s"]))
    rows.append({"label": path.parent.name, "prethrow_stable": summary["prethrow_stable"], "detach_delay_s": summary["detach_delay_s"], "detach_velocity_m_s": detach["cube_linear_velocity_w_m_s"]})
(root / "summary.json").write_text(json.dumps(rows, indent=2) + "\n")
print(json.dumps(rows, indent=2))
PY
