# xArm6 + G1 cube simulation

## Current result

The native runner now performs physical detach, rendered global-D435 tracking,
gravity-constrained fitting, learned intercept correction, bilateral catch and
0.5 s stable hold without post-initialization cube-state writes. The real-safe
candidate is 3/3; the wider fixed/learned cohort is 8/10. Reproduce with
`09_run_real_candidate_3.sh`; transfer artifacts are in `../real_handoff/`.

The milestone list below is the original bring-up roadmap and is retained only
as history.

This directory is the Isaac Sim/Isaac Lab side of the real toss-and-catch
project. It preserves the Panda reference's useful contract—paired arm `q/dq`
and separately timed gripper events—while rebuilding the robot, contacts and
trajectories for xArm6.

## Current first milestone

1. Import and inspect the self-contained xArm6 + G1 asset.
2. Spawn one small cube and establish a stable grasp/release.
3. Replay the scripted upward throw seed with simulator ground-truth state.
4. Move the arm to an actual intercept and close G1 for an open-loop catch.

The simulator must know cube physics to integrate motion, but these parameters
are explicitly excluded from policy observations. Cameras and learning come
after the single-environment contact sequence works.

## Hardware-independent checks

```bash
cd /path/to/toss_project/xarm_6
PYTHONPATH=src python sim/scripts/01_preview_reference.py
conda run --no-capture-output -n calib \
  python sim/tools/check_kinematic_parity.py
python sim/scripts/00_runtime_probe.py
```

The checked-in asset is rebuilt from the locally calibrated arm and an
official UFACTORY `xarm_ros2` checkout:

```bash
python sim/tools/build_xarm6_g1_urdf.py \
  --ufactory-root /path/to/xarm_ros2
```

## Isaac import

With Isaac Sim 6 available in the active Python environment:

```bash
python sim/scripts/02_import_urdf_to_usd.py --headless
```

The Isaac-specific script is isolated because the other simulation computer
may use a different Isaac release. The asset, scenario and control-reference
modules do not import Isaac and remain shared across versions.

## Important units

- xArm joints: radians and radians/second.
- Real G1 command: `0` closed, `850` open.
- Sim G1 `drive_joint`: `0.0` closed, `0.85 rad` open.
- G1 TCP: `0.172 m` along gripper base `+z`.
- Physics/control periods in the initial scenario: `5 ms / 20 ms`.
