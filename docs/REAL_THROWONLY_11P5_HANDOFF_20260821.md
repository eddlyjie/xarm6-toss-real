# Standard-G1 11.5-degree throw-only real handoff

Status: `sim_validated_real_unverified`. This is a high-energy soft-mat
throw-only checkpoint, not a recatch controller.

## Relationship to the existing trials

Keep the successful real 0.636/0.720 s micro-toss unchanged. Its measured
results are in `REAL_ROBOT_TEST_20260817.md`; it proves that the 20 ms servo,
1x arm timeline, and asynchronous G1 release/close can run together.

This checkpoint is a second, independent action. It uses the current
pose-rotation arm reference and the standard G1 to test whether the cube leaves
in the intended direction and visibly rotates forward. It sends no preclose or
final-close command. The v47 5-degree stable-regrasp is a third candidate; do
not merge the evidence from these three actions.

## Sim evidence and boundary

Native sim `v62_r90_two_stage_055`, with a fixed 35 mm and 25 g cube, produced:

| metric | value |
|---|---:|
| strict free flight | 0.394 s |
| signed forward rotation | 11.530 degrees |
| tumble-axis alignment | 0.974 |
| commanded peak joint speed | 1.7361 rad/s |
| commanded peak acceleration | 12.9340 rad/s2 |
| release command | 0.620 s |
| sim physical detach | 0.660 s |

Both `release_insert_geometry` and `release_retract_pad_geometry` are null. The
run used no roller, flipper, retract pad, or post-initialization cube-state
write. The 11.530-degree result is nominal sim evidence, not a real prediction.
Compact evidence is in:

```text
docs/media/j5_forward_rotation/standard_g1_throwonly_v62.json
```

## Hardware conditions

- Use the same light 35-40 mm cube and G1 370 to 520 at speed 5000.
- Remove the wrist D435, mount, and cable completely.
- Keep J4 near 165 degrees and J6 near -1.5 degrees static; J2/J3/J5 throw.
- Read, set, and read back `linear_spd_limit_factor=1.6`.
- Put a soft mat below the cube and keep an e-stop operator present.
- The global D435 records evidence only; it is not in high-speed control.

## Pull and inspect without connecting the robot

```bash
git pull --ff-only origin main

conda run --no-capture-output -n calib \
  python scripts/22_run_j5_dynamic_regrasp.py \
  --timeline real_handoff/standard_g1_throwonly_11p5deg_timeline.json \
  --controller real_handoff/standard_g1_throwonly_11p5deg_controller.json \
  --speed-scale 1.0
```

The plan must report zero robot commands, profile
`standard_g1_throwonly_11p5deg`, Probe status `not_used_for_throw_only`, release
time 0.62 s, and peak reference speed 1.73611027775 rad/s.

## Real execution order

These three commands do not operate G1. They only check the new arm reference:

```bash
conda run --no-capture-output -n calib python scripts/22_run_j5_dynamic_regrasp.py \
  --timeline real_handoff/standard_g1_throwonly_11p5deg_timeline.json \
  --controller real_handoff/standard_g1_throwonly_11p5deg_controller.json \
  --execute-empty-arm --speed-scale 0.25

conda run --no-capture-output -n calib python scripts/22_run_j5_dynamic_regrasp.py \
  --timeline real_handoff/standard_g1_throwonly_11p5deg_timeline.json \
  --controller real_handoff/standard_g1_throwonly_11p5deg_controller.json \
  --execute-empty-arm --speed-scale 0.5

conda run --no-capture-output -n calib python scripts/22_run_j5_dynamic_regrasp.py \
  --timeline real_handoff/standard_g1_throwonly_11p5deg_timeline.json \
  --controller real_handoff/standard_g1_throwonly_11p5deg_controller.json \
  --execute-empty-arm --speed-scale 1.0
```

Only after all three runs have no C60, joint/collision violation, or J4/J6 cable
clearance problem, run one cube trial:

```bash
conda run --no-capture-output -n calib python scripts/22_run_j5_dynamic_regrasp.py \
  --timeline real_handoff/standard_g1_throwonly_11p5deg_timeline.json \
  --controller real_handoff/standard_g1_throwonly_11p5deg_controller.json \
  --execute-throw-only --speed-scale 1.0
```

The runner first moves to the reference start, waits for cube placement, and
asks for a second operator confirmation. It sends G1 520 at 0.620 s. The
measured 25-44 ms G1 delay places expected physical detach at host
0.645-0.664 s. G1 then stays open and the cube must land on the soft mat.
Never use this command as a recatch action.

## Return data

Return the full 1x empty-arm and throw-only output directories from:

```text
toss_project_sim_handoff/toss_project/real_cube_demo/outputs/j5_dynamic_regrasp/
```

Include the global-camera native-speed video, controller error and command
count, 20 ms timing, actual q/dq, estimated tracking delay, G1 command/actual
position, observed detach time, and an operator label for direction, visible
rotation, separation, and any contact with the gripper or robot links.

Measure real cube rotation from the global-camera video. FK/Jacobian hand
angular velocity is only a release prior and is not observed cube rotation.
