# xArm 6 Real Cube Demo

This folder is the smallest real-robot path toward the toss-and-catch project:

1. identify the connected robot, G1 gripper, and two RealSense cameras;
2. capture synchronized-per-camera RGB/depth snapshots;
3. accept a cube handed directly to the fixed gripper pose;
4. run a slow fixed-place sequence;
5. only then add cube localization and throwing/catching.

The first pick-and-place is intentionally pose-taught rather than vision-controlled. The cameras are connected now so that the real scene, cube appearance, depth quality, and calibration can be checked before visual localization is coupled to motion.

## What was reused

- `../xarm_6`: the real xArm SDK adapter and the larger toss/catch method contracts.
- `../RobotCamCalib/RobotCamCalib`: xArm 6 base URDF, camera intrinsics, and both camera extrinsics.
- `../dro_real-master`: reference for the former project's simple joint-command workflow. It is not imported at runtime because it depends on a separate `rel` package and different hands.
- UFACTORY `xarm_ros2`, Humble branch: the G1 gripper kinematics and meshes appended to the local xArm 6 URDF.

The resulting model is `urdf/xarm6_with_gripper_g1.urdf`. It attaches the standard UFACTORY G1 model to `link_eef`. The SDK, not the URDF, drives the physical gripper.

## Confirmed hardware

- Robot: xArm 6 at `192.168.2.232`
- Gripper: UFACTORY xArm Gripper G1; the G1 SDK calls respond and report gripper firmware `3.6.0`
- Global RealSense D435: `317222073552`
- Wrist RealSense D435: `233622079809`
- GelSight: not present in the current USB inventory, and it is not part of the standard G1 model
- Wrist six-axis force/torque sensor: not installed; use the available joint effort and motor current for the first proprioceptive Probe

## Environment

The existing `calib` conda environment already contains the needed SDKs:

```bash
conda run -n calib python scripts/00_hardware_inventory.py
```

Run all commands from this directory.

## 1. Capture both cameras

Place the cube in the global camera view and run:

```bash
conda run -n calib python scripts/01_capture_cameras.py
```

The command writes RGB, raw depth, depth visualization, a combined preview, and metadata under `outputs/captures/`. The calibrated YAML files remain the source of geometric transforms. The metadata also records the factory intrinsics reported by each physical camera.

For the current yellow cube, run the deliberately simple first detector:

```bash
conda run -n calib python scripts/detect_cube.py
```

It reports the cube center in the global-camera image and transforms its median depth point into the robot base frame with `extrinsics_thirdview.yaml`. This output is diagnostic for now; it is not yet sent directly to the robot.

## 2. Teach fixed poses

Teaching is optional now. The simpler handoff demo below already contains fixed poses.

Use the xArm controller/app to put the arm in manual mode and move it to each pose. At every pose, run one command:

```bash
conda run -n calib python scripts/02_record_pose.py --name home
conda run -n calib python scripts/02_record_pose.py --name pregrasp
conda run -n calib python scripts/02_record_pose.py --name grasp
conda run -n calib python scripts/02_record_pose.py --name lift
conda run -n calib python scripts/02_record_pose.py --name preplace
conda run -n calib python scripts/02_record_pose.py --name place
```

The joint angles are stored in radians in `configs/pick_place.json`. `pregrasp` and `preplace` should be clear approach poses; `lift` should raise the cube before moving sideways.

## 3. Test the G1 gripper

First inspect the planned values:

```bash
conda run -n calib python scripts/03_test_gripper.py
```

After clearing the fingers and checking the values in `configs/hardware.json`, set `motion_confirmed` to `true` and run:

```bash
conda run -n calib python scripts/03_test_gripper.py --execute
```

The test prints the SDK's blocking full-open/full-close duration. It includes the
G1 stop-confirmation polling time, so it is only a rough upper bound on the
mechanical travel time.

For toss/catch, do not use the full `0–850` travel. After measuring the held-cube position, time a small release/catch aperture instead:

```bash
conda run --no-capture-output -n calib python scripts/09_test_partial_gripper.py \
  --held-position 370 --release-position 520 --execute
```

This partial-travel test sends each target without blocking and polls the G1
position. It reports command-return time, the first observed movement, and the
time at which the target aperture is reached. Physical cube detach will be
measured separately with the global camera.

The measured `370 -> 520` partial travel takes about `0.103 s`. To measure when
the held cube actually starts falling, put a soft mat below the handoff pose and
run:

```bash
conda run --no-capture-output -n calib python scripts/10_measure_detach.py --execute
```

The global D435 records aligned color/depth at 60 Hz. The output includes an
annotated video, an event image strip, and the frame-by-frame cube position in
the robot base frame under `outputs/detach_trials/`.

## 4. Run fixed pick-and-place

Dry-run first:

```bash
conda run -n calib python scripts/04_pick_and_place.py
```

If the printed sequence and taught poses are correct:

```bash
conda run -n calib python scripts/04_pick_and_place.py --execute
```

This is deliberately slow. Throwing is a later stage and must use a continuous release trajectory rather than scaling up this pick-and-place sequence.

## 5. Fixed handoff and place

This is the shortest real demo: the robot opens at the current handoff pose, waits for you to put the cube between the fingers, closes, and places it at the table location measured by the global camera.

```bash
conda run -n calib python scripts/05_handoff_place.py
```

The dry-run prints the hard-coded joint and TCP targets. Remove the cube currently occupying the destination. To execute, set `motion_confirmed` to `true` in `configs/hardware.json`, then run:

```bash
conda run -n calib python scripts/05_handoff_place.py --execute
```

## 6. Record torque/current signals

The robot can provide joint effort and motor current without GelSight. The read-only recorder is:

```bash
conda run -n calib python scripts/06_record_joint_signals.py --seconds 10
```

The proposed Probe and Detach data contract is described in `docs/TORQUE_PROBE_AND_DETACH.md`.

## 7. Run the first cube Probe

The first active Probe holds the hand-delivered cube and applies two smooth `0.04 rad` wrist tilts while streaming joint targets at 50 Hz. It records commanded/measured joints, velocity, effort, and current.

```bash
conda run --no-capture-output -n calib python scripts/07_probe_cube.py
conda run --no-capture-output -n calib python scripts/07_probe_cube.py --execute
```

The CSV and summary are written under `outputs/probes/`. This is an identification motion, not a throw trajectory.

After the cube run, repeat the identical motion with no object and the fingers fixed at the measured held-cube position. For the first cube run that position was `370`:

```bash
conda run --no-capture-output -n calib python scripts/07_probe_cube.py \
  --execute --condition empty --gripper-position 370
```

Then subtract the paired trials:

```bash
conda run -n calib python scripts/08_compare_probe.py \
  --cube outputs/probes/<cube-run> \
  --empty outputs/probes/<empty-run>
```

## 8. First observed ballistic toss and catch

The runtime does not take object mass or side length as inputs. The existing
Probe comparison is stored as an untrained feature posterior with no fabricated
physical mean or covariance. A later learned Probe model must infer inertial and
geometry beliefs from proprioception and camera observations.

The first cube attempt showed that simply braking after release does not put the
gripper at the cube's ballistic intercept. The global-camera recording from that
attempt estimates the grasped cube center at `[19.9, -8.3, -26.0] mm` in the TCP
frame. This is an observed geometry feature, not a supplied cube side length.

The revised release is higher and farther forward: the TCP is approximately
`[580, 18, 440] mm`, the fingers point `68.5 deg` above the horizon, and the G1
base is about `143 mm` below the observed cube center. The planner propagates the
measured release state under gravity, computes the catch TCP from the observed
grasp offset, and follows part of the cube's pose and velocity at interception.
No mass or side length is passed to this calculation.

The slow motion from the robot's current pose to this release pose is setup
relocation, so it can move forward or sideways. It is not the throw. The actual
prethrow keeps world-frame `x/y` fixed and moves the TCP vertically from about
`[580, 18, 280]` to `[580, 18, 440] mm` while building the release rotation.

Inspect the plan without motion:

```bash
conda run --no-capture-output -n calib python scripts/11_spin_toss_and_catch.py
```

Move slowly to the new high, forward, upward-facing release posture without
running the toss or moving the gripper:

```bash
conda run --no-capture-output -n calib python scripts/11_spin_toss_and_catch.py \
  --inspect-release
```

Run the arm trajectory once with an empty gripper:

```bash
conda run --no-capture-output -n calib python scripts/11_spin_toss_and_catch.py \
  --execute-empty
```

After inspecting that new posture and trajectory, verify the compensated G1
open/close timing with no object between the fingers:

```bash
conda run --no-capture-output -n calib python scripts/11_spin_toss_and_catch.py \
  --execute-empty-gripper
```

The current controller follows the commanded ServoJ path with about `0.09 s`
of repeatable delay. Dynamic separation in the failed cube recording occurs
about `0.025 s` after the open command, earlier than the `0.044 s` stationary
drop estimate. The revised schedule therefore delays the open command and uses
a `0.18 s` flight: open and close commands are approximately `0.105 s` apart,
matching the measured `370 -> 520` G1 travel time.

Only after inspecting that run, use a soft mat and run the cube trial:

```bash
conda run --no-capture-output -n calib python scripts/11_spin_toss_and_catch.py \
  --execute-cube
```

Each executed run records joint command/feedback signals, temperatures, exact
G1 command times, and 60 Hz global-camera color/depth under
`outputs/spin_toss/`. The current nominal plan predicts a `0.18 s` free interval
and about `0.324 rad` of horizontal object rotation. These are predictions to
compare against the video, not injected object-property labels.

Analyze an executed cube run and reproduce the read-only release/catch search
with:

```bash
conda run --no-capture-output -n calib python scripts/14_analyze_toss_run.py \
  --run outputs/spin_toss/<cube-run>
conda run --no-capture-output -n calib python scripts/15_search_observed_catch.py \
  --run outputs/spin_toss/<cube-run> --final-search
```
