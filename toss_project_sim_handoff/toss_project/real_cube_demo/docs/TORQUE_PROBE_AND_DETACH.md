# Torque-based Probe and Detach without GelSight

The current robot has no GelSight and no working wrist six-axis force/torque sensor. It does expose six-axis joint position, velocity, effort/torque, motor current, and G1 gripper position. These signals are sufficient for a first proprioceptive Probe and Detach model.

## What the signals can tell us

- Slow positive/negative wrist tilts change gravity torque and provide information about object mass and center of mass.
- A small bounded shake adds information about inertia.
- Changes in joint-effort and current residuals can indicate contact loading or gross slip.
- Gripper position and the commanded opening time describe the release actuator state.
- The global camera supplies the essential supervision: actual detach frame, flight velocity, angular velocity, and landing result.

These are lower-dimensional and less local than fingertip images. They will not directly reveal left/right contact patches, pressure maps, or small incipient slip. The real model should therefore call this branch `proprioceptive_contact`, even if it occupies the simulator model's existing temporal-contact input slot.

## Useful 18-D temporal input

The existing Detach GRU expects 18 values per frame. The real xArm version can use:

```text
[joint_velocity_1..6,
 motor_current_residual_1..6,
 command_minus_measured_joint_1..6]
```

The gripper command phase and position can stay in the action features. Joint effort remains in the recorded dataset as an auxiliary signal, but the first paired real experiment showed that its static held-object residual was zero while motor current carried a strong shoulder/elbow load offset. This preserves the model shape without pretending that joint signals are bilateral tactile images.

## Data collection

Record the same short motion twice:

1. empty G1 gripper, as the robot/gripper baseline;
2. cube held in the G1 gripper.

Subtract the time-aligned empty-run effort and current from the held-object run. Raw effort contains a large gravity component, so the paired residual is more useful than a single absolute threshold. Start with a slow tilt in each direction, then add a small wrist oscillation after the fixed pick-and-place works.

For Detach training, record the final pre-release window together with:

- commanded and measured arm motion;
- gripper open command and position;
- global-camera object track and first free-flight frame;
- object dimensions and measured mass;
- whether release was valid.

The Detach target remains the 13-D residual already defined in `xarm_6`: detach time, position, rotation, linear velocity, and angular velocity residuals. GelSight is helpful but is not required to create these labels.

## First signal check

The recorder never commands motion:

```bash
conda run -n calib python scripts/06_record_joint_signals.py --seconds 10
```

Run once without touching the arm, then run again while lightly loading the gripped cube in different directions. Compare the joint-effort ranges and CSV traces. Do not use the absent `get_ft_sensor_data()` stream: on this robot it returns no physical signal and can trigger the controller's missing-accessory communication error.
