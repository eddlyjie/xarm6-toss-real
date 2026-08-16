# Model sources

`xarm6_with_gripper_g1.urdf` combines:

- the xArm 6 model from `RobotCamCalib/RobotCamCalib/assets/robots/xarm6`;
- the UFACTORY G1 gripper links, joints, inertial values, and STL meshes from the Humble branch of `xArm-Developer/xarm_ros2`.

The gripper is fixed to the existing `link_eef`. Its movable `drive_joint` is mirrored by the other five finger joints, matching the official G1 model. `link_tcp` is 0.172 m along the gripper base z-axis.

The camera transforms stay in the calibration YAML files and are loaded separately; this avoids duplicating calibration values inside the robot model.

