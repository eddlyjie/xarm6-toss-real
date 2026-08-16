# xArm6 + G1 asset sources

This asset combines two descriptions:

- xArm6 arm kinematics, inertia and meshes: the locally calibrated
  `RobotCamCalib/RobotCamCalib/assets/robots/xarm6/xarm6_wo_ee.urdf`.
- G1 mechanism, inertia and meshes: UFACTORY's official `xarm_ros2`
  `humble` branch, specifically `xarm_description/urdf/gripper/` and
  `xarm_description/meshes/gripper/xarm/`.

The generated URDF uses the G1 branch, not the G2 mesh branch. The official
UF license is stored beside the asset as `UF_LICENSE`.

Rebuild from an official checkout with:

```bash
python sim/tools/build_xarm6_g1_urdf.py \
  --ufactory-root /path/to/xarm_ros2
```

The simulator command `drive_joint=0.0..0.85 rad` corresponds to the real G1
position command `0..850`; `joint_tcp` is 0.172 m from the gripper base.
