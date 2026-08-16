# Panda 控制流程与 xArm 6 迁移

## 1. Panda 实际在控制什么

仿真 runner 很大，但核心控制循环并不神秘：每个 physics/control step 读取机器人
`q/dq`、夹爪/触觉和相机观测，根据当前 phase 输出 joint reference 与 gripper command。
压缩后是：

```python
while not done:
    obs = read_robot_camera_tactile()
    phase = state_machine.update(obs, clock)

    if phase == PROBE:
        q_ref, dq_ref = probe_reference(clock)
    elif phase in (THROW, FLIGHT, CATCH):
        q_ref, dq_ref = selected_whole_arm_plan.sample(clock)
    elif phase == POSTCATCH_TRANSPORT:
        q_ref, dq_ref = selected_residual_plan.sample(clock)
    else:
        q_ref, dq_ref = hold_or_transition_reference(clock)

    gripper_ref = gripper_schedule(
        release_time=release_time,
        catch_time=catch_time,
        close_lead=close_lead,
    )
    command_arm(q_ref, dq_ref)
    command_gripper(gripper_ref)
```

`q_ref` 和 `dq_ref` 必须来自同一条时间参数化计划。只发 position、把 velocity 永远
设成零，会改变高速 release/catch 的跟踪行为。xArm 6 SDK 最终采用 position servo、
velocity control 还是 controller-side trajectory，需要真机电脑根据固件接口决定；但日志
必须同时保存计划值和编码器实测值。

## 2. 阶段与关键输入/输出

| 阶段 | 输入 | 输出控制 | xArm 6 第一版 |
|---|---|---|---|
| table pick | 全局/腕部 RGB-D、示教 grasp | approach、close、lift | 先用固定示教 pose |
| Active Probe | `q/dq`、夹爪、触觉/电流 | 小幅 tilt/chirp/shake | 先固定一条短 probe；无触觉时用 encoder/电流 |
| Detach | Probe posterior、release action、局部几何 | 离手时刻/pose/速度分布 | 先用标定 release delay，随后收数据学 residual |
| Flight | detach belief | 多时刻 6-D object belief | 全局相机先做简单轨迹拟合 |
| Catch candidates | 点云表面、flight belief、robot state | 时间、双接触点、hand pose/velocity | 先保留 3–5 个离线验证动作 |
| `J_catch` / actor | 候选、uncertainty、target pose | 选中 candidate 和 whole-arm `q/dq` | 先规则评分，再换 learned actor |
| gripper catch | `catch_time-close_lead`、接触信号 | preshape、close、hold | 先按时序闭合；有电流/触觉后做事件修正 |
| postcatch pose | 腕部多视角 | `T_HO` | 停稳后移动到 2–4 个观察位 |
| target transport | `T_WO_target`、`T_HO`、FK | residual joint plan | 先慢速小距离，验证 pose 后再加速度 |

## 3. Probe、Detach 与 Catch 的关系

Probe selector 从有限安全动作库中选一次激励，目标是降低与当前任务相关的物理不确定性，
并惩罚 slip、时间、能量和峰值力。Probe 输出的 posterior 进入 Detach ensemble；Detach
再预测真实离手状态。它们不是一个“摇一摇然后硬编码抛出”的脚本。

Detach residual 是 13 维：

```text
[delta_time,
 delta_position_xyz,
 delta_rotation_so3_xyz,
 delta_linear_velocity_xyz,
 delta_angular_velocity_xyz]
```

飞行 belief 在多个未来时间生成接触候选。候选成本 `J_catch` 同时考虑 target grasp
error、相对接触速度、冲击、滑移、机械臂运动、不确定性和 catch probability。于是
catch pose、catch time、approach velocity、close lead 是一起选的，不是先固定转角再求 IK。

## 4. target pose 如何改变动作

设请求物体目标为 `T_WO_target`，某个 toss/regrasp skill 预测接住后的手物关系为
`T_HO_skill`。该 skill 对应的目标手 pose 为：

```text
T_WH_goal(skill) = T_WO_target · inverse(T_HO_skill)
```

M3 coordinator 对每个 skill 做 IK/轨迹可行性、catch probability、飞行不确定性、碰撞、
关节路程和目标 pose error 的联合评分。目标 upright 时可以选 `low_spin`；目标 45°/90°
时可以选 `quarter_turn_regrasp`。如果所有目标都选择同一 skill，说明系统仍退化成固定动作，
不能称作 pose-conditioned。

真机推荐的渐进实现：

1. 为小 cube 示教/优化 3 个 skill：low-spin、quarter-turn、larger-turn。
2. 每个 skill 保存 throw `q/dq`、release sample、catch `q/dq`、close lead 和预期 `T_HO`。
3. 输入 target pose 后，对三个 skill 计算可达性和代价，选最合适的一条。
4. 先在 seen cube 上覆盖 0°/45°/90°，确认 target 改变时 skill 确实改变。
5. 冻结规则/模型，再换尺寸或质量不同的 unseen cube；不要在 unseen 上反复调参后仍称 unseen。

## 5. xArm 6 接线建议

现有 `../src/xarm6_toss/xarm_adapter.py` 只实现安全的连接、状态和夹爪薄封装；
`../src/xarm6_toss/trajectory.py` 能离线生成第一条 throw-only trajectory。真机电脑下一步应加：

```text
XArm6Client.command_joint_reference(q, dq)
CameraRig.read_global_frame()/read_wrist_frame()
PoseEstimator.estimate_object_pose(...)
TrialRecorder.write_robot_state(...)/write_video(...)
ActionLibrary.select(target_pose, physical_belief)
```

不要把 Panda 的 7 维关节数组删掉一维后发给 xArm。正确做法是保留 phase/candidate/
timing，使用 xArm 6 的 URDF、FK、IK、关节限制和实测时延重新 materialize 6-DoF 轨迹。

伪代码到官方 SDK 的典型映射（具体函数以真机 SDK 版本为准）：

```python
# 明确进入运动模式并由实验者确认后，才允许循环发送。
arm.set_mode(1)
arm.set_state(0)

for sample in xarm_joint_plan:
    # sample.q_rad 来自 xArm 6 自己的计划，不是 Panda q。
    arm.set_servo_angle_j(
        angles=sample.q_rad,
        speed=joint_speed_rad_s,
        mvacc=joint_accel_rad_s2,
        is_radian=True,
    )
    if sample.open_gripper:
        arm.set_gripper_position(open_position, wait=False)
    if sample.close_gripper:
        arm.set_gripper_position(closed_position, wait=False)
```

开始高速控制前先确认该 SDK 调用是 controller-side interpolation 还是逐点 servo。若控制频率
和时延无法稳定复现 release，则优先把整段 trajectory 下发到控制器，而不是依赖 Windows/
Python 用户态精确定时。

## 6. 对应的 sim 源码地图

真机 Codex 需要深入时，优先读这些文件，不要先钻进巨大的 runner：

```text
toss_probe/active_probe/selector.py        Probe 信息增益选择
toss_probe/detach/model.py                 Detach ensemble 与 13-D residual
toss_probe/detach/probe_conditioning.py    Probe posterior→Detach/flight
toss_probe/catch/candidates.py             动态双接触候选
toss_probe/catch/reward.py                 J_catch 与 CVaR
toss_probe/catch/policy.py                 candidate-relative whole-arm action
toss_probe/catch/capture_j.py              接触后的 absorption/centering
toss_probe/catch/postcatch_observation.py  接后 RGB-D 与 hand-object pose
toss_probe/catch/postcatch_transport_feasibility.py  接后目标可行规划
scripts/isaac_lateral_release_smoke.py      完整 Isaac 状态机与执行接线
```

常用离线命令：

```bash
cd /path/to/xarm_6
python scripts/01_preview_throw.py \
  --plan configs/throw_only_cube.json \
  --output outputs/preview_throw.csv
python scripts/10_method_pipeline_demo.py --target upright_forward
python scripts/10_method_pipeline_demo.py --target quarter_turn_forward
python sim_reference/panda_sequence_reference.py
python -m unittest discover -s tests -v
```

这些命令均不会默认运行仿真或移动真机。
