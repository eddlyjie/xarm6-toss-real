# 从仿真完整方法到 xArm 6 真机

这份文档解释项目真正的方法。`throw_only_cube.json` 只是让真机尽快产生第一个安全
demo 的启动步骤，不是最终研究方法。最终目标不是固定摆臂后补一次 IK，而是：

> 机器人拿起物体，通过主动 Probe 估计隐藏物理属性；根据目标最终 pose 选择抛掷、
> 飞行和动态接取方式；接住后再用视觉估计真实手物变换，完成目标姿态和位置。

真机第一天可以只做小 cube 的固定抛出，但代码结构必须保留下面的完整闭环。

## 1. 完整流程

```text
腕部/全局相机得到当前物体 pose 与点云
→ 桌面抓取、抬起并保持
→ Active Probe：小幅 tilt/chirp/shake/夹持变化
→ 估计 mass、CoM、inertia、contact/slip posterior
→ Detach model 预测开夹爪后的真实离手状态及不确定性
→ 对 release/throw skill 传播 6-D flight belief
→ 在多个飞行时刻生成动态双指接触候选
→ Ccatch / whole-arm policy 预测 catch action 与成功概率
→ M3 coordinator 针对最终 target pose 联合评估 Catch + residual IK + motion
→ 执行选中的 throw、release、flight、catch
→ Capture-J 选择接触后的 centering/absorption/preload 动作
→ 稳握至少 2 s
→ 腕部相机主动多视角估计真实 post-catch T_HO
→ xArm 6 全关节 residual planning 与短距离 transport
→ 到达 target object pose
```

目标 pose 不是最后才交给 IK。它必须在“选哪种抛法、飞行旋转和 regrasp”时就参与
M3 候选选择。这样系统才能对不同目标选择不同技能，而不是永远转一个固定角度。

## 2. 感知与坐标

真机只有一个腕部相机和一个全局相机也可以开始：

- 全局相机：桌面物体 pose、飞行轨迹、落点和实验视频；快速运动时是主相机。
- 腕部相机：抓取前局部点云；抓住后让机械臂停在 2–4 个观察 pose，形成主动多视角；
  接住后再次估计物体相对夹爪的 `T_HO`。
- 关节编码器：xArm 6 的 `q/dq`、末端 FK、真实轨迹跟踪。
- 夹爪/触觉：若没有 GelSight，第一版用夹爪位置、电流和是否滑落作为低维接触特征；
  以后再换成双 GelSight 时保持接口不变。

关键状态都用明确坐标表达：世界/机器人基座 `W`、手 `H`、物体规范坐标 `O`。
最终目标是 `T_WO_target`，抓住后的关系是 `T_HO`，因此理想手目标为：

```text
T_WH_goal = T_WO_target · inverse(T_HO_measured)
```

但这只解决“接住以后怎么去目标”。M3 还要提前选择一个更适合目标的 `T_HO`。

## 3. Active Probe 到底做什么

Probe 在物体仍被夹持时执行小幅、可恢复的激励。仿真状态机包含：

1. settle；
2. static hold；
3. 正负 roll/pitch tilt；
4. x/y/目标旋转轴 chirp；
5. bounded shake；
6. 可选的安全夹持力降低；
7. recover。

每个 `ProbeAction` 包含轴、幅度、频率范围、持续时间和夹持力比例。由腕部 F/T、
编码器和左右触觉时间序列拟合 10 维标准惯性参数：

```text
theta = [m, h_x, h_y, h_z, Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
h = m · CoM
```

同时估计接触力、左右不对称、接触丢失/滑移和 compliance 等 contact latent，得到
`PhysicalBelief(mean, covariance)`。

当前 sim 的 Probe selector 不是 PPO。它从有限动作库中做 model-based selection。若候选
动作 `a` 的预期信息矩阵是 `Lambda_a`，则 posterior 近似为：

```text
Sigma_after(a) = inverse(inverse(Sigma_before) + Lambda_a)
```

task-conditioned 模式使用任务 Jacobian `G`，选择：

```text
score_probe(a)
  = trace(G Sigma_before G^T) - trace(G Sigma_after(a) G^T)
  - w_slip P_slip(a)
  - w_time duration(a)
  - w_energy energy(a)
  - w_force peak_force_ratio(a)
```

这意味着 Probe 不是“随便摇一下”，而是优先观测对当前 release/flight/catch 最重要、
同时风险较低的物理方向。真机初版可以先固定一条短 Probe；收集数据后再启用 selector。

## 4. Detach model 学什么

Detach 指发送开夹爪命令后，物体真正失去双侧接触并进入自由飞行的时刻。命令时刻不等于
物理 detach 时刻，物体速度也不等于机械臂末端速度。

先用运动学得到 nominal detach state，再由网络预测 13 维 tangent residual：

```text
delta_detach = [
  delta_time,
  delta_position_xyz,
  delta_rotation_so3_xyz,
  delta_linear_velocity_xyz,
  delta_angular_velocity_xyz
]
```

模型输入分成四支：

- action：release 前末端速度/加速度、期望相对速度、开夹爪相位和速度、跟踪误差等；
- physics：Probe posterior 的 mass、CoM、inertia、covariance 与 contact latent；
- geometry：左右接触点/法向、曲率、边缘距离、抓取位姿、物体尺寸和 CoM lever arm；
- tactile：release 前固定窗口的左右触觉/夹爪时间序列。

网络输出 residual mean、aleatoric covariance、valid-release logit；多个 seed 组成 ensemble，
模型间差异给 epistemic covariance/OOD。训练是 supervised heteroscedastic regression +
validity classification，不是 RL。

所以正确的因果关系是：

```text
Probe action → physical posterior/tactile evidence
             → Detach model 对同一 release action 的预测改变
             → flight/catch candidates 改变
```

Detach model 不直接学习“Probe 手应该怎么动”；Probe selector 和 Detach predictor 是两个
模块。未来可以联合学习，但真机首版不需要把它们混成一个大策略。

## 5. Throw 与 6-D flight belief

一个 throw skill 至少包含 pre-throw 起点、关节轨迹、release sample 和 follow-through。
学习模型预测 detach state 后，用刚体飞行模型传播：

```text
x(t) = position, rotation, linear velocity, angular velocity
```

需要同时传播 posterior particles，而不是只看均值；接取候选的置信度和 CVaR 风险来自
这些样本。真机最早可以用相机实测飞行轨迹拟合简单 ballistic correction，再逐步接回
完整 Detach ensemble。

## 6. 动态 Catch 候选点怎么产生

候选不是在空间里随便采一个末端点。仿真做法是：

1. 在物体 CAD/点云表面采样点和法向；
2. 枚举两点距离在夹爪开度内、法向近似 antipodal 的接触对；
3. 在多个 candidate time 上把接触点随 6-D flight belief 传播到世界坐标；
4. 计算每个接触点的局部速度：平移速度加 `omega × lever_arm`；
5. 由接触连线构造 closing axis，由接近方向构造 hand rotation；
6. 可枚举平行夹爪的等价 180° approach flip、少量 roll 和保持原 pickup grasp 的对称分支；
7. 为每个候选记录 hand position/rotation、匹配线速度/角速度、夹爪宽度、antipodal
   quality 和预期 `T_HO`。

这些候选首先经过工作空间、关节可达、桌面/自碰撞、速度和时间约束，再交给学习模型和
目标协调器。真机可以先使用 3–5 个离线验证的 skill candidates；候选库不是永远的
hard-code，而是 CEM/仿真/真机数据产生的有限技能集。

## 7. Catch 的统一目标 J

仿真的候选级 `J_catch` 越小越好：

```text
J_catch =
    w_task        · task_grasp_error
  + w_relative    · relative_contact_velocity
  + w_impact      · impact_energy
  + w_slip        · slip_risk
  + w_motion      · arm_motion_cost
  + w_uncertainty · CVaR(failure_cost)
  - w_success     · log(P_catch)
```

当前默认权重分别是 `2.0, 1.5, 1.0, 1.2, 0.35, 1.0, 1.5`。CVaR 使用 posterior
samples 中最差一部分的平均失败代价，因此高均值但尾部很危险的动作不会被偏爱。

`task_grasp_error` 描述候选手物变换与任务需要的抓法之间的误差；
`relative_contact_velocity` 和 `impact_energy` 鼓励手指追上局部接触点速度；
`motion` 避免不必要的大关节摆动；`P_catch` 来自 Ccatch/rollout 标签。

## 8. Whole-arm policy 与 RL

whole-arm actor 的动作不是一个 IK 目标，而是 22 维连续动作：

```text
catch_time
3 × 3-D trajectory knots
catch rotation 6-D representation
match linear velocity xyz
close_lead
compliance_scale
execute_probability
```

候选本身提供基准 contact time、pose、局部速度和角速度；actor 学的是相对候选的 residual。
这比让网络从零输出整个轨迹稳定，也保留候选的物理含义。

训练顺序：

1. CEM 在每个训练 context 上搜索低 `J_catch`、可执行的 expert action；
2. Behavior Cloning 让 actor 拟合 CEM experts；
3. Asymmetric PPO 在 BC 附近优化 contextual rollout reward；
4. actor 只读部署时可获得的点云、Probe posterior、Detach belief、机器人 q/dq 和候选；
5. critic 训练时可以读 simulator true physics/state，部署时完全丢弃；
6. shape/material OOD 分开评估，不能把同一物体的相邻轨迹随机拆分成 unseen。

目录中的 `learning/models.py` 和 `learning/losses.py` 是这一结构的真机精简骨架，方便
真机电脑接入采集数据；它们不是已经训练好的 xArm checkpoint。

## 9. Capture-J：接触后的动作选择

Ccatch 决定在哪里、何时、以什么速度接近；第一次 pad 接触后，还要选择 centering、
momentum absorption、angular absorption 和 finger preload。Capture-J 用 Probe posterior、
视觉质量、接触几何、手物相对速度和候选动作预测：

- bilateral intercept；
- stable hold 250 ms；
- stable hold 2 s；
- 2 s hold fraction；
- 接后旋转是否被控制。

它在一个小动作库里最大化成功/稳握，减去动作幅度和 epistemic uncertainty。真机没有
触觉时可先固定 absorption 动作；有夹爪电流或 GelSight 后再训练 Capture-J。

## 10. 最终 target pose 如何真正参与选择

每个已经学到/验证的 toss-regrasp skill 都携带：

- safe `P_catch`；
- Detach/flight uncertainty；
- collision/contact risk；
- post-catch `T_HO` posterior samples；
- 对该 target pose 的 residual IK/path 结果。

M3 对每个 posterior sample 都求目标 pose 的 maintain-grasp IK，并计算：

```text
J_coord =
    2.00 · [-log(P_catch)]
  + 5.00 · [1 - robust_IK_fraction]
  + 0.25 · normalized_pose_residual
  + 0.50 · detach_flight_uncertainty
  + 2.00 · collision_contact_risk
  + 0.20 · normalized_joint_path
  + 0.30 · normalized_max_joint_swing
  + 0.35 · normalized_postcatch_EE_rotation
  + 0.05 · normalized_duration
```

然后从同时满足 Catch、robust IK 和风险约束的候选中选最小 `J_coord`。改变
`T_WO_target` 会改变 robust IK、pose residual 和运动代价，因此可能选择完全不同的
throw/catch skill。这就是“基于最终 pose 选择抛接情况”，不是固定旋转后再做 IK。

M2 则故意不看 target，永远选择最高 Catch-confidence skill；M2 与 M3 使用同一候选库，
二者差异正好衡量 coordination 是否有价值。

## 11. 四个主要方法/baseline

- **M0 Fixed-grasp Direct IK**：保持初始抓取，直接全关节移动物体到目标。
- **M1 Target-aware best-grasp Direct**：先根据目标在桌面选择较好的 antipodal pickup
  grasp，再抓起并直接移动；没有 toss。
- **M2 Fixed-confidence Toss–Regrasp–IK**：从候选库选择 target-independent 的最高
  `P_catch` 技能，接住后再做 IK。
- **M3 Coordinated Toss–Regrasp–IK（ours）**：使用同一候选库，但用最终 target pose、
  Catch、robust IK、motion 和 risk 联合选择技能。

真机老师只要求简单 demo 时，可以先交付 fixed throw；但论文/研究结论不能把它称作 M3。
至少应保留一个小型多技能演示：同一 cube、两个 target poses、2–3 个不同 release/catch
skills，展示 target 改变时选择结果确实改变。

## 12. seen/unseen 与训练安排

建议真机最小版本：

- seen：cube A，用于示教、轨迹调试和少量训练；
- transfer：cube B，尺寸或质量略有变化，只允许少量 target-independent calibration；
- target poses：至少 upright 与一个 30–60° 非共线旋转目标；
- 每个 target 同时跑 fixed throw 和 pose-conditioned selection；
- 若还不能接取，先比较离手成功、飞行时间、落点和旋转；接取完成后再报告 Catch/Task。

严格 unseen 需要物体级划分。cube B 一旦被反复用于调参，就应称 transfer/development，
不能再称 zero-shot unseen。

建议数据阶段：

```text
D0  示教轨迹 + 固定 release
D1  多次 throw-only，拟合 release timing / ballistic residual
D2  固定 Probe，训练 Detach residual ensemble
D3  多个 candidate catches，生成 CEM expert 并做 BC
D4  仿真 PPO refinement，真机只做小 residual/fine-tune
D5  冻结 skill library，比较 M2/M3 和 seen/transfer
```

## 13. 真机代码对应关系

复制到真机电脑后的关键文件：

```text
configs/method.example.json          小型 Probe/skill/target 示例
src/xarm6_toss/method/core.py        Probe、J_catch、M2/M3 selector
src/xarm6_toss/learning/models.py    Detach、pose/skill、catch actor/critic
src/xarm6_toss/learning/losses.py    supervised/BC/PPO losses
scripts/10_method_pipeline_demo.py   不连接机器人，演示 M2 与 M3 选择差异
```

原 sim 工程中最值得真机 Codex 对照阅读的源文件：

```text
toss_probe/active_probe/selector.py        Active Probe selection
toss_probe/control/state_machine.py        P0–P6 Probe state machine
toss_probe/detach/state.py                 13-D Detach residual
toss_probe/detach/model.py                 Detach ensemble network/loss
toss_probe/detach/probe_conditioning.py    Probe → Detach → flight
toss_probe/catch/candidates.py             动态 antipodal candidates
toss_probe/catch/reward.py                 J_catch
toss_probe/catch/policy.py                 22-D whole-arm actor/critic
toss_probe/catch/policy_training.py        CEM → BC → asymmetric PPO
toss_probe/catch/capture_j.py              post-contact Capture-J
toss_probe/catch/bridge.py                 M2/M3 target coordination
```

## 14. 当前真机实现状态

已经有：配置、离线 throw trajectory、只读 SDK、夹爪测试、方法数学、M2/M3 选择示例和
学习网络骨架。

尚未有：真实相机驱动、标定、xArm streaming throw、真实 Probe 执行、数据集采集、训练好
的 xArm Detach/Catch checkpoint、动态接取控制。真机电脑上的 Codex 应先完成能录像的
throw-only，然后沿上述模块逐个替换 placeholder，而不是继续堆固定 if/else。
