# xArm6 最小 learned closed-loop 自抛自接 goal

## 总目标

在不恢复、不替换当前 paused 长期 sim goal，也不改写 Panda 主线方法的前提下，
在 `/home/ubuntu/toss_project/xarm_6/` 建立一套 xArm6 + UFACTORY G1 针对单个轻质
3D 打印小 cube 的最小 learned closed-loop 自抛自接系统，并形成可交给真机电脑直接继续
验证的完整 handoff。

最终追求的不是论文级 xArm6 复刻，而是让真机在固定抓取和安全实验条件下尽快得到
2–3 次可信的同手抛出、自由飞行、接住并短时稳握。系统必须有真实参与控制的 learning
和 release 后闭环更新，但应保持小、稳、可解释，不引入 Panda 系统中的 GelSight、F/T、
复杂 Active Probe、完整 target-pose coordination 或大规模端到端 RL。

## 已知硬件与数据边界

- 机器人：xArm6，UFACTORY G1，控制周期 `20 ms`。
- 物体：边长略小于 `4 cm`、低填充率、较轻的 3D 打印 cube；在获得精确称重前，
  仿真使用围绕该描述的窄而合理的质量范围，不把真实质量作为 policy observation。
- 真机测得的 arm tracking delay 约 `90 ms`。
- G1 当前抓持位置约 `370`，用于 release/catch 的 partial-open 位置约 `520`；partial
  motion 约 `100 ms`，首次可观察运动约 `13–23 ms`。
- detach delay 的现有估计约 `25–44 ms`，应作为随机化中心而不是固定真值。
- 全局相机是 Intel RealSense D435，`640 x 480 @ 60 Hz`，实测约 `59.7 Hz`；已有
  intrinsics、`T_base_camera` 和 yellow-cube detection 代码。
- 无 GelSight，无腕部六维 F/T。可用信号为 `q/dq`、joint effort、motor current、
  gripper position 和两台 D435。首个 demo 允许固定 cube 摆放和 hard-coded 抓取 pose；
  wrist D435 只作可选的抓取/T_HO 检查，不得阻塞抛接主流程。release 后的飞行闭环必须
  使用 global D435 与编码器，不要求腕部相机在摆臂中持续看见 cube。

权威输入位于：

```text
xarm_6/toss_project_sim_handoff_20260816.tar.gz
xarm_6/toss_project_sim_handoff/toss_project/xarm_6/sim/
xarm_6/toss_project_sim_handoff/toss_project/real_cube_demo/
xarm_6/toss_project_sim_handoff/toss_project/RobotCamCalib/
```

完整 tar 已通过 `gzip -t`，包含 204 个归档条目。当前 staging 只作为合并来源；先将
需要的 sim、xArm core、测试、相机标定和真机接口整理到清晰的 `xarm_6/` 结构，验证后
再决定是否删除 archive/staging，不能提前丢失唯一反馈副本。

## 方法约束

### 1. 物理主流程

使用 Isaac Sim 6 / Isaac Lab 6 的 native PhysX contact：

```text
稳定抓持 cube
→ 低能量向上/略向前加速
→ G1 partial-open，cube 真实 detach
→ 短自由飞行
→ arm 执行 nominal intercept/absorption trajectory
→ global-camera observation 更新 catch timing/小幅 catch target
→ G1 提前 close
→ 双侧捕获并稳定保持至少 0.5 s
```

cube 只允许在 episode 初始化时放入夹爪。抓持、release、自由飞行、recontact 和 hold
期间不得写 cube pose/velocity，不得用约束、隐藏支撑或 evaluation-only state 制造成功。

### 2. 两台相机的职责

两台 D435 的标定和接口都保留，但首个成功 demo 只强制使用 global camera：

- wrist camera（可选）：机械臂静止或低速时采集局部 aligned RGB-D，可估计 cube center、
  可见表面、尺寸和姿态；若使用视觉抓取，则对近 4 cm cube 生成面中心附近、两指平行且
  开度可达的 antipodal grasp。第一版允许直接使用已验证的固定抓取 pose。
  抓住后可在 2–3 个静止观察 pose 复核 `T_HO` 和 grasp offset。腕部外参是
  `T_link_eef_camera`，每帧必须使用当前 FK 计算
  `T_base_camera = T_base_link_eef · T_link_eef_camera`，不能当固定世界外参。
- global camera：使用固定 `T_base_camera` 覆盖抓取、release 和 flight 区域；抓取前可交叉
  检查 cube world pose，但这不是必须的共同视野；release 后以约 60 Hz 追踪 cube 3-D
  center、拟合 ballistic state，驱动 learned timing/catch residual，同时保存评价视频。

不要求 wrist 与 global camera 同时看到 cube。跨相机状态交接统一通过 base frame：

1. 固定抓取配置直接提供 nominal `T_hand_object`；若启用 wrist observation，则用它更新；
2. 抓住后由当前 FK 计算 `T_base_hand`，并保存/复核 `T_hand_object`；
3. 持物摆臂期间用编码器 FK 和固定 `T_hand_object` 传播 object nominal pose；
4. release 附近允许存在短暂 blind gap，使用 nominal Detach/ballistic state 传播；
5. global camera 首次重新检测到 cube 后更新 flight state，并接管 learned catch correction。

如果 wrist 未启用，流程直接从固定 `T_hand_object` 和第 3 步开始，这仍是有效的第一版。
若能找到双相机都可见的静止 observation pose，可用它检查坐标和时间同步，但不能把这种
重叠当作运行前提。release/catch skill 搜索必须同时检查 global image bounds、depth range、
机器人/夹爪遮挡和首次可观测时刻，不能只检查 IK 与物理可达性。

第一版不训练端到端 RGB grasp network。cube 几何简单且已有 yellow-cube detector，优先用
颜色分割、aligned depth、已标定 intrinsics/extrinsics 和 cube 平面/边长约束得到稳定 grasp；
只有真实失败证据表明需要时才增加学习式 grasp correction。

Isaac 中按真实外参放置两台 camera，并用真实 intrinsics/distortion 对 rendered observation
做少量端到端检查。批量训练可使用等价的 state-level observation/noise model 提速，但必须
保留可选 wrist observation 与必需 global flight observation 的坐标、时序和可见性接口。

### 3. 固定 backbone

先建立一个本身接近成功的固定、低能量 xArm6 joint-space backbone。优先考虑近竖直
微抛和原路径下落，避免大幅横向追赶。使用 G1 `370 → 520 → 370` partial-open/close，
除非 PhysX 与真机证据说明需要更大开度；不要默认沿用 smoke config 的全开 `850`。

backbone 输出配对的 `q/dq/qdd`、20 ms 时间戳、release/close command time、预测
physical detach/catch time 和 follow-through/absorption 段。先验证关节限制、TCP 速度、
碰撞和 G1 mimic/drive/contact，再搜索少量有物理含义的参数。

### 4. 最小 learned residual

learning 不直接生成整条 6DoF 轨迹。采用 physics-guided ballistic estimator 加小型 residual
model；默认先用 supervised teacher/BC，只有证据表明不足时才做短 PPO fine-tune。

部署侧 observation 只允许包含：

- release 前后实际 `q/dq` 与 nominal tracking error；
- time since release、gripper command/position；
- 全局 D435 最近若干帧得到的 cube 3-D position、velocity/fit quality；
- nominal intercept、nominal close time 和有限历史窗口。

不得向 actor 暴露 simulator mass、friction、true cube pose/twist 或未来状态。训练 teacher、
critic、标签生成和离线评价可以使用 privileged truth，但最终 actor/inference 必须只读上面
的 deployable observation。

residual action 以容易转真的量为主：

- `delta_close_time_s`，初始限制在约 `±60 ms`；
- 可选的小幅 `delta_catch_z/xyz` 或等价 joint correction，初始空间幅度约 `20 mm`；
- 必要时一个小的 absorption/velocity-match scale。

这些界限可以依据真实可达性和仿真证据小幅调整，但不得扩展成无边界 full-trajectory actor。
在约 `0.18 s` 的短飞行里，timing correction 是第一优先；arm correction 只有在 observation
到 actuator deadline 仍有足够时间时才启用。

### 5. 最小闭环的严格含义

一次 learned rollout 必须在 physical release 后、close command deadline 前至少消费一次
deployable-style flight observation，并由该 observation 实际改变 close time 或 catch target。
仅离线录像、仅用网络预测但不改变命令、或只在 episode 开始选固定动作，都不算完成
closed loop。

训练时可用 state-level D435 observation model 提速，至少包含 60 Hz sampling、坐标外参、
测量噪声、host/camera latency 和偶发无效深度。最终必须再用真实 intrinsics/extrinsics 放置
Isaac camera，做少量 rendered native rollout，确认 cube 可见性、坐标方向和 online tracker
接口，而不是只在解析状态上成功。

## 实施顺序

1. 整理完整 handoff 到 `xarm_6/`，保留无关现有工作；补齐新 `flight.py`、
   `control_reference.py`、sim tests、self-contained xArm6+G1 asset、real configs 与相机标定。
2. 生成并检查 USD；建立小而独立的 xArm6 runner，不把 2 万行 Panda/GelSight runner
   整体复制过来。
3. 验证 articulation joint order、G1 mimic/drive gains、finger geometry、stable grasp、
   partial release、真实 detach 和双侧 catch contact。
4. 从 `natural_j5_candidate.json`、`upward_throw_smoke.json` 和真机 observed-catch 搜索结果
   生成 nominal backbone；必要时用 CEM/小范围搜索优化 release、retreat、intercept 和 close lead。
5. 先用 simulator truth 完成 nominal catch，再加入 real-centered delay、grasp offset、轻质 cube
   mass/inertia/friction 和 camera observation randomization。
6. 实现 online cube track/ballistic fit 与 learned residual dataset；训练小模型，冻结 checkpoint、
   normalization 和 action bounds。
7. 在完全相同 trials 上比较 fixed backbone 与 learned closed loop，保存成功和失败。
8. 生成真机 handoff：可执行数据、模型、推理接口、dry-run/preview、最短真机验证顺序和视频。

## 完成标准

只有同时满足下面结果，goal 才能标记完成：

- xArm6+G1 native PhysX runner 能从稳定抓持开始，完成真实 detach、自由飞行、双侧 catch
  和至少 `0.5 s` stable hold；过程中没有 object-state rewrite。
- frozen learned residual 在 nominal native 设置连续完成至少 `3/3`，并在围绕实测 delay、
  初始抓取和轻质 cube 物理量的小范围 native 扰动中达到至少 `8/10` catch-and-hold。
- 同条件 fixed-backbone baseline 被实际评估；结果能证明 learned observation/action 确实进入
  闭环，并报告它相对 fixed backbone 对 timing/position error 或 catch success 的影响。
- 至少三段代表性成功视频可直接观看；失败 trial 也保留并有简短 stage/reason。
- 最终 actor 不读取 hidden simulator physics/truth，rendered camera rollout 证明真实相机坐标与
  tracker 接口可工作。
- 真机 handoff 至少包含：
  - self-contained USD/URDF 与 Isaac/Python 版本；
  - nominal 和两个小 timing bracket 候选的 `q/dq/qdd`、20 ms timeline、G1 events；
  - residual checkpoint、normalization、observation/action schema；
  - online inference/dry-run 入口，默认不移动真机；
  - release/catch predicted state、训练随机化范围、评估摘要和视频；
  - 从空载 preview、空夹爪 partial-open、低速带 cube 到正式 2–3 次 demo 的简短步骤。

## 明确不做

本 goal 不要求：

- 迁移 Panda 7DoF trajectory/checkpoint；
- GelSight、F/T、完整 Active Probe/Detach ensemble；
- 多物体 seen/unseen、M0–M3 正式实验或最终 target-pose coordination；
- 大规模 PPO、端到端 RGB network 或论文级 camera domain randomization；
- 接后 transport、姿态精确控制或长时间稳握；
- 新的 hash、seal、sidecar、source capsule 或发布审计流程。

如果简单 residual 已达到可靠成功，不得为了“更像 Panda”继续扩大方法范围。

## 执行与报告要求

- 所有 Isaac、Python、CUDA、训练和测试都在 devserver 上运行；不得从 `R:\` 编辑。
- 不安装或升级依赖，除非用户明确批准；先使用
  `/home/ubuntu/IsaacLab-3.0.0-beta2/env_isaaclab` 和现有项目环境。
- 长 GPU run 开始前展示精确命令、输出、日志路径和主要风险，然后立即在 remote `tmux`
  中启动并监控。
- 保留当前仓库的无关未提交修改；不 reset，不删除历史 artifacts/videos/checkpoints。
- 每次方法修改后检查 `git diff`、`git diff --check`，运行最小 CPU/static test，再做 native rollout。
- 进展报告优先给出真实物理结果、成功/失败 stage、视频和剩余方法缺口，不扩展审计框架。
- 当前长期 sim goal 始终保持 paused；本 goal 是 `xarm_6/` 内的独立任务。
