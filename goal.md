# xArm6 最小 Probe–Toss–Catch 真机验证目标

## 最终目标

从原始真机 setup 重新建立一条 xArm6 + G1 最小但科学有效的闭环：

```text
固定抓取小 cube
→ 轻量 Active Probe
→ 估计 detach / flight belief
→ 释放后 third-view RGB-D 更新弹道
→ 用 J 评分并选择可达 catch candidate
→ bounded learned residual 修正 catch target
→ 同一夹爪重新接住并稳定保持
```

真机只要求边长约 38 mm 的轻量 3D 打印 cube 成功 2–3 次，不要求 GelSight、腕部
FT、复杂抓取学习、目标姿态 transport 或多物体泛化。抓取 pose 可以 hard-code，但
probe、camera observation、J 和 learned correction 必须真实进入控制，不能只记录不使用。

## 可用观测

真机和 policy 只能使用：

- xArm 实际 q、dq、joint effort/current 和 controller timestamp；
- G1 commanded/actual position；
- wrist D435：抓取前定位、抓取后 probe 期间的 hand-object observation；
- third-view D435：release 后 cube RGB-D center 和 timestamp；
- 已提供的相机内参、外参和 xArm/G1 URDF。

不得使用 cube 真值 pose、真值 velocity、质量、摩擦、接触标签或 simulator-only state
作为 policy 输入。Isaac 中 cube pose/velocity 只允许 episode 初始化写一次。

## 最小方法

1. 固定抓取：先用已知 cube pose 和示教/硬编码抓取点，不把抓取学习作为阻塞项。
2. Probe：执行一个短、小幅、可回到中心的安全 excitation；由 q/dq/effort/current、
   gripper position 和 wrist RGB-D 得到低维 posterior。posterior 至少影响 detach
   uncertainty、release timing 或 J，不能是装饰模块。
3. Detach prior：用实际 q/dq、FK/Jacobian、固定 T_hand_object 和实测 G1 delay 得到
   release position/velocity belief。
4. Flight tracking：third-view 60 Hz RGB-D 在 base frame 下做 gravity-constrained fit，
   覆盖 encoder prior 的 tracking、gripper delay 和滑移误差。
5. Catch candidates/J：在多个时间/位置候选中，用 catch probability、相对速度、
   uncertainty、IK/reachability 和 collision margin 计算 J，选择真实可执行的 candidate。
6. Learning：训练一个小型 residual，只修正 detach/intercept 或 J；输入必须是上述
   deployable observation，输出必须 bounded。不要做端到端 RGB/RL。
7. Catch：corrected candidate 进入 IK/Jacobian 和真实 joint target；G1 在 deadline
   nonblocking close；随后保持至少 0.5 s。

## 不可妥协的几何与视觉门槛

在任何“成功”或 handoff 之前必须同时满足：

- start、release、catch 的 TCP 都在 base 外侧工作区，水平半径至少 0.35 m；
- release 时 EE/tool 朝向明确对外：
  `dot(tool_axis_xy, tcp_position_xy) > 0`，并保存数值；
- 轨迹视频中机械臂不是向 base 内折，release 后不会向自身底座抛；
- physical detach 到首次重新接触至少 0.10 s；
- cube 与 gripper 的最大分离至少一个 cube 边长（约 38 mm）；
- spectator video 中至少连续 6 个 60 Hz frame 清楚看到自由飞行；
- third-view 在 detach 后至少提供 3 个有效 observation，并真实改变 catch command；
- wrist camera 至少清楚看到抓取或 probe 阶段的 cube；
- spectator camera 能在同一画面完整看到机器人、cube、release、flight、catch 和 hold；
- spectator camera 绝不进入 policy observation，只用于人工验收和论文视频；
- catch 必须 bilateral contact，并稳定保持至少 0.5 s；
- 我必须逐帧查看视频，不能只根据 JSON success flag 交付。

## 仿真验收

先通过单 trial，再固定方法跑至少 3 个扰动 trial：

- cube mass 覆盖约 20–50 g；
- 至少一个抓取 offset / camera noise / detach delay 扰动；
- 3 次均 physical detach、可见 free flight、camera-updated command、bilateral catch、
  stable hold；
- 报告并保存失败，不允许只挑成功视频；
- 输出同步的 spectator、third-view、wrist 视频或 frame montage；
- 保存 q/dq、effort、G1、probe posterior、detach belief、camera observations、J 候选、
  learned residual、selected catch 和实际 controller target。

最小对照：

- M0：固定 open-loop catch；
- M1：camera ballistic catch，不用 learned residual；
- M2：Probe + camera + J + learned residual。

对照服务于确认模块是否真实产生作用，不扩展成大规模 benchmark。

## 真机交接

只有仿真验收全部通过后才生成：

- 一个明确的 nominal timeline 和少量 timing bracket；
- 空载 0.25× / 0.5× / 1.0× preview；
- 不连接机器人的 observation/controller dry-run；
- third-view/wrist calibration 和 ROI 可见性检查；
- probe、detach、J、residual、IK/servo_j 的最小集成代码；
- 2–3 次真机 trial 的记录格式和停止条件；
- 一条正常全局 spectator 视角的完整成功视频，不能只给 1 秒且看不清的 policy-camera
  画面。

真机失败后优先带 q/dq、G1 delay、camera timestamps、detections 和视频回到 sim，
不在昂贵真机上做大范围动作搜索。

## 当前状态

旧的内折 EE / micro-toss handoff 已撤销并删除。旧 commit 只保留为 Git 历史，不得用于
真机。当前工作从原始 real setup、calibration 和 Panda 成功 pipeline 的方法结构重新开始。
