# xArm6 四物体真机 Pose-Conditioned 开环抛接开发目标

更新日期：2026-08-25

本文件是后续 Codex 挂载执行的真机开发目标。当前只要求维护本文件；挂载前不得连接或运动真实机械臂。
用户挂载后，Codex 应把本文件视为持续执行目标，直接读取已有代码、报告和 handoff，优先完成可上真机的轨迹、
标定文件、现场命令和结果整理。不要重新规划同一目标，也不要因为缺少现场数据而停掉所有离线工作；只有真实
机械臂连接、运动和需要操作者测量时才等待现场指令。

执行优先级固定为：四个物体各得到一个完整成功 demo → 每个物体增加可区分 pose → 重复统计与视频 →
M0–M3 对比。不要长期钻在一个物体或一个大角度上，也不要把主要精力投入 hash、seal、审计链和低概率防御。
真机结果优先于形式完整的 RL 或 baseline 矩阵。

## 0. 当前交付任务

当前 Sim/offline 轨迹已经能够覆盖四个物体，接下来工作的核心是把这些结果变成明天能在真机电脑上依次执行的
demo，而不是重新从零设计 Panda 仿真方法：

1. 先复现 O0 已有的真机 micro-toss 接住结果，并把它固定成当天保底 profile；
2. 用现场实测值完成 O0–O3 各自独立的 G1 held/release/preclose/close 标定；
3. O1、O2、O3 从最保守的 low profile 开始，各得到至少一次“离手—可见旋转—接住—保持”的完整成功；
4. 四物体均成功后，O0 做 low/medium/high，O1–O3 尽量补第二个明显可区分的 pose；
5. 角度以视频实测为准。能够稳定展示多个不同角度即可，40°/60°/80°均为 stretch target；
6. 最后再做重复次数、慢放 montage 和 M0–M3。RL/Detach/J 的补充只在它能实际提高 demo 时进行。

## 1. 最终目标

在 xArm6 + UFACTORY G1 上，用同一个 stock G1 完成四个轻量物体的真机开环抛接：

```text
操作者按固定标记放入物体
→ J2/J3/J5 协调抛出
→ 物体完全离开双侧夹指
→ 绕固定前滚翻或后滚翻轴产生可见 pose 变化
→ 同一 G1 在下降段重新接住
→ 稳定保持至少 0.5 s
```

真机只负责安全、可复现地执行 Sim/offline 选出的动作。相机用于录像和离线测角，不参与高速闭环；
不在真机 runner 中做 Probe、在线 pose estimation、在线轨迹优化或临场 RL。抓取起点、物体朝向和插入深度
允许通过治具或标记固定。输入是 `object context + desired rotation angle`，
输出是完整的 20 ms `q(t), dq(t)` 与 G1 event schedule。

论文与演示的核心表述是：同一个 object/pose-conditioned policy 根据物体尺寸、质量、惯量、抓取方式和目标
旋转角产生不同轨迹。导出的若干 profile 是 policy 的离线执行结果，不称为手工“技能库”。

## 2. 四个真机物体

下表尺寸保留用户的三轴实测顺序。后三个物体均让 G1 夹住最窄边；Sim object config 会重排局部轴，统一把
局部 `Y` 定义为夹爪闭合轴和主要前滚翻轴（例如 O1 config 为 `44.5 × 30 × 46 mm`）。每个物体需粘贴
不对称 marker，并固定抓取深度和朝向。

| ID | 尺寸 | 质量 | G1 夹持边 | 当前状态 |
|---|---:|---:|---:|---|
| O0 `cube38` | 38 × 38 × 38 mm | 8.0 g | 38 mm | 真机 micro-toss 已接住；Sim 4.59° / 6.48° / 7.87° 稳定 |
| O1 `cuboid30` | 44.5 × 46 × 30 mm | 20.0 g | 30 mm | Sim 2.96° / 5.71° / 6.57° 稳定；待实测 G1位置 |
| O2 `cuboid33` | 50.5 × 51 × 33.5 mm | 26.6 g | 33.5 mm | Sim 4.61° / 5.62° / 6.45° 稳定；待实测 G1位置 |
| O3 `cuboid38` | 57.5 × 58 × 38 mm | 37.0 g | 38 mm | Sim 4.40° / 5.58° / 6.85° 稳定；待实测 G1位置 |

实验当天重新称重并测量 marker、抓取深度、G1 held position。若实测与表格不同，更新 object config 后重新生成
profile，不能只改文件名。

每个物体的 Sim 模型使用真实三轴尺寸和质量，并计算长方体惯量：

```text
Ix = m (Y² + Z²) / 12
Iy = m (X² + Z²) / 12
Iz = m (X² + Y²) / 12
```

## 3. 已确认的真机约束

以下初值来自 `REAL_ROBOT_TEST_20260817.md`，新实验当天复测：

| 项目 | 已测值 |
|---|---:|
| arm command period | 20 ms `servo_j` |
| arm tracking lag | 约 80 ms |
| G1 speed | 5000 |
| O0 历史 G1 held / partial-open / close | 370 / 520 / 370（只作当天复测起点） |
| G1 first observed motion | open 22.64 ms；close 12.57 ms |
| G1 target reached | open 102.79 ms；close 102.62 ms |
| physical detach after open command | 约 25–44 ms |
| verified linear speed limit factor | 1.6 |
| reference peak joint speed | 1.7448 rad/s |
| reference peak joint acceleration | 13.0574 rad/s² |

硬件只有 xArm6、stock G1 和录像相机，无 GelSight、无 wrist F/T。当前阶段接受 open-loop demo 的限制，
但所有 profile 必须在 Sim 中使用真实 G1 geometry、release delay 和 arm lag验证。

## 4. 关节与动作约束

参考反腕姿态：

```text
J1=3.5°, J2=22°, J3=-46°, J4=176.8°, J5=53.3°, J6=-9.3°
```

它只是可达性 seed。正式动作遵守：

- 动态抛出和接取主结果只控制 J2、J3、J5；
- J1、J4、J6 在整段 timeline 中保持固定；
- J4 从机械上限适当退回，给 cable 和真机误差留余量；
- 物体绕固定轴做单轴前滚翻或后滚翻，不追求复杂三维自旋；
- EE 应先有向下的 backswing，再反转向上，release 后及时制动或撤离；
- 必须建立 object–gripper 相对分离，避免夹爪继续追着物体上升而吃掉可见飞行；
- 大角度不能以关节超限、剧烈碰撞或明显增加机械损耗为代价。

动作包含：

1. `backswing`：EE 竖直速度先为负；
2. `reversal/upstroke`：J2/J3/J5 平滑反转，产生向上速度和翻转角速度；
3. `detach plateau`：覆盖 G1 的物理离手延迟；
4. `brake/retract`：离手后制动，避免手继续追着物体上升；
5. `intercept`：按预测下降轨迹移动到接取位姿；
6. `preclose/close/hold`：根据 G1 首动与到位延迟发送命令并稳定保持。

## 5. 当前 O0 进度与角度目标

当前 8 g cube 已确认的最新 Sim 结果：

| measured rotation | free flight | catch | 真机包络 |
|---:|---:|---|---|
| 1.61°–1.71° | 0.055–0.058 s | stable | 手臂参考可继续整理 |
| 4.59° | 0.178 s | stable | 手臂参考通过，作为 low 档 |
| 7.87° | 0.266 s | stable | 已有可导出的 high 档 |
| 7.95°–8.01° | 0.269–0.271 s | stable | high 档鲁棒备选 |
| 12.3°–12.7° | 0.417–0.432 s | miss | throw-only 边界，不进入真机主结果 |

O0 的 low / medium / high 已统一导出，实测分别为 4.59° / 6.48° / 7.87°。目标角是 policy 输入，报告中使用
视频实测角度。20°以上不再作为当前真机硬指标；若机械条件允许，可作为 stretch，不能拖延四物体成功 demo。

## 6. 方法实现

### 6.1 参数化动作生成器

以 J2/J3/J5 的分段 quintic spline 为动作骨架。policy 输出：

- backswing/reversal/plateau/brake/intercept 的关节节点和持续时间；
- release/preclose/close 的 G1 时刻和开度；
- 抓取深度、release pose、catch pose 与动作强度；
- 目标旋转角对应的完整轨迹参数，而不是只选择一个固定角度动作。

### 6.2 Object/pose-conditioned policy

policy 输入至少包含：

```text
[dimensions, mass, principal inertia, grasp width/depth,
 friction prior, arm/G1 delay, desired rotation angle]
```

输出为轨迹参数分布或若干候选。先用 Sim 搜索得到的优质候选做 supervised warm start，再在 domain-randomized Sim
中按需要使用 RL 或 contextual policy optimization 微调。当前已有 supervised piecewise response 可以直接生成连续
pose；RL 属于可选增强，不能为了形式上使用 RL 拖延真机 demo。若使用 RL，必须记录训练数据、state/action、
reward 和它相对 supervised warm start 的实际增益。

### 6.3 Detach predictor

从 Sim rollout 和可用真机释放数据学习：

```text
commanded q/dq + object context + G1 schedule
→ detach time, object linear velocity, angular velocity, uncertainty
```

它用于补偿 G1 对角动量的吸收和不同物体惯量导致的 release 差异。旧 25/35 g cube 数据只能作为 prior；
四个新物体要用各自真实参数重新验证。

### 6.4 J candidate ranking

候选评分至少包含：

```text
J = angle error
  + catch position/velocity error
  + non-target-axis rotation
  + insufficient separation / flight time
  + joint speed/acceleration/margin cost
  + collision and mechanical-loss cost
  + uncertainty / low Sim success probability
```

每个 `(object, desired pose)` 先生成多个候选，再由 J 排序并进行 native Sim rollout。只有完整离手、单轴旋转、
下降段接住和稳定保持都通过的候选才能导出真机。

### 6.5 Sim-to-real

- 使用真实 20 ms arm period、约 80 ms tracking lag 和 G1 release/close delay；
- 对 mass、friction、grasp depth、delay 和 tracking gain做小范围 domain randomization；
- 真机只执行最终 q/dq 和 G1 schedule，不在线改轨迹；
- 真机视频测得的 detach time/angle/catch error只用于下一轮离线更新。

## 7. 多物体推进顺序

### Phase A：O0 三 pose 完成交付

状态：Sim 与 plan-only 交付完成，待真机按梯度验证。

- 固定 4.59° low 和 7.87° high；
- 补约 6° medium；
- 导出三档 20 ms timeline、G1 schedule、Sim 视频和 plan-only配置；
- 真机按 empty → soft-mat throw-only → recatch逐档验证。

### Phase B：四物体单档成功

状态：四物体 Sim 单档稳定接取完成；O1–O3 arm handoff 已导出，待现场 G1位置标定和真机验证。

- 实现 cuboid 三轴尺寸/惯量和最窄边抓取；
- O1/O2/O3 分别从低角度开始；
- 每个物体先得到一个 stable Sim profile，再交付真机；
- 优先保证四个物体都能明显离手、旋转并接住。

### Phase C：每物体增加 pose 变化

状态：Sim 已完成。O1/O2/O3 分别已有约 3–5° low 与约 6.5° high；待真机验证后再决定是否扩大角度。

- O1/O2/O3 各增加至少一个与低档可区分的较高角度；
- 优先调整 object-conditioned policy、release angular velocity、brake 与 catch timing；
- 不通过更换旋转轴或加入 J1/J4/J6 动态动作制造表面上的“不同 pose”。

角度使用逐级探索：

```text
已验证的 micro-toss
→ low：完整离手、可测旋转、稳定接住
→ medium：与 low 的视频角度明显可区分
→ high：在机械包络内尽量扩大
→ 20° / 30° / 40° / 60° / 80° 只作为逐级 stretch target
```

报告使用视频实测角度，不使用 profile 名称中的目标角冒充结果。40°以上不是最小交付门槛；若 stock G1、开环
误差或机械寿命限制了角度，优先保住四物体完整成功和多 pose 差异。

### Phase D：鲁棒性与视频

- 每个最终 profile 做重复试验；
- 保存正常速度、慢放、角度 overlay 和失败类型；
- 形成四物体 × pose 的结果表。

## 8. Baseline 与论文证据

至少保留以下对比：

| 方法 | 定义 |
|---|---|
| M0 fixed replay | O0 单一固定轨迹直接重放到所有物体 |
| M1 analytic scaling | 仅按质量/惯量和 ballistic time做解析缩放 |
| M2 search-only | 使用同一参数化轨迹搜索，但没有 learned Detach/J model |
| M3 full | object/pose-conditioned proposal + Detach predictor + J ranking + Sim validation |

主要指标：catch rate、measured rotation mean/std、angle error、free-flight duration、axis alignment、稳定保持时间、
失败类型。四个物体都来自当前开发集，因此先称作 multi-object transfer；只有额外未参与训练/调参的物体才能称为
unseen generalization。

## 9. 完成标准

### 最小可交付 demo

- O0/O1/O2/O3 四个物体各至少一个真机完整成功 profile；
- 每次物体完全离开双侧夹指，产生可见单轴旋转，再由同一 G1 接住并保持 ≥0.5 s；
- 主结果动态关节为 J2/J3/J5，J1/J4/J6固定；
- O0 至少三个可区分 pose 档位；
- O1/O2/O3 各至少一个成功档位，随后尽量增加第二档；
- 每个主 profile 至少运行 5 次并报告 catch rate 与实测角度，而不是只选单次视频。
- 若时间不足，先完成四物体各一个成功，再补 O0 多 pose；M0–M3 不得阻塞真机主视频。

### 理想完整结果

- 四个物体各有 low/high 两档，O0 有 low/medium/high；
- 同一 policy 对 object context 和 desired pose连续条件化；
- M0–M3 公平比较，M3在多物体成功率或角度误差上有明确优势；
- 生成一段四物体 montage 和每物体至少两档 pose 的慢放视频。

较大角度属于加分项。若受 stock G1、开环误差和机械寿命限制，论文中如实报告硬件边界，并用四物体成功、
多 pose 条件化和 baseline 对比证明方法，而不是强行追求 40°/60°。

## 10. 真机执行顺序

每个新 profile 必须由现场操作者明确启动：

```text
plan-only
→ 0.25× empty arm
→ 0.5× empty arm
→ 1.0× empty arm
→ 1.0× empty G1
→ soft-mat throw-only
→ guarded recatch
→ repeated demo trials
```

保存 commanded/actual q/dq、G1 command/position、controller status、侧视视频和人工 success label。异常振动、
关节跟踪丢失、物体飞出软垫范围或 G1碰撞时立即停止该 profile。

## 11. 交付目录

```text
configs/objects/
  cube38_8g.json
  cuboid30_20g.json
  cuboid33_26p6g.json
  cuboid38_37g.json
configs/open_loop_flip/<object>/<pose>.json
sim/tools/                 # candidate generation, training, Detach, J selection
sim/scripts/               # Isaac reproduction
real_handoff/<object>/<pose>/
  timeline.json
  g1_schedule.json
  plan.json
scripts/                   # plan-only / empty-arm / object replay
analysis/                  # video angle measurement and result aggregation
docs/media/                # selected Sim/real videos and tables
README.md                  # 真机电脑入口
```

## 12. 近期执行清单

1. [完成] O0 三档 Sim、20 ms timeline、G1 schedule 与 plan-only 交付；
2. [完成] Sim runner 支持 cuboid 三轴尺寸、质量和长方体惯量；
3. [完成] O1/O2/O3 object config，以及每物体 low/high 两档稳定 Sim recatch；
4. [完成] O1/O2/O3 low/high arm timeline 与 plan-only 文件；
5. [完成] O1/O2/O3 连续目标 5.5° 的 pose-conditioned Sim与 plan-only handoff；
6. [准备完成] 离线 G1 profile 标定工具；现场仍需逐物体实测整数位置；
7. [现场第一优先] 恢复 O0 已有 micro-toss baseline，并保存当天保底视频；
8. [现场] 实测 O0–O3 的 G1 held/release/preclose/close position，生成独立 profile；
9. [现场] 每个物体依次完成 empty arm → empty G1 → soft-mat throw-only → guarded recatch；
10. [现场] O1/O2/O3 每完成一个就立刻保存成功 profile 和视频；四物体均成功后再增加第二档 pose；
11. [结果] 每个最终 profile 做至少 5 次，离线测角并整理四物体 montage；
12. [可选方法] 补 M0/M1/M2 与 M3 的公平比较，不得阻塞主 demo。

挂载本文件授权离线开发、Sim、训练和 plan-only 验证。任何真实机械臂连接或运动仍需现场操作者明确启动。

## 13. 挂载后的第一批动作

1. 阅读 `README.md`、`REAL_ROBOT_TEST_20260817.md` 和 `docs/POSE_CONDITIONED_RESULTS_20260825.md`；
2. 运行最小 CPU tests 与 plan-only，确认 runner、object profile、timeline 和 G1 template一致；
3. 为四个物体生成一页现场命令表，分成 plan-only、empty、throw-only、recatch，禁止默认连接机器人；
4. 使用 `scripts/26_calibrate_open_loop_profile.py` 为每个物体记录 G1 实测位置；O1–O3 不得沿用 O0 的
   `370/520/370`；
5. 先复现 O0 baseline，再按 O1 → O2 → O3推进低角度完整成功；
6. 每完成一个物体就立即保存 profile、日志、正常速度视频、慢放和人工 label；
7. 四物体各一个成功后，再进行 angle ladder 和 M0–M3，不在单一失败角度上无限调参。

## 14. 明确禁止事项

- 禁止在未获现场操作者明确指令时连接 `192.168.2.232` 或发送任何运动命令；
- 禁止把相机包装成来不及工作的高速闭环；它只负责记录与离线测角；
- 禁止将 O0 的 G1 整数位置直接复制到 O1–O3；
- 禁止把 Sim 的 G1 drive radians 当成真机 G1 position；
- 禁止动态使用 J1/J4/J6 来伪造 pose 差异；
- 禁止为了追求 40°/60°/80°反复 miss，导致四物体主 demo没有完成；
- 禁止围绕 hash、seal、sidecar、审计链和极低概率 case扩展开发；
- 禁止把四个开发物体称为 formal unseen generalization；额外未参与训练/调参的物体才能作为 unseen。
