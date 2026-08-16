# xArm 6 真机自抛/接取项目

## 当前状态（2026-08-16）

最小 xArm6+G1 自抛自接闭环已经在 native Isaac PhysX 中实现：global D435 在释放后
拟合弹道，冻结的 learned residual 实际修正 catch target。真机速度候选 native 3/3，
扰动 cohort 8/10；三套 20 ms timeline、模型、dry-run、相机标定和视频见：

```text
real_handoff/README.md
real_handoff/manifest.json
```

下面是项目启动时的分阶段 roadmap，保留作背景；其中“当前只做 throw-only、接取以后再做”
已经过时，不能覆盖上面的权威 handoff。

## 真机电脑获取与更新

```bash
git clone https://github.com/eddlyjie/xarm6-toss-real.git
cd xarm6-toss-real

# 后续同步
git pull --ff-only

# 不连接、不控制机器人，只验证 release 后闭环计算
PYTHONPATH=src python scripts/20_closed_loop_dry_run.py \
  --model sim/models/intercept_residual_real_v1.json \
  --release-time-s 0.50 \
  --prediction-horizon-s 0.05
```

预期输出 `robot_commands_sent=0` 和 4 个 `post_release_update`。真机执行顺序、timeline、
相机坐标系和现有 SDK 集成点以 `real_handoff/README.md` 为准。

这个目录是 `toss_project` 的 xArm 6 真机代码库。它的第一目标不是一次性
复刻完整 Isaac Sim 系统，而是尽快在真机上建立一个可靠、可录像、可重复
迭代的最小闭环：

```text
xArm 6 抓住物体 → 执行一段可控摆臂 → 在运动中打开夹爪 → 物体明确离手
```

第一阶段只要物体安全离开夹爪、形成可观察的自由飞行，并落入缓冲区域，就算
成功。接取、最终姿态控制、seen/unseen 泛化和 baseline 对比都在这个可靠基础
上逐步增加。

## 1. 项目背景

`toss_project` 研究单机械臂同手自抛、自接，以及根据目标物体姿态选择抛掷、
空中旋转、接取和接后运动。仿真系统已经包含：

```text
RGB-D 物体定位
→ 抓取与主动 Probe
→ Detach/抛掷
→ 飞行状态预测
→ 动态接取候选选择
→ 夹爪接取与稳握
→ 根据目标 pose 做接后姿态与位置调整
```

可迁移到真机的核心思想包括：

- 用相机、关节编码器和夹爪状态构造真实观测，而不是读取仿真真值；
- 将目标物体 pose 作为动作条件，而不是永远执行一个固定旋转角；
- 从动作库或策略中联合选择 release 与 catch，而不是只在末端补一次 IK；
- 先在 seen 物体上建立稳定动作，再冻结参数测试 unseen 物体；
- 同时记录固定动作、pose-conditioned 规则和完整方法的对照结果。

不能直接从 Panda/Isaac Sim 搬到 xArm 6 的部分包括：

- Panda 的关节角、关节速度和轨迹；
- Panda 的 IK、关节限制、控制频率、增益和碰撞模型；
- 仿真中的 release/catch 时间常数；
- GelSight、相机和手眼标定参数。

这些必须在 xArm 6 上重新实现和标定。真机代码应保持简单，优先完成真实动作，
不要把仿真中历史性的审计、seal、hash 发布链移植到这里。

## 2. 第一天先确认的硬件

在真机电脑上补全以下信息：

- xArm 6 控制器 IP、固件版本和 Python SDK 版本；
- 末端夹爪型号、最大开口和开闭指令接口；
- 是否安装腕部/外部 RGB-D 相机；
- 是否有 GelSight 或其他触觉传感器；
- TCP、夹爪中心和相机到机械臂基座的标定；
- 实验桌高度、可用工作空间、缓冲箱位置；
- 首个测试物体的尺寸、质量和材质。

建议第一个物体使用质量轻、表面易夹持、掉落不易损坏的泡棉块或软包裹小盒子。
第一天不要从细长、易碎、过重或强反光物体开始。

## 3. 建议的代码结构

本目录已包含基础配置、离线轨迹预览、只读连接和夹爪测试；真机电脑上的 Codex
应在此基础上逐步补成：

```text
xarm_6/
├── README.md
├── configs/
│   ├── robot.yaml              # IP、速度、加速度、TCP、工作空间
│   ├── objects.yaml            # seen/unseen 物体定义
│   └── throw_only.yaml         # 首个 throw-only 动作参数
├── src/xarm6_toss/
│   ├── robot.py                # xArm 连接、状态、运动和急停封装
│   ├── gripper.py              # 夹爪接口
│   ├── trajectories.py         # 摆臂、release 和后续轨迹
│   ├── perception.py           # 后续相机/pose 接口
│   ├── action_library.py       # 多 pose 动作库与选择
│   └── logging.py              # JSON/CSV/视频记录
├── scripts/
│   ├── 00_check_connection.py
│   ├── 01_gripper_test.py
│   ├── 02_pick_and_place.py
│   ├── 03_throw_only.py
│   ├── 04_repeat_throw.py
│   ├── 05_pose_conditioned_throw.py
│   └── 06_catch.py
├── tests/                       # 只放有实际价值的 CPU/接口测试
├── data/                        # 标定、物体和轨迹数据
└── outputs/                     # 每次真机运行的日志和视频索引
```

不要在 Python import 时自动连接或移动机械臂。所有运动都应由明确的脚本入口触发。

## 4. 真机开发顺序

### M0：连通与状态读取

目标：程序能稳定连接 xArm 6，但不执行运动。

1. 安装并确认官方 xArm Python SDK 与控制器兼容。
2. 读取机械臂模式、状态、错误码、关节角和 TCP pose。
3. 实现 `--dry-run`，只打印将执行的指令。
4. 实现显式的 enable、clear error、stop 和 disconnect。

完成标准：连续读取状态，退出后连接正常释放，不发生运动。

### M1：夹爪与低速基础运动

目标：确认关节、TCP 和夹爪方向都正确。

1. 在低速度、低加速度下回到一个人工确认过的 home pose。
2. 分别测试夹爪打开、闭合和读取状态。
3. 在空载条件下完成一个很小的 TCP 位移并返回。
4. 保存命令时间、实际关节状态和错误码。

完成标准：动作方向与坐标系符合预期，夹爪能可靠开闭。

### M2：抓起物体

目标：用最简单的方法稳定抓起第一个 seen 物体。

1. 先使用人工测得或示教的固定抓取 pose，不急着接视觉。
2. 移动到预抓取 pose。
3. 低速下探并闭合夹爪。
4. 垂直抬起一小段距离，保持 1–2 秒。
5. 放回原位。

完成标准：连续多次完成抓起、保持和放回；失败时记录是定位、夹持还是滑落。

### M3：最小 throw-only

目标：抓住物体后完成一次低能量抛出。现阶段不要求接住，也不要求精确落点。

1. 在机械臂前方布置足够大的软垫或缓冲箱。
2. 从固定抓取姿态移动到 pre-throw pose。
3. 执行一条平滑、短距离的摆臂轨迹。
4. 在指定轨迹时刻打开夹爪。
5. 机械臂继续一个短 follow-through，再回到安全姿态。
6. 相机录像；记录命令关节轨迹、实际关节轨迹、release 指令时刻和夹爪状态。

第一版成功定义：

- 物体由夹爪主动释放，而不是抓取阶段掉落；
- 物体明确离开末端并出现可观察的自由飞行；
- 物体落入预先布置的缓冲区域；
- 机械臂本身没有报错或中途停止。

先从很小的摆动幅度开始，再逐步增大。不要在第一版同时加入视觉、学习策略和
接取控制，否则失败时无法判断是哪一部分造成的。

### M4：重复性和简单落点

目标：让 throw-only 从“偶尔能做”变成可重复动作。

1. 固定同一个 seen 物体、抓取 pose、起始关节状态和 release 参数。
2. 先做少量重复实验，记录成功、掉落和未释放。
3. 用相机估计落点和飞行时间。
4. 只调少数有物理意义的参数：摆臂幅度、速度和 release 时刻。
5. 稳定后再扩大到多个起始 pose。

不要只保存成功视频；每次尝试都保存简短结果和失败原因。

### M5：pose-conditioned throw

目标：目标 pose 真正改变抛掷动作，而不是始终执行一个固定角度。

推荐先用动作库，而不是立即端到端训练：

1. 为同一个 seen 物体定义 3–5 个目标姿态。
2. 为每个姿态采集或优化一条 release 动作。
3. 输入目标 pose，选择最近的已验证动作。
4. 记录实际选择的动作、目标 pose、release 状态和最终物体姿态。
5. 对相邻目标做插值时，先验证关节空间轨迹和 release 时刻是否连续。

这个阶段至少比较：

- 固定 throw：所有目标使用同一动作；
- pose-conditioned rule/IK：根据目标姿态选择或优化动作；
- learned policy：使用仿真/真机数据训练的目标条件策略。

### M6：加入接取

接取是后续目标，不应阻塞第一个真机 demo。

建议顺序：

1. 先固定抛掷，只做预定义接取轨迹；
2. 再用相机估计飞行位置，调整接取时刻和位置；
3. 然后加入目标姿态条件的接取方向；
4. 最后再加入触觉闭合、稳握和接后 transport。

接取失败要区分：物体未到达、时间不同步、空间偏差、夹爪未闭合和抓住后滑落。

### M7：seen/unseen 泛化

seen/unseen 必须按物体划分，而不是把同一物体的相邻轨迹随机拆到训练和测试。

建议第一版：

- seen：2–3 个容易夹持的盒状/柱状软物体，用于开发动作和模型；
- unseen：至少 1 个训练时完全未使用的尺寸或形状；
- 所有方法在相同起始条件、目标 pose 集和尝试次数下比较；
- 先报告抓取成功率和 throw-only 成功率，再报告落点/姿态误差；
- 接取稳定后再增加 catch success 和 full-task success。

同一个 unseen 物体一旦用于反复调参，就应转入 development/seen，不再把后续结果
称作 zero-shot。

## 5. 每次运行至少保存什么

建议每个 trial 单独一个目录：

```text
outputs/YYYYMMDD_HHMMSS_trialNNN/
├── config.yaml
├── result.json
├── robot_state.csv
└── video.mp4                 # 有相机时
```

`result.json` 最少记录：

- 物体 ID、seen/unseen 标签和目标 pose；
- 使用的动作/策略名称；
- 抓取是否成功；
- release 指令是否发出、物体是否离手；
- throw-only 是否成功；
- 若可测，飞行时间、落点和最终姿态；
- 人工备注和失败原因。

不需要围绕这些文件建立复杂的 hash、seal 或发布流程。首先保证数据字段清楚、
失败也保存、视频能对应到 trial 即可。

## 6. 最小必要的真机安全边界

这个项目不是安全攻防项目，但真机运动需要基本边界：

- 实验者始终能按到急停；
- 首轮使用软物体、低速和大缓冲区域；
- 抛掷方向不得朝向人员、屏幕、相机或其他易损设备；
- 程序启动时不自动运动；
- 每条新轨迹先 `dry-run`，再低速空载执行，最后带物体；
- 控制器报错、连接断开或状态异常时停止继续发送轨迹。

除此之外，代码应优先服务真实实验，不要为没有实际证据的极低概率 case 堆叠
大量防御逻辑。

## 7. 给真机电脑 Codex 的第一批任务

将此目录复制到真机电脑后，可以直接让 Codex 按下面顺序开发：

1. 检查 xArm 6 型号、SDK、夹爪和相机接口，补全 `configs/robot.yaml`。
2. 实现 `robot.py` 和 `00_check_connection.py`，只读状态、不运动。
3. 实现 `01_gripper_test.py` 和小范围低速关节/TCP 测试。
4. 实现固定示教 pose 的 `02_pick_and_place.py`。
5. 实现可配置摆臂轨迹与 release 时刻的 `03_throw_only.py`。
6. 建立简单的 trial 输出目录和视频/状态日志。
7. 先完成一个泡棉块的可靠 throw-only demo，再讨论接取和学习策略。

开发时优先使用官方 SDK 的非阻塞/连续轨迹接口；若控制器只能接收离散 waypoint，
应在发送前生成时间参数化轨迹，并实际记录控制周期和跟踪误差。不要把 Panda 的
关节值直接替换成 xArm 6 关节值。

## 8. 当前交接状态

当前包含两层：第一层是可立即接真机的连接、夹爪和 throw trajectory starter；第二层是
从 sim 提取的 Active Probe、Detach、dynamic catch、目标 pose coordination 和 RL
代码骨架。后者用于指导完整方法开发，不代表已经训练出 xArm 6 checkpoint。

## 9. 已附带的 starter kit

本目录现在包含一套不会默认移动机械臂的起步代码：

```text
configs/robot.example.json       # 机器人、夹爪和相机占位配置
configs/throw_only_cube.json     # 小 cube 的第一条平滑 throw 计划
src/xarm6_toss/config.py         # 配置加载与必要字段验证
src/xarm6_toss/trajectory.py     # 两段 quintic 关节轨迹与 release 事件
src/xarm6_toss/xarm_adapter.py   # 官方 SDK 的只读/夹爪薄封装
scripts/00_check_connection.py   # 只读连接和状态快照
scripts/01_preview_throw.py      # 离线生成轨迹 CSV，不连接机械臂
scripts/02_gripper_test.py       # 默认 dry-run，显式参数才执行
scripts/10_method_pipeline_demo.py # 离线展示 M2/M3 的 target-conditioned 差异
tests/test_trajectory.py         # 不需要真机的 CPU 测试
docs/REAL_ROBOT_HANDOFF.md       # 给真机电脑 Codex 的短交接
docs/SIM_TO_REAL_METHOD.md       # 仿真完整方法、J、RL、baselines 与迁移路线
configs/method.example.json      # 两目标、两技能的协调示例
src/xarm6_toss/method/           # Probe、J_catch、M2/M3 核心选择逻辑
src/xarm6_toss/learning/         # Detach、actor/critic 与 BC/PPO loss 骨架
```

先在任意装有 Python 3 的电脑运行：

```bash
python scripts/01_preview_throw.py \
  --plan configs/throw_only_cube.json \
  --output outputs/preview_throw.csv
python scripts/10_method_pipeline_demo.py --target upright_forward
python scripts/10_method_pipeline_demo.py --target quarter_turn_forward
python -m unittest discover -s tests -v
```

`robot.example.json` 中 `hardware_confirmed` 默认是 `false`，示例关节角全部是
占位值。真机电脑必须先用 xArm Studio/示教确认 home、抓取、pre-throw、release
和 follow-through，再将这些值写入新的本机配置。不要直接把示例角度用于带物体
运动。

## 10. Panda 仿真参考包

`sim_reference/` 是给真机电脑和后续 Codex 的可视化交接包，包含五段精选视频、
Panda 的控制时间线、`q/dq` 与夹爪时序参考代码，以及到 xArm 6 的逐项映射：

```bash
cd xarm_6
python sim_reference/panda_sequence_reference.py
ffplay sim_reference/videos/03_successful_full_pipeline_panda.mp4
```

先看 `sim_reference/README.md`，再看
`sim_reference/PANDA_CONTROL_AND_PORTING.md`。其中成功视频与开发失败视频已明确
分开标注；参考代码不会连接或移动真机。
