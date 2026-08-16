# xArm6 最小 Probe–Toss–Catch 真机验证目标

## 最终目标

从原始真机 setup 重新建立一条 xArm6 + G1 最小但科学有效的闭环：

```text
固定抓取小 cube
→ 轻量 Active Probe
→ 估计 detach / flight belief
→ 释放后 third-view / wrist RGB-D 依可见性接力更新弹道
→ 用 J 评分并选择可达 catch candidate
→ bounded learned residual 修正 catch target
→ 同一夹爪重新接住并稳定保持
```

真机只要求边长约 38 mm 的轻量 3D 打印 cube 成功 2–3 次，不要求 GelSight、腕部
FT、复杂抓取学习、目标姿态 transport 或多物体泛化。抓取 pose 可以 hard-code，但
probe、camera observation、J 和 learned correction 必须真实进入控制，不能只记录不使用。

## 执行边界

这是一个先在 Isaac Sim/Lab 跑通、再把同一套 deployable observation/controller 交给真机的
最小 sim-to-real goal。顺序固定为 sim 单次成功 → 3 次扰动验收 → handoff/dry-run → 真机
2–3 次成功。固定抓取、规则小 cube、短 Probe 和最小 camera/J/residual 闭环已经足够；任何
不能直接提高这条交付链真机成功率的复杂模块都不进入当前 scope。

## 可用观测

真机和 policy 只能使用：

- xArm 实际 q、dq、joint effort/current 和 controller timestamp；
- G1 commanded/actual position，以及由实测 G1 position→jaw aperture mapping 得到的
  closing-axis contact width；
- wrist D435：抓取/probe observation，以及 release 后随 EE 运动获得的 cube RGB-D
  center、depth、confidence 和 timestamp，尤其是 catch 前的 terminal observation；
- third-view D435：cube 落在其实际 FOV 内时的 RGB-D center、depth、confidence 和
  timestamp；不能假设它是覆盖整个抛接空间的全局相机；
- 已提供的相机内参、外参和 xArm/G1 URDF。

两台 policy camera 按时间戳异步接力；无需同时看到 cube，也不指定 third-view 永远是
主相机。estimator 必须消费当前真正可见且通过 gating 的 observation。

不得使用 cube 真值 pose、真值 velocity、simulator true mass、摩擦、接触标签或其他
simulator-only state 作为 policy 输入；由 deployable Probe 信号估计的 effective payload
是允许且必须使用的 observation。Isaac 中 cube pose/velocity 只允许 episode 初始化写一次。

## 真机反馈是仿真硬约束

以下原始真机文件是权威输入，不能把数值另抄一份后脱离源文件使用：

- `toss_project_sim_handoff/toss_project/real_cube_demo/configs/hardware.json`；
- `toss_project_sim_handoff/toss_project/RobotCamCalib/RobotCamCalib/outputs/intrinsics_new.yaml`；
- `toss_project_sim_handoff/toss_project/RobotCamCalib/RobotCamCalib/outputs/extrinsics_thirdview.yaml`；
- `toss_project_sim_handoff/toss_project/RobotCamCalib/RobotCamCalib/outputs/extrinsics_wrist_new.yaml`；
- `toss_project_sim_handoff/toss_project/real_cube_demo/` 中的实机时序和接口实现。

仿真、搜索、训练和 handoff 必须共同遵守：

- xArm6 command period 为 0.02 s；当前真机 transfer cap 为 joint speed 0.45 rad/s、
  joint acceleration 1.5 rad/s²。URDF 的 3.14 rad/s velocity limit 只是机械/模型上限，
  不是允许仿真超过当前真机配置的依据；
- URDF joint bounds：J1 [-3.14, 3.14]、J2 [-1.92, 2.0944]、
  J3 [-3.927, 0.19198]、J4 [-3.14, 3.14]、J5 [-1.69297, pi]、J6 [-pi, pi]；
- arm tracking delay nominal 为约 0.09 s，必须进入 command/measurement 对齐和
  trajectory execution model，并在空载执行中重新辨识；
- G1 held/partial-open/close/full-open 分别为 370/520/370/850，speed 5000，命令是
  nonblocking；detach delay 必须在实测 0.025–0.044 s 内采样，而不是瞬时释放；
- cube 边长约 38 mm、轻量低填充 3D print；得到实测质量前，仿真质量覆盖 20–50 g，
  simulator true mass 不得作为 policy observation；Probe 从真实可用信号估计的
  effective payload posterior 允许且必须进入 downstream belief；
- third-view D435 serial `317222073552`，wrist D435 serial `233622079809`，分辨率
  640×480；两者使用 raw intrinsic K：fx 597.4084346880913、fy 595.7611918577373、
  cx 316.83407708591676、cy 242.68429790012132；
- third-view raw `X_CammountCam` 为
  `R=[[-0.0150689073,-0.5894024786,-0.8076989824],`
  `[-0.9997740237,-0.0032319837,0.0210108489],`
  `[-0.0149943164,0.8078330722,-0.5892205852]],`
  `t=[1.0069862113,0.0003598100,0.6473656984]`；
- wrist raw `X_CammountCam` 为
  `R=[[0.0146068394,0.9994237319,-0.0306405670],`
  `[-0.9998694352,0.0143878488,-0.0073554334],`
  `[-0.0069103429,0.0307440061,0.9995034033]],`
  `t=[0.0695039316,0.0385871165,0.0248719289]`；
- raw calibration YAML 的 frame semantics 是权威定义。third-view 固定在 base frame，
  wrist camera 附着 `link_eef` 并每帧重算 pose；必须写 conversion test 后才能进入
  Isaac/控制代码，不能把 co-moving wrist image 中的 pixel motion 直接当作 cube velocity；
- 两台 policy camera 不要求同时看到 cube：third-view 在实际可见时提供固定视角的
  中段 flight observation；wrist 除抓取/probe 外，还必须用于 release 后和 catch 前的
  terminal tracking。estimator 应支持 third→wrist、wrist-only 和短暂全丢失后的 belief
  propagation，而不是绑定固定 camera order；
- 每个 wrist detection 必须使用该图像 `camera_timestamp_s` 对齐/插值得到的 q 做 FK，
  再由 `T_base_eef(q) @ X_CammountCam` 变换到 base frame；不得使用处理完成时的最新 q；
- 真机 motion recorder 请求 640×480@60 Hz，并保存 `camera_timestamp_s`、
  `host_received_s` 和 frame number。仿真使用真机 dry-run 测得的实际 rate、clock offset、
  receive/processing latency 和 dropped-frame pattern；
- 另设正常全局 spectator camera，仅用于人工验收和论文视频，绝不进入 policy
  observation；
- camera timestamps、实际可见 ROI、measurement noise 和 dropout 必须进入仿真；不能
  用理想真值相机代替标定后的成像几何。

这些约束必须进入 executable config 和 simulator，而不只是文档。每个可交付候选必须
输出 `real_constraints_report.json`，至少记录实际 max |qdot|、max |qddot|、全部 joint
margin、TCP 水平半径与 outward dot、所用 arm/G1 delay 分布、两台 policy camera 的
逐帧 cube visibility 和有效 timestamp 数量。任何一项越界都不是可移交结果。

`real_cube_demo/configs/natural_j5_candidate.json` 只能作为失败反例，禁止复用其中的
轨迹或姿态：它明确 `throw_execution_ready=false`，旧 release TCP 水平半径约 0.20 m，
tool axis 朝向 base 内侧。

如果 0.45 rad/s / 1.5 rad/s² 的当前 transfer cap 下无法同时达到可见 free flight 和
catch 门槛，必须提交定量 infeasibility evidence，等待真机操作者明确批准新的实机上限；
不得为得到好看的仿真结果静默提高限制。

## 最小方法

1. 固定抓取：先用已知 cube pose 和示教/硬编码抓取点，不把抓取学习作为阻塞项。
2. Probe：执行一个短、小幅、可回到中心的安全 excitation；由 q/dq/effort/current、
   gripper position 和 wrist RGB-D 得到低维 posterior。posterior 至少影响 detach
   uncertainty、release timing 或 J，不能是装饰模块。
   对规则 cube，不要求把质量精确到克，但 Probe 必须用 time-aligned empty/held motor-current
   和 effort residual 输出 effective payload、CoM-offset、held/slip posterior；信号弱时输出宽
   uncertainty，而不是删除 mass branch。posterior 必须收紧 detach/release belief 或改变 J。
   G1 actual position 必须经实测 mapping 转成 jaw aperture/projected width，并与 wrist RGB-D
   联合估计 side length、grasp offset 和 orientation confidence。它不等于三个轴的完整尺寸，
   但对基本对齐的规则 cube 足够。Probe 仍应短小，不能阻塞 ballistic catch。
3. Detach prior：用实际 q/dq、FK/Jacobian、固定 T_hand_object 和实测 G1 delay 得到
   release position/velocity belief。
4. Flight tracking：对 third-view 和 time-aligned wrist RGB-D 做 visibility-gated、
   asynchronous fusion，在 base frame 下做 gravity-constrained fit；没有 observation 时
   propagate belief，有任一相机重新看见 cube 时更新，而不是依赖理想连续 tracking。
5. Catch candidates/J：在多个时间/位置候选中，用 catch probability、相对速度、
   uncertainty、IK/reachability、collision margin 和 predicted wrist-FOV visibility 计算 J，
   优先选择既可达、又能在 catch 前给 wrist camera 留出 terminal observation window 的
   candidate。
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
- detach 后两台 policy camera 合计至少提供 3 个有效 RGB-D observation，并真实改变
  catch command；不要求这些 observation 来自 third-view；
- wrist camera 除清楚看到抓取/probe 外，还须在首次重新接触前最后 0.10 s 内提供至少
  2 个有效 cube observation，其中至少一个真实改变 terminal decision；
- wrist terminal observation 若剩余时间大于已辨识 arm tracking delay，可触发 bounded
  intercept correction；若已经来不及改变 arm target，则必须改变 G1 close deadline、
  catch confidence 或 reject decision，不能只记录；
- third-view 不保证覆盖整个抛接轨迹，但其每帧真实 visibility 必须记录；看见时必须被
  estimator 使用，看不见时不得以 simulator truth 补观测；
- spectator camera 能在同一画面完整看到机器人、cube、release、flight、catch 和 hold；
- spectator camera 绝不进入 policy observation，只用于人工验收和论文视频；
- catch 必须 bilateral contact，并稳定保持至少 0.5 s；
- 我必须逐帧查看视频，不能只根据 JSON success flag 交付。

## 仿真验收

先通过单 trial，再固定方法跑至少 3 个扰动 trial：

- cube mass 覆盖约 20–50 g；
- simulator true mass 只用于评估 Probe posterior 的 calibration/error，不能进入 policy；
  同时报告 G1/wrist 估计的 projected width、side length 和 grasp offset；
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
