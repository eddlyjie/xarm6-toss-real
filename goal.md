# xArm6 camera-under-forearm 定轴翻滚抛接 v3

## 2026-08-18 stable-upgrade 停止 checkpoint

本轮已停止继续扫 controller/timing 参数。以 stable-recovered reference 做 throw-only 时，cube
达到 0.121 s 严格 all-link free-flight、56.9 mm 上升、30.4 mm 最大 hand-relative
separation、内部 apex、下降段首次重新接触和 0.961 tumble-axis alignment；这说明当前动作本身
已经能产生合格的短腾空窗口。但是 detach→apex 只有 2.83°、整段目标轴旋转约 3.15°，且没有
稳定接住，因而 strict goal 仍未完成。

把 close 延后、降低 G1 stiffness、推迟/提前 catch servo、增加 early-J1 lateral motion 都没有
同时保住腾空并形成 bilateral stable catch。最后的 soft-close run 保住 0.116 s free-flight、内部
apex 和下降段首次接触，但仍只是右侧单边接触。v18 高抛分支在 80 ms lag 和当前 1× 真机
joint envelope 内也没有可达的在线 recatch，不继续围绕它加 controller 特例。

本轮没有保留新的 controller 源码改动，也不改变真机默认交付：继续使用
`bash sim/scripts/13_run_stable_camera_closed_loop.sh`。它是已经三 seed 重复的“小旋转 + camera
correction + 稳定接取”版本；本轮 long-flight 结果只作后续 reference-level throw/catch 协同设计的
证据。完整结果和失败边界见 `docs/STABLE_UPGRADE_CLOSEOUT_20260818.md`。

## 2026-08-18 strict v18 收束 checkpoint

本轮停止继续扩展 controller。`camera_under_tumble_v18_mid_pose` 的 throw-only 已达到
1.137 s all-link robot-free、123.2 mm 上升、内部 apex、0.914 axis alignment 和
4.303° detach→apex 目标轴旋转，且 command/reference 机械门槛通过；但没有 recatch，
整段 signed target rotation 也只有 6.842°，所以 strict goal 未完成。

最接近的 catch 实验有 0.131 s free-flight，但只发生上升段右指单边轻触，未形成 bilateral
stable catch。未验证的 velocity/lateral-pre/two-stage 接球接口不进入真机默认源码。
当前真机仍使用下方 stable-recovered checkpoint；strict v18 的完整证据边界见
`docs/STRICT_V18_CHECKPOINT_20260818.md`。

## 2026-08-18 实用 checkpoint（strict goal 继续保留）

已修正 IsaacLab `xyzw` quaternion contract 与 wrist-camera extrinsic，并重复验证一个可优先
移交真机的“小旋转 + 稳定接取 + camera correction”分支。三次不同 camera seed 均完成
0.078 s all-link robot-free、约 52.9 mm 上升、目标轴 alignment 0.987、detach→apex 2.13°、
双侧接触率 1.0 和稳定接取。third-view 在每次控制窗口 7/7 检出，detach 后实际触发 5 次
ballistic/controller update；Probe/J 也实际选择并覆盖了 nominal timing。推荐单命令入口是：

```bash
bash sim/scripts/13_run_stable_camera_closed_loop.sh
```

该 checkpoint 是为了尽快做 2–3 次真机验证，不改变下方 strict goal，也不得被写成 strict
完成。它仍未满足 strict 的 0.12 s、25 mm 全离手分离、内部 apex 和 5° detach→apex
门槛。修正后的长飞行 `outputs/quaternion_contract_v2/v3_throwonly` 达到 1.124 s free-flight、
65.3 mm 上升、0.893 alignment、3.07° detach→apex 和 11.50° signed target-axis rotation，
但尚未 recatch，也未同时达到 strict rotation 门槛，只可作为下一阶段动作证据。旧输出曾报告的
12.63° 是错误 quaternion 顺序的派生结果，禁止继续引用。

真机绝对时刻不能直接照抄 sim：先复测 G1 25–44 ms detach delay 和约 80 ms arm lag，
再以实测 detach 为时间原点安排 catch。完整边界和执行顺序见
`docs/STABLE_RECOVERED_HANDOFF_20260818.md`。
无视觉更新的 fallback 仍为 `bash sim/scripts/12_run_stable_recovered.sh`。

## 最终目标

在保留现有真机 micro-toss baseline 和当前明显腾空 recatch baseline 的前提下，开发一条新的
xArm6 + G1 固定 cube 抛接分支：

```text
约 38–40 mm 轻量 3D 打印 cube 固定抓取
→ 静态旋转 wrist roll，使 wrist D435 位于 forearm 下侧
→ 保留朝外且有足够上抛/上翘空间的姿态
→ 由受约束 angular Jacobian 选择的 J2/J3/J5 协调 flick 产生指定水平翻滚轴角速度
→ G1 physical detach
→ cube 在完全无机器人接触时沿指定轴可见翻滚并经过 apex
→ actual q/dq release prior + ballistic propagation 预测下降段
→ brake/retract 后同一 G1 重新接住并稳定保持
```

本轮只针对同一只规则小 cube，真机最终做 2–3 次成功即可。不追求 90° 大翻转、目标姿态
transport、多物体泛化、端到端 RGB/RL、GelSight 或腕部 F/T。成功率优先，但必须是真正的
空中定轴翻滚，不能再用接取碰撞产生的姿态变化充当飞行旋转。

## 必须纠正的两个误解

### 1. “翻腕”指 J6 静态换边，不是搜索正 J5

用户要求的是绕 gripper/tool roll axis 静态旋转 EE 最前端，使 wrist camera 从 forearm 上侧换到
下侧，从而避免继续上翘时 camera/housing 先靠近机械臂。它主要是 J6 branch 选择；J4/J5 只负责
配合保持 TCP、夹爪方向和 clearance。不得再把“翻腕”解释成以正 J5 为目标。

“camera 在下方”必须在 `link5` / forearm 局部几何中判断，并使用 D435 housing 和 cable keepout；
不能用 camera optical center 的 world-Z 高低替代。`configs/wrist_camera_real.json` 的 optical
extrinsic 是权威输入，但它本身不是完整碰撞外形。

当前成功 trajectory 在 detach 时 J6≈0.175 rad，仍是未换边姿态。约 180° 的直接等价解
`J6≈-2.966 rad` 离下限只剩约 0.175 rad，因此只作搜索参照，不能默认直接采用。优先搜索能把
camera housing 放到 forearm 下侧、同时保留 joint margin 的约 120–160° roll branch。

### 2. 目标是水平 tumble，不是绕 Z yaw 或绕 tool roll

在 detach 时定义：

```text
d_finger = 两根夹爪手指的延伸方向，由实际 FK 得到并归一化
z_world  = [0, 0, 1]
a_tumble = normalize(d_finger × z_world)
```

`a_tumble` 是同时垂直于手指延伸方向和 world Z 的水平轴。cube 应像向前翻跟头一样围绕该轴
翻滚。若 `||d_finger × z_world||` 太小，说明姿态无法稳定定义目标轴，该 pose 直接拒绝。

当前已交付结果不满足这个定义：release angular velocity 约
`[0.058, -0.039, -0.268] rad/s`，主要绕 `-world Z`，与目标轴约差 83°；detach→apex 只转约
1.66°，到 0.85 s 只转约 2.52°。0.86 s 后角速度突然升高，而现有 evaluator 只检查左右 finger
sensor，没有覆盖 gripper base/palm。此前报告的 19.45° 很可能主要包含漏检的近接取碰撞，不能
作为本 goal 的旋转成功证据。该结果保留为“明显腾空并接住”baseline，不删除、不冒充 tumble。

## 权威真机事实

开始执行前必须读取仓库根目录的 `REAL_ROBOT_TEST_20260817.md`。冻结保留：

- xArm6 + G1，20 ms `servo_j`；
- 已真机完整执行的 reference peak：1.74483445 rad/s、13.0573925 rad/s²；
- arm tracking lag 约 80 ms；
- G1 370 → 520 → 370，speed 5000；
- baseline release / close command：0.636 / 0.720 s；
- physical detach delay：25–44 ms；
- `linear_spd_limit_factor=1.6` 后 1× baseline 无 C60、无 20 ms 超期；
- global D435 serial `317222073552`，wrist D435 serial `233622079809`；
- 相机配置以 `configs/global_camera_real.json` 和 `configs/wrist_camera_real.json` 为准。

旧 baseline 和现有输出只允许读取、复现和对照；新分支不得覆盖它们。

## 机械臂限制：所有阶段的硬门槛

任何 pose、search sample、reference waypoint、插值点、brake/retract、catch correction 和最终
servo command 都必须满足本节。不能先生成超限动作再依赖 controller clipping；超限候选直接失败。

### Joint position hard bounds

来自收到的 xArm6 URDF：

| Joint | Hard range (rad) |
|---|---:|
| J1 | [-3.14000, 3.14000] |
| J2 | [-1.92000, 2.09440] |
| J3 | [-3.92700, 0.19198] |
| J4 | [-3.14000, 3.14000] |
| J5 | [-1.69297, 3.14159] |
| J6 | [-3.14159, 3.14159] |

所有连续轨迹点必须在 hard bounds 内。handoff candidate 的全轨迹 minimum joint margin 必须
≥0.15 rad，目标 ≥0.25 rad；低于 0.15 rad 的 wrist-under branch 不得移交真机。

### Joint velocity / acceleration / servo step

URDF joint velocity hard limit 为 3.14 rad/s，但本轮真机 transfer 使用更严格的已验证 1× envelope：

```text
max_i,t |qdot_cmd[i,t]|  <= 1.74483445 rad/s
max_i,t |qddot_cmd[i,t]| <= 13.0573925 rad/s²
control period           = 0.020 s
max joint step           <= 0.0348967 rad per command
max qdot change          <= 0.261148 rad/s per command
```

不得再使用 3.10 rad/s / 20 rad/s² 的旧 sim search cap 生成 handoff candidate。若 SDK/firmware 后续
提供更低的实时限制，取更低值。URDF 没有给出可信 acceleration hard limit，因此在获得 controller
query 前，13.0573925 rad/s² 是不可突破的上限。

### Effort、Cartesian speed 和 controller limits

URDF arm effort limits 为 `[50, 50, 32, 32, 32, 20]`。仿真必须报告每个 joint 的 peak applied
effort；任何超过对应值的 candidate 失败。

真机执行前必须读取 controller 当前 linear speed limit/factor。每个 trajectory sample 用 FK/Jacobian
计算 TCP linear/angular speed，要求 requested Cartesian speed 不超过 controller 实际报告值，并保留
至少 10% margin。出现或预测 C60 的 candidate 失败，不能通过临时提高 controller limit 掩盖。

### Collision、clearance 和工作区

必须对 5 ms physics trajectory 和 20 ms command trajectory 同时检查：

- 全机器人连续 swept self-collision；
- G1 fingers/base、link4/link5/link6、wrist D435 housing 之间的 collision；
- wrist camera cable keepout 与 forearm 的 clearance；
- cube 在计划 free-flight 区间与所有 robot/gripper links 的 collision；
- TCP 位于 base 外侧，抛出方向不朝 base；
- throw-only 预测落点位于软垫区域。

camera-under-forearm 不是视觉偏好，而是 motion-clearance 条件。必须保存 wrist-under pose 的
link-frame 数值、最小 clearance 和 spectator 截图；只看关节角或 wrist image 不算验证。

## 动作设计

### Phase A：寻找 camera-under-forearm wrist branch

1. 从已经真机执行的 start/release pose 出发，保持 TCP 在外侧工作区。
2. 扫描 J6 的换边候选，优先约 ±120°、±140°、±155°，由 link5-frame housing/cable clearance
   选择正确符号；不得默认 `+π` 或 `-π`。
3. 用 J4/J5/必要时 J2/J3 做受约束 IK，使 gripper finger direction 和上抛空间合理。
4. 对 start、flick start、detach、brake、retract、intercept、hold 以及全部插值点运行本 goal 的
   position/velocity/acceleration/effort/collision gate。
5. 若没有满足限制的 underside branch，停止并报告几何不可行，不得退回旧腕姿后仍称完成本 goal。

### Phase B：产生指定轴角速度

J6 的主要任务是静态把 camera 移到 forearm 下侧；动态 tumble 不能再靠纯 J6 roll。detach 前
80–160 ms 使用 angular Jacobian 设计协调 flick：

```text
Jv(q) qdot ≈ desired upward/outward TCP velocity
Jw(q) qdot ≈ omega_tumble * a_tumble
```

现有 release-pose Jacobian 的实算结果表明 J2/J3/J5 是目标轴和向上速度的主要自由度，J4 仅在
clearance/IK 需要时参与；不得继续凭关节编号猜测 flick 轴。在全部机械约束下搜索
`omega_tumble`，先尝试 0.8、1.2、1.6 rad/s。目标轴
alignment 比角速度大小优先。允许 2–4 mm 固定偏心抓取帮助传递 torque，但必须可复现并保持
prethrow stable；不得利用随机滑动制造旋转。

旋转只能由 detach 前真实 gripper–cube contact motion 产生。初始化后禁止直接写 cube pose、
quaternion、linear velocity 或 angular velocity。

### Phase C：detach、brake/retract 和 catch

- 使用 actual q/dq、FK/Jacobian、固定 `T_hand_object` 和 25–44 ms detach prior 建立 release state；
- G1 开始打开后等待真实 contact loss，不把 command time 当 detach time；
- detach 后立即在 qdot/qddot 限制内 brake，并向下/外侧 retract，避免 wrist/forearm 追随 cube；
- cube 经过 apex 后用 ballistic prior 预测下降段 intercept；
- catch correction 必须经过同一机械限制和 collision gate；
- G1 close 后双侧接触并稳定保持至少 0.5 s。

## Contact 与旋转证据必须重做

现有左右 finger force 不足以定义 free flight。新的 evaluator 必须覆盖 cube 与以下所有对象：

```text
left/right fingers
gripper base / palm
link_eef / link6
wrist camera collision proxy
其他可能进入 catch workspace 的 robot links
```

free flight 从所有 robot–cube contact 消失后开始，到任意 robot/gripper link 首次重新接触前结束。
旋转只在这个严格区间内计算，并输出逐帧 quaternion、angular velocity、contact pair 和 rotation axis。

单条 tumble success 必须同时满足：

- physical detach 和严格 contact-free flight ≥0.12 s；
- hand-relative separation ≥25 mm；
- apex 位于严格 free-flight 内，且在下降段才重新接触；
- detach angular velocity 与 `a_tumble` 的轴对齐
  `abs(dot(normalize(omega_detach), a_tumble)) >= 0.85`；
- detach→apex 围绕 `a_tumble` 的累计旋转 ≥5°；
- 整段严格 free-flight 的 signed tumble rotation ≥12°，优先 15–35°；
- 非目标 yaw/roll 分量不能大于目标 tumble 分量；
- 任何接取碰撞之后的旋转都不计入；
- `catch_stable=true`，双侧接触并保持 ≥0.5 s。

cube 必须使用轻量非对称颜色/角标，让 spectator 视频能分辨轴和转角；角标不得明显改变质量、
摩擦或夹持宽度。

## 最小闭环、Probe/J 和 camera

主控制链保持真机可部署：

```text
actual q/dq + FK/Jacobian
→ physical-detach release state
→ gravity / torque-free ballistic propagation
→ constrained intercept candidate set
→ J 选择可达 timing / bounded correction
→ constrained servo_j + G1 close
```

固定抓取点可以 hard-code。Probe 使用 paired empty/held q/dq/effort/current/G1 position 输出
effective payload、held/slip 和 detach uncertainty；posterior 至少要在 conservative flick 与 nominal
tumble candidate 之间改变 J，不能只记录。learning 只允许小型 bounded residual 或 candidate
selection，不做端到端视觉策略。若本轮没有训练 residual，必须称为 sim-tuned bias，不能写成 learned。

camera 角色：

- global/spectator：完整拍到 wrist-under pose、release、apex、定轴翻滚、catch、hold，只用于验收；
- third-view：录像和可选 bounded correction，不保证全程可见；
- wrist：换到 forearm 下侧后用于抓取 pre-check 和机会观测，不要求飞行全程可见；
- 两台 policy camera 同时丢失时，ballistic nominal catch 仍须执行；
- simulator truth 和 spectator 永不进入 policy。

## 开发顺序

1. 归档当前 v2 goal，冻结所有已有成功输出。
2. 修复 all-link contact/free-flight evaluator，加入 rotation-axis decomposition 测试。
3. 加入 D435 housing/cable proxy，并搜索 camera-under-forearm J6 branch。
4. 用 constrained angular Jacobian 生成 J4/J5 flick；先做纯运动学 limit/collision report。
5. 通过后再运行 Isaac contact dynamics，先 throw-only，再 catch。
6. 固定 nominal cube 至少得到 2 次完整 tumble + catch success；失败 trial 原样保留。
7. 输出 normal/slow spectator、third-view、wrist 视频和 trajectory/summary。
8. 生成真机 config；真机仍按 0.25× → 0.5× → 1.0× empty preview → throw-only → 最多 2–3 次
   catch trial 的顺序执行。

## 每次 trial 必须输出

- 全时序 q/qdot/qddot/effort 和每个 joint 的 hard-limit margin；
- 20 ms max joint step、max qdot change、TCP linear/angular speed；
- controller linear-speed limit/factor 与 margin；
- self-collision、camera/forearm/cable minimum clearance；
- wrist D435 在 link5 frame 的位置、housing corners 和 underside 判定；
- `d_finger`、`a_tumble`、detach omega、axis alignment；
- all-link contact pairs、严格 free-flight 起止、detach→apex 和全程 signed tumble rotation；
- ballistic prior、J ranking、selected intercept、bounded residual；
- catch contact、hold duration 和三路视频路径。

summary 中必须有一个总门槛：

```text
mechanical_limits_pass
and camera_under_forearm
and strict_contact_free_flight
and target_axis_tumble
and descending_bilateral_catch
and stable_hold
```

只有上述全部为 true 才能标记 `tumble_toss_success=true`。

## 完成定义

本 goal 完成必须同时具备：

1. wrist camera 确实位于 forearm 下侧，并有数值 clearance 和正常全局视角证明；
2. 所有 planned/commanded motion 严格不超过 joint、velocity、acceleration、effort、Cartesian 和
   collision limits；
3. cube 在严格无任何 robot contact 的区间围绕 `a_tumble` 可见翻滚，而不是 yaw 或接取撞转；
4. 至少 2 条固定 nominal 完整成功：detach、apex、≥12° target-axis tumble、descending catch、
   bilateral stable hold；
5. actual q/dq ballistic catch、Probe-conditioned J 和 bounded correction 使用真机可用 observation；
6. 输出可复现 runner/config/tests、完整视频、summary/trajectory 和诚实的 real handoff；
7. 状态在真机成功前保持 `sim_validated_real_unverified`，不得把 Isaac success 写成真机成功。
