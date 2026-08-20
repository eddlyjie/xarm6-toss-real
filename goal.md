# xArm6 forward-rotation release-mediated regrasp v4

更新日期：2026-08-20

> 本节是当前唯一生效的 goal。下方 v3 内容仅作历史证据，不再作为本轮完成条件。

## 核心任务

针对同一只约 35–40 mm 的轻量规则 cube，开发并移交一个最小动态 regrasp：

~~~text
固定抓取
→ Probe 判断 held/slip 与动力学不确定度
→ J4 静态换到外翻 wrist branch，J6 使 wrist camera 朝下
→ 上抛并产生真实 contact loss
→ cube 在离手段产生可测的前向小角度旋转
→ actual detach q/dq + ballistic prior 预测短时轨迹
→ J 选择 regrasp reference/timing
→ 同一 G1 双侧重新抓稳
~~~

主目标是“真实离手 + 前向 pose change + 稳定 recapture”，不是夹爪原地开闭，也不再要求
明显高抛、内部 apex 和 ≥12° flight-only tumble 同时成立。大幅 ballistic toss 保留为 stretch goal。

## 当前证据与判断

已经分别证明：

- stable camera closed loop：三 seed 均稳定接住，all-link contact-free 0.078 s，
  最大 hand-relative separation 14.4 mm，detach→apex rotation 2.13°；
- long-flight throw-only：contact-free 1.124 s，flight-only signed target-axis rotation 11.50°，
  但没有 recatch；
- 真机 baseline：1× arm reference、20 ms servo_j、G1 370→520→370 和同手 rapid recatch 可运行，
  但尚未证明明显 pose change 或 runtime closed loop。

所以“小角度旋转”和“稳定接取”并非分别做不到；当前问题是两者尚未在同一中等能量 reference
上同时成立。继续扫 close time 不会解决这个结构问题。

当前 stable reference 的 J4≈0，J5 从 -1.509 rad 运动到约 -1.58 rad，而 J5 hard lower limit
为 -1.693 rad；动态 brake 段只剩约 0.11 rad margin。这直接支持搜索 J4 等价换边 branch，
把 J5 搬到正值且有更大上翘空间的工作区。

## J4/J5/J6 的职责

### J4 静态换 branch

当前 stable reference 的 J4 接近 0，J5 工作在负下限附近。新分支不再靠 J6 单独“反腕”，而是
搜索 J4 的腕部等价换边：

~~~text
J4 candidate: ±140°、±150°、±160°
目标 J5 工作区: 从负下限附近移到正值且保留足够 flick/brake margin
J6: 在保持 finger/catch geometry 后，使 wrist camera optical axis 朝下
~~~

不能简单把 J4 加 π 后直接执行。J4、J5、J6 构成耦合 wrist orientation，必须以 FK/IK 保持
TCP 位置、finger direction 和外侧工作区，再由连续 joint/collision gate 选择符号与角度。

优先目标是让整条 throw/brake/regrasp 轨迹的 J4/J5/J6 minimum margin ≥0.25 rad；最低不得
低于 0.15 rad。若 ±160° 逼近 J4 hard limit，则使用 ±140° 或 ±150°，不强求视觉上的整 180°。

### J5-dominant forward flick

J5 是本轮主要动态自由度。J4 branch 冻结后，用 angular Jacobian 检查 J5 对 a_forward 的贡献，
由 J2/J3 配合提供 upward/outward TCP velocity，J5 提供前向 angular velocity。J4/J6 在 release
窗口原则上保持静态，只在 IK 和 orientation compensation 必要时做小幅平滑变化。

候选必须同时满足：

- J5 不再贴近 -1.693 rad lower limit；
- detach 前 a_forward angular velocity projection 为期望符号；
- TCP 仍向上/外侧，不朝 base；
- brake 后 J5 有足够反向行程；
- 不利用 joint clipping 生成 flick。

### J6 与 wrist camera

J6 的任务是让 wrist camera 朝下并保持 finger opening 方向，不承担主要抛掷角速度。使用
configs/wrist_camera_real.json 的 T_link_eef_camera 和当前 FK 判断 optical axis，而不是用 J6
正负号猜测。

wrist camera 对本轮控制是可选的：

- global/third-view 是 release、飞行、pose marker 和验收的主相机；
- wrist camera 可以不接入 policy，只保留抓取前或接住后的局部录像；
- 若相机、housing 和 cable仍装在真机上，仿真必须保留其 collision proxy；
- 若整个 wrist camera/mount/cable 都拆除，才可以从碰撞模型中移除；
- 固定 cube 的 grasp point 允许 hard-code，不依赖 wrist vision。

## 前向旋转定义

detach 时由 FK 定义：

~~~text
d_finger = 两根手指延伸方向
z_world  = [0, 0, 1]
a_forward = normalize(d_finger × z_world)
~~~

旋转符号选择为 cube 朝机器人外侧“向前翻”的方向。每次 trial 分别输出：

- detach angular velocity 在 a_forward 上的投影；
- contact-free interval 内的 signed forward rotation；
- 非目标轴 rotation；
- pre-grasp 到 stable post-catch 的 hand-object orientation change。

接取碰撞之后的旋转不能冒充 flight-only rotation，但 release-mediated regrasp 的最终 pose change
允许由短飞行和受控 recapture interaction共同产生，二者必须分别报告。

## 动作设计

### Phase A：J4 换 branch + J6 camera-down IK

从真实 start/release TCP 附近，对 J4 ±140°、±150°、±160° 做受约束 IK。J5 优先选择正值且
保留 flick/brake margin 的解；J6 再用于保持 finger direction 并让 camera optical axis 朝下。

每个候选沿 start、release、detach、brake、regrasp、hold 全轨迹验证：

- TCP 位于 base 外侧；
- finger opening 对准预计 regrasp corridor；
- J4/J5/J6 以及全关节 minimum margin ≥0.15 rad，目标 ≥0.25 rad；
- G1、link4–6、cube 和仍保留的 camera housing/cable 无碰撞；
- J5 对 a_forward 的 angular Jacobian contribution 足够且符号正确。

先保存静态 spectator、camera optical axis 和 link-frame 数值，再运行带 cube physics。

### Phase B：中等能量 release

保留已经真机运行过的 pre-release upstroke，围绕 stable 与 long-flight reference 之间生成少量
结构候选，不做参数网格：

1. stable reference；
2. stable + 更早/更强的 reference-level brake；
3. stable + 小 J2/J3/J5 forward angular component；
4. 候选 3 + 固定 2–4 mm grasp offset。

目标是 contact-free 0.06–0.15 s、10–30 mm separation，而不是先追求最高 apex。

### Phase C：brake/retract 与局部 recapture

physical detach 后使用预先规划的 20–100 ms brake/retract，使 hand 不再与 cube 等速上升，
但 finger capture center仍留在局部可达 corridor。reference 控制点必须是两指 capture center，
不能继续控制 gripper_base 让 cube 落到 palm。

G1 close 和 arm re-entry 相对 physical detach/intercept 调度。80 ms tracking lag 必须进入 reference，
不得依赖临时 online IK 在 apex 后追赶。

### Phase D：最小 closed loop

Probe 不做精确称重，只输出 held/slip、effective payload interval、detach-delay uncertainty 和
contact asymmetry。J 至少在以下两个冻结候选间选择：

~~~text
stable_recapture
forward_pose_change_recapture
~~~

selected candidate 必须改变 arm reference、close timing 或 correction bound，不能只写日志。

运行时至少满足两项：

1. actual q/dq + physical detach estimate 改变 intercept/close schedule；
2. Probe posterior 改变 J selection；
3. timestamped third-view observation 改变 bounded catch target。

sim-trained J/residual 在真机前冻结，不用 2–3 次真机成功做在线训练。

## 真机约束

沿用 REAL_ROBOT_TEST_20260817.md：

~~~text
control period                 0.020 s
max joint speed               1.74483445 rad/s
max joint acceleration        13.0573925 rad/s²
max joint step                0.0348967 rad
max qdot change/command       0.261148 rad/s
minimum joint margin          0.15 rad，目标 ≥0.25 rad
arm tracking lag              ≈80 ms
G1                            370→520→370, speed 5000
physical detach delay         25–44 ms
~~~

任何 reference sample 超过 joint/Cartesian/effort limit 或产生 self-collision 都失败，不能依赖 clipping。

## 主任务成功门槛

### Sim

冻结同一配置运行至少 5 个 seed，至少 3 次同时满足：

- all-link physical contact loss；
- continuous contact-free ≥0.060 s；
- hand-relative separation ≥10 mm；
- detach omega 与 a_forward alignment ≥0.75；
- contact-free signed forward rotation ≥4°；
- pre-grasp 到 stable post-catch orientation change ≥5°；
- bilateral stable hold ≥0.5 s，catch_stable=true；
- 至少两项 runtime closed-loop 条件真实改变 command；
- 通过 1× 真机 joint、collision 和 clearance gate；
- spectator 与 third-view 视频能辨认 contact loss 和 forward pose change。

不要求 internal apex，也不要求首次接触一定发生在下降段。

### Real

保留全部 3–5 次尝试，至少 2 次满足：

- global video 或等效 timestamp evidence 证明 contact loss 和可见 separation；
- 带不对称角标的 cube 显示 pre/post orientation change ≥5°；
- 同一 G1 recapture 并稳定保持 ≥0.5 s；
- 无 C60、20 ms overrun、joint/collision violation；
- actual-detach adaptation、Probe/J selection 或 third-view correction 至少一项进入真实 command。

若最后一项没有实现，只能称 real open-loop dynamic regrasp baseline。

## Stretch goal

主任务完成后才尝试：

- all-link contact-free ≥0.12 s；
- separation ≥25 mm；
- internal apex 与下降段首次 recontact；
- forward-axis alignment ≥0.85；
- detach→apex flight-only rotation ≥5°；
- 整段 flight-only signed forward rotation ≥12°；
- bilateral stable catch ≥0.5 s。

stretch 失败不影响主任务完成。继续 stretch 时优先扩大 reference-level brake/retract 可达域或修改
finger capture geometry，不再围绕 close time 做 sweep。

## 开发顺序

1. 从真机电脑回传实际执行过的 scripts/22_run_empty_handoff.py，使 fresh clone 可执行。
2. 明确 wrist camera 是完整拆除、仅断开数据，还是保留使用；据此建立真实附件碰撞模型。
3. 搜索 J4 ±140° 到 ±160° 等价 branch，以 J5 正值工作区和 J6 camera-down 为选择目标。
4. 在最佳 J4/J5/J6 pose 上先复现 stable throw/catch baseline。
5. 只生成 3 个 J5-dominant 中等能量 brake/retract + forward angular candidates，先跑 throw-only。
6. 选择满足 0.06–0.15 s flight、10–30 mm separation 和 ≥4° forward rotation 的候选。
7. 写入 capture-center recapture reference，恢复 bilateral stable catch。
8. 接入 actual-detach adaptation 与 Probe/J，再加 optional third-view bounded residual。
9. 冻结参数跑 5 seeds并生成 spectator/third-view 视频。
10. 真机按 empty 0.25×→0.5×→1×→throw-only 软垫→3–5 次 regrasp 执行。

## 每次 trial 输出

- config 与 arm/G1 command timeline；
- commanded/actual q、dq、effort/current；
- detach、all-link contact pair、first recontact、bilateral contact、stable hold timestamps；
- cube pose/quaternion、linear/angular velocity；
- forward flight-only rotation、非目标 rotation、pre/post hand-object pose change；
- Probe posterior、J scores、selected candidate；
- ballistic prediction、camera residual、实际 command delta；
- joint/Cartesian limits、collision 与附件 clearance；
- spectator、third-view 视频和 failure stage。

不新增 hash、seal、sidecar 或发布审计流程。

## 完成定义

本轮 goal 只在以下条件同时成立时完成：

1. J4 换 branch、J5 正值工作区和 J6 camera-down 解通过连续 joint/collision/attachment-clearance gate；
2. sim 冻结配置达到主任务 3/5 success；
3. contact loss、forward rotation、closed-loop action change 和 stable recapture均有直接证据；
4. GitHub fresh clone 包含真实 runner、配置、文档和复现命令；
5. 真机至少 2 次达到 dynamic regrasp 主门槛。

只完成 sim 时标记 sim_validated_real_unverified；真机只跑固定 timing 时标记
real_open_loop_baseline。

---

## 以下为 v3 历史记录

下方原 goal 只用于追溯旧 strict tumble 实验、真机限制来源和失败边界。与本节冲突时，以 v4 为准。

# xArm6 camera-under-forearm 定轴翻滚抛接 v3

## 2026-08-18 passive-release + clearance checkpoint

在 stable reference 上把 G1 改成“10 ms 主动打开完成后卸载 sim drive”，消除了主要横向
release impulse：detach cube `vy` 从约 -0.153 m/s 降到 -0.003 m/s，tumble-axis alignment
从 0.961 提高到 0.992。再用 bounded Cartesian preposition 沿 x 跟随、沿 z 保持 clearance，
已经得到 0.138 s 严格 all-link free-flight、35.4 mm 最大 separation、内部 apex、下降段首次
重新接触、2.79° detach→apex 和 3.98° 整段目标轴旋转，command/reference 机械门槛通过。

但首次重新接触对象仍是 `gripper_base`，没有 finger bilateral contact，`catch_stable=false`；
因此这不是新 handoff，更不是 strict success。passive sim drive 只模拟 G1 完成开指后不再持续
对 cube 施力，真机能否等效必须由 G1 position/current 和实际 detach 录像验证，不能发送 sim
stiffness/effort 数值给真机。

下一步不再扫 close time：需要把 full-IK follow 提前并写进受约束 reference，使指尖夹持中心在
下降段进入 cube，而不是让 cube 落到 gripper base；online servo 仅做 actual-q/dq ballistic
residual。先恢复 bilateral stable catch，再提高 J5-dominant detach omega 以满足 5°/12° 旋转
门槛。默认真机交付仍是 `sim/scripts/13_run_stable_camera_closed_loop.sh`。

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
