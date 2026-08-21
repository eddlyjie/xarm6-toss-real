# xArm6 stock-G1 0/20/30/40° ballistic stable regrasp v6

更新日期：2026-08-21

> 本节是当前唯一生效的 goal。真机硬件条件冻结为现有 xArm6、原装 G1 双橡胶软垫和当前相机安装状态；不增加 insert、roller、flipper、低摩擦覆盖层或其他机械改装。下方 v5/v4/v3 只作历史证据。

## 最终交付

针对同一只约 35–40 mm 的轻量规则 3D 打印 cube，建立原装 G1 条件下可控的前翻 dynamic regrasp：

~~~text
固定抓取 + Probe posterior + requested rotation
→ target-conditioned J 选择 throw/release/brake/catch action
→ J4/J6 保持静态，J2/J3/J5 协调上抛和前翻
→ G1 actual position + actual q/dq 判断真实离手
→ Detach model 预测真实 Δt/Δv/Δω
→ 传播 cube 平移弹道与 torque-free orientation
→ 根据目标角度选择下降段 intercept 和 G1 close lead
→ 同一原装 G1 双侧重新抓稳并保持 ≥0.5 s
~~~

paper-facing target family 改为 `0° / 20° / 30° / 40°`。每个 target 必须在同一 trial 中同时完成真实离手、目标轴旋转、同手 recapture 和稳定保持。throw-only 只允许作为一次软垫 commissioning/诊断，不计入任何 target 的成功结果，也不能替代 stable regrasp。

## 固定硬件与真机约束

- Robot：现有 UFACTORY xArm6；控制周期 20 ms；约 80 ms tracking lag。
- Gripper：原装 G1、原装双橡胶软垫，`370→520→370`、speed 5000；首次运动约 22.64 ms，完整行程约 102.79 ms，physical detach delay 约 25–44 ms。
- Cube：边长约 35–40 mm、轻量低填充 3D 打印件；固定抓点允许硬编码。
- Grasp action：允许在不改变硬件的前提下沿非 closing-axis 方向使用约 4–6 mm 可重复 offset；closing-axis offset 若被 G1 自动居中，不作为方法变量。
- Wrist branch：J4 保持约 165°静态反腕 branch，J6 保持约 −1.5°静态；J2/J3/J5 提供 upward/outward velocity 与 forward rotation。
- Camera：global/third-view 用于录像、离线角度和真机结果标注；wrist camera 按真机当前安装状态建模并做碰撞检查。camera 不进入高速主 policy。
- 禁止要求真机更换软垫、增加 release insert、改变夹爪结构或超过机械臂现有限制。

~~~text
max joint speed               1.74483445 rad/s
max joint acceleration        13.0573925 rad/s²
max joint step                0.0348967 rad
max qdot change/command       0.261148 rad/s
minimum joint margin          0.15 rad，目标 ≥0.25 rad
linear speed limit factor     1.6
~~~

任何 reference sample 超过 joint、Cartesian、effort、self-collision、camera/cable clearance gate 都失败；不得依赖 clipping、sim-only object-state write 或理想瞬时开爪形成结果。

## 当前证据边界

- `v86_fast_early_avoid`：标准 G1、同一 trial，strict flight 0.332 s、signed forward rotation 9.840°、axis alignment 0.994、双侧稳定接回并保持；`v87` 完全复现。冻结入口为 `sim/scripts/15_run_stock_g1_10deg_regrasp.sh`。
- `v96_detach_minus10ms`：在冻结动作内把 G1 command/detach 前移 10 ms，strict flight 0.342 s、signed rotation 9.927°、cube detach omega 0.606 rad/s、双侧稳定接回。
- `v100_grasp_rot_plus10`：小抓取姿态修正后 strict flight 0.341 s、signed rotation 9.963°、cube detach omega 0.617 rad/s、双侧稳定接回。这是当前最大同一 trial stable regrasp；相对 v96 增益只有 0.036°，不能外推到 20°。
- `v92_r10cfh_direct_110`：strict flight 0.426 s、signed rotation 12.602°，随后先触地且没有 catch；它是当前 release height 下的 ground-limited throw boundary。
- `v108_early_burst_plateau`：release height 0.487 m、apex 0.570 m、actual hand omega 3.335 rad/s，最终双侧接回；gripper base 在 apex 区提前 recontact，strict rotation 只有 5.104°，因此不提升可交付角度。
- `v110/v117/v118/v119`：高 release 的延后下降段候选得到 9.91–10.19° strict rotation，但都先撞 gripper base、没有双侧稳定接回。J4/J6 静态的 J1/J2/J3/J5 catch mask 已验证机械限制，仍受固定腕部 catch workspace 的 x–z 耦合限制。
- `v103/v104/v105/v120` 表明降低 G1 effort、增加预开量或把 friction 从 1.2/0.9 降到 0.8/0.6 均未提高稳定离手角速度；这些支路停止继续 sweep。
- 真机 `0.636/0.720 s` baseline：操作者确认同夹爪 recatch，视觉上接近 rapid release–recatch，尚无可靠旋转角度和 learned closed-loop 证据。
- 上述 stable、throw-only、ground/base recontact 结果必须分别报告。当前 `8–12° + same-trial bilateral stable recatch` 第一门槛已完成；20°、30°、40°及对应 Probe/J 多 seed gate 仍未完成。

## 物理先验与最小闭环

规则 cube 在短时无接触飞行中的控制先验为：

~~~text
p(t) = p_detach + v_detach t + 0.5 g t²
R(t) = R_detach Exp([ω_detach]× t)
~~~

规则 cube 的三个主惯量接近，短飞行内 `ω_detach` 可近似常量。重力平移与质量无关；低填充造成的惯量、摩擦和释放差异进入 Probe posterior 与 Detach residual，无需把质量当作精确已知先验。

真实离手时：

1. 根据 G1 actual position threshold 或 measured-delay fallback 得到 `t_detach`；
2. 冻结 actual arm `q/dq`，用 xArm6 FK/Jacobian 和固定 `T_hand_cube` 得到 rigid-grasp release prior；
3. Detach model 预测标准 G1 橡胶接触造成的 `[Δt, Δp, ΔR, Δv, Δω]`；
4. 对多个下降段 `t_catch` 传播 `p(t), R(t)`，并检查 IK、joint margin、collision 和 G1 closing lead；
5. J 根据 requested angle、predicted pose error、catch probability、Probe uncertainty 和动作代价选择 executable candidate；
6. selected candidate 必须真实改变 arm reference、release lead、brake/retract、catch time 或 G1 close time中的至少一项。

## 开发顺序

### A. 先整合高能量 throw 与 catch

状态：已由 `v86/v87/v96/v100` 完成 8–12°第一门槛；后续结果只有在同一 trial 同时提高旋转并稳定接住时才替换冻结版本。

- 以 `v62` 的标准 G1 11.53° reference 为起点，接入 v47 已验证的 detach observer、ballistic prior、catch servo 和双侧 stable-hold 判定；
- physical detach 后立即制动 J5，并让 J2/J3 将手向下/侧向撤离；约 80 ms tracking lag 必须在 reference 中前馈补偿；
- release 后的手不能继续以接近 cube 的速度追随上升；catch candidate 位于下降段；
- 第一门槛固定为 `8–12° + same-trial bilateral stable recatch`；达到前停止扩大 throw-only 角度。

### B. 在原装 G1 内扩大角度

- 提高 release height，并把 catch height 放在 release height 下方，利用工作空间得到约 0.35–0.50 s 可控飞行；
- 调整 G1 command lead，使 25–44 ms physical detach 落在 J2/J3/J5 forward omega 高值区；
- 尝试在 G1 opening contact window 内维持受限的 forward angular acceleration，利用惯性载荷形成可重复接触顺序；
- 搜索非 closing-axis 的 4–6 mm 抓取 offset，要求 throw 前稳定且同一真机抓点可复现；
- detach 后先撤手，再根据目标角度进入下降段 intercept；飞行时间和 catch timing共同形成角度梯度。

### C. 形成 0/20/30/40° stable ladder

- 0°：flight-only absolute rotation ≤5°，同手稳定接回；
- 20°：actual signed rotation `20°±7°`；
- 30°：actual signed rotation `30°±8°`；
- 40°：actual signed rotation `40°±10°`；
- 20/30/40°均要求 axis alignment ≥0.85、strict all-link contact loss、无飞行中提前 recontact、bilateral stable hold ≥0.5 s；
- target response 必须单调，20/30/40°相邻 nominal angle 至少相差 7°；
- 同一 target 的 throw-only 与另一次 catch 结果不得拼接。

### D. Probe/J 与真机交接

- Probe 只需估计 held/slip、有效动力学范围和 detach uncertainty；不得把轻 cube 的噪声 residual 包装成精确称重；
- J 使用 actual-detach prediction 选择 target-conditioned action，至少一次消融证明关闭 Detach adaptation 或 target conditioning 会改变命中率/角度误差；
- sim先冻结20°和30°skill，各运行5个dynamics seeds，至少3/5 same-trial stable success；40°保存全部seed结果并报告真实成功率；
- 真机顺序：0.25×→0.5×→1× empty；最多一次软垫 throw-only 检查离手方向；随后运行20° regrasp，最多3–5次，目标至少2次完整成功；
- 20°真机成功且无C60、碰撞、线缆或落点问题后才尝试30°；40°是真机stretch；
- 每次保存 commanded/actual q/dq、G1 position与event timestamp、detach state、selected J candidate、global-camera原速视频和stable-hold标签。

## Completion definition

本轮 goal 只在以下条件同时成立时完成：

1. 标准 G1、无任何机械改装的 sim 首先得到 `8–12° + same-trial stable recatch`，证明高能量throw与catch已经整合；
2. sim nominal分别得到20°、30°、40° same-trial stable regrasp，并满足各自角度误差、axis alignment、strict flight和≥0.5 s stable hold要求；
3. 0/20/30/40° target选择不同的executable action并形成单调响应，Probe/J与Detach prediction实际改变command；
4. 冻结20°和30°skill各达到至少3/5 stable success；40°全部seed如实报告；
5. GitHub fresh clone包含真机runner、冻结配置、原装G1说明、软垫commissioning命令、global-camera录像说明和对应sim视频；
6. 真机至少2次完成约20°目标的真实contact loss、可见前翻、同手recapture和≥0.5 s stable hold；
7. 真机若只能达到更低角度，paper claim按真机证据缩小，sim结果单独标记 `sim_validated_real_unverified`。

开发过程中，任何新增结果只有在同一 trial 同时旋转并稳定接住时才提升“可交付版本”的角度。单独提高throw-only角度只作为诊断数据，不发布为新的真机regrasp版本。

---

## 以下为 v5/v4/v3 历史记录

# xArm6 0–90° pose-conditioned release-mediated regrasp v5

更新日期：2026-08-21

> 历史记录：本节不再作为当前完成条件；仅用于追溯 release-transfer、0/30/60/90°旧目标和 passive-insert 探索。

## Paper-facing objective

针对同一只约 35–40 mm 的轻量规则 cube，建立可控的前翻 dynamic regrasp skill family：

~~~text
fixed grasp + Probe posterior + requested target rotation
→ target-conditioned J selects throw/release/brake/catch reference
→ J4/J6 static camera-under branch, J2/J3/J5 coordinated flick
→ measured-G1 release and true all-link contact loss
→ Detach model predicts actual Δt/Δv/Δω
→ ballistic propagation and catch timing adaptation
→ same G1 bilateral stable recapture
~~~

paper-facing targets 固定为 `0° / 30° / 60° / 90°`。这不是四个后处理标签：target 改变时，
pre-detach q/dq、G1 release timing、brake/retract 或 catch timing 必须实际改变，flight-only signed
forward rotation 必须随 target 单调变化。`120° / 180°` 是 stretch。规则 cube 必须贴不对称
角标或使用不同颜色的面，使 90° cube symmetry 不会掩盖视觉和 pose measurement。

`v47` 的 5.055°、5/5 nominal stable regrasp 只作为真机 commissioning/fallback，证明控制、
Probe/J、detach observer 和双侧接取链路可运行；它不计入本 goal 的 pose-changing result，不能
作为论文最终角度。

## Method hypothesis

当前瓶颈不是 xArm6 已被证明只能转个位数，而是 release-mediated angular transfer 与早期
recontact：现有 reference peak joint speed 约 0.906 rad/s，低于真机已验证 1× 上限
1.74483445 rad/s。按实际 v35 detach Jacobian 重新施加 Cartesian gate 后，J2/J3/J5 在 conservative
TCP 1.44 m/s 下的 forward hand omega 上限约为 3.53 rad/s，在真机已设置的 1.6 m/s gate 下约为
3.82 rad/s；后一解主要使用 J3=-1.745、J5=+1.745 rad/s，仍位于 1× joint envelope。若 release
近似完整保留该角速度，90°需要约 0.41–0.45 s strict flight；当前 35–40% retention 不足以直接
达到该角度，因此必须同时提高 release transfer 并延长无 recontact flight。

Detach model 显式学习 G1 有限响应、grasp offset、摩擦和 Probe posterior 造成的
`[Δt, Δp, ΔR, Δv, Δω]`。同一 arm reference 必须比较 ideal instantaneous、10 ms sim 与真机
measured-G1 profile（22.64 ms first motion、102.79 ms full travel、25–44 ms physical detach）。
每次输出 pre-open/actual-detach twist、angular retention、rigid-hand residual、strict flight
rotation 和 first recontact。Detach model 的作用不仅是解释损失；J 必须根据预测 `Δω` 增加
pre-detach omega、延长 flight 或改变 release action，从而命中 requested angle。

2026-08-21 measured-G1 native `v49` 已把该 residual 直接写入 summary：detach 为 0.633 s，即
command 后 48 ms；G1 motion start 到 detach 的 contact window 为 26 ms；cube forward omega 从
pre-open 1.455 rad/s 降到 detach 0.571 rad/s，retention=0.392，relative-to-hand angular transfer
为 0.364，forward `Δω=-0.996 rad/s`。detach linear residual 很小，而 0.698 s 的左指 recontact
把 strict flight 限制为 0.065 s、rotation 限制为 1.90°。该结果是 timing/contact failure evidence，
不是能力上限；compact evidence 保存于 `docs/media/j5_forward_rotation/release_transfer_v49.json`。

2026-08-21 `v50–v65` expansion checkpoint 已把“机械臂速度不足”和“G1 detach transfer不足”分开：

- `v52` measured-G1 full-open 是原始最佳：strict flight 0.338 s、forward rotation 9.878°、
  hand/cube detach omega 2.636/0.599 rad/s，transfer=22.7%；
- `v60` 后移 release 后 hand omega 提高到 3.346 rad/s、flight 延长到 0.404 s，但 cube omega
  只有 0.554 rad/s、transfer=16.6%，最终 11.259°；
- `v62` 对称 two-stage pre-open 保持夹持且达到当前最佳 11.530°，但 transfer 仍为 16.5%；
- cube `±30°` edge placement 在 throw 前掉落，closing-axis 4 mm 偏心在 settle 时被自动居中；
- `v63–v65` 的 J1 inertial-preload reference 虽通过 1× joint/TCP gate，但约 80 ms tracking dynamics
  把短时 lateral pulse 滤掉，actual detach J1 velocity 约 0.001 rad/s，左右接触顺序没有改变，
  结果仍为 11.30–11.56°。这些候选不得进入真机运行入口。

因此标准 G1 + controller-only symmetric opening 的当前可信能力 checkpoint 是约 11.5°，不是90°。
机械臂已能提供约 3.1–3.35 rad/s forward hand omega 和约 0.40 s flight，主瓶颈是 opening 最后
数毫秒只保留 16–23% angular velocity。下一 active method branch 是可拆卸的 passive unilateral
release insert/roller：cube 仍为规则 cube，insert 只在一侧指面形成可复现 rolling detach；其
geometry、contact material 和真机安装图必须公开，并把标准 G1结果作为 hardware-matched baseline。
若不接受该最小硬件改动，则 paper claim 必须降到小角度离手 regrasp，不能以 sim 90°代替真机。

## Development gates

### A. Release transfer identification

- 固定 arm reference，完成三种 G1 profile 的 controlled ablation；
- summary 直接保存 actual-minus-rigid-hand `Δv/Δω` 与 angular retention；
- 调整 measured-G1 command lead，使 physical detach 落在 J2/J3/J5 omega peak；
- 设计 simultaneous 与可实现的 asymmetric/rolling release candidate，提高角速度保留率；
- 不允许以 PhysX-only 瞬时无摩擦开爪作为最终高角度方案。

### B. Throw-only controllability

- 在同一 1× real envelope 内生成 `30° / 60° / 90°` 三档 reference；
- 每档 all-link strict contact-free，axis alignment ≥0.85，actual angle 对 target 误差 ≤12°；
- physical detach 后立即 brake/retract，避免任何 finger/base/camera early recontact；
- 至少达到 90° throw-only 后才把 90°称为 kinematic/dynamic capability；
- 通过 90°后才尝试 120°，180°只作 stretch，不靠 joint clipping 或超 1×实现。

### C. Target-conditioned stable regrasp

- 先恢复 30° same-trial bilateral stable recapture，再做 60°，最后做 90°；
- 0° low-spin skill 与 30/60/90° skills 进入同一个 J candidate library；
- actual detach q/dq 与 learned residual 必须改变 ballistic intercept/catch schedule；
- selected target 的同一 trial 必须同时具有目标 flight rotation 和 ≥0.5 s stable hold；
- throw-only 与 recatch 结果始终分开报告，不得拼接。

### D. Frozen validation and real handoff

- 选定 60°或90° skill 冻结后运行 5 个 dynamics seeds，至少 3/5 stable success；
- third-view/global spectator 提供全程证据；camera 不进入高速主 policy；
- 真机依次 empty 0.25×→0.5×→1×、软垫 throw-only 15°→30°→45°→60°→90°；
- 只在前一档无 C60、joint/collision violation 且落点安全时进入下一档；
- 最终选定 60°或90°做最多 3–5 次 regrasp，至少保留 2 次完整成功。

## Fixed real envelope

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

J4=165°附近只负责静态 wrist branch，J6 只负责 camera/cable 下置；J4/J6 不作为动态前翻作弊轴。
J2/J3/J5 共同提供 upward/outward translation 与 forward rotation。任何 reference sample 超过
joint/Cartesian/effort/collision gate 都失败，不得依赖 clipping。

## Completion definition

本轮 goal 只在以下条件同时成立时完成：

1. controlled G1 ablation 和 release-transfer residual 已进入 summary/Detach model；
2. sim 分别得到 30°、60°、90°机械合规 strict throw-only，actual error 均 ≤12°；
3. `0/30/60/90°` target 会选择不同 executable action，并在 sim 中形成单调 angle response；
4. 至少 30°、60°、90°各有 same-trial stable recapture，选定的 60°或90° frozen skill 达到 3/5；
5. Probe/J 与 actual-detach adaptation 真实改变 command，而不只是写日志；
6. GitHub fresh clone 包含真机 runner、配置、角标说明、软垫递进命令和第三视角视频；
7. 真机选定的 60°或90° target 至少 2 次完成 contact loss、可见 rotation、同手 recapture 与
   ≥0.5 s stable hold；若真机只能达到更低角度，必须缩小 paper claim，不能用 sim 结果代替。

只完成 sim 时标记 `sim_validated_real_unverified`；真机只跑固定 timing 或个位数 rotation 时标记
`real_open_loop_commissioning_baseline`。camera 只用于离线证据，不是本 goal 的主闭环传感器。

---

## 以下为 v4/v3 历史记录

# xArm6 forward-rotation release-mediated regrasp v4

更新日期：2026-08-21

> 历史记录：本节不再作为当前完成条件；仅用于追溯 v47 commissioning、J4/J5 branch 与旧门槛。

## 核心任务

针对同一只约 35–40 mm 的轻量规则 cube，开发并移交一个最小动态 regrasp：

~~~text
固定抓取
→ Probe 判断 held/slip 与动力学不确定度
→ J4 静态换到外翻 wrist branch，J6 将 camera housing/cable 转到腕部下侧
→ 上抛并产生真实 contact loss
→ cube 在离手段产生可测的前向小角度旋转
→ actual detach q/dq + ballistic prior 预测短时轨迹
→ J 选择 regrasp reference/timing
→ 同一 G1 双侧重新抓稳
~~~

主目标是“真实离手 + 前向 pose change + 稳定 recapture”，不是夹爪原地开闭，也不再要求
明显高抛、内部 apex 和大角度 flight-only tumble 必须一次同时成立。但是新 J4/J5 branch 已把
J5 从旧负下限附近释放到 +82.5°、保留约 97.5° 正向行程，因此本轮必须认真尝试扩大前向旋转，
不能只停在当前 2.13° stable baseline。开发分成两个结果：先用 throw-only 证明 12–30° 前翻能力，
再把能量收回到可稳定接取的 5–20° dynamic regrasp。只有同一 trial 同时旋转并接住时，才能称
large-rotation regrasp；两个分支的结果不得混写。

## 2026-08-20 v47 达成 checkpoint

本轮已经在同一 trial 达到本 goal 的最小 5° dynamic regrasp，不再把 throw-only 与 stable
trial 拼接成结果：

- `v46`：proprioceptive controller，strict flight 0.173 s，signed forward rotation 5.039°，
  0.821 s 起双侧接触并稳定保持；
- `v47`：paired Probe gate 通过，J 在两个可执行候选中选择 `dynamic_5deg_g1_observer`
  （J=0.6799，fallback=0.8450），strict flight 0.173 s，signed rotation 5.055°，稳定双侧接取；
- 两次 sim 都只用 actual q/dq + FK ballistic prior；sim G1 position/effort response 在 0.615 s
  冻结 release state，与 evaluation-only physical detach 同刻，误差 0 ms；真机没有已验证的 G1
  motor-current API，因此 runtime 改用 fixed-pose camera 标定出的 detach-position threshold；
- J4=165°、J6=−1.5° 保持静态，J2/J3/J5 reference peak 0.906 rad/s、8.929 rad/s²，
  joint mechanical limits 通过；
- wrist camera/mount/cable 按 camera-removed 分支处理；spectator/third-view 只录像，未进入 policy。
- 2026-08-21 冻结 v47 配置的五次独立 native process repeat 达到 5/5 success：strict flight
  0.173 s、separation 15.48 mm、axis alignment 0.984、flight-only forward rotation 5.055°、
  pre/post hand-object orientation change 8.816°、bilateral fraction=1.0、stable catch；最后 0.5 s
  相对姿态波动最大 0.008°。五次只改变 record-only camera seed，camera 不进入控制，因此该结果
  证明 nominal repeatability，不宣称物理随机化 robustness。

v47 真机 handoff 分支从此冻结，不再为追角度修改其 arm reference。根据 2026-08-21 用户对
“全臂甩动 + J5 手腕甩动仍有明显余量”的判断，允许另开不覆盖 v47 的 rotation-expansion 分支：
先在同一 1× 真机 envelope 内让 J2/J3 提供 upward/outward 速度、J5 在 physical detach 保持前翻
速度峰值，并在 detach 后立即 brake/retract，目标是把已达 9.599° 的 throw-only 推过 12°；随后
回退能量尝试 8–10° stable recatch。无论该分支结果如何，v47 的 arm reference、G1 two-stage close、
Probe/J 和 G1 detach observer 继续原样移交真机，先做空载/软垫 throw-only，再做最多 2–3 次 cube
recatch。sim `.48/.65 rad` 不是 G1 真机位置；真机映射为约 `441/370`，最终以 G1 actual position
标定为准。可执行入口为 `scripts/22_run_j5_dynamic_regrasp.py`，标定入口为真机包内更新后的
`real_cube_demo/scripts/10_measure_detach.py`。

## 2026-08-21 Detach model 主线：显式建模 G1 release impulse

当前 rotation-expansion 分支不再把开爪后的角速度损失简单归因于“xArm 不够快”。初步 native
sim 记录显示，cube 在开爪前随手运动时的前翻角速度约为 `1.46–1.93 rad/s`，真实 contact loss
后只保留约 `0.53 rad/s`，对应约 `27–34%` angular retention。该现象应作为 Detach model 的
研究对象：G1 command-to-motion delay、有限开爪速度、指尖摩擦、grasp offset 和 Probe posterior
共同决定真实 detach time、linear velocity 与 angular velocity residual。

先固定同一条 J2/J3/J5 arm reference，只比较三种 release profile：

1. ideal instantaneous open；
2. 当前 10 ms sim transition；
3. 真机测得的 G1 response：首次运动约 22.64 ms、完整 `370→520` 行程约 102.79 ms，physical
   detach delay 约 25–44 ms。

每个 profile 必须输出 `pre-open hand/cube twist`、`actual detach twist`、`Δt`、`Δv`、`Δω`、
`eta_omega=(omega_cube_detach·a_forward)/(omega_cube_preopen·a_forward)`、strict flight rotation
与首次 recontact。只有 measured-G1 profile 也重复出现显著 angular residual，才允许把它写成
G1 release-mediated Detach 结论；当前 10 ms 结果只能作为 hypothesis，不得直接冒充真机机制。
Detach predictor 延续已有 13-D residual 语义 `[Δt, Δp, ΔR, Δv, Δω]`，由 Probe posterior 与
release action 条件化；J 使用预测的 actual post-detach state 选择 catch timing/reference。这样
即使 G1 只保留部分角速度，也不是方法失败，而是被显式预测并用于闭环协调的 detach dynamics。

## 2026-08-20 当前执行 checkpoint

本轮已经按上述职责实际执行，而不是继续审计旧资产：

- J4 固定 165°、J6 固定 −1.5°；J4 只换左右 branch，不参与前翻；
- 1.6 rad/s reference 只由 J2/J3/J5 产生，reference peak 为 0.906 rad/s、8.929 rad/s²；
- hand-local `[+4, 0, +24] mm` 固定抓点使 cube 绕开掌座；
- throw-only 达到 0.163 s strict flight、109.3 mm separation、internal apex、4.747° signed tumble；
- Probe/J stable trial 中 posterior 实际改变 J，选择 `stable_0640`，随后 0.049 s 离手、
  1.416° signed tumble、bilateral fraction=1.0 并稳定抓回；
- 2.0 rad/s candidate 虽满足 command/reference joint limits，但掌部更早追上 cube，结果劣于 1.6，
  因此不作为真机首选；
- camera 只录像，未进入上述 policy。

当前可移交的是“一条接近 5° 的 throw-only 证据 + 一条最小 Probe/J closed-loop stable regrasp”。
它们仍是两个 trial，不能宣称已经满足本 goal 的 5–20° dynamic regrasp 或下方 12° strict tumble
完成定义。下一步真机先复现 throw-only detach/timing，再启用 `stable_0640` close；sim 若继续开发，
重点是 G1 release angular-momentum transfer，而不是继续增加 J5 速度。

## 2026-08-20 最新动作决策：固定 J4，释放 J5 的大角度能力

现场关节语义按用户观察修正并冻结：J4 负责左右 wrist branch / 动作平面选择，不负责 cube
正前翻。J4 只在进入蓄势姿势前移动到约 165°，整段 throw、brake、regrasp 保持静态；不得再用
J4 的动态旋转制造或放大前翻角度。J6 也保持静态，仅负责 finger opening 方向以及 camera/cable
位于腕部下侧。真正的前翻由 J5 主导，J2/J3 配合生成向上、向外的 TCP 速度和离手后的回撤。

把 J4 换到这一 branch 的直接收益，是 J5 从旧的负下限附近搬到约 +82.5°，获得接近 97.5°
的正向可用角度和更大的无碰撞 brake 行程。因此当前 4.747° throw-only 不是这套构型的能力上限，
必须继续开发大角度 forward rotation；但不得把“J5 可用角度大”等同于允许超过真机 1× 的
速度、加速度、servo step、joint margin 或 collision gate。

下一阶段只做下面两个结构实验，不展开 close-time 网格：

1. `J5 rotation capability`：固定 J4/J6，在 1× 真机 envelope 内重新设计 J2/J3/J5 的
   accelerate→detach→brake reference，先达到 8°，再验证 ≥12° flight-only forward rotation；
   只有前一档保持 axis alignment、clearance 和 joint margin 后才尝试 20°。
2. `rotation-preserving recapture`：从合规 throw-only reference 回退能量，在 physical detach
   前按约 80 ms lag 提前发出 brake/retract reference，但 catch controller 接管时必须保留 nominal
   commanded `q/dq`，不得重置成 actual state 而消掉上抛动量。先恢复 ≥5° 的同一-trial stable
   recapture，再向 10–20°推进。

每个候选必须分别报告 release 时 J5 对 `a_forward` 的 angular-velocity contribution、cube detach
omega、strict contact-free rotation、最大 separation、首次 recontact link 和 stable bilateral hold。
大角度 throw-only 与小角度 stable recatch 仍是不同证据；在同一 trial 达到 ≥5° 并稳定接住前，
不得称为 large-rotation regrasp。camera 继续只作 spectator/third-view 录像和离线证据，不进入主控制。

## 2026-08-20 camera-removed / early-brake checkpoint

本轮确认腕部 D435/mount 的实体状态会直接改变可用抛掷空间。只有真机把 wrist camera、mount 和
cable 全部拆除时，才允许采用 `--wrist-camera-hardware-removed` 分支；此时仿真 wrist 视频只是
virtual recording，不能当作真机相机证据。

当前最佳 camera-removed throw-only 为 `v35_1p6_no_wrist_camera_oriented_ground`：

- strict all-link contact-free flight 0.330 s，最大 hand-relative separation 397.8 mm；
- apex 位于 strict flight 内，detach 后上升 24.2 mm；
- signed target-axis rotation 9.599°，axis alignment 0.981；
- reference peak speed 0.906 rad/s、acceleration 8.929 rad/s²、minimum margin 0.260 rad；
- 首次 renewed contact 是 0.945 s 的 oriented cube-ground contact，不是 robot link；
- 无 recatch，因此 `tumble_toss_success=false`，不能移交为完整 regrasp。

`v37_2p4h_early_strong_brake_no_camera` 把 brake reference 提前到 0.540 s、reverse velocity scale
提高到 −0.75；reference 仍通过 1× 限制（1.000 rad/s、12.500 rad/s²、0.260 rad margin），但 cube
在 0.700 s 撞到 gripper base。撞前 strict flight 只有 0.085 s、2.408°，比 v36 仅延迟约 7 ms，
因此该 2.4 rad/s high-upward reference 判定失败，禁止进入真机 timeline。

暂停点的判断：J4 翻转确实提供了 J5 joint-space margin，但当前瓶颈是 detach 后 gripper base 的
Cartesian 退让方向/速度，而不是 J5 上翘角度不足。恢复开发时应从 v35 的 1.6 reference 出发重做
collision-aware retract；不要继续提高 throw target，也不要继续扫 G1 close time。

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
搜索 J4 的腕部等价换边。按真机界面和用户现场观察，J4 负责左右/换边；它只用于到达蓄势姿势，
release flick 全程 qdot4≈0，不参与 cube 前翻。正前翻与上抛只由 J2/J3/J5 的协调速度产生：

~~~text
J4 candidate: ±140°、±150°、±160°
目标 J5 工作区: 从负下限附近移到正值且保留足够 flick/brake margin
J6: 在保持 finger/catch geometry 后，将 camera housing/cable 转到腕部下侧
~~~

不能简单把 J4 加 π 后直接执行。J4、J5、J6 构成耦合 wrist orientation，必须以 FK/IK 保持
TCP 位置、finger direction 和外侧工作区，再由连续 joint/collision gate 选择符号与角度。

优先目标是让整条 throw/brake/regrasp 轨迹的 J4/J5/J6 minimum margin ≥0.25 rad；最低不得
低于 0.15 rad。若 ±160° 逼近 J4 hard limit，则使用 ±140° 或 ±150°，不强求视觉上的整 180°。

用户给出的真机观察 pose 是权威 IK seed：

~~~text
degrees = [3.5, 9.8, -25.7, 175.4, 82.5, -1.5]
radians = [0.06109, 0.17104, -0.44855, 3.06178, 1.43990, -0.02618]
~~~

URDF FK/Jacobian 检查得到：

- pose 全部位于 hard bounds 内；
- J5 对 TCP 上升的 gain 约 +0.277 m/rad，对 a_forward 的 angular gain 约 +1.00；
- J5 到 upper limit 仍有 97.5°，明显优于旧负 J5 branch；
- TCP 约为 [0.544, 0.054, 0.194] m；
- J4=175.4° 距 upper hard limit 只剩约 4.5°/0.079 rad，不满足 0.15 rad handoff margin。

因此该 pose 用作 branch seed，不直接作为动态真机命令。第一候选先把 J4 收到约 165°，再由
J5/J6 和必要的 J2/J3 IK 补偿 TCP/finger orientation；165° 约有 0.26 rad J4 margin，达到目标。

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

新 branch 的目的就是使用更大的 J5 正向可用行程。先以 detach target angular velocity
0.8、1.2、1.6 rad/s 做三个 reference-level 候选；若 1× velocity/acceleration envelope 允许，
再增加一个不超过 1.74483445 rad/s 的上界候选。J5 angle range 只是可达域，实际 qdot/qddot 仍必须
受真机门槛限制。

### J6、wrist camera 与真机观测

J6 的任务是把 camera housing/cable 放到腕部下侧并保持 finger opening 方向，不承担主要抛掷
角速度。“housing 在下侧”和“optical axis 朝地”不是同一个条件：D435 optical axis 基本沿 tool
方向，J6 只能让偏置安装的相机绕 tool axis 换侧。

对用户 seed pose，J6=-1.5° 时 camera center 在 link5 frame 的局部 z 约为 -36.8 mm，world-Z
比 link5 origin 低约 128 mm，支持“相机实体位于下侧”的现场观察。

本轮真机主 policy 不使用 camera：

- wrist camera 不进入 grasp、release 或 catch controller；固定 grasp point hard-code；
- 若允许，优先将 wrist camera、mount 和 cable 一起拆除；
- 若相机、housing 和 cable仍装在真机上，仿真必须保留其 collision proxy；
- global/third-view 仅用于离线 contact-loss、pose change 和成功录像，不进入主 command；
- sim 可保留 optional camera-residual ablation，但不能把它当作真机完成条件。

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

### Phase A：J4 换 branch + J6 camera-under geometry

从真实 start/release TCP 附近，对 J4 ±140°、±150°、±160° 做受约束 IK。J5 优先选择正值且
保留 flick/brake margin 的解；J6 再用于保持 finger direction 并让已安装的 housing/cable 位于下侧。

每个候选沿 start、release、detach、brake、regrasp、hold 全轨迹验证：

- TCP 位于 base 外侧；
- finger opening 对准预计 regrasp corridor；
- J4/J5/J6 以及全关节 minimum margin ≥0.15 rad，目标 ≥0.25 rad；
- G1、link4–6、cube 和仍保留的 camera housing/cable 无碰撞；
- J5 对 a_forward 的 angular Jacobian contribution 足够且符号正确。

先保存静态 spectator、camera center/housing 的 link-frame 数值，再运行带 cube physics。

### Phase B：中等能量 release

保留已经真机运行过的 pre-release upstroke，围绕 stable 与 long-flight reference 之间生成少量
结构候选，不做参数网格：

1. stable reference；
2. stable + 更早/更强的 reference-level brake；
3. stable + 小 J2/J3/J5 forward angular component；
4. 候选 3 + 固定 2–4 mm grasp offset。

目标是 contact-free 0.06–0.15 s、10–30 mm separation，而不是先追求最高 apex。

### Phase B2：J5 rotation expansion

在同一 J4/J6 静态 pose 上，只让 J2/J3/J5 参与动态 reference：

1. J2/J3 提供主要 upward/outward TCP velocity；
2. J5 正向加速，同时贡献 upward velocity 与 a_forward angular velocity；
3. physical detach 后三关节按 1× acceleration envelope brake；
4. cube 落入软垫，先不加入 catch controller。

throw-only 依次验证 8°、12°、20° 三档 flight-only signed forward rotation。每一档必须保存
detach omega、axis alignment、all-link contact-free interval 和落点；若某档破坏 axis alignment、
工作区或机械门槛，不继续提高能量。

得到至少 12° 的机械合规 throw-only 后，才加入局部 recapture。recapture 版本从较低能量回退，
先要求 ≥5° flight-only forward rotation，目标 10–20°。不得靠延迟 close 细扫代替 reference 设计。

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

真机主闭环必须同时满足：

1. actual q/dq + physical detach estimate 改变 intercept/close schedule；
2. Probe posterior 改变 J selection；

timestamped third-view correction 仅作为 optional sim/real ablation，不属于主链。sim-trained J/residual
在真机前冻结，不用 2–3 次真机成功做在线训练。

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
- contact-free signed forward rotation ≥5°，目标 10–20°；
- pre-grasp 到 stable post-catch orientation change ≥5°；
- bilateral stable hold ≥0.5 s，catch_stable=true；
- actual-detach adaptation 与 Probe/J selection 都真实改变 command；
- 通过 1× 真机 joint、collision 和 clearance gate；
- spectator/third-view evaluation 视频能辨认 contact loss 和 forward pose change。

不要求 internal apex，也不要求首次接触一定发生在下降段。

### Real

保留全部 3–5 次尝试，至少 2 次满足：

- global video 或等效 timestamp evidence 证明 contact loss 和可见 separation；
- 带不对称角标的 cube 显示 pre/post orientation change ≥5°；
- 同一 G1 recapture 并稳定保持 ≥0.5 s；
- 无 C60、20 ms overrun、joint/collision violation；
- actual-detach adaptation 与 Probe/J selection 至少一项进入真实 command。

若最后一项没有实现，只能称 real open-loop dynamic regrasp baseline。

## Stretch goal

主任务完成后才尝试：

- all-link contact-free ≥0.12 s；
- separation ≥25 mm，目标 30–60 mm；
- internal apex 与下降段首次 recontact；
- forward-axis alignment ≥0.85；
- detach→apex flight-only rotation ≥8°；
- 整段 flight-only signed forward rotation ≥15°，目标 20–30°；
- bilateral stable catch ≥0.5 s。

stretch 失败不影响主任务完成。继续 stretch 时优先扩大 reference-level brake/retract 可达域或修改
finger capture geometry，不再围绕 close time 做 sweep。

另设独立的 throw-only rotation capability gate：在不要求 recatch 时，至少一条机械合规 reference
达到 ≥12° flight-only forward rotation；目标 20–30°。该结果不能冒充 stretch catch success。

## 开发顺序

1. 从真机电脑回传实际执行过的 scripts/22_run_empty_handoff.py，使 fresh clone 可执行。
2. 明确 wrist camera/mount/cable 是完整拆除还是仍留在机械臂上；据此建立真实附件碰撞模型。
3. 从用户 seed 搜索 J4 约 160–170° 邻域及必要的另一符号解，以 J5 正值工作区为核心目标。
4. 在最佳 J4/J5/J6 pose 上先复现 stable throw/catch baseline。
5. 固定 J4/J6，只生成 J2/J3/J5 的 0.8、1.2、1.6 rad/s rotation candidates。
6. throw-only 逐级验证 8°、12°、20°，至少保留一条 ≥12° 的机械合规 reference。
7. 从合规 throw-only 回退能量，选择 0.06–0.15 s flight、10–30 mm separation 和 ≥5° rotation 候选。
8. 写入 capture-center recapture reference，恢复 bilateral stable catch，目标 rotation 10–20°。
9. 接入 actual-detach adaptation 与 Probe/J；camera 只用于 evaluation。
10. 冻结参数跑 5 seeds并生成 spectator/third-view evaluation 视频。
11. 真机按 empty 0.25×→0.5×→1×→throw-only 软垫→3–5 次 regrasp 执行。

## 每次 trial 输出

- config 与 arm/G1 command timeline；
- commanded/actual q、dq、effort/current；
- detach、all-link contact pair、first recontact、bilateral contact、stable hold timestamps；
- cube pose/quaternion、linear/angular velocity；
- forward flight-only rotation、非目标 rotation、pre/post hand-object pose change；
- Probe posterior、J scores、selected candidate；
- ballistic prediction、实际 command delta，以及 optional camera ablation residual；
- joint/Cartesian limits、collision 与附件 clearance；
- spectator、third-view evaluation 视频和 failure stage。

不新增 hash、seal、sidecar 或发布审计流程。

## 完成定义

本轮 goal 只在以下条件同时成立时完成：

1. J4 换 branch、J5 正值工作区和 J6 camera-under geometry 通过连续 joint/collision/attachment gate；
2. sim 冻结配置达到主任务 3/5 success；
3. contact loss、forward rotation、closed-loop action change 和 stable recapture均有直接证据；
4. 至少一条独立 throw-only reference 达到 ≥12° flight-only forward rotation，并明确不冒充 recatch；
5. GitHub fresh clone 包含真实 runner、配置、文档和复现命令；
6. 真机至少 2 次达到 dynamic regrasp 主门槛。

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
