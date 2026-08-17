# xArm6 可见旋转抛接 v2：正 J5 腕姿、角速度注入与 brake/retract

## 最终目标

在已经成功的真机 micro-toss / rapid release–recatch baseline 上，新建而不是覆盖一条
可见旋转抛接分支：

```text
固定抓取约 38–40 mm 的轻量 3D 打印 cube
→ 正 J5（EE 前腕反转）外侧上抛姿态
→ 上抛末段给 cube 注入小而可控的角速度
→ G1 physical detach
→ 机械臂立即 brake / retract，不再跟随 cube 一起上升
→ cube 独立上升并产生可见的小角度旋转
→ release prior + ballistic belief 预测下降段 intercept
→ third-view 可见时做一次或数次 bounded correction
→ 同一 G1 重新接住并稳定保持
```

本轮不追求大幅翻转、目标姿态 transport、多物体泛化、GelSight、腕部 F/T 或端到端 RL。
cube 只要在空中产生肉眼和数值都能确认的姿态变化即可：硬门槛为净旋转至少 8°，优先目标
15–45°。接住优先于旋转角度，不为追求 90° 翻转牺牲真机成功率。

## 已有真机事实：必须作为新仿真的权威输入

权威报告是仓库根目录的 `REAL_ROBOT_TEST_20260817.md`。本轮开始时先读该文件，不再沿用
旧 goal 中已经被真机推翻的假设。

已经验证并必须冻结保留的 baseline：

- xArm6 + G1 使用 20 ms `servo_j` 和 1× timeline 完整执行；
- 真机 reference peak 约 1.7448 rad/s、13.0574 rad/s²；
- arm tracking lag 重复测得约 80 ms，而不是旧仿真使用的 90 ms；
- G1 使用 370 / 520 / 370、speed 5000；
- 最终真机 release / close command 为 0.636 / 0.720 s；
- physical detach 预计发生在 0.661–0.680 s；
- 1× 在 `linear_spd_limit_factor=1.6` 后可完整执行，20 ms 超期周期为 0；
- 13:38、13:39、13:42 三次 controller-complete，操作者确认能够 recatch；
- 该结果是有效 open-loop micro-toss baseline，但不是明显飞行、旋转或 learned closed loop。

旧 `0.45 rad/s / 1.5 rad/s²` 是早期保守 transfer cap，不再用来否决已经在真机完整运行的
1× reference。新候选不得默认比已验证的 1× 更激烈；若 qdot、qdd、TCP linear speed 或
Cartesian factor 超过真机已验证 envelope，必须单独报告并由操作者批准。

当前视觉效果不明显的根因也已由真实 q/dq + FK 确认：detach 附近 TCP 向上约
1.33–1.36 m/s，cube 理论上能上升约 90–94 mm，但 release 后 TCP 继续上升了近似距离，
夹爪一直追随 cube，object–gripper 相对分离很小。继续只调 G1 几毫秒不是本轮方法。

## 数据同步边界

`REAL_ROBOT_TEST_20260817.md` 已经到达 devserver，但报告中列出的部分真机文件当前仍未出现：

```text
scripts/22_run_empty_handoff.py
src/xarm6_toss/real_timeline.py
tests/test_real_timeline.py
outputs/real_empty_handoff/20260817_*/
```
`configs/global_camera_real.json` and `configs/wrist_camera_real.json` are already synced; in the paragraph below, the missing handoff items are only the real runner, timeline module, tests, and raw logs.

运动设计可以先依据报告中的实测数值和已有 URDF/FK 开始；但最终 handoff 前必须把上述真机
脚本、相机 JSON 和至少三条最终 baseline 的 q/dq/current/G1 logs 同步回来。文件缺失时如实
记录，不得伪造 raw evidence，也不得退回依赖已经失效的旧相机 YAML 路径。

## 姿态选择：正 J5 是主分支，旧反腕是 fallback

当前 sim trajectory 使用 J5≈-1.51 rad，离 J5 下限只剩约 0.18 rad，且更激烈 follow-through
可能接近自身结构。这个姿态及其真机成功 baseline 原样保留，只作 fallback。

新主分支使用用户提出的“EE 前面 joint 反转”思路，即搜索正 J5 的自然腕姿，而不是简单给
旧轨迹某个 joint 加 π。已有真机 `natural_j5_candidate.json` 提供起点：

```text
start seed   [0.0611, -0.1038, -1.1218, 0.0227, 2.3599, 0.3316]
release seed [0.0611,  0.3180, -1.4167, 0.0227, 2.9312, 0.3316]
```

这些只作 IK/search seed，不是可直接执行的真机轨迹。必须用 xArm6 URDF 在连续轨迹上检查：

- 所有关节 bounds 和至少 0.15 rad 的动态 margin；
- start、detach、brake、retract、intercept 和 hold 的连续 swept self-collision；
- G1、wrist camera 与 forearm/link 的碰撞及线缆空间；
- TCP 始终位于 base 外侧，水平半径优先 ≥0.35 m；
- 抛出方向不朝向 base，软垫区域覆盖 throw-only 落点；
- 真机 third-view 和 wrist 的实际 FOV，不要求两台同时看到 cube。

若正 J5 family 无法同时满足 collision、外侧工作区和 descending catch，可回到已验证负 J5
baseline 加 brake/retract；不能为了坚持“反转”而提交自碰或贴 joint limit 的结果。

## 如何让 cube 真正旋转

旋转必须来自 detach 前的真实接触运动，不能在 detach 后写 simulator cube quaternion 或
angular velocity。Isaac 中 cube pose/velocity 仍只允许 episode 初始化写一次。

首选实现：

1. 保留已经验证的 whole-arm upstroke 和竖直 release velocity；
2. 在 detach 前约 80–140 ms 加入平滑 J6 roll，必要时配合 J4/J5，使 tool angular velocity
   在 detach 时约为 0.8–2.5 rad/s；
3. 先搜索低档 0.8 / 1.2 / 1.6 rad/s，不直接追求最大 spin；
4. G1 从 370 向 520 运动时，cube 通过真实双指接触继承角速度并自然 detach；
5. detach 后不再继续 roll 追随 cube，机械臂进入 brake/retract。

若纯 J6 roll 因 cube 夹持对称、摩擦不足或相机视角导致姿态变化不明显，可测试小的 J4/J6
协调角速度或 2–4 mm 可重复偏心抓取。偏心只作为第二选择，必须保持固定抓取可复现，不能把
随机滑动包装成 controlled rotation。

cube 本身几何近似对称。真机实验前在不同面贴颜色/非对称角标，或使用已有 3D 打印纹理，
否则 spectator video 无法可靠区分 2° 和 20°。标记不得明显改变质量、摩擦或夹持宽度。

## 如何让 cube 与夹爪分开

保留真实 baseline 的 pre-release upstroke，但在预计 detach 后立即改变手部运动：

- 以 real release q/dq 和 25–44 ms G1 detach delay 形成 release belief；
- detach 后约 60–100 ms 内平滑制动主要上抛 joints；
- 同时让 TCP 停止上升，或向下/外侧撤离约 30–60 mm；
- 不允许 brake/retract 轨迹在 cube 尚未 detach 时夹击或扫到 cube；
- 下降段 intercept 初始搜索以报告中的约 0.20–0.26 s after detach 为中心；
- 报告给出的 `q_stop` 和约 0.863 s host intercept 只作 seed，必须在 Isaac contact dynamics
  和实际控制 lag 下重新优化。

目标不是让夹爪在原地无限等待，而是形成可解释的三段运动：upstroke/release、brake/retract、
descending catch。catch 可以主要依赖规则 cube 的弹道 prior，只需小幅 pose correction。

## 最小闭环和 J

camera 不作为本轮真机控制的硬依赖。单次抛接的主链固定为：

```text
actual q/dq + FK/Jacobian
→ nominal physical-detach time 的 6-D release state
→ gravity / torque-free ballistic propagation
→ bounded intercept residual
→ J 选择可达的 catch timing / lateral correction
→ servo_j + G1 close
```

控制器必须使用实际 q/dq，而不是只重放 commanded trajectory。质量不进入理想重力弹道；Probe
只需输出 effective payload / held / slip posterior，并让 J 选择已经在 sim 标定的 timing 或 residual
候选。固定 35 g 开发 cube 允许使用 sim 学得的 13.5 mm lateral residual；它必须单独记录，不能
伪装成 camera update，也不能外推成 20–50 g 全范围泛化。

third-view 和 wrist 可以录像、做离线轨迹/旋转测量，或在真实可见时提供 bounded correction；
两者没有有效 observation 时仍必须完成 nominal catch。spectator/global camera 只用于人工验收。

## Isaac 实施和验收

先冻结旧真机 micro-toss baseline，再建立独立 visible-spin 分支。主候选如果正 J5 搜索不能同时
满足外侧工作区和 EE 朝外，应按前述 fallback 使用已验证的负 J5 outward family，不能为了形式上
“翻腕”缩到 base 附近。

单条成功必须同时满足：

- `cube_state_writes_after_initialization == 0`；
- `prethrow_stable == true`；
- 连续离手至少 0.12 s，相对 hand 分离至少 25 mm；
- 飞行净旋转至少 8°，优先 15–45°；
- apex 位于自由飞行内部，并在下降段重新接触；
- `catch_stable == true`、双侧接触并稳定保持至少 0.5 s；
- `visible_spin_toss_success == true`；
- policy 使用 proprioceptive / deployable observation，simulator truth 和 spectator 不进入控制；
- reference qdot/qdd 不超过已经真机执行的约 1.7448 rad/s / 13.0574 rad/s²，新增 motion 仍需
  0.25 / 0.5 / 1.0× 空载检查。
