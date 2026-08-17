# xArm6 visible-spin sim → real handoff

日期：2026-08-17

## 当前结论

固定约 38 mm、35 g sim cube 已完成一条不依赖 camera 的可见旋转同夹爪抛接：actual q/dq
形成 detach state，ballistic prior 传播到 catch，13.5 mm bounded lateral residual 进入 J1 servo，
随后 G1 在下降段闭合。third-view、wrist 和 spectator 只录像，不进入控制。

最终 nominal 结果：

```text
outputs/visible_spin_natural_proprio_v1_marked/
```

关键数值：

- detach：0.690 s；
- continuous free flight：0.245 s；
- maximum hand-relative separation：49.4 mm；
- rise after detach：55.1 mm；
- net cube rotation：19.45°；
- pre-contact vertical velocity：-0.094 m/s；
- bilateral contact fraction：1.0；
- `catch_stable=true`；
- `visible_spin_toss_success=true`；
- reference peak qdot / qdd：1.7448 rad/s / 13.0574 rad/s²；
- `observation_mode=proprioceptive`、`camera_control_enabled=false`。

## 先看视频

```text
outputs/visible_spin_natural_proprio_v1_marked/spectator.mp4
outputs/visible_spin_natural_proprio_v1_marked/spectator_slow_0p25x.mp4
outputs/visible_spin_natural_proprio_v1_marked/spectator_third_view.mp4
outputs/visible_spin_natural_proprio_v1_marked/spectator_wrist.mp4
```

sim cube 有一个无质量、无碰撞的红色角标，只用于让旋转在视频中可辨认。真机 cube 也应贴轻量
非对称标记。

## 复现

```bash
cd /home/ubuntu/toss_project/xarm_6
bash sim/scripts/11_run_visible_spin.sh
```

默认输出到 `outputs/visible_spin_natural_proprio_v1/`。runner 使用：

- motion reference：`sim/configs/outward_vertical_real_detach_v7.json`；
- 35 g、38 mm cube，hand-local y offset -2 mm；
- sim open command 0.655 s、physical detach 约 0.690 s；
- catch close 0.800 s、intercept 0.840 s；
- actual q/dq ballistic prior；
- lateral residual `[0, 0.0135, 0] m`；
- 20 ms zero-order hold；
- camera recording enabled but camera control disabled。

## 为什么没有使用正 J5 / 强制 J6 spin

URDF/FK 搜索显示收到的正 J5 natural seed 在 release 时 TCP 半径约 0.20 m，tool axis 指向 base。
更大范围搜索也没有找到同时满足正 J5、外侧半径和 EE 朝外的候选，因此按 goal 的 fallback 使用
已经真机验证的负 J5 outward family。

显式 J6 角速度注入也做过 Isaac 接触实验，但没有作为最终默认：

- `visible_spin_v2_seed_20260817`：J6 约 1.0 rad/s，离手 0.230 s、净旋转 5.23°，catch 不稳定；
- `visible_spin_v2_centered_1p2`：J6 约 1.2 rad/s，净旋转 2.87°，未接住。

规则 cube 在对称夹持下不会可靠继承纯 J6 roll。最终 nominal 的 19.45° 来自真实接触和 2 mm
固定偏心，不是在 detach 后写入 cube angular velocity。`sim/configs/outward_visible_spin_v2.json`
保留显式 J6 候选作为 negative evidence，但默认 runner 不使用它。

## Camera 到底怎么用

本轮 camera 不是 catch 的硬依赖：当前腕姿下 wrist 很容易被手腕/夹爪遮挡，third-view 也不保证
连续覆盖飞行。控制依赖 actual q/dq release prior 和弹道传播，因此两路同时丢失仍会执行 nominal
catch。camera 的实际职责是：

- global/spectator：人工确认 cube 是否独立离手、经过 apex、旋转并重新接住；绝不进 policy；
- third-view：记录主实验视频；若真机检测稳定，可之后增加一次 bounded lateral update；
- wrist：抓取前检查和近场录像；当前腕姿下不把飞行末端可见性作为成功条件。

仓库已有 `configs/global_camera_real.json` 和 `configs/wrist_camera_real.json`。不要退回旧的缺失 YAML
路径，也不要为了强行让 wrist 看见 cube 而改变已经可接取的腕姿。

## 给真机电脑的执行顺序

先更新仓库并看证据：

```bash
cd ~/toss_project/xarm6-toss-real
git pull origin main
```

然后：

1. 保留真机电脑上尚未同步的 `scripts/22_run_empty_handoff.py`、
   `src/xarm6_toss/real_timeline.py` 和已有 raw logs；本次提交不会覆盖它们。
2. 先按 `REAL_ROBOT_TEST_20260817.md` 复跑冻结的 micro-toss baseline，确认 G1 370→520→370、
   20 ms servo、约 80 ms lag 和 `linear_spd_limit_factor=1.6` 没有变化。
3. 把 `real_handoff_visible_spin/controller_config.json` 的 timing、ballistic prior 和 13.5 mm lateral
   seed 接入真机本地 runner。不要直接运行 Isaac 的 `11_run_visible_spin.sh` 控制真机。
4. 新分支必须依次做 0.25×、0.5×、1.0× empty preview；检查 joint limit、C60、自碰、线缆、
   q/dq 和 G1 event。通过后先做 throw-only，并在 cube 外侧放软垫。
5. catch 先试 close 0.780 s，再试 0.800 s，每个只做少量 trial。出现轨迹偏差、C60、异常电流、
   cube 朝 base 飞或线缆风险时立即停止并回到冻结 baseline。
6. cube 贴一个几乎无质量的非对称彩色角标，global camera 录全程；third-view/wrist 能看见多少就
   保存多少，不因丢帧阻塞控制。

## Nominal repeat

`outputs/visible_spin_natural_proprio_hold_bias13p5/summary.json` is the second
complete nominal run: 0.245 s free flight, 19.45 deg rotation,
`catch_stable=true`, and `visible_spin_toss_success=true`.

## 已知边界

当前结果只证明固定 35 g sim development cube 的 nominal success。20 g、50 g 和偏移 detach timing
诊断均未形成同样稳定的 catch，因此不能宣称质量/时延泛化。最终 sim catch controller 的 command
acceleration cap 是 20 rad/s²，高于旧 1× throw reference 的 13.0574 rad/s²；它必须经过真机 empty
preview，不能直接下发。真机 visible-spin 仍未验证，真实成功后才把状态改成 real validated。
