# J5 forward-rotation sim→real handoff

更新日期：2026-08-20
状态：`sim_validated_real_unverified`

## 结论

新分支按真机观察 pose 工作：J4 只负责左右换 branch 并固定在 165°，J6 固定在 −1.5°，
真正的前翻与上抛由 J2/J3/J5 完成。相机不进入真机主 policy；spectator、third-view、wrist
只用于录像和离线核对。

本轮保留两个不能混写的结果：

| 结果 | strict flight | separation | apex | signed forward rotation | catch |
|---|---:|---:|---|---:|---|
| `throwonly_4p75deg` | 0.163 s | 109.3 mm | internal | 4.747° | 无 |
| `probe_j_regrasp_1p42deg` | 0.049 s | 10.6 mm | 否 | 1.416° | 双侧稳定 |

throw-only 证明该 branch 已产生真实离手、内部 apex 和接近 5° 的目标轴前翻；它没有抓回。
Probe/J 版本证明最小闭环可以在同一 trial 中完成 Probe、J selection、ballistic servo 和稳定抓回；
它不是“大角度”结果。当前诚实边界是：大角度与稳定抓回尚未在同一 trial 同时成立。

## 动作与真机数值

蓄势 pose：

```text
J1=3.5°, J2=9.8°, J3=-25.7°, J4=165.0°, J5=82.5°, J6=-1.5°
```

职责：

- J4/J6 在 throw reference 中静止；J4 只换左右 branch，J6 只保持 camera housing/cable 在下侧。
- J2/J3/J5 生成向上/向外 TCP velocity 与前翻角速度。
- 固定抓点使用 hand-local `[+4, 0, +24] mm` 偏置；`+24 mm` 靠近指尖增加掌座净空，
  `+4 mm` 让 cube 绕开掌座。真机必须把这个偏置映射到同一 gripper frame，不能凭图猜符号。
- cube nominal 为 35 mm、25 g；policy 不读取 sim mass，Probe 输出 effective payload/held/slip。

1.6 rad/s reference 的离线门槛：

```text
control period                 0.020 s
reference peak joint speed    0.906 rad/s
reference peak acceleration   8.929 rad/s²
catch commanded peak speed    1.156 rad/s
catch commanded acceleration  13.057 rad/s²
J4 minimum hard-limit margin  about 0.26 rad
```

真机 G1 事件：

```text
open command   t=0.585 s, position=520
detach prior   25–44 ms after open command
stable close   t=0.640 s, position=370
held           position=370
firmware speed 5000
```

0.640 s close window 很窄。真机先重新测 G1 actual position/detach delay；若 measured delay 与
25–44 ms 不一致，按 physical detach 对齐 close，不要照抄 wall-clock time。

## Probe/J 与 closed loop

完整 sim 链路：

```text
paired empty/held Probe
→ held/slip gate + effective payload/detach uncertainty
→ J ranking conservative 0.640 vs aggressive 0.700 candidate
→ actual q/dq + 35 ms detach prior
→ gravity-constrained ballistic propagation
→ J1–J3 Cartesian catch servo，J4/J6 locked
→ G1 close
```

成功 trial 的 posterior：effective payload `20±6 g`、held probability `0.9999999994`、
slip probability `0`。J 选择 `j5_forward_rotation_stable_0640`；这不是只记录 posterior，
selected controller 实际覆盖了 CLI timing。

相机不参与该结果。third-view/wrist 丢帧不会阻止 nominal ballistic catch；global/spectator 永不进入 policy。

## Pull 后先看什么

```text
docs/media/j5_forward_rotation/throwonly_4p75deg/spectator.mp4
docs/media/j5_forward_rotation/throwonly_4p75deg/third_view.mp4
docs/media/j5_forward_rotation/throwonly_4p75deg/wrist.mp4
docs/media/j5_forward_rotation/probe_j_regrasp_1p42deg/spectator.mp4
docs/media/j5_forward_rotation/probe_j_regrasp_1p42deg/third_view.mp4
docs/media/j5_forward_rotation/probe_j_regrasp_1p42deg/wrist.mp4
```

同目录的 `summary.json` 与 `probe_j.json` 是数值证据。

## Sim 复现

Probe/J stable regrasp：

```bash
bash sim/scripts/15_run_j5_probe_j_regrasp.sh
```

大角度 throw-only：

