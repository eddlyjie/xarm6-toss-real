# J5 forward-rotation sim→real handoff

更新日期：2026-08-20
状态：`sim_validated_real_unverified`

## 结论

新分支按真机观察 pose 工作：J4 只负责左右换 branch 并固定在 165°，J6 固定在 −1.5°，
真正的前翻与上抛由 J2/J3/J5 完成。相机不进入真机主 policy；spectator、third-view、wrist
只用于录像和离线核对。

本轮保留三个不能混写的结果：

| 结果 | wrist hardware | strict flight | separation | apex | forward rotation | catch |
|---|---|---:|---:|---|---:|---|
| `throwonly_9p60deg_camera_removed` | 已拆除 | 0.330 s | 397.8 mm | internal | 9.599° | 无，最后落地 |
| `throwonly_4p75deg` | 保留 collision proxy | 0.163 s | 109.3 mm | internal | 4.747° | 无 |
| `probe_j_regrasp_1p42deg` | 保留 collision proxy | 0.049 s | 10.6 mm | 否 | 1.416° | 双侧稳定 |

camera-removed throw-only 是当前最清楚的离手与前翻证据：axis alignment 0.981，首次 renewed contact
是 oriented cube-ground contact，不是机械臂；它仍低于 12°且没有抓回。
Probe/J 版本证明最小闭环可以在同一 trial 中完成 Probe、J selection、ballistic servo 和稳定抓回；
它不是“大角度”结果。当前诚实边界是：大角度与稳定抓回尚未在同一 trial 同时成立。

不要把 `2p4h` high-upward candidate 交给真机。即使把 brake reference 提前到 0.540 s 并把
reverse velocity scale 提到 −0.75，v37 仍在 0.700 s 撞到 gripper base；撞前只有 0.085 s strict
flight 和 2.408° rotation。J4 翻转提供了关节行程，但没有自动解决掌座的 Cartesian 退让问题。

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
docs/media/j5_forward_rotation/throwonly_9p60deg_camera_removed/spectator.mp4
docs/media/j5_forward_rotation/throwonly_9p60deg_camera_removed/third_view.mp4
docs/media/j5_forward_rotation/throwonly_9p60deg_camera_removed/wrist_virtual_only.mp4
docs/media/j5_forward_rotation/throwonly_4p75deg/spectator.mp4
docs/media/j5_forward_rotation/throwonly_4p75deg/third_view.mp4
docs/media/j5_forward_rotation/throwonly_4p75deg/wrist.mp4
docs/media/j5_forward_rotation/probe_j_regrasp_1p42deg/spectator.mp4
docs/media/j5_forward_rotation/probe_j_regrasp_1p42deg/third_view.mp4
docs/media/j5_forward_rotation/probe_j_regrasp_1p42deg/wrist.mp4
```

同目录的 `summary.json` 与 `probe_j.json` 是数值证据。`wrist_virtual_only.mp4` 只是仿真附加视角；
camera-removed 真机没有对应 wrist stream。

## Sim 复现

Probe/J stable regrasp：

```bash
bash sim/scripts/15_run_j5_probe_j_regrasp.sh
```

大角度 throw-only：

```bash
bash sim/scripts/14_run_j5_rotation_ladder.sh \
  1p6 \
  outputs/j5_forward_rotation/reproduce_9p60deg_camera_removed \
  --wrist-camera-hardware-removed
```

## 真机怎么用

当前 `real_handoff/j5_forward_rotation_timeline.json` 仍对应 1.6 rad/s 的 stable/low-rotation
reference，不是失败的 `2p4h`。真机按下面顺序进行：

1. 先确认 J4=165° branch 的整段 empty preview 无 self-collision，J4/J6 在 throw 中保持静止；
2. 腕部相机若仍安装，只执行既有 collision-proxy/stable branch；不要使用 camera-removed 结论；
3. camera、mount、cable 全拆除后仍执行已跟踪的 stable timeline：0.25×、0.5×、1.0× empty preview
   通过后，最多做 2–3 次 cube trial，并根据实测 physical detach delay 对齐 `stable_0640` G1 close；
4. 9.599° 当前只是 sim throw-only evidence，没有单独导出的真机 throw-only JSON；不要临场删除 close
   command。`2p4h` 更不得进入真机。

reference evidence 通过 joint position/speed/acceleration/20 ms step 门槛。Isaac actual-state 的
finite-difference acceleration 在初始化 controller transfer 出现 269.6 rad/s² 单样本尖峰，因此
`sim_actual_acceleration_matches_transfer_envelope=false`；这不是可下发的 reference acceleration，
但也不能拿它宣称真机 dynamics 已验证。真机 empty preview 必须保留。
