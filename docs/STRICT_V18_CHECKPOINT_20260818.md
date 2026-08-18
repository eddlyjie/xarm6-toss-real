# Strict v18 throw checkpoint — 2026-08-18

## 当前结论

本轮在 `camera_under_tumble_v18_mid_pose` 上得到了一条明显、干净、轴方向正确的
throw-only 轨迹，但没有得到下降段 bilateral stable recatch。因此：

- 真机近期做 2–3 次最小验证时，仍优先使用已经发布的 `stable-recovered` 版本；
- `v18` 只作为下一阶段明显腾空/翻滚动作 checkpoint，不得直接作为真机 catch 默认动作；
- 本轮未验证完的 velocity servo、lateral pre-servo、two-stage G1 接球接口已从交付源码撤回；
- 相关输出保留在 devserver，未删除，但不进入 Git/真机默认入口。

推荐真机 handoff 仍见：

```text
docs/STABLE_RECOVERED_HANDOFF_20260818.md
sim/scripts/13_run_stable_camera_closed_loop.sh
```

## v18 权威 throw-only 结果

结果目录：

```text
outputs/strict_v18/mid_pose_e20_s160/
```

| 指标 | 结果 | strict gate |
|---|---:|---:|
| physical detach | 0.623 s | detected |
| all-link robot-free duration | 1.137 s | ≥0.12 s，pass |
| rise after detach | 123.2 mm | visible |
| free-flight apex | 0.782 s | internal，pass |
| tumble-axis alignment | 0.914 | ≥0.85，pass |
| detach→apex target rotation | 4.303° | ≥5°，fail |
| full target-axis signed rotation | -6.842° | magnitude ≥12°，fail |
| non-target rotation | 0.293 rad | larger than target，fail |
| minimum wrist-camera proxy clearance | 36.1 mm | positive |
| command/reference mechanical envelope | pass | pass |
| recatch | none | fail |

`free_flight_rotation_deg=20.34°` 是总姿态变化，不是目标轴翻滚，不能拿来替代
`free_flight_signed_tumble_rotation_deg`。该 run 后段落地仍处于“无机器人接触”区间，
因此整段 signed rotation 只用于诊断；真正可靠的飞行旋转证据是 detach→apex 的 `4.303°`。

reference envelope 峰值为 `1.740 rad/s / 12.429 rad/s²`，minimum joint margin
`0.259 rad`。trajectory 的 1 ms actual-dq 有数值尖峰，不能把有限差分得到的
`234.7 rad/s²` 当作真机 command acceleration；真机 transfer 仍以 20 ms command envelope
和空载 preview 为准。

## 可复现命令

```bash
cd /home/ubuntu/toss_project/xarm_6
env -u CONDA_PREFIX \
  /home/ubuntu/IsaacLab-3.0.0-beta2/isaaclab.sh -p \
  sim/scripts/04_native_release_smoke.py \
  --config sim/configs/camera_under_tumble_v18_mid_pose.json \
  --output outputs/strict_v18_checkpoint \
  --observation-mode physics \
  --cube-size-m 0.035 --cube-mass-kg 0.025 \
  --cube-offset-hand-m 0 0 0.0065 \
  --held-drive-rad 0.56 --held-gripper-effort-limit-n 4 \
  --partial-open-drive-rad 0.39 \
  --release-gripper-effort-limit-n 20 \
  --release-gripper-stiffness 160 \
  --settle-s 0.5 --post-release-s 1.0 \
  --release-time-s 0.62 \
  --gripper-open-command-time-s 0.60 \
  --release-drive-transition-s 0.01 \
  --detach-delay-prior-s 0.035 \
  --arm-tracking-delay-s 0.08 \
  --arm-drive-interpolation linear \
  --arm-sim-effort-scale 2 \
  --arm-sim-stiffness-scale 2.5
```

## 最接近 strict catch 的实验边界

`outputs/strict_v18/velocity_soft_k020_c072` 曾达到：

```text
strict free-flight = 0.131 s
rise               = 119.2 mm
first contact       = 0.754 s
precontact vz       = +0.285 m/s
bilateral contact   = none
catch stable        = false
```

它已越过 0.12 s 门槛，但仍在上升段由右指单边轻触，随后 cube 被推出。固定远期
intercept、较晚 servo、soft/firm two-stage G1、world-frame catch bias 和未隔离的 J1 pre-servo
均未形成 bilateral catch。最后一个 J1 pre-servo 实现还扰动了 J2–J6 throw reference，已撤回，
不能作为继续运行的入口。

## 下一步边界

若继续 strict 分支，只做一件核心工作：在不改变 J2–J6 throw `q/dq` 的条件下，把 J1 横向
预居中与 full catch controller 分离，并先证明接触前 `5–8 mm` 横向误差被消除。之后再把
detach→apex 从 `4.303°` 小幅提高到 `≥5°`。在 bilateral descending recatch 出现前，不再把
这些实验接口加入真机 handoff。
