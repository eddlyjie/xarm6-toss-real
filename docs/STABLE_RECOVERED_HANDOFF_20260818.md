# Stable-recovered cube toss/catch handoff — 2026-08-18

## 结论

本仓库现在有一条适合优先拿到真机做 2–3 次验证的最小闭环：固定抓取 35 mm、
25 g 轻 cube，静态 J6 `+155°` camera-under 候选姿态，上抛后用 actual q/dq release
state 做 ballistic propagation，三次 bounded catch update 后由同一 G1 重新接住。
paired empty/held Probe 产生 posterior，J 根据 posterior 选择稳定接取 controller，并且
确实覆盖了命令行 nominal timing；cube 的真值质量没有进入 policy。

推荐入口：

```bash
cd /home/ubuntu/toss_project/xarm_6
bash sim/scripts/12_run_stable_recovered.sh
```

默认输出：

```text
outputs/stable_recovered_probe_j/run.log
outputs/stable_recovered_probe_j/summary.json
outputs/stable_recovered_probe_j/probe_j.json
outputs/stable_recovered_probe_j/spectator.mp4
outputs/stable_recovered_probe_j/spectator_third_view.mp4
outputs/stable_recovered_probe_j/spectator_wrist.mp4
```

本文件完成前又用上述 runner 原样复现了一次，证据目录是
`outputs/stable_recovered_probe_j_handoff_20260818`；三路视频均为 2.10 s、126 帧。

## 已复现结果

| Run | all-link free-flight | rise | target-axis alignment | detach→apex rotation | first contact | bilateral | stable |
|---|---:|---:|---:|---:|---:|---:|---|
| `goal_stable_recovered_repeat1` | 0.097 s | 56.2 mm | 0.960 | 2.50° | 0.717 s | 0.785 s | yes |
| `goal_stable_recovered_repeat2_video` | 0.097 s | 56.2 mm | 0.960 | 2.50° | 0.717 s | 0.785 s | yes |
| `goal_stable_recovered_probe_j_repeat3` | 0.078 s | 52.9 mm | 0.986 | 2.11° | 0.698 s | 0.786 s | yes |
| `stable_recovered_probe_j_handoff_20260818` | 0.078 s | 52.9 mm | 0.986 | 2.11° | 0.698 s | 0.786 s | yes |

四次都在 `0.620 s` 发生 physical detach，bilateral contact fraction 都是 1.0，
并通过 0.5 s stable-hold 判定。前两次是同一 reference 的确定性重复；第三次故意给 runner
`0.70 / 0.74 / 0.74 s` nominal catch timing，Probe/J 随后把实际 controller 改为：

```text
catch servo start = 0.680 s
catch close       = 0.720 s
catch intercept   = 0.720 s
```

因此 Probe/J 不是只写日志。`probe_j.json` 中：

```text
probe_used_for_control = true
j_used_for_control     = true
selected candidate     = stable_recovered_probe_conditioned
effective payload      = 0.020 ± 0.006 kg
held / slip probability = 1.0 / 0.0
projected width        = 0.035 m
J(stable) / J(long)    = 0.575 / 3.907
```

## 证据边界

这不是 `goal.md` 的 strict 完成，不能写成明显定轴翻滚成功：

- strict 要求 all-link robot-free ≥0.12 s；稳定版是 0.097 s，Probe/J 版是 0.078 s；
- strict 要求完全离手段 hand-relative separation ≥25 mm；连续 free-flight 最大值约 14.4 mm；
- stable 版 apex 在 free-flight 末端，不满足“内部 apex”；
- strict 要求 detach→apex ≥5°；当前是 2.11–2.50°；
- actual simulated q/dq 的有限差分 acceleration 会被接触/drive 数值尖峰污染，不能拿它
  证明真机 command acceleration 合规；command/reference envelope 自身合规，真机仍须空载预览。

另一个 `outputs/goalv3_lagcomp_z0065_throwonly_exact` 确实展示更长的飞行：0.924 s
all-link free-flight、65.3 mm 上升、0.861 alignment、12.63° signed target-axis rotation，
但没有 recatch。它只说明旋转动作有潜力，不能替代稳定接取 evidence。

## 控制调用关系

```text
fixed grasp
  → paired empty/held q, dq, actuator-effort Probe
  → payload/held/slip/detach-uncertainty posterior
  → J ranks stable vs long-flight catch candidates
  → selected timing overwrites nominal controller
  → actual q/dq + FK at physical detach
  → ballistic state propagation
  → 3 bounded IK catch updates
  → G1 close and bilateral hold
```

没有 wrist F/T、GelSight 或质量先验。Probe 不是精确秤：真机上的轻 cube current/effort
residual 很可能接近噪声，因此 posterior 必须保留不确定度；它的作用是 held/slip gate 和
保守 controller selection，而不是假装得到精确质量。

## 相机角色

控制默认是 `proprioceptive`。third-view 和 wrist 都会录制，但不假定每帧能看到 cube：

- third-view：release/flight/catch 的主要真机证据，有观测时可用于低频 correction；
- wrist：grasp/probe 和接取前机会观测；camera-under 后未必持续看见 cube；
- spectator：仅 Isaac 审核和好看的全局视频，永不进入控制。

真机相机仍使用：

```text
global D435 serial = 317222073552
wrist D435 serial  = 233622079809
configs/global_camera_real.json
configs/wrist_camera_real.json
```

第一轮真机不应因偶发漏帧停止 ballistic controller；相机观测只在 timestamp/quality 合格时
做 bounded correction。若还没有把视觉更新接入真机 runner，先录制并用 q/dq 闭环完成最小验证。

## sim 与真机数值不能混用

仿真 gripper drive 是：

```text
held 0.56 rad → open 0.39 rad → close 0.56 rad
```

真机 G1 是：

```text
held 370 → open 520 → close 370, speed 5000
```

禁止把 sim rad 发给 G1。稳定 sim 的 open command / physical detach / close command 是
`0.600 / 0.620 / 0.720 s`；其中 20 ms sim detach delay 略低于真机实测 25–44 ms，
所以这些绝对时刻不是已批准的真机 timeline。

真机先保留 `REAL_ROBOT_TEST_20260817.md` 已验证 baseline：release/close command
`0.636 / 0.720 s`，约 80 ms arm tracking lag。新动作应重新测量本次 G1 detach 和 arm lag，
然后以实测 physical detach 为 `t=0`：当前 sim geometry 的 catch servo start 约为
`detach + 60 ms`，close/intercept 约为 `detach + 100 ms`。真机 scheduling 还必须把约
80 ms arm response 纳入 command 提前量，不能把这两个相对值直接当网络发送延时。

## 真机执行顺序

1. Pull 后先读 `REAL_ROBOT_TEST_20260817.md` 和本文件，确认使用同一只约 35–40 mm cube。
2. 运行 disconnected dry-run 和 0.25×、0.5×、1.0× preview；核对 joint bounds、20 ms
   step、velocity/acceleration 和 controller 当前 linear speed factor。
3. 用同一 probe trajectory 各采一次 empty/held q、dq、effort/current。不要把 sim posterior
   当真机测量；当前真机 current 到 posterior 的自动接线仍需在真机仓完成。
4. 先无 cube 执行 0.25×，再逐级到 1.0×；确认无 C60、无 20 ms overrun，并复测约 80 ms lag。
5. 先让 cube throw-only 落到软垫，确认实际 detach、运动方向、J6/camera housing/cable clearance。
6. 仍以已验证 G1 `370 → 520 → 370` 为唯一真机映射，先跑 stable-recovered timing；操作者
   保持急停可达，做 2–3 次即可。
7. 保存 commanded/actual q,dq,current/effort、G1 timestamps、两个 D435 视频和人工 catch label。
8. 只有 stable-recovered 真机可靠后，才尝试 12.63° long-flight 分支；它目前没有 recatch。

现有真机侧离线检查入口：

```bash
python scripts/20_closed_loop_dry_run.py --output real_handoff/disconnected_dry_run.json
python scripts/21_preview_handoff.py --speed-scale 0.25 --output real_handoff/preview_025x.json
python scripts/21_preview_handoff.py --speed-scale 0.5  --output real_handoff/preview_05x.json
python scripts/21_preview_handoff.py --speed-scale 1.0  --output real_handoff/preview_1x.json
```

## 关键文件

```text
sim/configs/camera_under_tumble_stable_recovered.json
sim/configs/probe_j_stable_recovered_v1.json
sim/scripts/12_run_stable_recovered.sh
sim/scripts/04_native_release_smoke.py
src/xarm6_toss/flight.py
src/xarm6_toss/motion_limits.py
REAL_ROBOT_TEST_20260817.md
```

当前发布状态只能写：`sim_validated_real_unverified`。最直接的下一步不是继续扩大 sim search，
而是把 stable-recovered 的相对 timing 接进真机脚本，完成空载预览、throw-only 和 2–3 次接取。
