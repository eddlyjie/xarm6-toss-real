# xArm6 5° dynamic regrasp 真机交接

状态：`sim_validated_real_unverified`。这是当前唯一推荐的新真机候选；此前 1.42° stable 版本保留为 fallback。

## 已经证明的结果

`v46` 和 `v47` 使用同一 J4=165°、J6=−1.5° branch 与同一 1.6 reference，均完成真实
all-link contact loss、前向旋转和同一 G1 双侧重新抓稳：

| trial | policy state | Probe/J | strict flight | signed rotation | stable bilateral |
|---|---|---:|---:|---:|---:|
| v46 | proprioceptive G1 observer | no | 0.173 s | 5.039° | yes |
| v47 | proprioceptive G1 observer | yes | 0.173 s | 5.055° | yes |

2026-08-21 又以冻结 v47 配置启动五个独立 Isaac native process，结果为 5/5 success：每次均为
0.173 s strict flight、15.48 mm hand-relative separation、0.984 axis alignment、5.055° flight-only
forward rotation、bilateral fraction=1.0 和 stable catch。由逐帧 `T_hand_object` 计算，抓前到稳定
接后的 orientation change 为 8.816°，最后 0.5 s 的相对姿态波动最大 0.008°。五次只改变录像相机
dropout seed；`camera_seed_affects_control=false`，因此这是 nominal process repeatability，不是物理
随机化 robustness。结构化结果见
`docs/media/j5_forward_rotation/dynamic_regrasp_5p05deg_proprio_probe_j/frozen_validation_5run.json`。

v47 的 Probe gate 通过，J 选择 `dynamic_5deg_g1_observer`（0.6799），没有选择
`stable_1p4deg_fallback`（0.8450）。G1 observer 在 0.615 s 冻结 actual q/dq + FK release
state，与 evaluation-only physical detach 同刻。controller 没有读取 cube pose、contact truth 或 camera。

视频：

```text
docs/media/j5_forward_rotation/dynamic_regrasp_5p05deg_proprio_probe_j/spectator_slow_0p5x.mp4
docs/media/j5_forward_rotation/dynamic_regrasp_5p05deg_proprio_probe_j/third_view_slow_0p5x.mp4
```

## pull 后先做什么

```bash
git pull
python scripts/21_preview_handoff.py \
  --timeline real_handoff/j5_forward_rotation_timeline.json \
  --speed-scale 0.25 \
  --output real_handoff/j5_dynamic_preview_025x.json
conda run --no-capture-output -n calib python scripts/22_run_j5_dynamic_regrasp.py --speed-scale 0.25
```
最后一条默认是 plan-only：不导入 xArm SDK、不连接机械臂、不发送命令。先确认它打印的 wrist
branch、G1 position、detach-relative offsets 和 reference peak speed 正确。

权威输入只有：

```text
real_handoff/j5_forward_rotation_timeline.json
real_handoff/j5_dynamic_regrasp_controller.json
sim/configs/probe_j_j5_dynamic_regrasp_v2.json
scripts/22_run_j5_dynamic_regrasp.py
src/xarm6_toss/real_dynamic_regrasp.py
toss_project_sim_handoff/toss_project/real_cube_demo/scripts/10_measure_detach.py
```

不要再使用旧 `nominal_timeline.json` 作为这条反腕动作的 reference。

## 真机动作与闭环

1. wrist D435、mount 和 cable 必须全部拆除；若硬件仍在，不能使用这个 camera-removed 候选。
2. 先用硬编码抓点抓 35–40 mm 轻 cube；J4 静态到 165° branch，J6 静态 −1.5°。
3. 空载/held 各跑一次低幅 Probe，计算 held/slip posterior；gate 不过就停止，J 若选 fallback 就按
   fallback timing，不要强行执行 dynamic 候选。
4. arm reference 以 20 ms `servo_j` 发送。J2/J3/J5 动，J1/J4/J6 保持；先 0.25× 空载，再 0.5×
   空载，最后 1× 空载。runner 会读取、设置并回读确认 `linear_spd_limit_factor=1.6`；这是 2026-08-17
   真机 1× baseline 消除 C60 后的实测设置，并会写入每次 `summary.json`。
5. 1× cube trial 在 arm t=0.585 s 异步发送 G1 `370→520`，speed=5000。
6. 先用下方 global-camera fixed-pose calibration 得到物理离手时的 G1 actual position。runtime
   根据真机 G1 首次运动约 22.64 ms 的结果，从 release 后 20 ms 才开始轮询 actual position，
   随后每 5 ms 一次；越过阈值就冻结 actual q/dq，经 FK/Jacobian 与固定
   `xarm_gripper_base_link → cube center` offset 得到 cube release position/velocity，再按重力传播。
   真机使用 `[4.000, 0, 137.436] mm`；sim summary 中 `[4,0,24] mm` 是另一个 hand-local
   placement frame，不能复制到真机 FK。没有 calibration threshold 时使用 35 ms fallback；已有
   threshold 时不会在 35 ms 提前截断，而会等待完整的 25–44 ms 实测窗口，若到上界仍未越过再
   fallback。每次 summary 保存 G1 read count/duration；真机包没有可用的 G1 motor-current signal。
7. 从 observed detach 计时：+5 ms 启动 bounded J1–J3 catch servo；+65 ms 发 G1 preclose≈441；
   +165 ms 发最终 close=370；目标 intercept 为 +185 ms，+215 ms 停止更新。J4–J6 全程锁定。
8. G1 441 只是 sim 0.48 rad 的初始线性映射；先用空夹爪 actual-position log 检查。sim 0.65 rad
   是快速闭合的 actuator proxy，真机最终命令仍是 370，绝不能把 `.48/.65` 当 G1 position 发送。

camera 不进入 runtime controller。global/third-view 只做一次 detach-position 标定和 trial 录像；
wrist camera 在本分支不存在。

## 最短真机试验顺序

```text
A. 0.25× empty preview
B. 0.5× empty preview
C. 1× empty preview + G1 520→441→370 timing
D. fixed-pose cube release + global camera，标定 detach-position threshold
E. 1× cube throw-only，下面放软垫，不闭合
F. 最多 2–3 次 dynamic regrasp
```

对应命令如下；每条带 `--execute-*` 的命令都会先等待人工确认：

```bash
conda run --no-capture-output -n calib python scripts/22_run_j5_dynamic_regrasp.py \
  --execute-empty-arm --speed-scale 0.25
conda run --no-capture-output -n calib python scripts/22_run_j5_dynamic_regrasp.py \
  --execute-empty-arm --speed-scale 0.5
conda run --no-capture-output -n calib python scripts/22_run_j5_dynamic_regrasp.py \
  --execute-empty-g1 --speed-scale 1.0

cd toss_project_sim_handoff/toss_project/real_cube_demo
conda run --no-capture-output -n calib python scripts/07_probe_cube.py --execute --condition empty
conda run --no-capture-output -n calib python scripts/07_probe_cube.py --execute --condition cube
conda run --no-capture-output -n calib python scripts/08_compare_probe.py \
  --empty outputs/probes/<empty_run> --cube outputs/probes/<cube_run>
conda run --no-capture-output -n calib python scripts/10_measure_detach.py --execute
cd ../../..

conda run --no-capture-output -n calib python scripts/22_run_j5_dynamic_regrasp.py \
  --execute-throw-only \
  --detach-result toss_project_sim_handoff/toss_project/real_cube_demo/outputs/detach_trials/<run>/detach_result.json

conda run --no-capture-output -n calib python scripts/22_run_j5_dynamic_regrasp.py \
  --execute-cube \
  --probe-comparison toss_project_sim_handoff/toss_project/real_cube_demo/outputs/probe_comparisons/<run>/summary.json \
  --detach-result toss_project_sim_handoff/toss_project/real_cube_demo/outputs/detach_trials/<run>/detach_result.json
```

E 确认 actual detach、J5 空间和 cube 离手方向后才做 F。目标只是得到 2–3 个成功视频，不需要
提高角度。若第一次 recatch miss，带回完整 log，只修 observed-detach-relative 的 preclose/final-close
offset 或 ballistic intercept，不提高 arm reference。

`--execute-cube` 必须同时拿到通过 gate 的 paired Probe summary 和 detach calibration；否则 runner
拒绝执行。`--execute-throw-only` 不闭合，可用于软垫下先核对离手方向。

## 必须回传的记录

每次保存：

- commanded/actual q、dq、joint effort/current，时间戳至少覆盖 −0.2 至 +0.5 s；
- G1 commanded/actual position、position read duration、三个 event 的 monotonic timestamp；
- observer detach timestamp、冻结的 release q/dq、预测 intercept；
- Probe posterior、J ranking、selected candidate；
- third-view 原始视频与人工标签：detach、明显 pose change、left/right/bilateral contact、stable hold；
- controller error code 与 measured arm tracking delay。
timeline 中途出现 C60/SDK error 时，runner 先 stop，并仍保存已发送 command 对应的 partial
`signals.csv`、`summary.json` 与录像，然后以非零状态退出。

## 仿真复现

GPU/Isaac 环境中：

```bash
bash sim/scripts/15_run_j5_probe_j_regrasp.sh
```

成功标准不是旧的 `tumble_toss_success`（它要求 12° visible tumble），而是同一 trial：

```text
strict_contact_free_flight = true
abs(free_flight_signed_tumble_rotation_deg) >= 5
catch_stable = true
bilateral_contact_fraction >= 0.9
joint_mechanical_limits_pass = true
observation_mode = proprioceptive
sim detach_state_source = g1_release_response
real detach_state_source = calibrated_g1_position (or measured_delay_fallback)
probe_used_for_control = true
j_used_for_control = true
```

Runtime invariant: catch servo continuously takes over from the nominal command
`q/dq`; actual `q/dq` is used for FK, ballistic release state, and position error,
but never replaces the command seed. `first_catch_update` logs both states so the
measured real tracking lag remains visible. The runner timestamps every arm command
separately from the following state read and writes
`execution.arm_tracking.estimated_delay_s`, active-joint RMS values, and fit RMS to
`summary.json`; use this measured value when transferring the next sim timing update.
`execution.release_state` also contains the exact frozen actual
`joint_position_rad` and `joint_velocity_rad_s` used by FK/Jacobian, together with
the resulting cube position/velocity, so the real ballistic prior can be replayed.
Its angular fields are explicitly named `rigid_grasp_*_prior`: they are the hand
angular velocity inferred from arm q/dq, not a measured post-detach cube omega.
In v47 this FK/Jacobian prior is 1.553 rad/s on the forward axis, while the first
contact-free simulated cube state is 0.532 rad/s after G1 contact transfer. Real
cube rotation therefore remains a third-view measurement; never report the hand
prior as observed cube rotation.
Each recorded cube trial writes both `global_color.avi` at the native 60 fps and
`global_color_slow_0p25x.avi` at 15 fps using the same frames. Their paths, frame
counts, fps, and playback durations are stored under
`global_camera.color_videos` in `summary.json`; review the slow file when judging
the small forward pose change.
