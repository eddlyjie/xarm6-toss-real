# xArm6 four-object pose-conditioned open-loop toss/catch

This repository is the handoff package for the xArm6 + stock UFACTORY G1 demo.
The real robot replays an offline-selected 20 ms J2/J3/J5 arm reference and a
measured G1 event schedule. Cameras record the result; they are not part of the
high-speed control loop. Read [`goal.md`](goal.md) and
[`REAL_ROBOT_TEST_20260817.md`](REAL_ROBOT_TEST_20260817.md) before hardware use.

## Onsite entry for the four-object demo

Start with the complete Chinese runbook:
[`docs/FOUR_OBJECT_REAL_ROBOT_RUNBOOK.md`](docs/FOUR_OBJECT_REAL_ROBOT_RUNBOOK.md),
or keep the compact
[`docs/FOUR_OBJECT_ONSITE_COMMANDS.md`](docs/FOUR_OBJECT_ONSITE_COMMANDS.md)
open beside the robot. The
[`four-object Sim reference`](docs/FOUR_OBJECT_SIM_REFERENCE.md) links every
low/next/high profile to its tracked video and 20 ms timeline. Activate the Python environment prepared on the
real-robot computer, then run the fully offline environment preflight:

```bash
python scripts/28_check_real_robot_environment.py \
  --output real_handoff/onsite_environment.json
```

It checks the installed package versions, local hardware configuration, all
four object profiles, and G1 calibration state without importing the xArm SDK
or attempting a network connection. After it passes, rebuild the handoff report:

```bash
python scripts/27_check_four_object_handoff.py \
  --output real_handoff/four_object_plan_check.json
```

O0 is the first staged real baseline. O1–O3 must each receive measured real G1
held/release/preclose/close positions through
`scripts/29_prepare_object_commissioning.py` before any G1 or object trial.
The lower-level `scripts/26_calibrate_open_loop_profile.py` remains available.
Rerun `scripts/28_check_real_robot_environment.py` after writing a bundle; the
object then changes from `WAIT` to `PASS ... ready via commissioning bundle`.

After each full recatch run, record the observed result directly from the
runner output so the object, profile, requested angle, and four G1 positions do
not have to be copied by hand:

```bash
python scripts/31_record_real_trials.py record-from-runner \
  --runner-summary <RUN_OUTPUT>/summary.json --trial-id o1_low_01 \
  --angle-summary <ANGLE_OUTPUT>/summary.json --rotation-axis forward_tumble \
  --detached yes --caught yes --hold-s <SECONDS> --video <VIDEO> \
  --output real_results/O1/o1_low_01.trial.json --write
```

The recorder is offline and never connects to the robot. It reads the signed
image-plane rotation from `scripts/25_measure_cube_rotation.py`; use the
`--measured-angle-deg` fallback only when a valid marker summary is unavailable.
The complete command sequence is in the onsite command card.

Once one cuboid's low profile has been caught successfully, reuse that same
object's measured G1 positions to generate its staged next/high profiles:

```bash
python scripts/32_prepare_pose_ladder.py \
  --commissioning-bundle real_handoff/cuboid30/low/<LABEL>/commissioning_bundle.json \
  --write
```

This creates separate plan-only, empty-G1, throw-only, and object paths for
both higher poses. It does not connect to the robot; each generated pose still
has to pass the complete onsite execution ladder.

## Current Sim result matrix

All listed runs use the same xArm6/G1 model, fixed J1/J4/J6, and dynamic
J2/J3/J5. Each result contains continuous finger-free flight, target-axis
rotation, bilateral recatch, and a stable hold.

| Object | Size / mass | Low | Continuous target 5.5° | High | Real status |
|---|---|---:|---:|---:|---|
| `cube38` | 38×38×38 mm / 8 g | 4.59° | 6.48° medium | 7.87° | O0 G1 positions available; staged real verification pending |
| `cuboid30` | 44.5×46×30 mm / 20 g | 2.96° | 5.71° | 6.57° | arm ready; G1 positions must be measured |
| `cuboid33` | 50.5×51×33.5 mm / 26.6 g | 4.61° | 5.62° | 6.45° | arm ready; G1 positions must be measured |
| `cuboid38` | 57.5×58×38 mm / 37 g | 4.40° | 5.58° | 6.85° | arm ready; G1 positions must be measured |

The O1–O3 high runs also pass the Sim `obvious_toss_success` condition. These
successful transfer results currently use fixed-reference replay and are
labelled `M0_fixed_replay_transfer_candidate` in the profiles. They are useful
demo candidates and a baseline, but they do not yet prove the learned
object-conditioned M3 method.

## Handoff layout

```text
configs/objects/                         measured geometry, mass, inertia, grip axis
configs/open_loop_flip/<object>/<pose>  profile and evidence
real_handoff/<object>/<pose>/timeline.json
real_handoff/<object>/<pose>/plan.json
real_handoff/<object>/<pose>/g1_schedule*.json
outputs/<run>/summary.json               Sim metrics
outputs/<run>/spectator.mp4              normal-speed Sim video
```

The three new cuboids intentionally ship with
`g1_schedule.template.json`: arm timelines are validated, while unknown real G1
positions remain `null`. Their profiles allow `empty_arm` only and refuse G1 or
object execution until onsite calibration is filled in.

Plan-only commands never import the xArm SDK or connect to the robot:

```bash
python scripts/24_run_cube_open_loop_demo.py --angle-deg 5
python scripts/24_run_cube_open_loop_demo.py --angle-deg 6.5
python scripts/24_run_cube_open_loop_demo.py --angle-deg 8

python scripts/24_run_cube_open_loop_demo.py \
  --profile configs/open_loop_flip/cuboid30/high_6p5deg.json
python scripts/24_run_cube_open_loop_demo.py \
  --profile configs/open_loop_flip/cuboid33/high_6p5deg.json
python scripts/24_run_cube_open_loop_demo.py \
  --profile configs/open_loop_flip/cuboid38/high_6p5deg.json
```

The new cuboid arm references can be previewed without operating G1:

```bash
python scripts/24_run_cube_open_loop_demo.py \
  --profile configs/open_loop_flip/cuboid30/low_3deg.json \
  --speed-scale 0.25 --execute-empty-arm
```

Repeat at 0.5× and 1.0× only after the previous stage passes. Before enabling
G1 for O1/O2/O3, measure the real held, release, preclose, and close positions
at the marked grasp depth; update both low/high schedules and profiles; then
run the relevant tests and the empty-G1 stage. Sim drive radians must never be
copied into the real G1 integer position field.

Selected videos are under `docs/media/four_object_open_loop/`; the authoritative
metrics remain in each source output directory.

## Historical single-cube development notes

## Current real-demo deployment entry

The current hardware target is one marked yellow cube, approximately 38 mm and
8 g, on xArm6 with the stock UFACTORY G1. The real controller is intentionally
open loop: simulation/offline code selects a complete angle-conditioned arm and
G1 profile; the real runner replays it and records robot state. Cameras are for
video and offline angle measurement only.

Read [`goal.md`](goal.md) and `REAL_ROBOT_TEST_20260817.md` before running any
hardware command. The measured starting parameters are:

```text
arm control period                 20 ms servo_j
arm tracking lag                  about 80 ms
G1 held / partial-open / close    370 / 520 / 370
G1 speed                          5000
physical detach delay             25--44 ms after open command
G1 full target travel             about 103 ms
linear speed limit factor         1.6
real micro-toss release / close   0.636 / 0.720 s
```

Available exact-angle profiles:

| Angle | Status | Permitted stage |
|---:|---|---|
| 0° | real micro-toss baseline | empty previews, throw-only, cube recatch |
| 5° | Sim stable-recatch reference | empty previews and soft-mat throw-only |
| 8° | Sim open-loop J2/J3/J5 stable recatch | staged empty, throw-only, then cube |
| 10° | Sim 9.84° stable-recatch reference | empty previews and soft-mat throw-only |
| 20°+ | planned | no trajectory yet |

The 8° profile is the first converted real-reference candidate. It uses 92
commands at 20 ms, keeps J1/J4/J6 fixed, passes the measured joint envelope,
and achieved 0.266 s free flight, 7.87° total rotation, and stable bilateral
recatch with the measured 38 mm / 8 g cube in Sim. Real execution is pending.

The 5° and 10° Sim successes used detach-relative catch correction. Their
references are useful for commissioning, but the new open-loop runner will not
allow a cube-recatch command until an open-loop Sim replay has passed. It never
silently substitutes the nearest available angle.

Plan-only commands do not import the xArm SDK or connect to the robot:

```bash
python scripts/24_run_cube_open_loop_demo.py --angle-deg 0
python scripts/24_run_cube_open_loop_demo.py --angle-deg 5
python scripts/24_run_cube_open_loop_demo.py --angle-deg 8
python scripts/24_run_cube_open_loop_demo.py --angle-deg 10

python scripts/24_run_cube_open_loop_demo.py --angle-deg 0 \
  --output-plan real_handoff/open_loop_cube/baseline_0deg_plan.json
```

On the real computer, use this commissioning order. Every command below still
asks the operator to confirm a clear workspace and accessible e-stop before the
timeline begins.

```bash
# Preserved real baseline: arm only, then G1 without an object.
python scripts/24_run_cube_open_loop_demo.py --angle-deg 0 \
  --speed-scale 0.25 --execute-empty-arm
python scripts/24_run_cube_open_loop_demo.py --angle-deg 0 \
  --speed-scale 0.5 --execute-empty-arm
python scripts/24_run_cube_open_loop_demo.py --angle-deg 0 \
  --speed-scale 1.0 --execute-empty-arm
python scripts/24_run_cube_open_loop_demo.py --angle-deg 0 \
  --speed-scale 1.0 --execute-empty-g1

# Object commands only after the empty ladder and a soft mat are ready.
python scripts/24_run_cube_open_loop_demo.py --angle-deg 0 --execute-throw-only
python scripts/24_run_cube_open_loop_demo.py --angle-deg 0 --execute-cube
```

After the 0° baseline is reconfirmed, commission the new 8° reference in the
same order. The cube command is the last step, not the first:

```bash
python scripts/24_run_cube_open_loop_demo.py --angle-deg 8 --speed-scale 0.25 --execute-empty-arm
python scripts/24_run_cube_open_loop_demo.py --angle-deg 8 --speed-scale 0.5 --execute-empty-arm
python scripts/24_run_cube_open_loop_demo.py --angle-deg 8 --speed-scale 1.0 --execute-empty-arm
python scripts/24_run_cube_open_loop_demo.py --angle-deg 8 --execute-empty-g1
python scripts/24_run_cube_open_loop_demo.py --angle-deg 8 --execute-throw-only
python scripts/24_run_cube_open_loop_demo.py --angle-deg 8 --execute-cube
```

Real logs are written under
`toss_project_sim_handoff/toss_project/real_cube_demo/outputs/open_loop_cube_demo/`.
The runner records commanded/actual q, dq, effort/current, fixed G1 event times,
controller status, and setup values. Use an external side-view phone at 120/240
fps with an asymmetric marker on the cube for the actual rotation measurement.

Generate and print a 25--30 mm ArUco marker for one cube face:

```bash
python scripts/25_measure_cube_rotation.py generate-marker \
  --output real_handoff/open_loop_cube/aruco_4x4_id0.png
```

Record from the side with the camera optical axis aligned to the intended
forward-tumble axis. After the run, measure only the free-flight interval:

```bash
python scripts/25_measure_cube_rotation.py measure \
  --video /path/to/side_view.mp4 \
  --start-s 1.20 --end-s 1.55 \
  --output-dir /path/to/angle_measurement
```

The command writes `angle_measurements.csv`, `summary.json`, and an annotated
video. The value is 2-D image-plane rotation, so keep the side camera fixed.

## 2026-08-21 stock-G1 10-degree stable-regrasp checkpoint

The frozen same-trial result uses the stock G1 with no release insert:
0.332 s strict all-link free flight, 9.840 degrees signed forward tumble,
0.994 target-axis alignment, bilateral recapture, and stable hold.

Evidence committed to Git:

```text
docs/media/stock_g1_10deg_v86/global.mp4
docs/media/stock_g1_10deg_v86/third_view.mp4
docs/media/stock_g1_10deg_v86/wrist.mp4
docs/media/stock_g1_10deg_v86/summary.json
```

Isaac reproduction:

```bash
bash sim/scripts/15_run_stock_g1_10deg_regrasp.sh
```

Dedicated real-hardware plan/runner:

```bash
python scripts/23_run_stock_g1_10deg_regrasp.py
```

The command above is plan-only. It loads the 10-degree dynamic-regrasp
timeline and controller, not the throw-only profile. Hardware stages use the
same entrypoint with `--execute-empty-arm --speed-scale 0.25`, then 0.5 and
1.0, followed by `--execute-empty-g1 --speed-scale 1.0`. A cube run requires
both a passing paired-Probe summary and a calibrated detach result:

```bash
python scripts/23_run_stock_g1_10deg_regrasp.py --execute-cube \
  --probe-comparison <probe-summary.json> \
  --detach-result <detach-result.json>
```

Status remains `sim_validated_real_unverified`. The frozen sim and real
candidate assume the wrist-camera hardware, mount, and cable are removed.

## 2026-08-21 standard-G1 11.5-degree throw-only checkpoint

A real soft-mat trial entry now freezes native sim `v62` without any
roller, flipper, or release insert. The nominal sim result has 0.394 s strict
free flight, 11.530 degrees forward rotation, and 0.974 axis alignment. The
reference peaks at 1.7361 rad/s and 12.9340 rad/s2, inside the received 1x real
command envelope.

This is `sim_validated_real_unverified` throw-only: it never commands a catch
and does not predict 11.5 degrees on hardware. It does not replace the v47
stable-regrasp candidate or the successful 0.636/0.720 s real micro-toss.

```bash
python scripts/22_run_j5_dynamic_regrasp.py \
  --timeline real_handoff/standard_g1_throwonly_11p5deg_timeline.json \
  --controller real_handoff/standard_g1_throwonly_11p5deg_controller.json
```

See `docs/REAL_THROWONLY_11P5_HANDOFF_20260821.md` for the empty-arm ladder and
the one permitted soft-mat cube command.

## 2026-08-20 deployable 5° dynamic regrasp

当前推荐真机接手的是 `v47`：paired Probe posterior 经过 J 选择
`dynamic_5deg_g1_observer`，controller 只用 actual q/dq、FK、G1 actual position 与
camera-calibrated detach-position threshold，不读取 cube physics truth，也不需要 runtime camera
控制。真机包没有已验证的 G1 motor-current API，因此不再把 motor current 写成真机触发条件。
冻结 v47 配置的五次独立 native process repeat 均达到 0.173 s strict all-link free flight、
15.48 mm separation、5.055° 前翻和稳定双侧接取；抓前到稳定接后的 hand-object orientation
change 为 8.816°，最后 0.5 s 相对姿态波动不超过 0.008°。五次只改变 record-only camera
seed，camera 不进入控制，因此这是冻结 nominal repeatability，不冒充物理随机化 robustness。
真机入口是 `scripts/22_run_j5_dynamic_regrasp.py`；默认只打印 plan，不连接机械臂。

先看 Git 内慢放与 third-view：

```text
docs/media/j5_forward_rotation/dynamic_regrasp_5p05deg_proprio_probe_j/spectator_slow_0p5x.mp4
docs/media/j5_forward_rotation/dynamic_regrasp_5p05deg_proprio_probe_j/third_view_slow_0p5x.mp4
```

复现和真机 detach-relative timing 见
[`docs/DYNAMIC_REGRASP_5DEG_HANDOFF_20260820.md`](docs/DYNAMIC_REGRASP_5DEG_HANDOFF_20260820.md)。
真机仍未验证，因此状态是 `sim_validated_real_unverified`。

## 2026-08-20 J5 forward-rotation checkpoint

当前新分支固定 J4=165°、J6=−1.5°，只用 J2/J3/J5 做前翻；真机主 policy 不需要 camera。
已有两个诚实分开的结果：

- throw-only：0.163 s strict free flight、109.3 mm separation、internal apex、4.747° 前翻；
- Probe/J regrasp：Probe 与 J 实际进入控制，0.049 s 离手、1.416° 前翻、双侧稳定抓回。

先看 Git 内的视频：

```text
docs/media/j5_forward_rotation/throwonly_4p75deg/spectator.mp4
docs/media/j5_forward_rotation/probe_j_regrasp_1p42deg/spectator.mp4
```

复现最小闭环：

```bash
bash sim/scripts/15_run_j5_probe_j_regrasp.sh
```

完整 branch 职责、真机 20 ms timeline、G1 370→520→370 timing 和交接顺序见
[`docs/J5_FORWARD_ROTATION_HANDOFF_20260820.md`](docs/J5_FORWARD_ROTATION_HANDOFF_20260820.md)。
4.747° throw-only 不能冒充“旋转并抓回”；真机成功前仍是 `sim_validated_real_unverified`。

## 2026-08-18 strict v18 收束

明显腾空的 `v18` 已达到 123.2 mm 上升、0.914 轴对齐和 4.303° detach→apex 翻滚，
但尚未 recatch，也未达到 5°/12° rotation gate。边界与复现命令见
[`docs/STRICT_V18_CHECKPOINT_20260818.md`](docs/STRICT_V18_CHECKPOINT_20260818.md)。

## 2026-08-18 stable-recovered handoff

当前建议真机先接手的是 `stable_recovered` 小旋转稳定接取版，而不是下方旧的
`visible-spin` 声明。两次相同 reference 重复都完成了 0.097 s 全机器人链路无接触、
56.2 mm 上升、目标翻滚轴 alignment 0.960、detach→apex 2.50° 旋转和双指稳定接取；
第三次由 paired Probe posterior + J 实际把 nominal catch timing 改为
`0.68 / 0.72 / 0.72 s` 后也稳定接住。

一条命令复现：

```bash
bash sim/scripts/12_run_stable_recovered.sh
```

先看已有视频：

```text
outputs/stable_recovered_probe_j_handoff_20260818/spectator.mp4
outputs/stable_recovered_probe_j_handoff_20260818/spectator_third_view.mp4
outputs/stable_recovered_probe_j_handoff_20260818/spectator_wrist.mp4
```

完整指标、Probe/J 调用关系、sim→real timing 换算和真机执行顺序见
[`docs/STABLE_RECOVERED_HANDOFF_20260818.md`](docs/STABLE_RECOVERED_HANDOFF_20260818.md)。
这是一版实用 checkpoint，不是 strict goal 完成：稳定版连续 free-flight 最大分离只有
14.4 mm、apex 不是严格内部点、detach→apex 旋转只有 2.1–2.5°；另有 12.63°
长飞行 throw-only，但尚未接回。真机状态仍是 `sim_validated_real_unverified`。

下面 2026-08-17 的 19.45° 结果已经由更完整的 all-link contact 检查判定为可能混入
接取碰撞，只保留作历史记录，不能作为定轴飞行旋转结论。

## 2026-08-17 visible-spin update

The nominal Isaac native result now shows clear free flight, cube rotation,
and a stable bilateral recatch: 0.245 s continuous flight, 49.4 mm
hand-relative separation, 19.45 deg net rotation, and descending recatch.
Control uses actual q/dq at release, ballistic propagation, and a bounded
13.5 mm lateral residual. Third-view, wrist, and spectator cameras are
record-only; `camera_control_enabled=false`.

Start with the slow-motion evidence:

```text
outputs/visible_spin_natural_proprio_v1_marked/spectator_slow_0p25x.mp4
```

Reproduce with `bash sim/scripts/11_run_visible_spin.sh`. Full metrics,
all three videos, negative J5/J6 evidence, and the real-robot execution order
are documented in `docs/VISIBLE_SPIN_HANDOFF_20260817.md`. The camera/Probe
section below is the preserved previous candidate and is superseded by this update.

Scope: this is a nominal fixed 38 mm, 35 g sim cube success. It is not yet
real-robot validated and does not establish full 20--50 g or 25--44 ms
detach-delay robustness.

当前可移交候选是反腕、朝外工作区的 xArm6 + G1 固定 cube 抛接。反腕是有意选择：换取更大的
抛接空间；third_view 负责 release/flight 的可见段，wrist 只做 grasp/probe 和机会观测。Isaac native learned
controller 在固定约 38 mm、35 g cube 上完成 3/3 stable bilateral catch；三次都实际运行了
paired empty/held Probe、Probe-conditioned J、policy cameras、bounded learned residual 和 90 ms arm lag。

关键结果：

- Probe/J stable candidate：上升 56.1–58.6 mm、连续离手 95/95/145 ms、3/3 catch；
- Probe 从 paired actuator effort 得到约 35 g effective payload、held=1、slip=0；J 三次均选择
  `stable_third_view_learned`；
- clear physics candidate：连续离手 245 ms，过 apex 后下降再接并稳定保持；
- rendered-camera clear diagnostic：离手 160 ms，但 bilateral fraction 0.667，不算稳定成功；
- sim detach delay 35 ms，位于真机反馈的 25–44 ms 范围；
- 每次离手后 1 个 camera-updated command、10 个 learned updates，8 mm bounded residual；
- 90 ms arm lag 已通过把 release/catch timeline 同步后移补偿；
- wrist 本次没有看到 cube，belief 由 release prior、ballistic propagation 和 third-view
  observation 维持；spectator 从不进入控制。
- 1× reference 超过当前真机 `0.45 / 1.5` cap，而且 sim actual acceleration 峰值约
  `90.2 rad/s²`；只有 0.25× 空载 preview 在当前 cap 内，full-speed 不可直接下发。

先看结果：

```text
outputs/final_probe_j_seed_20260861_v3/spectator_slow_0p25x.mp4
outputs/final_probe_j_seed_20260861_v3/spectator_third_view.mp4
outputs/final_probe_j_seed_20260861_v3/spectator_wrist.mp4
outputs/final_clear_camera_seed_20260841/spectator_zoom_slow_0p4x.mp4
real_handoff/real_constraints_report.json
real_handoff/sim_probe_j_evidence.json
```

以下命令必须在已安装 `numpy` 的项目/xArm Python 环境中运行；系统裸 `python3` 不一定可用。

真机电脑 pull 后先运行完全不连接机器人的检查：

```bash
python scripts/20_closed_loop_dry_run.py \
  --output real_handoff/disconnected_dry_run.json

python scripts/21_preview_handoff.py --speed-scale 0.25 \
  --output real_handoff/preview_025x.json
python scripts/21_preview_handoff.py --speed-scale 0.5 \
  --output real_handoff/preview_05x.json
python scripts/21_preview_handoff.py --speed-scale 1.0 \
  --output real_handoff/preview_1x.json
```

真机 timeline、真实 G1 数值、相机角色、执行顺序和已知风险见
[docs/REAL_ROBOT_HANDOFF.md](docs/REAL_ROBOT_HANDOFF.md)。本候选只验证固定的这一只 cube；
真机前必须做 paired empty/held current Probe、重新测 arm/G1 latency，并先执行 0.25× 空载预览。
sim 已消费同结构的 paired actuator-effort Probe 并让 posterior 改变 J，但尚未消费真机 current；
strict clear-flight stable success 仍是 physics observation。不能把 nominal timing 当成跨物体参数，
也不能把 1× 当成已批准真机动作。
## Pose-conditioned warm-start update (2026-08-25)

The repository now contains continuous target-pose generation in addition to
the fixed low/high endpoint profiles. Stable same-object Sim trials define a
piecewise response from desired rotation to action strength; the selected
action interpolates the validated 20 ms J2/J3/J5 command references.

| Object | Requested | Measured | Free flight | Stable recatch | Handoff |
|---|---:|---:|---:|---|---|
| `cuboid30` / 20 g | 5.5 deg | 5.71 deg | 0.193 s | yes | `real_handoff/cuboid30/pose_conditioned_5p5deg/` |
| `cuboid33` / 26.6 g | 5.5 deg | 5.62 deg | 0.190 s | yes | `real_handoff/cuboid33/pose_conditioned_5p5deg/` |
| `cuboid38` / 37 g | 5.5 deg | 5.58 deg | 0.189 s | yes | `real_handoff/cuboid38/pose_conditioned_5p5deg/` |

O2's first intermediate action caught successfully at 4.78 deg. Adding that
stable trial to the response model produced the corrected 5.62 deg result.
This is seen-object pose conditioning, not formal unseen-object generalization.
Both profiles remain `empty_arm` only until real G1 positions are calibrated.
