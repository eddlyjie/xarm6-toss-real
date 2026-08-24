# xArm6 四物体真机当天操作手册

本手册只覆盖当前 camera-free open-loop demo。相机负责录像和离线测角。所有未带 `--execute-*` 的命令
都是离线检查；带执行参数的命令会连接真机，只能由现场操作者明确运行。

## 1. 当天目标与顺序

1. 恢复 O0 已有 micro-toss baseline，先保存一条当天保底成功视频；
2. 标定 O1、O2、O3 各自的 G1 held/release/preclose/close整数位置；
3. 按 O1 → O2 → O3 各完成一个低角度完整抛接；
4. 四物体都有完整成功后，再增加第二档 pose；
5. 每个最终 profile重复至少 5 次并保存侧视正常速度与慢放视频。

## 2. 开机前离线检查

先激活真机电脑自己的 Python环境，并确认 `numpy` 与兼容的 `xarm-python-sdk` 可导入。下面保留的
`../.venv/bin/python` 是 devserver路径；若真机电脑没有这个 sibling environment，将所有命令前缀替换为当前
环境的 `python`。不要在实验当天临时升级 SDK。

随后在 `xarm_6` 仓库内运行：

```bash
../.venv/bin/python scripts/28_check_real_robot_environment.py \
  --output real_handoff/onsite_environment.json

../.venv/bin/python scripts/27_check_four_object_handoff.py \
  --output real_handoff/four_object_plan_check.json
```

该命令检查 O0–O3 的 baseline 和 next-pose 共八条 timeline、J2/J3/J5合同、机械包络及 G1 标定状态，
不会导入 xArm SDK，也不会连接机器人。预期 O0 的 G1 标定完整，O1–O3 显示待标定。

默认 hardware config 位于：

```text
toss_project_sim_handoff/toss_project/real_cube_demo/configs/hardware.json
```

若真机电脑使用不同配置，所有 `--execute-*` 命令可显式增加 `--hardware-config <PATH>`。

O0 baseline的单独 plan-only：

```bash
../.venv/bin/python scripts/24_run_cube_open_loop_demo.py \
  --profile configs/open_loop_flip/cube38/low_5deg.json \
  --output-plan real_handoff/cube38/low/onsite_plan.json
```

## 3. O1–O3 的 G1 现场标定

每个物体先测四个整数：

- `held_position`：固定标记深度下可靠夹持、又不过分挤压；
- `release_position`：能可靠完全离手；
- `preclose_position`：位于 held 与 release之间，用于提前收口；
- `close_position`：最终接取保持位置，通常接近 held，但以实测为准。

不要把 O0 的 `370/520/441/370` 复制到新物体，也不要使用 Sim drive radians。

以 O1 low 为例，先把 `<...>` 换成现场实测整数：

```bash
# Preview one candidate. This does not read hardware config.
../.venv/bin/python scripts/30_measure_g1_position.py \
  --object O1 --purpose held --position <CANDIDATE>

# Move G1 only; type MOVE G1 at the interactive prompt.
../.venv/bin/python scripts/30_measure_g1_position.py \
  --object O1 --purpose held --position <CANDIDATE> \
  --output real_handoff/cuboid30/low/20260826/held_measurement.json \
  --execute

# Preferred: create empty_g1, throw_only, and object stages together.
../.venv/bin/python scripts/29_prepare_object_commissioning.py \
  --object O1 --label 20260826 \
  --held-position <HELD> --release-position <RELEASE> \
  --preclose-position <PRECLOSE> --close-position <CLOSE> \
  --write

# Low-level fallback: create one stage only.
../.venv/bin/python scripts/26_calibrate_open_loop_profile.py \
  --template-profile configs/open_loop_flip/cuboid30/low_3deg.json \
  --output-profile configs/open_loop_flip/real_calibrated/cuboid30/low_empty_g1.json \
  --output-schedule real_handoff/cuboid30/low/g1_schedule.empty_g1.json \
  --held-position <HELD> \
  --release-position <RELEASE> \
  --preclose-position <PRECLOSE> \
  --close-position <CLOSE> \
  --stage empty_g1
```

工具只生成 profile、schedule和 plan，不连接机器人。`throw_only` 与 `object` 阶段使用同一组实测值、不同的
输出文件名重新生成，避免提前解锁 object motion：

```bash
# 确认 empty_g1 通过后生成 throw-only profile
../.venv/bin/python scripts/26_calibrate_open_loop_profile.py \
  --template-profile configs/open_loop_flip/cuboid30/low_3deg.json \
  --output-profile configs/open_loop_flip/real_calibrated/cuboid30/low_throw_only.json \
  --output-schedule real_handoff/cuboid30/low/g1_schedule.throw_only.json \
  --held-position <HELD> --release-position <RELEASE> \
  --preclose-position <PRECLOSE> --close-position <CLOSE> \
  --stage throw_only

# 确认 soft-mat throw-only 通过后生成完整 object profile
../.venv/bin/python scripts/26_calibrate_open_loop_profile.py \
  --template-profile configs/open_loop_flip/cuboid30/low_3deg.json \
  --output-profile configs/open_loop_flip/real_calibrated/cuboid30/low_object.json \
  --output-schedule real_handoff/cuboid30/low/g1_schedule.object.json \
  --held-position <HELD> --release-position <RELEASE> \
  --preclose-position <PRECLOSE> --close-position <CLOSE> \
  --stage object
```

O2、O3 只替换 template 与输出目录：

```text
O2 template: configs/open_loop_flip/cuboid33/low_5deg.json
O3 template: configs/open_loop_flip/cuboid38/low_4p5deg.json
```

## 4. 每条 profile 的执行梯度

下面用 `<PROFILE>` 表示刚生成的 calibrated profile。前三条只运动空机械臂：

```bash
../.venv/bin/python scripts/24_run_cube_open_loop_demo.py --profile <PROFILE>
../.venv/bin/python scripts/24_run_cube_open_loop_demo.py --profile <PROFILE> \
  --speed-scale 0.25 --execute-empty-arm
../.venv/bin/python scripts/24_run_cube_open_loop_demo.py --profile <PROFILE> \
  --speed-scale 0.5 --execute-empty-arm
../.venv/bin/python scripts/24_run_cube_open_loop_demo.py --profile <PROFILE> \
  --speed-scale 1.0 --execute-empty-arm
```

随后才运行 G1、软垫 throw-only 和完整接取：

```bash
../.venv/bin/python scripts/24_run_cube_open_loop_demo.py --profile <EMPTY_G1_PROFILE> \
  --execute-empty-g1
../.venv/bin/python scripts/24_run_cube_open_loop_demo.py --profile <THROW_ONLY_PROFILE> \
  --execute-throw-only
../.venv/bin/python scripts/24_run_cube_open_loop_demo.py --profile <OBJECT_PROFILE> \
  --execute-object
```

O0 使用 `--execute-cube`；O1–O3 使用 `--execute-object`。runner 在连接前打印 plan，移动到 start pose后要求
操作者确认放置物体和工作区清空。

真实运行的 `signals.csv` 与 `summary.json` 默认保存到：

```text
toss_project_sim_handoff/toss_project/real_cube_demo/outputs/open_loop_object_demo/
```

## 5. 现场固定条件

- 动态关节只有 J2/J3/J5，J1/J4/J6固定；
- 同一物体始终按 marker 朝向和标记深度装入；
- 物体下方和预计飞行方向铺软垫，急停由独立操作者看守；
- 侧视手机优先使用 120 fps 或 240 fps，画面同时包含 G1、物体和静止参考方向；
- 每次记录 profile、物体、实测 G1整数、是否完全离手、是否接住、保持时间和视频文件；
- 若出现 C60、tracking异常、G1/cable碰撞、明显冲击或飞出软垫范围，停止该 profile并回到离线修改。

## 6. 多 pose 推进

每个物体先得到一个完整成功，随后再运行 `next_pose_profile`。顺序是 low → continuous 5.5° → high。
报告使用视频实测角度。20°、30°及更大角度属于 stretch，不应在四物体主结果完成前长期调试。

O1–O3 的 low 成功后，直接复用该物体已经实测的 G1位置生成 next/high 分阶段 profiles：

```bash
../.venv/bin/python scripts/32_prepare_pose_ladder.py \
  --commissioning-bundle real_handoff/cuboid30/low/<LABEL>/commissioning_bundle.json \
  --write
```

生成的 `pose_ladder_bundle.json` 包含两档各自的 plan-only、空臂、空 G1、throw-only 和 recatch 命令；每一档
仍需从 0.25×空臂开始。

## 7. 现场记录表

| Object | Profile | held | release | preclose | close | 完全离手 | 实测角度 | 接住 | 保持≥0.5s | Video |
|---|---|---:|---:|---:|---:|---|---:|---|---|---|
| O0 | baseline | 370 | 520 | 441 | 370 |  |  |  |  |  |
| O1 | low |  |  |  |  |  |  |  |  |  |
| O2 | low |  |  |  |  |  |  |  |  |  |
| O3 | low |  |  |  |  |  |  |  |  |  |

```bash
# Measure the free-flight interval, then save one independent trial record.
../.venv/bin/python scripts/25_measure_cube_rotation.py measure \
  --video <VIDEO> --start-s <DETACH_TIME> --end-s <RECATCH_TIME> \
  --output-dir <ANGLE_OUTPUT>

../.venv/bin/python scripts/31_record_real_trials.py record-from-runner \
  --runner-summary <SUMMARY_JSON> --trial-id o1_low_01 \
  --angle-summary <ANGLE_OUTPUT>/summary.json \
  --rotation-axis forward_tumble \
  --detached yes --caught yes --hold-s <SECONDS> \
  --video <VIDEO> \
  --output real_results/O1/o1_low_01.trial.json --write

../.venv/bin/python scripts/31_record_real_trials.py summarize \
  --input-root real_results --output real_results/summary.json
```
