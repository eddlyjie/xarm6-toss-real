# xArm6 四物体现场命令卡

这张命令卡按真实执行顺序排列。`[OFFLINE]` 只读本地文件；`[ROBOT]` 会连接并运动 xArm6，必须由现场
操作者确认软垫、急停和净空后手动运行。真机电脑若已激活 Python 环境，统一使用 `python`。

四物体各档位的 Sim 动作、视频、profile 与 timeline 对照见
[`FOUR_OBJECT_SIM_REFERENCE.md`](FOUR_OBJECT_SIM_REFERENCE.md)。

## 0. 开机后先做离线预检

```bash
# [OFFLINE] 检查 Python/SDK、hardware config、四物体 profile 和 G1 标定状态
python scripts/28_check_real_robot_environment.py \
  --output real_handoff/onsite_environment.json

# [OFFLINE] 重新生成四物体 handoff 报告
python scripts/27_check_four_object_handoff.py \
  --output real_handoff/four_object_plan_check.json
```

预检必须显示 Python、`xarm-python-sdk`、NumPy、SciPy、OpenCV ArUco、hardware config 和四物体文件均为 `PASS`。O0 G1
应为 `PASS`；O1–O3 在标定前显示 `WAIT` 属于正常状态。预检本身不会导入 SDK或连接机器人。

## 1. 当天先恢复 O0 保底结果

```bash
# [OFFLINE] 检查计划
python scripts/24_run_cube_open_loop_demo.py \
  --profile configs/open_loop_flip/cube38/low_5deg.json

# [ROBOT] 空臂速度梯度
python scripts/24_run_cube_open_loop_demo.py \
  --profile configs/open_loop_flip/cube38/low_5deg.json \
  --speed-scale 0.25 --execute-empty-arm
python scripts/24_run_cube_open_loop_demo.py \
  --profile configs/open_loop_flip/cube38/low_5deg.json \
  --speed-scale 0.5 --execute-empty-arm
python scripts/24_run_cube_open_loop_demo.py \
  --profile configs/open_loop_flip/cube38/low_5deg.json \
  --speed-scale 1.0 --execute-empty-arm

# [ROBOT] 空 G1、软垫抛出、最后才完整接取
python scripts/24_run_cube_open_loop_demo.py \
  --profile configs/open_loop_flip/cube38/low_5deg.json --execute-empty-g1
python scripts/24_run_cube_open_loop_demo.py \
  --profile configs/open_loop_flip/cube38/low_5deg.json --execute-throw-only
python scripts/24_run_cube_open_loop_demo.py \
  --profile configs/open_loop_flip/cube38/low_5deg.json --execute-cube
```

O0 low 接住后立刻保存日志和保底视频。当天不先追大角度。

## 2. O1–O3 逐物体标定

| Object | Template | Calibrated output stem |
|---|---|---|
| O1 | `configs/open_loop_flip/cuboid30/low_3deg.json` | `cuboid30/low` |
| O2 | `configs/open_loop_flip/cuboid33/low_5deg.json` | `cuboid33/low` |
| O3 | `configs/open_loop_flip/cuboid38/low_4p5deg.json` | `cuboid38/low` |

对当前物体实测 `<HELD> <RELEASE> <PRECLOSE> <CLOSE>` 后，用一个命令生成 `empty_g1`、`throw_only`、
`object` 三阶段文件和执行顺序。下面以 O1 为例；`--label` 使用当天日期或本次实验名：

```bash
# [OFFLINE] Preview a single candidate without reading hardware config.
python scripts/30_measure_g1_position.py \
  --object O1 --purpose held --position <CANDIDATE>

# [ROBOT] Move only G1 after the dry-run; type MOVE G1 at the prompt.
python scripts/30_measure_g1_position.py \
  --object O1 --purpose held --position <CANDIDATE> \
  --output real_handoff/cuboid30/low/20260826/held_measurement.json \
  --execute

# [OFFLINE]
python scripts/29_prepare_object_commissioning.py \
  --object O1 --label 20260826 \
  --held-position <HELD> --release-position <RELEASE> \
  --preclose-position <PRECLOSE> --close-position <CLOSE> \
  --write
```

O2、O3 分别使用 `--object O2`、`--object O3`。工具默认只预览；`--write` 才创建七个文件，并拒绝覆盖
同名实验。生成的 `commissioning_bundle.json` 已包含从 plan-only 到完整接取的可复制命令。O0 的历史整数
位置只能作为 O0 起点，不能复制给 O1–O3。

## 3. 每个新物体的执行梯度

下面依次使用刚生成的 `<EMPTY_G1_PROFILE>`、`<THROW_ONLY_PROFILE>` 和 `<OBJECT_PROFILE>`：

```bash
# [OFFLINE]
python scripts/24_run_cube_open_loop_demo.py --profile <EMPTY_G1_PROFILE>

# [ROBOT] 空臂 0.25x → 0.5x → 1.0x
python scripts/24_run_cube_open_loop_demo.py --profile <EMPTY_G1_PROFILE> \
  --speed-scale 0.25 --execute-empty-arm
python scripts/24_run_cube_open_loop_demo.py --profile <EMPTY_G1_PROFILE> \
  --speed-scale 0.5 --execute-empty-arm
python scripts/24_run_cube_open_loop_demo.py --profile <EMPTY_G1_PROFILE> \
  --speed-scale 1.0 --execute-empty-arm

# [ROBOT] G1 → 软垫抛出 → 完整接取
python scripts/24_run_cube_open_loop_demo.py --profile <EMPTY_G1_PROFILE> \
  --execute-empty-g1
python scripts/24_run_cube_open_loop_demo.py --profile <THROW_ONLY_PROFILE> \
  --execute-throw-only
python scripts/24_run_cube_open_loop_demo.py --profile <OBJECT_PROFILE> \
  --execute-object
```

严格按 O1 → O2 → O3 推进。每完成一个物体就保存成功 profile、G1整数、日志和视频，不等到最后统一整理。

## 4. 四物体成功后再做 pose 梯度

每个 O1–O3 的 low profile 成功后，用该物体已经实测的 G1位置离线生成 next/high 两档。以 O1 为例：

```bash
# [OFFLINE] Creates 13 versioned profile/schedule/bundle files; never connects to the robot.
python scripts/32_prepare_pose_ladder.py \
  --commissioning-bundle real_handoff/cuboid30/low/<LABEL>/commissioning_bundle.json \
  --write
```

生成的 `pose_ladder_bundle.json` 为 next/high 分别列出完整执行命令。每一档仍然必须从 plan-only 和 0.25×空臂
重新开始，不能因为 low 已成功就跳级。

| Object | Low | Next pose | High |
|---|---|---|---|
| O0 | `cube38/low_5deg.json` | `cube38/medium_6p5deg.json` | `cube38/high_8deg.json` |
| O1 | `cuboid30/low_3deg.json` | `cuboid30/pose_conditioned_5p5deg.json` | `cuboid30/high_6p5deg.json` |
| O2 | `cuboid33/low_5deg.json` | `cuboid33/pose_conditioned_5p5deg.json` | `cuboid33/high_6p5deg.json` |
| O3 | `cuboid38/low_4p5deg.json` | `cuboid38/pose_conditioned_5p5deg.json` | `cuboid38/high_6p5deg.json` |

每条新 pose 都重新走 plan-only、空臂、空 G1、throw-only、object 梯度。报告角度取侧视视频实测值；profile
名称中的目标角只表示 policy 输入。四物体各一个完整成功之前，不投入时间追 20°以上。

## 5. 每次运行马上记录

```text
object / profile / held / release / preclose / close
是否完全离手 / 实测旋转角 / 是否接住 / 保持时间
normal-speed video / slow-motion video / output summary path
```

```bash
# [OFFLINE] Measure the marked object over the free-flight interval first.
python scripts/25_measure_cube_rotation.py measure \
  --video <VIDEO> --start-s <DETACH_TIME> --end-s <RECATCH_TIME> \
  --output-dir <ANGLE_OUTPUT>

# [OFFLINE] Preferred: read object/profile/target/G1 positions from the runner output.
# Preview by omitting --write.
python scripts/31_record_real_trials.py record-from-runner \
  --runner-summary <SUMMARY_JSON> --trial-id o1_low_01 \
  --angle-summary <ANGLE_OUTPUT>/summary.json \
  --rotation-axis forward_tumble \
  --detached yes --caught yes --hold-s <SECONDS> \
  --video <VIDEO> \
  --output real_results/O1/o1_low_01.trial.json --write

python scripts/31_record_real_trials.py summarize \
  --input-root real_results --output real_results/summary.json
```

`record-from-runner` 会自动读取本次执行的 object、profile、目标角、四个 G1整数和视频测角结果，并在 runner
报错时禁止把该次标成完整成功。只有历史执行没有 `summary.json` 或 marker测角不可用时，才使用兼容的手工字段。

出现 tracking error、异常振动、G1/cable 干涉或物体飞出软垫范围时，停止当前 profile，保留日志，回到离线
调整。不要在现场临时升级 SDK，也不要跳过低速空臂阶段。
