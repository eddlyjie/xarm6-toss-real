# xArm6 固定 cube 真机交接

## 已验证的 sim 结果

当前 handoff 对应 `outward_minimal_v1`，不是 Git 历史中的 inward-facing 旧候选。

| 项目 | 结果 |
|---|---:|
| 固定 cube | 38 mm，sim nominal 35 g |
| learned validation | 3/3 bilateral catch |
| arm limits | 0.45 rad/s，1.5 rad/s² |
| arm tracking delay | 90 ms |
| G1 measured sim detach | 35 ms |
| 最大相对离手 | 26.3 mm |
| cube 上升 | 105 mm |
| policy updates | 3 camera + 2 learned |
| hold | bilateral 0.5 s |

完整数值以 `real_handoff/real_constraints_report.json` 为准。最终视频：

```text
outputs/final_handoff_nominal/spectator.mp4
outputs/final_handoff_nominal/spectator_slow_0p4x.mp4
outputs/final_handoff_nominal/spectator_third_view.mp4
outputs/final_handoff_nominal/spectator_wrist.mp4
outputs/final_handoff_nominal/three_view.mp4
```

## Controller 结构

```text
fixed grasp
→ q/dq + calibrated T_hand_object 形成 detach prior
→ gravity-constrained ballistic propagation
→ third-view / wrist 任一有效 timestamped RGB-D center 异步更新
→ 至少 3 个 camera samples 后启用 12 mm bounded learned residual
→ 0.005 rad/20 ms bounded Jacobian pose correction
→ G1 nonblocking close
```

相机不是逐帧状态源。看不见时继续传播 ballistic belief；任一相机重新看见时更新。
spectator 只录像。当前腕部视角在短 flight 中没有看到 cube，这是已记录的真实限制，不得用
sim truth 填补。

## 真机数值

权威配置是 `real_handoff/controller_config.json` 和 `real_handoff/nominal_timeline.json`。

```text
arm command period       0.020 s
release G1 command       t=0.690 s, position=520
detach prior             0.030 s
catch servo window       0.720–0.780 s
catch G1 command         t=0.780 s, position=370
ballistic intercept      t=0.815 s
held / close             position=370
G1 firmware speed        5000
joint6 roll offset       +0.785398 rad
```

注意：sim USD 的 held/open drive 值 `0.56/0.39 rad` 只服务于 USD mimic joint，绝不能发给
真机。真机只使用 UFACTORY position `370/520/370`。

## 真机电脑执行顺序

1. `git pull` 后核对当前 commit 和 `real_handoff/manifest.json`。
2. 测量这只 cube 的质量与边长。若质量明显偏离约 35 g，先把数据返回 sim；已有 25–45 g
   sweep 不是 3/3。
3. 核对相机 serial：third-view `317222073552`，wrist `233622079809`；以原始 calibration
   YAML 为准，不重新手抄外参。
4. 运行 disconnected controller replay：
   使用真机项目已有的 Python 环境，并先确认 `python -c "import numpy"` 成功；不要默认使用系统
   裸 `python3`。

   ```bash
   python scripts/20_closed_loop_dry_run.py \
     --output real_handoff/disconnected_dry_run.json
   ```

   预期 `robot_commands_sent=0`、4 个 camera commands、2 个 learned updates。
5. 生成 0.25× / 0.5× / 1.0× preview；这些脚本同样不会连接机器人：

   ```bash
   python scripts/21_preview_handoff.py --speed-scale 0.25 --output real_handoff/preview_025x.json
   python scripts/21_preview_handoff.py --speed-scale 0.5  --output real_handoff/preview_05x.json
   python scripts/21_preview_handoff.py --speed-scale 1.0  --output real_handoff/preview_1x.json
   ```

6. 复制 `configs/robot.example.json` 为本地配置，填 IP 后先运行只读连接：

   ```bash
   python scripts/00_check_connection.py --config configs/robot.local.json
   ```

7. 空夹爪执行 arm reference，重新测实际 q/dq tracking delay；再单独测 G1 370→520 的 actual
   position timing。只有两者与 handoff 相符后才夹 cube。
8. 第一次带 cube 用低处软垫，先录 throw-only，再启用 catch；目标只要得到 2–3 次成功。

## 必须记录

每个 trial 保存 controller timestamp、q/dq、G1 commanded/actual position、两台相机的
`camera_timestamp_s` / `host_received_s` / frame number、detections、ballistic state、residual、
selected intercept、实际 joint target，以及 spectator 视频。失败也保留。

## 已知风险与回退

- nominal close window 很窄：sim 中 `0.780 s` 成功，`0.785 s` 已出现失败。真机必须先重测
  G1 delay，不能直接大范围扫真机。
- 当前固定 cube 的三次 nominal learned rollout 成功；25–45 g 扰动未通过，不能宣称质量
  泛化。
- wrist 当前角度没有 terminal detection；它仍用于抓取/probe，并保留异步更新接口。若真机
  wrist 看见 cube，可进入同一 tracker；看不见不阻塞。
- 真机若失败，把 q/dq、G1 delay、camera timestamps、detections 和视频带回 sim，优先修
  detach prior 与 close deadline，不在真机上盲扫动作。
