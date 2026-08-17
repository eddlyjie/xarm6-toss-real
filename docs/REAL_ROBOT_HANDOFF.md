# xArm6 固定 cube 真机交接

## 已验证的 sim 结果

当前 handoff 对应 `outward_vertical_real_detach_v7`。反腕是有意保留的：它换来更大的朝外
抛接空间；third_view 是 flight 主相机但不能假定全程在 FOV 内，wrist terminal 可见性不是门槛。

| 项目 | stable transfer candidate |
|---|---:|
| 固定 cube | 38 mm，sim nominal 35 g |
| Probe/J camera validation | 3/3 bilateral stable catch |
| continuous free flight | 95/95/145 ms |
| cube 上升 | 56.1–58.6 mm |
| post-detach camera / learned updates | 1 / 10 |
| residual | 8 mm bounded |
| G1 measured sim detach | 35 ms，真机范围 25–44 ms |
| paired Probe posterior | effective payload≈35 g，held=1，slip=0 |
| arm tracking lag in sim | 90 ms，timeline 已补偿 |
| wrist terminal detections | 0，非门槛 |

三次 nominal 都真实运行了 paired Probe、Probe-conditioned J、policy cameras 和 learned residual。
另外：physics observation 下已有 245 ms、过 apex 后下降再接的严格成功；rendered camera 下
已有 160 ms 明显飞行，但 bilateral fraction 仅 0.667，不算 stable catch。两者不能包装成
camera closed-loop 3/3；当前真机第一候选仍是上表的稳定版。

完整数值以 `real_handoff/real_constraints_report.json` 为准。最终视频：

```text
outputs/final_probe_j_seed_20260861_v3/spectator.mp4
outputs/final_probe_j_seed_20260861_v3/spectator_slow_0p25x.mp4
outputs/final_probe_j_seed_20260861_v3/spectator_third_view.mp4
outputs/final_probe_j_seed_20260861_v3/spectator_wrist.mp4
outputs/final_clear_camera_seed_20260841/spectator_zoom_slow_0p4x.mp4
real_handoff/sim_probe_j_evidence.json
```

## Controller 结构

```text
fixed grasp
→ paired empty/held Probe（sim 已进入控制；真机 current preflight 尚未运行）
→ Probe posterior 收紧 detach uncertainty，并由 J 选择 catch candidate
→ q/dq + calibrated T_hand_object + measured G1 delay 形成 detach belief
→ gravity-constrained ballistic propagation
→ third_view 主相机与 wrist 机会观测异步更新；丢帧时传播 belief
→ 1 个有效 camera sample 后启用 8 mm bounded residual
→ joint1 lateral correction，q2–q6 跟 nominal reference
→ G1 nonblocking close
```

third_view 是 flight/catch 主相机；wrist 只用于 grasp、Probe 和机会观测，两者不要求同时看见。
spectator/global camera 只录像。sim 已消费 paired empty/held actuator effort，posterior 实际改变 J；
但它没有消费真机 current，因此真机仍必须运行同一路径 Probe 并通过 held/slip gate。

## 真机数值

权威配置是 `real_handoff/controller_config.json` 和 `real_handoff/nominal_timeline.json`。

```text
arm command period          0.020 s
real measured arm lag       about 0.090 s
release G1 command          t=0.745 s, position=520
detach prior                0.030 s
stable catch servo/close    t=0.790 s
vision control end          t=0.990 s
ballistic intercept         t=0.930 s
held / close                position=370
G1 firmware speed           5000
joint6                      0.177245 rad, baked; extra offset=0
reference peak              1.745 rad/s, 13.057 rad/s²
sim actual peak             1.546 rad/s, 90.189 rad/s²
current transfer cap        0.45 rad/s, 1.5 rad/s²
```

0.25× preview 为 `0.436 rad/s / 0.816 rad/s²`，在当前 cap 内；0.5× 和 1× 均超 cap。
1× 还存在 sim actual acceleration 超过 20 rad/s² command cap 的问题，不能直接下发真机。
注意：sim USD 的 `0.56/0.30/0.60 rad` 只服务于 USD mimic joint，绝不能发给
真机。真机只使用 UFACTORY position `370/520/370`。

## 真机电脑执行顺序

1. `git pull` 后核对当前 commit 和 `real_handoff/manifest.json`。
2. 测量 cube 边长与质量，并先做同一路径的 paired Probe。以下前两条不带 `--execute` 时只预览；
   核对中心姿态后再分别加 `--execute`：

   ```bash
   cd toss_project_sim_handoff/toss_project/real_cube_demo
   python scripts/07_probe_cube.py --condition empty
   python scripts/07_probe_cube.py --condition cube
   python scripts/08_compare_probe.py \
     --empty outputs/probes/<empty> --cube outputs/probes/<cube>
   ```

   现有 comparison 会保存 paired current/effort residual；held/slip 与 timing posterior 尚需接入
   controller，这也是当前明确的真机缺口。规则 cube 不要求质量精确到克。
3. 核对相机 serial：third-view `317222073552`，wrist `233622079809`；以原始 calibration
   YAML 为准，不重新手抄外参。
4. 运行 disconnected controller replay：
   使用真机项目已有的 Python 环境，并先确认 `python -c "import numpy"` 成功；不要默认使用系统
   裸 `python3`。

   ```bash
   python scripts/20_closed_loop_dry_run.py \
     --output real_handoff/disconnected_dry_run.json
   ```

   预期 `robot_commands_sent=0`、4 个 camera commands、4 个 learned updates。
5. 生成 0.25× / 0.5× / 1.0× preview；这些脚本同样不会连接机器人：

   ```bash
   python scripts/21_preview_handoff.py --speed-scale 0.25 --output real_handoff/preview_025x.json
   python scripts/21_preview_handoff.py --speed-scale 0.5  --output real_handoff/preview_05x.json
   python scripts/21_preview_handoff.py --speed-scale 1.0  --output real_handoff/preview_1x.json
   ```

   只有 0.25× 在当前 transfer cap 内；0.5×/1× 文件只用于审阅，不代表允许执行。

6. 复制 `configs/robot.example.json` 为本地配置，填 IP 后先运行只读连接：

   ```bash
   python scripts/00_check_connection.py --config configs/robot.local.json
   ```

7. 空夹爪只执行 0.25× reference，重新测实际 q/dq tracking delay；再单独测 G1 370→520 的 actual
   position timing。只有两者与 handoff 相符后才夹 cube。
8. 操作者根据 tracking、空间和软垫条件决定是否批准更高动态。第一次带 cube 先录 throw-only，
   再启用 stable catch；得到 2–3 次即可。clear-flight `t=0.890 s` close 只作第二阶段升级。

## 必须记录

每个 trial 保存 controller timestamp、q/dq、G1 commanded/actual position、两台相机的
`camera_timestamp_s` / `host_received_s` / frame number、detections、ballistic state、residual、
selected intercept、实际 joint target，以及 spectator 视频。失败也保留。

## 已知风险与回退

- stable candidate 的 close window 很早且很窄；真机必须先重测
  G1 delay，不能直接大范围扫真机。
- 当前固定 cube 的三次 Probe/J + camera + learned rollout 成功；20–50 g 不是正式成功率，
  不能宣称质量泛化。
- sim paired Probe 已进入控制，但真实 paired current Probe 尚未运行；没有真机 posterior 时不能
  声称完成 real Probe-conditioned coordination。
- stable 3/3 仍不是 strict obvious-flight 3/3；seed 20260863 已过 apex 并下降再接，但只有 1 个
  post-apex spectator frame，要求是 2 个。
- wrist 当前反腕角度没有 terminal detection；它仍用于抓取/probe，并保留异步更新接口。若真机
  wrist 看见 cube，可进入同一 tracker；看不见不阻塞。
- 1× reference 超过当前 transfer cap，sim actual acceleration 也达到约 90.2 rad/s²；完成 retiming
  且操作者明确批准前不得执行。
- 真机若失败，把 q/dq、G1 delay、camera timestamps、detections 和视频带回 sim，优先修
  detach prior 与 close deadline，不在真机上盲扫动作。
