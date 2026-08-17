# xArm6 fixed-cube toss/catch

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
