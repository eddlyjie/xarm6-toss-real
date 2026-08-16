# xArm6 fixed-cube toss/catch

当前可移交候选是朝外的 xArm6 + G1 固定 cube 自抛自接，不再使用旧的 inward-facing
姿态。Isaac native learned controller 在固定约 38 mm、35 g cube 上完成 3/3：真实 detach、
third-view 异步弹道更新、bounded residual、小幅 catch pose correction、双指重新夹住并保持。

关键结果：

- command limits：0.45 rad/s、1.5 rad/s²，arm tracking delay 90 ms；
- sim detach delay 35 ms，位于真机反馈的 25–44 ms 范围；
- cube 相对 gripper 最大离手 26.3 mm，最高上升约 105 mm；
- 每次离手后 3 个 camera updates、2 个 learned updates；
- 三次均 bilateral catch，0.5 s hold；
- wrist 本次没有看到 cube，belief 由 release prior、ballistic propagation 和 third-view
  observation 维持；spectator 从不进入控制。

先看结果：

```text
outputs/final_handoff_nominal/spectator_slow_0p4x.mp4
outputs/final_handoff_nominal/three_view.mp4
real_handoff/real_constraints_report.json
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
25–45 g mass sweep 没有 3/3，因此真机前必须称重/做短 probe，并重新测 arm/G1 latency，
不能把 nominal timing 当成跨物体通用参数。
