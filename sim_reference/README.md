# Panda 仿真参考包

这个目录让真机电脑能直接看到 Panda 在仿真中怎样完成 Probe、抛出、自由飞行、
接取和接后运动。它不是另一套审计材料，也不是要求 xArm 6 逐关节照搬 Panda。
真正要迁移的是状态机、观测、动作条件和控制时序。

## 建议观看顺序

| 文件 | 时长 | 用途 | 结果边界 |
|---|---:|---|---|
| `videos/01_active_probe_panda.mp4` | 12.75 s | settle、tilt、chirp、shake、grip reduction、recover | Probe 教学演示；物体相对手的 pose 在这段早期演示中是 prescribed |
| `videos/02_throw_and_detach_panda.mp4` | 3.25 s | 左向蓄势、右向加速、运动中开夹、真实 detach 和自由飞行 | 只展示 throw/detach，未执行 catch |
| `videos/03_successful_full_pipeline_panda.mp4` | 17.50 s | 拿取、Probe、抛出、自由飞行、同手接住、稳握和接后前送 | 固定小物体完整成功 run；同配置复跑 5/5，不代表跨物体泛化 |
| `videos/04_pose_conditioned_45deg_development.mp4` | 86.17 s | 45° target 下的多姿态/接后规划开发轨迹 | `passed=false`，只用于理解 target pose 如何进入开发流程，不能当成功样本 |
| `videos/05_toss_vs_direct_comparison.mp4` | 18.83 s | Toss–IK 与 Direct maintain-grasp 同屏 | 历史对比素材，不是当前方法的正式总体结果 |

若电脑安装了 `ffmpeg`：

```bash
ffplay videos/01_active_probe_panda.mp4
ffplay videos/02_throw_and_detach_panda.mp4
ffplay videos/03_successful_full_pipeline_panda.mp4
ffplay videos/04_pose_conditioned_45deg_development.mp4
ffplay videos/05_toss_vs_direct_comparison.mp4
```

没有 `ffplay` 时，用系统播放器直接打开 MP4 即可。

## 看视频时应该关注什么

不要只看“物体飞起来了”。逐段看下面的因果关系：

1. Panda 先稳定抓取和抬升，建立统一的 hand-object state。
2. Probe 在不释放物体的情况下激励质量、质心、惯量和接触/滑移特征。
3. Detach 模型预测开夹命令到真实离手之间的延迟与 6-D 状态残差。
4. 抛出后，策略不是追一个固定空间点，而是在多个时间和接触对之间选动态 catch。
5. Whole-arm reference 同时包含关节位置 `q(t)` 和关节速度 `dq(t)`；夹爪依据
   `release_time`、`catch_time` 和 `close_lead` 单独调度。
6. 接住后重新估计 `T_HO`，再根据请求的 `T_WO_target` 选择 residual motion。
7. 完整方法的 target pose 在候选/skill coordination 阶段就参与评分，不是最后才补 IK。

## 本目录的代码

运行：

```bash
python panda_sequence_reference.py
```

它会打印一条压缩的 Panda 控制时间线，并演示两个不同 target orientation 会选择
不同的 toss/regrasp skill。代码只生成离线 reference，不连接 Isaac 或 xArm。

完整原理见 `../docs/SIM_TO_REAL_METHOD.md`；具体 Panda→xArm 映射见
`PANDA_CONTROL_AND_PORTING.md`。
