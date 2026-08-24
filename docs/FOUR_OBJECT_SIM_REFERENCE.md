# 四物体真机调试用 Sim 动作参考

本页把已经进入 Git 的四物体 Sim 视频、profile 和 20 ms timeline 放在同一个位置，供真机现场对照动作方向、
离手时机和接取阶段。视频只证明对应 Sim rollout；它们不代表真机已经成功，也不能替代 O1–O3 的 G1实测标定。

所有主动作固定 J1/J4/J6，只动态执行 J2/J3/J5。相机只负责录像和离线测角。表中的“目标角”是 policy 输入，
“Sim实测角”才是对应 rollout 的结果。

## 视频、profile 与 timeline

| 物体 | 档位 | 目标角 | Sim实测角 | 视频 | Profile | 20 ms timeline |
|---|---|---:|---:|---|---|---|
| O0 38 mm cube / 8 g | low | 5.0° | 4.59° | [MP4](media/four_object_open_loop/O0_cube38_low_4p59deg.mp4) | [JSON](../configs/open_loop_flip/cube38/low_5deg.json) | [JSON](../real_handoff/cube38/low/timeline.json) |
| O0 | next | 6.5° | 6.48° | [MP4](media/four_object_open_loop/O0_cube38_medium_6p48deg.mp4) | [JSON](../configs/open_loop_flip/cube38/medium_6p5deg.json) | [JSON](../real_handoff/cube38/medium/timeline.json) |
| O0 | high | 8.0° | 7.87° | [MP4](media/four_object_open_loop/O0_cube38_high_7p87deg.mp4) | [JSON](../configs/open_loop_flip/cube38/high_8deg.json) | [JSON](../real_handoff/cube38/high/timeline.json) |
| O1 44.5×46×30 mm / 20 g | low | 5.0° | 2.96° | [MP4](media/four_object_open_loop/O1_cuboid30_low_2p96deg.mp4) | [JSON](../configs/open_loop_flip/cuboid30/low_3deg.json) | [JSON](../real_handoff/cuboid30/low/timeline.json) |
| O1 | next | 5.5° | 5.71° | [MP4](media/pose_conditioned_20260825/o1_cuboid30_target5p5_measured5p71.mp4) | [JSON](../configs/open_loop_flip/cuboid30/pose_conditioned_5p5deg.json) | [JSON](../real_handoff/cuboid30/pose_conditioned_5p5deg/timeline.json) |
| O1 | high | 6.5° | 6.57° | [MP4](media/four_object_open_loop/O1_cuboid30_high_6p57deg.mp4) | [JSON](../configs/open_loop_flip/cuboid30/high_6p5deg.json) | [JSON](../real_handoff/cuboid30/high/timeline.json) |
| O2 50.5×51×33.5 mm / 26.6 g | low | 5.0° | 4.61° | [MP4](media/four_object_open_loop/O2_cuboid33_low_4p61deg.mp4) | [JSON](../configs/open_loop_flip/cuboid33/low_5deg.json) | [JSON](../real_handoff/cuboid33/low/timeline.json) |
| O2 | next | 5.5° | 5.62° | [MP4](media/pose_conditioned_20260825/o2_corrected_target5p5_measured5p62.mp4) | [JSON](../configs/open_loop_flip/cuboid33/pose_conditioned_5p5deg.json) | [JSON](../real_handoff/cuboid33/pose_conditioned_5p5deg/timeline.json) |
| O2 | high | 6.5° | 6.45° | [MP4](media/four_object_open_loop/O2_cuboid33_high_6p45deg.mp4) | [JSON](../configs/open_loop_flip/cuboid33/high_6p5deg.json) | [JSON](../real_handoff/cuboid33/high/timeline.json) |
| O3 57.5×58×38 mm / 37 g | low | 5.0° | 4.40° | [MP4](media/four_object_open_loop/O3_cuboid38_low_4p40deg.mp4) | [JSON](../configs/open_loop_flip/cuboid38/low_4p5deg.json) | [JSON](../real_handoff/cuboid38/low/timeline.json) |
| O3 | next | 5.5° | 5.58° | [MP4](media/pose_conditioned_20260825/o3_corrected_target5p5_measured5p58.mp4) | [JSON](../configs/open_loop_flip/cuboid38/pose_conditioned_5p5deg.json) | [JSON](../real_handoff/cuboid38/pose_conditioned_5p5deg/timeline.json) |
| O3 | high | 6.5° | 6.85° | [MP4](media/four_object_open_loop/O3_cuboid38_high_6p85deg.mp4) | [JSON](../configs/open_loop_flip/cuboid38/high_6p5deg.json) | [JSON](../real_handoff/cuboid38/high/timeline.json) |

## 现场应该观察什么

每条 timeline 都按 `throw → flight → precatch → catch` 排列。现场低速空臂和软垫抛出阶段重点观察：

1. J1/J4/J6 保持固定，主动作只来自 J2/J3/J5；
2. EE 在 throw 内完成下摆、反转和上扬，而不是从起点持续向上翘；
3. release 后物体与双侧夹指之间出现清楚的间隙，手臂及时制动或撤离，不继续追着物体上升；
4. 物体主要绕同一前滚翻/后滚翻轴旋转，没有明显侧向飞出；
5. precatch 在物体下降段接近预测位置，close 后能保持至少 0.5 s；
6. 真机视频中的角度、离手时间和接取误差单独记录，不用 profile 名称或 Sim角度替代实测。

所有现有 profile 的 release command 都在 0.62 s。low 档的 preclose/close 通常为 0.76/0.81 s；O1–O3 high
为 0.80/0.86 s；next 档是 low/high 之间的连续插值。O0 high 使用更晚的 0.84/0.92 s。真实物理离手会受
G1 延迟影响，因此这些是命令时刻，不是保证的 detach 时刻。

## 真机使用方式

1. 先按 [现场命令卡](FOUR_OBJECT_ONSITE_COMMANDS.md) 完成 O0 low 保底和 O1–O3 独立 G1标定；
2. low 完整接住后，用 `scripts/32_prepare_pose_ladder.py` 生成同一物体的 next/high 分阶段 profiles；
3. 打开本页对应 MP4 作为动作形态参考，同时从 plan-only、0.25×空臂重新开始；
4. 每次完整运行后，用 `scripts/31_record_real_trials.py record-from-runner` 保存真实结果；
5. 真机动作与视频明显不一致时，先检查物体方向、抓取深度、G1位置和 arm/G1时延，再离线调整。不要在现场
   直接编辑 20 ms joint timeline。

## 证据边界

O1–O3 的 low/high 是固定参考迁移候选；next 是 object/pose-conditioned warm-start 的连续动作。它们共同提供
真机 demo 的候选梯队，但仍属于 Sim 结果。四物体真机成功、重复 catch rate、实测角度分布以及 M0–M3 对比，
都必须由新的现场 trial records 支撑。
