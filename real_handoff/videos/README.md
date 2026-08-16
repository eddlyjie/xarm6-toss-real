# 真机交接视频

优先看：

- `xarm6_toss_catch_3_trials_8x_slow_zoom.mp4`：25 g、35 g、45 g 三次成功依次播放；
  左侧是完整 global-camera frame 和红色 ROI，右侧是 ROI 放大，时间为 8 倍慢放。
- 原始 60 FPS 视频仍在
  `../../sim/outputs/real_candidate_learned_3/trial_01.mp4` 到 `trial_03.mp4`。

当前 real-safe candidate 是低能量 micro-toss：三次自由竖直位移约 6.8–7.0 mm，因此原始
1.584 s 视频肉眼很难分辨离手和重新接触。慢放/放大视频用于检查这条已有结果，不应把它
描述成明显的大抛物线。如果真机 demo 需要肉眼可见的飞行弧线，必须先在仿真中把离手高度
和 free-flight window 扩大，再生成新的真机 timeline。
