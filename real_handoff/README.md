# xArm6 + G1 最小自抛自接真机交接

## 现在可以交什么

这不是把 Panda policy 硬迁移到 xArm6。当前交付是一条固定抓取、低能量近竖直微抛的
xArm6 joint-space backbone，加上释放后 global D435 弹道估计和冻结的小型 learned
intercept residual。腕部 D435 只做可选的抓取前检查。

仿真证据：

- 真机速度候选 `qdot_max=1.756 rad/s`、`qdd_max=18.611 rad/s²`。
- 25 g、35 g、45 g 和 1 mm 抓取偏置的 real-candidate native trials 为 3/3，均为双侧
  contact、0.5 s stable hold，并且均在 physical detach 后消费 global-camera observation
  和 learned action。
- 更宽的 10-trial native cohort 为 fixed 8/10、learned 8/10；learned 将平均 intercept
  error 从 20.13 mm 降到 17.51 mm。两次失败是接取失败，不是初始化掉落。
- cube 在 episode 初始化后没有 pose/velocity rewrite、隐藏支撑或约束。

这些是 sim 结果，不等于真机已经成功。真机首先必须空载 preview 并确认 20 ms streaming、
实际 q/dq、G1 时延和相机时间戳。

## 交付文件

- `manifest.json`：版本、峰值、实测时延、模型、相机和结果入口。
- `timelines/nominal.{json,csv}`：默认 arm q/dq/qdd 和 G1 event。
- `timelines/release_early.{json,csv}`：G1 partial-open 提前 20 ms。
- `timelines/release_late.{json,csv}`：G1 partial-open 推迟 20 ms。
- `../sim/models/intercept_residual_real_v1.json`：真机速度版本 checkpoint、normalization、
  feature order 和 20 mm action bound。
- `../scripts/20_closed_loop_dry_run.py`：只读 JSONL/stdin observation，默认且永远不连接机器人。
- `../configs/global_camera_real.json`：global D435 intrinsics 和固定 `T_base_camera`。
- `../configs/wrist_camera_real.json`：wrist D435 的 `T_link_eef_camera`。
- `../sim/assets/xarm6_g1/`：本地 USD/URDF/meshes，不依赖 Nucleus。
- `../sim/outputs/real_candidate_learned_3/trial_0{1,2,3}.mp4`：三个代表性成功视频。

## 初速度和闭环到底从哪里来

开夹爪命令本身不能告诉我们 cube 的真实初速度。运行时分两段：

1. 在 nominal detach 附近读取实际 `q/dq`。由 FK 得到手的 pose，由 Jacobian 得到
   `v_hand, omega_hand`。固定抓取 offset 给出
   `p_cube = T_base_hand * p_hand_cube` 和
   `v_cube = v_hand + omega_hand × r_hand_cube`。这是 encoder prior。
2. global D435 首次重新看见 cube 后，把带 camera timestamp 的 RGB-D center 变到 base
   frame。最近 2–6 帧在已知 gravity 下最小二乘拟合 `p0, v0`，覆盖 G1 延迟、arm tracking
   delay 和轻微滑移造成的 prior 误差。
3. 解析弹道把 fitted state 向前预测 50 ms。learned residual 只读时间、相机帧数、fit RMS、
   position/velocity innovation，输出不超过 20 mm 的 catch-target correction。
4. corrected intercept 转成小幅 IK/Jacobian joint correction；G1 在固定 deadline close。

因此 hard-coded 的是抓取和安全 backbone，不是 release 后的 cube state。global camera 的
观测实际改变接取目标，满足最小 closed loop。

## 相机用法

global D435 是必需的 flight camera：640×480@60 Hz，aligned depth，使用 camera timestamp。
黄色 cube detector 应先使用 encoder prior 投影出的约 80×80 px ROI，再做 HSV/depth median；
这个视角很斜，整图 contour threshold 会漏掉只有几到几十像素的远处 cube。

wrist D435 不要求在摆臂或飞行时看到 cube。若用它复核抓取，逐帧计算
`T_base_camera = T_base_link_eef(q) @ T_link_eef_camera`，然后保存 `T_hand_object`。
两台相机不需要同时看见 cube；它们通过 base frame 和 encoder propagation 交接。

## 不动机器人的检查

在 `xarm_6/` 下运行：

```bash
PYTHONPATH=src python scripts/20_closed_loop_dry_run.py \
  --model sim/models/intercept_residual_real_v1.json \
  --release-time-s 0.50 \
  --prediction-horizon-s 0.05
```

预期 `robot_commands_sent=0`、4 个 post-release updates。真实 detector 也可以按同一 JSONL
schema 通过 stdin 流入：先一条 `encoder_detach_prior`，随后若干条
`global_camera_position`。

## 最短真机顺序

1. 备好软垫、低功率/低速模式和急停。称 cube，首个用约 35 g、边长约 38 mm。
2. 空夹爪单独验证 `370 → 520 → 370`，记录实际 position、首次运动、contact-loss proxy
   和总行程时间。不要把 sim 的 `0.56/0.40 rad` 当成真机 G1 命令。
3. 只加载 `timelines/nominal.csv` 的 q，先 0.25× time scale 空载，再 0.5×，最后才 1.0×。
   每次保存 commanded/actual q/dq；不得超过已检查的 controller limit。
4. 1.0× 空载时加入 G1 events，但夹爪内不放 cube。确认 arm streaming 没有丢周期，G1
   nonblocking command 不阻塞 20 ms servo loop。
5. 固定抓取 cube，低速带物检查不会滑落；global D435 确认 release ROI 在 image bounds。
6. 正式 nominal 一次。若 cube 到 close 时仍未完全离手，使用 `release_early`；若明显过早
   下落，使用 `release_late`。只在这三个候选中选，不现场扩大动作幅度。
7. 选定 timing 后冻结参数，连续做 2–3 次，保留所有视频、q/dq、G1 position、camera
   timestamps、detector observations 和 controller outputs。

## 接到现有真机代码

handoff archive 中已有可用的 xArm SDK wrapper、`servo_j`、FK/IK、nonblocking G1 command
和 `MotionCameraRecorder`：

```text
toss_project_sim_handoff/toss_project/real_cube_demo/
```

最小集成点是：

- 用 timeline q 替换 `19_natural_throw_empty.py` 的旧 reference；不要改 20 ms monotonic loop。
- timeline 的 G1 event 用 `command_gripper_position()`，不能 `wait=True`。
- release 前由实际 q/dq 和固定 `T_hand_object` 调用 `set_encoder_detach_prior()`。
- global detector 每帧输出 base-frame center，调用 `add_global_camera_position()`。
- 将 `corrected_intercept_base_m - grasp_offset_base_m` 交给现有 IK，限制为相对 nominal
  最多 20 mm，再送下一拍 `servo_j`；close deadline 后不再更新。

第一轮真机如果来不及接入在线 IK，先用 camera 只核对 prior/fit，但这不算最终 learned
closed-loop demo。正式的 2–3 次结果必须保存至少一次 detach 后 controller command，且该
command 实际进入 joint target。

## 已知风险

- 真机 arm tracking delay 约 90 ms，sim 只以 transfer config 和先验体现，无法替代实际
  q/dq 对齐。
- real G1 detach 估计 25–44 ms；sim 的几何阈值报告约 65–80 ms，两者定义不同，所以提供
  ±20 ms release bracket，不能把 sim threshold 当真机定时真值。
- global camera 角度刁钻且 cube 很小；ROI projection、aligned depth 和 timestamps 是必需项。
- 当前 learning 修正 catch xyz，不直接生成整条轨迹，也不估质量/摩擦。对首个 2–3 次 demo
  足够；不要临时扩成端到端 RGB/RL。
