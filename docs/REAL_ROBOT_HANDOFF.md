# 旧版 throw-only 交接（已被替代）

此文件来自最初只有 throw-only 时的计划，不能再作为当前执行依据。现在已有 xArm6+G1
native catch、global-camera ballistic tracker、learned intercept residual、真机安全速度版本、
三条 timing timeline 和 3/3 transfer-candidate 视频。

当前权威交接请读 `xarm_6/real_handoff/README.md` 和 `xarm_6/real_handoff/manifest.json`。

## 当前目标

老师对第一版真机实验要求较简单。不要先实现完整自抛自接；先用 xArm 6 和一个
小 cube 做出稳定 throw-only：

```text
固定抓取 → 抬起 → 平滑摆臂 → 运动中开夹爪 → cube 飞入软垫/缓冲箱
```

1 个 cube 成功即可形成首个 demo；时间允许再换第 2 个尺寸或质量略有变化的
cube。第二个 cube 不要求 zero-shot 接取，优先证明同一基础流程可以迁移。

## 已有代码的边界

- `00_check_connection.py` 只读状态，不会 enable motion。
- `01_preview_throw.py` 只生成 CSV，不连接机械臂。
- `02_gripper_test.py` 默认 dry-run；只有同时传 `--execute` 且配置中
  `hardware_confirmed=true` 才会控制夹爪。
- 示例 joint poses 是占位值，不能直接执行。
- 高速 `set_servo_angle_j` throw runtime 尚未接通，必须在真机电脑结合固件、
  实际控制周期和示教轨迹实现。

## 建议当天完成的顺序

1. 记录 xArm 6 IP、固件、SDK、夹爪型号和单位。
2. 运行只读连接脚本，保存第一次状态快照。
3. 通过 xArm Studio 示教并记录：home、pre-grasp、grasp、lift、pre-throw、
   release、follow-through。
4. 确认夹爪 open/closed position 后运行夹爪测试。
5. 把示教的 6 关节 pose 写入新的 throw plan，不覆盖 example。
6. 用 preview 脚本画/检查关节轨迹，先低速空载回放。
7. 实现模式 1 的 `set_servo_angle_j` 固定周期 streaming；记录每条命令返回码。
8. 空载通过后再夹持泡棉 cube，以低能量向缓冲箱抛出。
9. 保存全局相机视频、release 时刻、实际 q/dq 和 result.json。

## 两个相机怎么用

全局相机安装在高处斜侧方，覆盖抓取区、飞行区和落地区。它是飞行阶段的主要
相机。腕部相机主要用于抓取前定位；之后可让机械臂停在几个观察 pose，用同一
腕部相机采集主动多视角。不要假设腕部相机在快速摆臂和飞行中始终看得见物体。

第一版 throw-only 可以完全不依赖在线视觉控制：相机只录像和评价。第二阶段再
把全局相机估计的 cube pose 与用户指定 target pose 输入动作选择器。

## 真机 policy 的简化路线

仿真 actor 不能直接用于 xArm 6：它包含 Panda 7DoF q/dq、三固定 RGB-D 视角、
Probe 和 GelSight detach 特征。真机先采用分层版本：

```text
阶段 A：固定示教动作 + 固定 release 时刻
阶段 B：当前 cube pose + target pose → 从 3–5 条动作中选择
阶段 C：根据真实落点/飞行时间拟合 release correction
阶段 D：收集足够 trials 后训练 xArm 6 专用 target-conditioned policy
```

阶段 B 的最小输入只需要：

- 当前 cube pose（抓取前由全局/腕部相机估计）；
- 目标 cube pose（实验配置给定）；
- xArm 6 当前 6DoF q/dq；
- cube ID、尺寸和质量的测量值；
- 候选动作库。

不要在只有几个真机样本时训练大网络。先让动作库和简单插值工作，再把真实数据
用于 residual correction。

## 最小实验表

先做下列三组就足够形成清楚结果：

| 组别 | 物体 | 方法 | 主要指标 |
|---|---|---|---|
| A | cube seen 01 | 固定动作 | 是否抓起、是否离手、是否进入缓冲区 |
| B | cube seen 01 | 调整 release 时刻/动作 | throw-only 成功率、落点散布 |
| C | cube 02 | 冻结 B 的方法或少量标定 | 是否可迁移、失败类型 |

每组先做少量开发尝试。动作稳定后再固定参数，连续记录一小组 trials。不要为了
凑成功率丢掉失败视频。

## 高速 throw runtime 的实现提示

官方 SDK 中 `set_servo_angle` 用于普通关节目标；`set_servo_angle_j` 用于模式 1
下的高频 joint servo，并且只执行最新指令。实现时需要：

- 先以模式 0 普通低速运动到 start pose；
- 切换模式 1 并进入 state 0；
- 使用 `time.monotonic()` 按固定周期发送 6 关节目标；
- 在轨迹的 release sample 发非阻塞夹爪打开指令；
- 继续发送 follow-through，再停止并切回普通模式；
- 若 SDK 返回非零、机械臂状态异常或周期严重超时，停止继续发送。

先实测网络和控制周期。示例配置使用 20 ms 仅作为离线预览，不代表控制器的最终
周期。最终动作应以实际 q/dq 跟踪和视频为准。
