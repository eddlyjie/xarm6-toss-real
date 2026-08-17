# xArm6 + G1 真机 micro-toss / rapid release–recatch 实验记录

日期：2026-08-17
真机工程：`xarm6-toss-real`
用途：保留本次真机测试结果、限制、解释和后续仿真方向，供另一台 Isaac Sim / Isaac Lab 电脑接手。

## 1. 结论

本次已经得到一个应当冻结保留的真机 baseline：xArm6 使用 1× joint timeline 上抛，G1 在运动中部分打开并重新闭合，操作者确认最终版本能够用同一个夹爪重新接住 cube。

准确名称应是：

> xArm6 + G1 同夹爪 micro-toss / rapid release–recatch

它是一个有效的真机 demo，但目前自由飞行在视觉上不明显，更像极短暂释放后的 drop-catch。它证明了 1× 手臂轨迹、20 ms `servo_j`、异步 G1 timing 和同夹爪快速接取可以在真机上协同工作；它还没有证明明显抛物线、cube 旋转或相机/学习闭环接取。

最终保留版本在 13:38、13:39、13:42 连续完成了三次控制执行：三次均无控制器错误，均发送 45 个 arm command，均无周期超过 20 ms。操作者确认该版本能够接住 cube。当前日志没有自动视觉 success label，因此不能仅根据三个目录机械地宣称“3/3 抛接成功”。

## 2. 硬件与软件

| 项目 | 真机信息 |
|---|---|
| Robot | UFACTORY xArm6 |
| Robot IP | `192.168.2.232` |
| Robot firmware | `v2.5.1` |
| xArm Python SDK | `1.18.4` |
| Controller | 20 ms `servo_j` |
| Gripper | UFACTORY G1，firmware `3.6.0` |
| G1 speed | `5000` |
| Cube | 约 38–40 mm 边长，轻量 3D 打印件；本次没有把质量当作已知先验 |
| GelSight | 未安装 |
| Wrist F/T | 未安装 |
| Global D435 | serial `317222073552` |
| Wrist D435 | serial `233622079809` |

可用的真机信号是 q、dq、joint effort、motor current 和 G1 position。global / wrist 相机在当前最终 demo 中没有进入控制。

仓库内可直接交给仿真的相机配置：

- `configs/global_camera_real.json`
- `configs/wrist_camera_real.json`

旧的 `toss_project_sim_handoff/toss_project/real_cube_demo/configs/hardware.json` 仍指向原工程中的 `RobotCamCalib/RobotCamCalib/outputs/*.yaml`。这些原始目录没有完整打包在当前仓库中；仿真侧不要依赖该旧路径，应优先使用上面的两个 JSON。真机曾保存双相机彩色检查图：

```tex
toss_project_sim_handoff/toss_project/real_cube_demo/outputs/captures/
  20260817_125124/cameras_color.png
```

## 3. 最终冻结的执行方式

最终 baseline 命令：

```bash
cd ~/toss_project/xarm6-toss-real

python scripts/22_run_empty_handoff.py
  --speed-scale 1.0
  --execute-cube-toss-catch
```

关键参数：

| 参数 | 值 |
|---|---:|
| arm timeline duration | 0.72 s |
| arm command period | 0.020 s |
| reference peak joint speed | 1.7448 rad/s |
| reference peak joint acceleration | 13.0574 rad/s² |
| G1 held | 370 |
| G1 partial open | 520 |
| G1 catch close | 370 |
| real release command | 0.636 s |
| real catch-close command | 0.720 s |

`real_handoff/nominal_timeline.json` 中仍保留仿真 handoff 的 `0.745 / 0.810 s` G1 event。真机脚本会在输出中打印 `real_cube_g1_override`，最终 baseline 以 `0.636 / 0.720 s` 为准。

这条命令会：

1. 移动到固定 handoff pose；
2. G1 打开到 850，由操作者把 cube 放入；
3. G1 夹到 370；
4. 慢速移动到反腕的 timeline start；
5. 以 1× 执行 arm timeline，并非阻塞地发送 `520 → 370`；
6. 记录 commanded/actual q、dq、effort、current、G1 event 和控制时间。

## 4. 真机测试过程

### 4.1 Paired Probe

空载和持 cube 使用相同 probe trajectory，各记录 480 个样本：

```tex
toss_project_sim_handoff/toss_project/real_cube_demo/outputs/probes/
  20260817_124314_empty/
  20260817_124430_cube/

toss_project_sim_handoff/toss_project/real_cube_demo/outputs/probe_comparisons/
  20260817_124509/
```

Probe center：

```tex
[0.061075, -1.21475, -0.300198, 0.022688, 1.439898, 0.331612] rad
```

paired comparison 已保存 position、velocity、effort 和 motor-current residual。cube 很轻，effort residual 相对噪声并不足以可靠反推出精确质量；这些数据适合当作 sim posterior / feature 输入，不应包装成精确称重结果。最终 baseline 也没有在运行时读取该 posterior。

### 4.2 G1 实测

G1 `370 → 520 → 370` 结果：

```tex
outputs/g1_partial/20260816_192859.json
```

| 动作 | command return | first observed motion | target reached |
|---|---:|---:|---:|
| 370 → 520 | 1.36 ms | 22.64 ms | 102.79 ms |
| 520 → 370 | 1.36 ms | 12.57 ms | 102.62 ms |

固定姿态、相机观测得到的真机 detach delay 范围约为 25–44 ms。注意 detach 发生在夹爪尚未走到 520 之前，因此 detach delay 与完整 G1 travel time 不是同一个量。

### 4.3 空载 arm timeline

| speed scale | 输出 | 结果 |
|---:|---|---|
| 0.25× | `outputs/real_empty_handoff/20260817_124951_025x_empty/` | 完整运行；lag 约 80 ms；无超期周期 |
| 0.5× | `outputs/real_empty_handoff/20260817_125810_0p5x_empty/` | 完整运行；lag 约 80 ms；无超期周期 |
| 1.0× 初次 | `outputs/real_empty_handoff/20260817_132132_1p0x_empty/` | 在 0.66 s 附近触发 C60；部分数据已保存 |

0.5× arm 虽然稳定，但 cube 的上抛视觉效果不足。两个 0.5× throw-only 记录如下：

```tex
outputs/real_empty_handoff/20260817_131218_0p5x_cube_throw_only/
outputs/real_empty_handoff/20260817_131618_0p5x_cube_throw_only/
```

对应 release command 分别是 1.425 s 和 1.225 s；操作者观察为释放偏晚、cube 更接近直接下落，因此后续转到 1×。

### 4.4 1× C60 限制及处理

初次 1× 失败不是 joint speed 超限。保存的 controller error 是：

```tex
C60: Linear speed exceeded limit in servo_j mode
get_c60_error_info(): configured=1200.0 mm/s, requested≈1212.5 mm/s
```

当时：

- normal joint speed upper limit 约 4.0 rad/s；
- reference peak joint speed 只有 1.7448 rad/s；
- reduced mode 已关闭，之前临时检查过的 reduced joint setting 已恢复；
- 真正阻止命令的是 `servo_j` Cartesian linear-speed factor。

xArm SDK 的默认 linear factor 是 1.2。本次将其设为 1.6：

```python
arm.get_linear_spd_limit_factor()
arm.set_linear_spd_limit_factor(1.6)
```

设置后 1× timeline 可以完整执行。该 factor 是 robot/controller setting，不写在 timeline 文件中；以后机器人重启或换真机时应先读取实际值。不要把这次问题误判为“某个关节只能到 0.45 rad/s”或“关节命令超过 20 rad/s”。

### 4.5 1× throw-only 与第一版 recatch

1× throw-only：

```tex
outputs/real_empty_handoff/20260817_132815_1p0x_cube_throw_only/
```

结果：

- 39 个 arm command 完整发送；
- release command 实际发送于 0.6007 s；
- tracking lag 约 80 ms；
- maximum command lateness 约 0.10 ms；
- 没有 C60、没有 20 ms 超期周期。

第一版 recatch 使用 `0.600 / 0.665 s`：

```tex
outputs/real_empty_handoff/20260817_133212_1p0x_cube_toss_catch/
```

这两个 G1 command 只相隔 65 ms，夹爪很快反向闭合，cube 没有形成明显可见的自由飞行。该版本说明 G1 足够快，但不是最终保留 timing。

### 4.6 最终保留版本

最终 `0.636 / 0.720 s` timing 的三个 controller-complete 记录：

```tex
outputs/real_empty_handoff/20260817_133810_1p0x_cube_toss_catch/
outputs/real_empty_handoff/20260817_133932_1p0x_cube_toss_catch/
outputs/real_empty_handoff/20260817_134247_1p0x_cube_toss_catch/
```

共同结果：

- 每次发送 45 个 arm command；
- actual release command 为 0.6364–0.6365 s；
- actual catch-close command 为 0.7205–0.7206 s；
- arm tracking lag 均约 80 ms；
- lag-aligned RMS：q2 约 0.0046 rad、q3 约 0.0076–0.0077 rad、q5 约 0.0111–0.0112 rad；
- maximum command lateness 约 0.06–0.12 ms；
- 20 ms 超期周期为 0；
- controller error 为 0。

操作者观察：该版本明显优于第一版，G1 能够接住 cube，但自由飞行仍不明显，视觉上仍接近 cube 直接落入夹爪。这是当前应该冻结保留的真机 baseline。

## 5. 为什么速度很高，但飞行仍不明显

从真实 q/dq 通过 URDF FK/Jacobian 计算，在 1× 运动的约 0.64 s：

```tex
TCP position ≈ [0.5559, 0.0000, 0.5094] m
TCP velocity ≈ [-0.188, 0.000, 1.333] m/s
```

仅按重力弹道，1.333 m/s 的竖直速度对应约 90.6 mm 的理论上升，因此问题并不是 arm 不够快。

最终 timing 的真实离手预计在 0.661–0.680 s。接近 0.68 s 时，TCP 竖直速度约 1.36 m/s，cube 的理论弹道上升约 94 mm；但当前 arm timeline 随后也让 TCP 继续上升到最终姿态，TCP 自身又上升了接近相同距离。结果是夹爪一直追随 cube，object–gripper 相对分离很小，所以肉眼看起来像直接下落或快速交接。

这也解释了为什么继续提高 G1 speed 或继续微调几毫秒不会从根本上产生明显飞行。明显飞行需要改变 release 后的 arm motion，而不是只改变夹爪。

## 6. 当前证据边界

真机已经验证：

- xArm6 反腕 start pose 可达；
- 1× timeline 可以稳定运行；
- 20 ms `servo_j` loop 无丢周期；
- 约 80 ms tracking lag 可重复；
- G1 370 / 520 / 370 可以异步配合 arm；
- 操作者确认最终版本能够完成同夹爪 recatch。

真机尚未验证：

- 明显可见的长时间自由飞行；
- cube 在空中明显旋转；
- global camera 在 detach 后形成有效 observation；
- ballistic tracker / learned residual 实际修改 catch target；
- 修改后的 target 进入真实 `servo_j` command；
- Probe posterior 或 J 在最终真机 baseline 中实际改变动作。

因此这个结果应作为 open-loop real baseline，而不是完整的 learned closed-loop method。相机、Probe、J 和 residual 在 sim handoff 中存在，但没有进入这条最终真机命令。

## 7. 给仿真侧的下一步思路（尚未实施）

当前版本先保持不变。若以后要做肉眼明显的 toss，正确方向是：

1. 保留已经验证的 pre-release upstroke 和离手速度；
2. detach 后让 TCP 立即制动或向下/侧向撤离，不再继续追随 cube；
3. 用实际 release q/dq 建立 ballistic prior；
4. 在 cube 经过 apex 后，把夹爪移动到下降段 intercept；
5. 最后再加入 global D435 的一次或数次 observation 修正 intercept。

基于本次真实 q/dq 做过一个只读的候选计算：若在约 0.64 s 离手，并用约 80 ms 制动，候选 stop 和 intercept 为：

```tex
candidate brake q_stop:
[0.00000, -0.14359, -0.72195, 0.00003, -1.08021, 0.17725] rad

candidate brake TCP:
[0.5457, 0.0000, 0.5625] m

predicted apex separation from stopped hand:
about 37.6 mm

predicted descending intercept:
about 0.223 s after release
host time about 0.863 s
position about [0.5138, 0.0000, 0.5625] m
```

这些数值只是利用真实日志得到的设计线索，没有写入最终脚本，也没有在真机验证。仿真侧可以据此设计 release 后 brake/retract trajectory，再回真机；不要覆盖当前已经成功的 baseline。

一个合理但不必机械化的目标是：让自由飞行达到约 0.15–0.25 s、相对分离达到约一个 cube 边长或更多，并且在下降段接触。是否“明显”仍应结合视频整体判断，不必只依赖单一阈值。

## 8. 交给仿真电脑时必须一起带走的本地内容

本次真机产生的部分代码和输出目前是本地修改或未纳入版本管理。只在另一台电脑执行普通 pull，可能看不到完整实验材料。交接前至少确认以下内容已经随仓库或单独数据包带走：

```tex
docs/REAL_ROBOT_TEST_20260817.md
scripts/22_run_empty_handoff.py
src/xarm6_toss/real_timeline.py
tests/test_real_timeline.py

outputs/g1_partial/20260816_192859.json
outputs/real_empty_handoff/20260817_124951_025x_empty/
outputs/real_empty_handoff/20260817_125810_0p5x_empty/
outputs/real_empty_handoff/20260817_132132_1p0x_empty/
outputs/real_empty_handoff/20260817_132815_1p0x_cube_throw_only/
outputs/real_empty_handoff/20260817_133212_1p0x_cube_toss_catch/
outputs/real_empty_handoff/20260817_133810_1p0x_cube_toss_catch/
outputs/real_empty_handoff/20260817_133932_1p0x_cube_toss_catch/
outputs/real_empty_handoff/20260817_134247_1p0x_cube_toss_catch/

toss_project_sim_handoff/toss_project/real_cube_demo/outputs/probes/
toss_project_sim_handoff/toss_project/real_cube_demo/outputs/probe_comparisons/
```

还应保留以下本地修改：

```tex
toss_project_sim_handoff/toss_project/real_cube_demo/scripts/07_probe_cube.py
toss_project_sim_handoff/toss_project/real_cube_demo/src/real_cube_demo/robot.py
```

## 9. 一句话交接

真机已经得到可重复执行、能够 recatch 的 1× micro-toss baseline；它的主要限制不是 arm speed，而是 release 后 TCP 继续追随 cube，导致相对自由飞行不明显。请在 sim 中保留当前 baseline，并另开 brake/retract 分支设计明显飞行，不要覆盖现有真机成功版本。
