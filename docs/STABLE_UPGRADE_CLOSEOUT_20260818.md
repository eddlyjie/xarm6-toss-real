# Stable-upgrade closeout — 2026-08-18

## 结论

本轮在不修改 stable-recovered throw reference 的前提下，确认它自然能够产生一个接近 strict
门槛的短腾空：throw-only 达到 0.121 s all-link robot-free、56.9 mm 上升、30.4 mm 最大
hand-relative separation、内部 apex，并在下降段首次重新接触。缺口是目标轴旋转仍只有约 3°，
而所有延迟 close / catch-servo 组合都没有形成 bilateral stable catch。

因此现在应停止 controller timing sweep。当前可以交给真机的仍是已经重复验证的
stable-recovered closed loop；新结果只是证明“动作有足够腾空空间”，不是新的成功抛接版本。
发布状态仍为 `sim_validated_real_unverified`。

## 权威结果

| Run | free-flight | rise | max separation | apex | first recontact | alignment | detach→apex | signed target rotation | stable catch |
|---|---:|---:|---:|---|---|---:|---:|---:|---|
| stable camera closed loop | 0.078 s | 52.9 mm | 14.4 mm | 不在严格区间内部 | 上升段 | 0.987 | 2.13° | 小旋转 | yes |
| `strict_stable_upgrade/throwonly` | 0.121 s | 56.9 mm | 30.4 mm | internal | 下降段，0.741 s | 0.961 | 2.83° | 3.15° | no |
| `strict_stable_upgrade/openloop_soft_close0735` | 0.116 s | 56.8 mm | 28.6 mm | internal | 下降段，0.736 s | 0.961 | 2.83° | 3.02° | no |
| strict v18 throw-only | 1.137 s | 123.2 mm | 大幅离手 | internal | 无 recatch | 0.914 | 4.30° | 6.84° | no |

`openloop_soft_close0735` 在 0.735 s 发出低 stiffness close，首次接触前 cube 垂直速度约
-0.067 m/s，但仍没有首次双侧接触，`catch_stable=false`。它不能作为 catch 成功，也没有必要
录制交付视频。

## 本轮验证过但不保留为 handoff 的方向

- 单纯把 close 从 0.720 s 延后到 0.740 s：能够把首次接触推到下降段，但 cube 已横向离开
  pinch 区域，未形成双侧接触。
- 在 0.720/0.740 s 才启动 arm catch servo：保住约 0.117–0.121 s free-flight，但仍未接住。
- 降低 catch G1 stiffness：降低了接触冲击，没有解决横向 capture 问题。
- 提前加入 J1 lateral reference：污染 release，free-flight 降到 0.053 s，axis alignment 降到
  0.608。
- 更大 partial-open：破坏稳定 release，axis alignment 降到 0.423。
- 对 strict v18 高抛做 lagged online servo：80 ms lag 下 q1 correction 在 apex 后才有效；满足
  1× velocity/acceleration envelope 的 IK/reference 也无法及时到达下降段 intercept。

这些实验输出完整保留在：

```text
outputs/strict_stable_upgrade/
outputs/strict_v20/
```

其中的 `camera_under_tumble_stable_early_lateral_v1.json` 和
`camera_under_tumble_v20_j1_feedforward.json` 是失败实验配置，不是 handoff 配置。

## 当前应交给真机的版本

唯一推荐入口不变：

```bash
cd /home/ubuntu/toss_project/xarm_6
bash sim/scripts/13_run_stable_camera_closed_loop.sh
```

交接说明和数值边界：

```text
docs/STABLE_RECOVERED_HANDOFF_20260818.md
REAL_ROBOT_TEST_20260817.md
```

该版本已经完成三次不同 camera seed 的 sim 重复：physical detach 后用 actual q/dq ballistic
state、third-view measurement 和 bounded catch update，最后形成 bilateral stable hold。Probe/J
会覆盖 nominal timing；spectator 只验收，不进入 policy。真机仍必须重新测 physical detach delay
和约 80 ms arm lag，不能直接照抄 sim 绝对时刻。

## 仓库收束状态

- `sim/scripts/04_native_release_smoke.py` 已恢复到当前 Git HEAD；本轮 controller 特例没有进入
  tracked source。
- stable handoff 的 runner/config/docs 未被覆盖。
- 失败 run 和实验 config 保留用于复盘，没有删除生成物。
- strict success 仍未达到：没有任何新 run 同时满足 ≥12° target-axis tumble、下降段 bilateral
  catch 和 0.5 s stable hold。

## 收束后的结构性复核

后续只做了 release/clearance 结构性复核，没有改变默认 handoff。权威新诊断是：

| Run | free-flight | max separation | apex | alignment | detach→apex | signed target | first recontact | catch |
|---|---:|---:|---|---:|---:|---:|---|---|
| `passive_after_transition_throwonly` | 0.079 s | 25.7 mm | contact 前未到 apex | 0.992 | 2.27° | 2.27° | 上升段 inner knuckle | no |
| `passive_followx_clearz_close0728_track076` | 0.132 s | 24.7 mm | internal | 0.992 | 2.79° | 3.81° | 0.752 s，下降段 base | no |
| `passive_followx_clearz_fullik` | 0.138 s | 35.4 mm | internal | 0.992 | 2.79° | 3.98° | 0.758 s，下降段 base | no |

G1 在 10 ms 主动 open transition 后卸载 sim drive，把 detach cube 横向速度从约
-0.153 m/s 降到 -0.003 m/s，同时保住目标轴对齐。这说明原 stable branch 的大部分横漂来自
持续的开指 drive reaction，而不是 ballistic model。沿 x 跟随、沿 z clearance 已把首次接触
推到下降段，但 cube 仍落到 gripper base；提前 close 或把 control window 延长到 0.760 s 均未
产生 bilateral finger contact。

passive sim drive 不是 G1 真机命令。真机仍只使用 370 → 520 → 370 position mapping；是否存在
等效的 post-open unload 必须根据 position/current 和实际 detach 录像判断。以上 run 只用于下一条
reference 的物理设计，不进入真机默认配置。

## 若后续继续开发

不要继续细扫 close time。下一步应把 full-IK follow 提前写进整条受约束 reference：release 后
先保持 z clearance，同时让 finger midpoint 沿 x 跟随 ballistic cube；下降段再降低 finger center，
避免 gripper base 成为首次接触对象。online servo 只处理 actual-q/dq belief residual。恢复
bilateral stable catch 后，再提高 J5-dominant detach omega，使 detach→apex 从 2.79° 提高到 ≥5°、
整段从 3.98° 提高到 ≥12°。所有候选继续受 1× 真机 envelope 和 80 ms lag 约束。
