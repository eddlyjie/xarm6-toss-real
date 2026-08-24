# Four-object xArm6 Sim handoff — 2026-08-24

## Result summary

| Object | Profile | Measured target-axis rotation | Continuous free flight | Axis alignment | Stable bilateral recatch | Real arm envelope |
|---|---|---:|---:|---:|---|---|
| O0 38 mm cube, 8 g | low | 4.59° | 0.178 s | 0.969 | yes | pass |
| O0 38 mm cube, 8 g | medium | 6.48° | 0.219 s | 0.901 | yes | pass |
| O0 38 mm cube, 8 g | high | 7.87° | 0.266 s | 0.901 | yes | pass |
| O1 44.5×46×30 mm, 20 g | low | 2.96° | 0.150 s | 0.975 | yes | pass |
| O1 44.5×46×30 mm, 20 g | high | 6.57° | 0.222 s | 0.942 | yes | pass |
| O2 50.5×51×33.5 mm, 26.6 g | low | 4.61° | 0.172 s | 0.956 | yes | pass |
| O2 50.5×51×33.5 mm, 26.6 g | high | 6.45° | 0.218 s | 0.923 | yes | pass |
| O3 57.5×58×38 mm, 37 g | low | 4.40° | 0.148 s | 0.969 | yes | pass |
| O3 57.5×58×38 mm, 37 g | high | 6.85° | 0.232 s | 0.935 | yes | pass |

All profiles dynamically command J2/J3/J5 and hold J1/J4/J6 fixed. O1–O3
high profiles pass the obvious-toss condition in addition to stable recatch.

## Evidence and videos

Selected normal-speed videos are in `docs/media/four_object_open_loop/` and
follow the table order. Full outputs contain `summary.json`, `trajectory.json`,
normal/third/wrist videos, and run logs:

```text
outputs/cube8g_smalltier_r10cfm_close081_v44
outputs/cube8g_smalltier_r10cfh_close086_v47
outputs/cube8g_stock_g1_10deg_j235_centered_v13_export
outputs/cuboid30_20g_low_v4
outputs/cuboid30_20g_medium_v1
outputs/cuboid33_26p6g_low_v1
outputs/cuboid33_26p6g_medium_v1
outputs/cuboid38_37g_low_v1
outputs/cuboid38_37g_medium_v1
```

Each corresponding handoff directory contains a 92-sample, 20 ms arm
timeline and a plan-only JSON. O1–O3 use G1 templates with `null` positions;
their profiles permit empty-arm preview only until onsite calibration.

## Scientific boundary

The O1–O3 results reuse the O0 low/high J2/J3/J5 references with
object-specific geometry, mass, grip width, release preload and catch aperture.
They establish a useful M0 fixed-replay transfer baseline and viable demo
trajectories. They do not by themselves establish the final learned
object/pose-conditioned M3 claim. The next method experiment must compare:

- M0: fixed O0 replay;
- M1: analytic mass/inertia scaling;
- M2: parameter search without learned Detach/J proposal;
- M3: object/pose-conditioned proposal with Detach prediction and J ranking.

Use identical object sets, desired angles and Sim randomization for all four.
Report catch rate and angle error over repeats; do not turn the nine selected
single runs into a robustness claim.

## Onsite sequence

1. Re-measure each object and mark fixed orientation/grasp depth.
2. Calibrate held, release, preclose and close G1 integer positions.
3. Fill both low/high `g1_schedule.template.json` and profile event positions.
4. Enable only `empty_g1`; run empty arm at 0.25×, 0.5× and 1.0× first.
5. After empty-G1 passes, enable and run soft-mat throw-only.
6. Enable guarded object recatch only after the earlier stages pass.
7. Record at least five object trials per reported profile and retain every run.

The Sim drive radians used during development are not real G1 position values.
