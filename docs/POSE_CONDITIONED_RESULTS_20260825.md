# Object/pose-conditioned warm-start results - 2026-08-25

## Implemented method

Input: object dimensions, mass, principal box inertia, grip width, and desired
rotation angle. The supervised response model maps desired angle to a continuous
action strength `alpha`. It then interpolates validated low/high 20 ms
J2/J3/J5 q/dq/ddq commands at every control tick. J1/J4/J6 remain fixed.

Direct interpolation of the five coarse quintic phases was rejected because it
exceeded the real handoff speed and acceleration envelope. Tick-space
interpolation is a convex combination of two executable references and passes
the same mechanical limits before Sim execution.

## New continuous-pose results

| Object | Training actions | Requested | Action alpha | Measured | Error | Free flight | Catch |
|---|---|---:|---:|---:|---:|---:|---|
| O1 cuboid30, 20 g | low/high | 5.5 deg | 0.704 | 5.705 deg | 0.205 deg | 0.193 s | stable bilateral |
| O2 cuboid33, 26.6 g, first trial | low/high | 5.5 deg | 0.483 | 4.776 deg | 0.724 deg | 0.209 s | stable bilateral |
| O2 cuboid33, 26.6 g, corrected | low/first/high | 5.5 deg | 0.707 | 5.621 deg | 0.121 deg | 0.190 s | stable bilateral |
| O3 cuboid38, 37 g, first trial | low/high | 5.5 deg | 0.449 | 5.109 deg | 0.391 deg | 0.173 s | stable bilateral |
| O3 cuboid38, 37 g, corrected | low/first/high | 5.5 deg | 0.573 | 5.582 deg | 0.082 deg | 0.189 s | stable bilateral |

O2 demonstrates the intended offline learning loop: execute a safe proposal in
Sim, retain the stable measured response, update the piecewise response, and
generate a corrected action. No joint trajectory was manually edited between
the first and corrected O2 trials.

## Evidence and handoff

- O1 result: `outputs/pose_conditioned_cuboid30_5p5deg_v2/`
- O2 first result: `outputs/pose_conditioned_cuboid33_5p5deg_v1/`
- O2 corrected result: `outputs/pose_conditioned_cuboid33_5p5deg_v2/`
- O3 corrected result: `outputs/pose_conditioned_cuboid38_5p5deg_v2/`
- O1 handoff: `real_handoff/cuboid30/pose_conditioned_5p5deg/`
- O2 handoff: `real_handoff/cuboid33/pose_conditioned_5p5deg/`
- O3 handoff: `real_handoff/cuboid38/pose_conditioned_5p5deg/`
- Candidate builder: `sim/tools/build_object_pose_conditioned_candidate.py`
- Generic Sim runner: `sim/scripts/16_run_pose_conditioned_candidate.sh`

Both handoff profiles are plan-only/empty-arm until real G1 integer positions
are measured onsite. The Sim drive-radian values are not real G1 positions.

## Evidence boundary and next experiment

These are seen-object pose-conditioning results. They do not establish formal
unseen-object generalization or RL performance. Repeat statistics and fair M0
fixed replay / M1 inertia scaling / M2 discrete search / M3 continuous policy
comparisons remain the next Sim work.
