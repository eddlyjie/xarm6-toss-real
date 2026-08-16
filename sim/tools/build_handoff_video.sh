#!/usr/bin/env bash
set -euo pipefail

TASK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$TASK_ROOT"

OUTPUT="real_handoff/videos/xarm6_toss_catch_3_trials_8x_slow_zoom.mp4"
mkdir -p "$(dirname "$OUTPUT")"

FILTER_COMPLEX="[0:v]setpts=8*PTS,split=2[f0][c0];[f0]drawbox=x=240:y=360:w=160:h=120:color=red:t=3,drawtext=text='TRIAL 1 - 35 g - 8x slow':x=18:y=18:fontsize=24:fontcolor=black:box=1:boxcolor=white@0.8[F0];[c0]crop=160:120:240:360,scale=640:480:flags=lanczos,drawtext=text='ROI zoom':x=18:y=18:fontsize=24:fontcolor=black:box=1:boxcolor=white@0.8[C0];[F0][C0]hstack=2[T0];[1:v]setpts=8*PTS,split=2[f1][c1];[f1]drawbox=x=240:y=360:w=160:h=120:color=red:t=3,drawtext=text='TRIAL 2 - 25 g - 8x slow':x=18:y=18:fontsize=24:fontcolor=black:box=1:boxcolor=white@0.8[F1];[c1]crop=160:120:240:360,scale=640:480:flags=lanczos,drawtext=text='ROI zoom':x=18:y=18:fontsize=24:fontcolor=black:box=1:boxcolor=white@0.8[C1];[F1][C1]hstack=2[T1];[2:v]setpts=8*PTS,split=2[f2][c2];[f2]drawbox=x=240:y=360:w=160:h=120:color=red:t=3,drawtext=text='TRIAL 3 - 45 g - 8x slow':x=18:y=18:fontsize=24:fontcolor=black:box=1:boxcolor=white@0.8[F2];[c2]crop=160:120:240:360,scale=640:480:flags=lanczos,drawtext=text='ROI zoom':x=18:y=18:fontsize=24:fontcolor=black:box=1:boxcolor=white@0.8[C2];[F2][C2]hstack=2[T2];[T0][T1][T2]concat=n=3:v=1:a=0,format=yuv420p[out]"

ffmpeg -y -v error \
  -i sim/outputs/real_candidate_learned_3/trial_01.mp4 \
  -i sim/outputs/real_candidate_learned_3/trial_02.mp4 \
  -i sim/outputs/real_candidate_learned_3/trial_03.mp4 \
  -filter_complex "$FILTER_COMPLEX" \
  -map "[out]" -r 60 -c:v libx264 -crf 18 -preset medium -movflags +faststart \
  "$OUTPUT"

ffprobe -v error \
  -show_entries stream=width,height,r_frame_rate,nb_frames:format=duration,size \
  -of default=noprint_wrappers=1 "$OUTPUT"
