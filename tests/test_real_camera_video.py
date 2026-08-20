from pathlib import Path
import sys
import types

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
REAL_DEMO_SRC = (
    ROOT
    / "toss_project_sim_handoff"
    / "toss_project"
    / "real_cube_demo"
    / "src"
)
sys.path.insert(0, str(REAL_DEMO_SRC))
sys.modules.setdefault("pyrealsense2", types.ModuleType("pyrealsense2"))

from real_cube_demo.realsense import write_color_video


def test_real_camera_writer_creates_quarter_speed_review_video(tmp_path):
    width = 64
    height = 48
    source_fps = 60.0
    frames = []
    for index in range(20):
        frame = np.zeros((height, width, 3), dtype=np.uint8)
        frame[:, :, 1] = 5 * index
        cv2.putText(
            frame,
            str(index),
            (4, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
        )
        frames.append(frame)

    raw = write_color_video(
        tmp_path / "global_color.avi",
        frames,
        fps=source_fps,
        width=width,
        height=height,
    )
    slow = write_color_video(
        tmp_path / "global_color_slow_0p25x.avi",
        frames,
        fps=0.25 * source_fps,
        width=width,
        height=height,
    )

    assert raw["frame_count"] == slow["frame_count"] == 20
    assert slow["duration_s"] == 4.0 * raw["duration_s"]
    capture = cv2.VideoCapture(str(tmp_path / slow["path"]))
    try:
        assert int(capture.get(cv2.CAP_PROP_FRAME_COUNT)) == 20
        assert abs(
            capture.get(cv2.CAP_PROP_FPS) - 15.0
        ) < 0.1
    finally:
        capture.release()
