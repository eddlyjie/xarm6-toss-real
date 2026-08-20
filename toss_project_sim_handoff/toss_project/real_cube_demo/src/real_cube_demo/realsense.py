"""Sequential RGB/depth snapshot capture for the two RealSense cameras."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from pathlib import Path
import threading
import time
from typing import Any

import cv2
import numpy as np
import pyrealsense2 as rs

from .config import CameraConfig


@dataclass(frozen=True)
class CaptureMetadata:
    role: str
    serial: str
    depth_scale_m: float
    factory_color_intrinsics: dict[str, Any]
    calibration_intrinsics_yaml: str
    calibration_extrinsics_yaml: str


def write_color_video(
    path: Path,
    frames: list[np.ndarray],
    *,
    fps: float,
    width: int,
    height: int,
) -> dict[str, Any]:
    writer = cv2.VideoWriter(
        str(path),
        cv2.VideoWriter_fourcc(*"MJPG"),
        fps,
        (width, height),
    )
    if not writer.isOpened():
        raise RuntimeError(f"could not open MJPG video writer for {path}")
    try:
        for frame in frames:
            writer.write(frame)
    finally:
        writer.release()
    return {
        "path": path.name,
        "fps": float(fps),
        "frame_count": len(frames),
        "duration_s": len(frames) / float(fps),
    }


class MotionCameraRecorder:
    """Record global-camera RGB/depth without blocking the servo loop."""

    def __init__(
        self,
        camera: CameraConfig,
        *,
        width: int = 640,
        height: int = 480,
        fps: int = 60,
    ):
        self.camera = camera
        self.width = width
        self.height = height
        self.fps = fps
        self.pipeline = rs.pipeline()
        self.align = rs.align(rs.stream.color)
        self.frames: list[dict[str, Any]] = []
        self.depth_scale_m = 0.0
        self._running = threading.Event()
        self._thread: threading.Thread | None = None
        self._error: Exception | None = None

    def start(self) -> None:
        config = rs.config()
        config.enable_device(self.camera.serial)
        config.enable_stream(
            rs.stream.color,
            self.width,
            self.height,
            rs.format.bgr8,
            self.fps,
        )
        config.enable_stream(
            rs.stream.depth,
            self.width,
            self.height,
            rs.format.z16,
            self.fps,
        )
        profile = self.pipeline.start(config)
        self.depth_scale_m = float(
            profile.get_device().first_depth_sensor().get_depth_scale()
        )
        for _ in range(30):
            self.align.process(self.pipeline.wait_for_frames())
        self._running.set()
        self._thread = threading.Thread(target=self._capture, daemon=True)
        self._thread.start()

    def _capture(self) -> None:
        try:
            while self._running.is_set():
                frame_set = self.align.process(
                    self.pipeline.wait_for_frames(timeout_ms=100)
                )
                color_frame = frame_set.get_color_frame()
                depth_frame = frame_set.get_depth_frame()
                self.frames.append(
                    {
                        "host_received_s": time.monotonic(),
                        "camera_timestamp_s": color_frame.get_timestamp() / 1000.0,
                        "frame_number": int(color_frame.get_frame_number()),
                        "color": np.asanyarray(color_frame.get_data()).copy(),
                        "depth": np.asanyarray(depth_frame.get_data()).copy(),
                    }
                )
        except Exception as error:
            if self._running.is_set():
                self._error = error

    def stop(self) -> None:
        self._running.clear()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self.pipeline.stop()
        if self._error is not None:
            raise self._error

    def save(self, output_dir: Path, trajectory_start_host_s: float) -> dict[str, Any]:
        color_frames = [frame["color"] for frame in self.frames]
        raw_video = write_color_video(
            output_dir / "global_color.avi",
            color_frames,
            fps=self.fps,
            width=self.width,
            height=self.height,
        )
        slow_video = write_color_video(
            output_dir / "global_color_slow_0p25x.avi",
            color_frames,
            fps=0.25 * self.fps,
            width=self.width,
            height=self.height,
        )

        depth_path = output_dir / "global_depth_raw.npz"
        np.savez_compressed(
            depth_path,
            depth=np.stack([frame["depth"] for frame in self.frames]),
        )
        frame_records = [
            {
                "frame_number": frame["frame_number"],
                "camera_timestamp_s": frame["camera_timestamp_s"],
                "host_received_s": frame["host_received_s"],
                "time_from_trajectory_start_s": (
                    frame["host_received_s"] - trajectory_start_host_s
                ),
            }
            for frame in self.frames
        ]
        metadata = {
            "role": self.camera.role,
            "serial": self.camera.serial,
            "resolution": [self.width, self.height],
            "requested_fps": self.fps,
            "depth_scale_m": self.depth_scale_m,
            "frame_count": len(self.frames),
            "frames": frame_records,
            "color_videos": {
                "raw": raw_video,
                "slow_0p25x": slow_video,
            },
        }
        (output_dir / "global_camera.json").write_text(
            json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
        )
        return metadata


def connected_devices() -> list[dict[str, str]]:
    devices = []
    for device in rs.context().query_devices():
        devices.append(
            {
                "name": device.get_info(rs.camera_info.name),
                "serial": device.get_info(rs.camera_info.serial_number),
                "firmware": device.get_info(rs.camera_info.firmware_version),
            }
        )
    return devices


def capture_camera(camera: CameraConfig, output_dir: Path) -> Path:
    camera_dir = output_dir / camera.role
    camera_dir.mkdir(parents=True, exist_ok=True)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(camera.serial)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    profile = pipeline.start(config)
    align = rs.align(rs.stream.color)
    try:
        frames = None
        for _ in range(20):
            frames = align.process(pipeline.wait_for_frames())
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        color = np.asanyarray(color_frame.get_data())
        depth = np.asanyarray(depth_frame.get_data())

        color_profile = color_frame.profile.as_video_stream_profile()
        intr = color_profile.get_intrinsics()
        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        metadata = CaptureMetadata(
            role=camera.role,
            serial=camera.serial,
            depth_scale_m=float(depth_scale),
            factory_color_intrinsics={
                "width": intr.width,
                "height": intr.height,
                "fx": intr.fx,
                "fy": intr.fy,
                "ppx": intr.ppx,
                "ppy": intr.ppy,
                "model": str(intr.model),
                "coeffs": list(intr.coeffs),
            },
            calibration_intrinsics_yaml=str(camera.intrinsics_path),
            calibration_extrinsics_yaml=str(camera.extrinsics_path),
        )

        cv2.imwrite(str(camera_dir / "color.png"), color)
        np.save(camera_dir / "depth_raw.npy", depth)
        depth_vis = cv2.applyColorMap(
            cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET
        )
        cv2.imwrite(str(camera_dir / "depth.png"), depth_vis)
        (camera_dir / "metadata.json").write_text(
            json.dumps(asdict(metadata), indent=2) + "\n", encoding="utf-8"
        )
    finally:
        pipeline.stop()
    return camera_dir / "color.png"
