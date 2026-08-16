"""Simple yellow-cube observation from a saved global-camera capture."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from pathlib import Path

import cv2
import numpy as np
import yaml

from .config import CameraConfig


@dataclass(frozen=True)
class CubeObservation:
    pixel_uv: tuple[float, float]
    bbox_xywh: tuple[int, int, int, int]
    depth_m: float
    point_camera_m: tuple[float, float, float]
    point_base_m: tuple[float, float, float]


def detect_yellow_cube(capture_dir: Path, camera: CameraConfig) -> CubeObservation:
    camera_dir = capture_dir / camera.role
    color = cv2.imread(str(camera_dir / "color.png"))
    depth = np.load(camera_dir / "depth_raw.npy")
    metadata = json.loads((camera_dir / "metadata.json").read_text())

    hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([20, 120, 120]), np.array([40, 255, 255]))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(contour) < 200:
        raise RuntimeError("yellow cube was not found in the global-camera image")

    moments = cv2.moments(contour)
    u = moments["m10"] / moments["m00"]
    v = moments["m01"] / moments["m00"]
    x, y, width, height = cv2.boundingRect(contour)

    contour_mask = np.zeros(mask.shape, dtype=np.uint8)
    cv2.drawContours(contour_mask, [contour], -1, 255, -1)
    depth_values = depth[(contour_mask > 0) & (depth > 0)]
    depth_m = float(np.median(depth_values)) * float(metadata["depth_scale_m"])

    intrinsic = yaml.safe_load(camera.intrinsics_path.read_text())
    K = np.asarray(intrinsic["K"], dtype=float)
    distortion = np.asarray(intrinsic["dist"], dtype=float)
    normalized = cv2.undistortPoints(
        np.asarray([[[u, v]]], dtype=float), K, distortion
    )[0, 0]
    point_camera = np.asarray(
        [normalized[0] * depth_m, normalized[1] * depth_m, depth_m, 1.0]
    )

    extrinsic = yaml.safe_load(camera.extrinsics_path.read_text())
    X_base_camera = np.asarray(extrinsic["X_CammountCam"], dtype=float)
    point_base = X_base_camera @ point_camera
    return CubeObservation(
        pixel_uv=(float(u), float(v)),
        bbox_xywh=(x, y, width, height),
        depth_m=depth_m,
        point_camera_m=tuple(float(value) for value in point_camera[:3]),
        point_base_m=tuple(float(value) for value in point_base[:3]),
    )


def save_observation(
    capture_dir: Path, observation: CubeObservation, camera_role: str = "global"
) -> tuple[Path, Path]:
    camera_dir = capture_dir / camera_role
    color = cv2.imread(str(camera_dir / "color.png"))
    x, y, width, height = observation.bbox_xywh
    u, v = observation.pixel_uv
    cv2.rectangle(color, (x, y), (x + width, y + height), (0, 0, 255), 2)
    cv2.circle(color, (round(u), round(v)), 4, (255, 0, 0), -1)
    bx, by, bz = observation.point_base_m
    cv2.putText(
        color,
        f"base=({bx:.3f}, {by:.3f}, {bz:.3f}) m",
        (max(0, x - 30), min(color.shape[0] - 8, y + height + 24)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 0, 255),
        1,
        cv2.LINE_AA,
    )
    image_path = capture_dir / "cube_detection.png"
    json_path = capture_dir / "cube_detection.json"
    cv2.imwrite(str(image_path), color)
    json_path.write_text(
        json.dumps(asdict(observation), indent=2) + "\n", encoding="utf-8"
    )
    return image_path, json_path

