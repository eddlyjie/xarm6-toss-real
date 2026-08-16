#!/usr/bin/env python3
"""Capture one warmed-up RGB/depth snapshot from each configured camera."""

from datetime import datetime
from pathlib import Path

import cv2

import _bootstrap
from real_cube_demo.config import DEMO_ROOT, load_hardware
from real_cube_demo.realsense import capture_camera


def main() -> None:
    hardware = load_hardware()
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = DEMO_ROOT / "outputs" / "captures" / stamp
    color_paths: list[Path] = []
    for camera in hardware.cameras:
        print(f"capturing {camera.role} camera {camera.serial}")
        color_paths.append(capture_camera(camera, output_dir))

    images = [cv2.imread(str(path)) for path in color_paths]
    preview_path = output_dir / "cameras_color.png"
    cv2.imwrite(str(preview_path), cv2.hconcat(images))
    print(f"saved capture: {output_dir}")
    print(f"combined preview: {preview_path}")


if __name__ == "__main__":
    main()

