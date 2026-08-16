#!/usr/bin/env python3
"""Detect the yellow cube in a saved global-camera capture."""

import argparse
from pathlib import Path

import _bootstrap  # noqa: F401
from real_cube_demo.config import DEMO_ROOT, load_hardware
from real_cube_demo.cube import detect_yellow_cube, save_observation


def latest_capture() -> Path:
    captures = sorted((DEMO_ROOT / "outputs" / "captures").iterdir())
    return captures[-1]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--capture", type=Path)
    args = parser.parse_args()
    capture_dir = args.capture or latest_capture()
    global_camera = next(
        camera for camera in load_hardware().cameras if camera.role == "global"
    )
    observation = detect_yellow_cube(capture_dir, global_camera)
    image_path, json_path = save_observation(capture_dir, observation)
    print(f"cube pixel: {tuple(round(v, 1) for v in observation.pixel_uv)}")
    print(f"cube depth: {observation.depth_m:.3f} m")
    print(
        "cube in robot base: "
        f"{tuple(round(v, 4) for v in observation.point_base_m)} m"
    )
    print(f"annotated image: {image_path}")
    print(f"observation: {json_path}")


if __name__ == "__main__":
    main()

