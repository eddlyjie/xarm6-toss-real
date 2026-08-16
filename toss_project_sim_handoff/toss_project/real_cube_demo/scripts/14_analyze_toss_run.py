#!/usr/bin/env python3
"""Measure cube motion relative to the real TCP from a recorded toss run."""

import argparse
import csv
import json
from pathlib import Path

import cv2
import numpy as np
import yaml

import _bootstrap  # noqa: F401
from real_cube_demo.config import DEMO_ROOT, load_hardware
from real_cube_demo.robot import PickPlaceRobot
from real_cube_demo.spin_toss import pose_matrix


def observe_cube(
    color: np.ndarray,
    depth: np.ndarray,
    depth_scale_m: float,
    intrinsic: np.ndarray,
    distortion: np.ndarray,
    base_from_camera: np.ndarray,
) -> dict | None:
    hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([20, 120, 120]), np.array([40, 255, 255]))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(contour) < 80.0:
        return None
    moments = cv2.moments(contour)
    u = float(moments["m10"] / moments["m00"])
    v = float(moments["m01"] / moments["m00"])
    contour_mask = np.zeros(mask.shape, dtype=np.uint8)
    cv2.drawContours(contour_mask, [contour], -1, 255, -1)
    depth_values = depth[(contour_mask > 0) & (depth > 0)]
    if not depth_values.size:
        return None
    depth_m = float(np.median(depth_values)) * depth_scale_m
    normalized = cv2.undistortPoints(
        np.asarray([[[u, v]]], dtype=float), intrinsic, distortion
    )[0, 0]
    point_camera = np.asarray(
        [normalized[0] * depth_m, normalized[1] * depth_m, depth_m, 1.0]
    )
    return {
        "pixel_uv": [u, v],
        "point_base_m": (base_from_camera @ point_camera)[:3],
        "area_px": float(cv2.contourArea(contour)),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", type=Path)
    args = parser.parse_args()
    run_dir = args.run or sorted((DEMO_ROOT / "outputs" / "spin_toss").glob("*_cube"))[-1]

    summary = json.loads((run_dir / "summary.json").read_text())
    frame_metadata = json.loads((run_dir / "global_camera.json").read_text())
    signal_rows = list(csv.DictReader((run_dir / "signals.csv").open()))
    signal_times = np.asarray([float(row["time_s"]) for row in signal_rows])
    measured_joint = np.asarray(
        [
            [float(row[f"joint_position_rad_{joint}"]) for joint in range(1, 7)]
            for row in signal_rows
        ]
    )

    hardware = load_hardware()
    camera = next(value for value in hardware.cameras if value.role == "global")
    intrinsic_yaml = yaml.safe_load(camera.intrinsics_path.read_text())
    extrinsic_yaml = yaml.safe_load(camera.extrinsics_path.read_text())
    intrinsic = np.asarray(intrinsic_yaml["K"], dtype=float)
    distortion = np.asarray(intrinsic_yaml["dist"], dtype=float)
    base_from_camera = np.asarray(extrinsic_yaml["X_CammountCam"], dtype=float)
    depth_frames = np.load(run_dir / "global_depth_raw.npz")["depth"]

    video = cv2.VideoCapture(str(run_dir / "global_color.avi"))
    color_frames = []
    while True:
        ok, frame = video.read()
        if not ok:
            break
        color_frames.append(frame)
    video.release()

    records = []
    with PickPlaceRobot(hardware) as robot:
        for index, metadata in enumerate(frame_metadata["frames"]):
            time_s = float(metadata["time_from_trajectory_start_s"])
            if time_s < signal_times[0] or time_s > signal_times[-1]:
                continue
            observation = observe_cube(
                color_frames[index],
                depth_frames[index],
                float(frame_metadata["depth_scale_m"]),
                intrinsic,
                distortion,
                base_from_camera,
            )
            if observation is None:
                continue
            joint = tuple(
                float(np.interp(time_s, signal_times, measured_joint[:, axis]))
                for axis in range(6)
            )
            tcp_transform = pose_matrix(robot.forward_kinematics(joint))
            relative_base = observation["point_base_m"] - tcp_transform[:3, 3]
            relative_tool = tcp_transform[:3, :3].T @ relative_base
            records.append(
                {
                    "time_s": time_s,
                    "frame_number": metadata["frame_number"],
                    "pixel_uv": observation["pixel_uv"],
                    "area_px": observation["area_px"],
                    "relative_tool_m": relative_tool.tolist(),
                    "tcp_rotation": tcp_transform[:3, :3].tolist(),
                    "tcp_translation_m": tcp_transform[:3, 3].tolist(),
                }
            )

    baseline_values = np.asarray(
        [record["relative_tool_m"] for record in records if 0.30 <= record["time_s"] <= 0.58]
    )
    baseline = np.median(baseline_values, axis=0)
    camera_from_base = np.linalg.inv(base_from_camera)
    for record in records:
        record["relative_change_m"] = float(
            np.linalg.norm(np.asarray(record["relative_tool_m"]) - baseline)
        )
        attached_point_base = (
            np.asarray(record["tcp_rotation"]) @ baseline
            + np.asarray(record["tcp_translation_m"])
        )
        attached_point_camera = camera_from_base @ np.append(attached_point_base, 1.0)
        projected, _ = cv2.projectPoints(
            attached_point_camera[:3].reshape(1, 1, 3),
            np.zeros(3),
            np.zeros(3),
            intrinsic,
            distortion,
        )
        attached_uv = projected[0, 0]
        record["attached_prediction_uv"] = attached_uv.tolist()
        record["pixel_separation_from_attached"] = float(
            np.linalg.norm(np.asarray(record["pixel_uv"]) - attached_uv)
        )

    result = {
        "run": str(run_dir),
        "planned_physical_release_s": summary["physical_release_time_s"],
        "planned_physical_catch_s": summary["physical_catch_time_s"],
        "baseline_relative_tool_m": baseline.tolist(),
        "records": records,
    }
    output_path = run_dir / "cube_motion_analysis.json"
    output_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")

    print(f"saved analysis: {output_path}")
    print("time_s  relative_change_mm  pixel_from_attached  cube_pixel_uv")
    for record in records:
        if 0.55 <= record["time_s"] <= 1.02:
            print(
                f"{record['time_s']:.3f}  "
                f"{1000.0 * record['relative_change_m']:.1f}  "
                f"{record['pixel_separation_from_attached']:.1f}  "
                f"{[round(value, 1) for value in record['pixel_uv']]}"
            )


if __name__ == "__main__":
    main()
