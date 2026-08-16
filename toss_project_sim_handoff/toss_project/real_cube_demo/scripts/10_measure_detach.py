#!/usr/bin/env python3
"""Measure fixed-pose cube release timing with the global RealSense camera."""

import argparse
from datetime import datetime
import json
import time

import cv2
import numpy as np
import pyrealsense2 as rs
import yaml

import _bootstrap  # noqa: F401
from real_cube_demo.config import DEMO_ROOT, load_handoff_plan, load_hardware
from real_cube_demo.robot import PickPlaceRobot


WIDTH = 640
HEIGHT = 480
FPS = 60
PRE_FRAMES = 30
POST_FRAMES = 48
DROP_THRESHOLD_M = 0.005


def observe_cube(
    color: np.ndarray,
    depth: np.ndarray,
    *,
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
    area = float(cv2.contourArea(contour))
    if area < 100.0:
        return None

    moments = cv2.moments(contour)
    u = float(moments["m10"] / moments["m00"])
    v = float(moments["m01"] / moments["m00"])
    x, y, width, height = cv2.boundingRect(contour)

    contour_mask = np.zeros(mask.shape, dtype=np.uint8)
    cv2.drawContours(contour_mask, [contour], -1, 255, -1)
    depth_values = depth[(contour_mask > 0) & (depth > 0)]
    point_base_m = None
    depth_m = None
    if depth_values.size:
        depth_m = float(np.median(depth_values)) * depth_scale_m
        normalized = cv2.undistortPoints(
            np.asarray([[[u, v]]], dtype=float), intrinsic, distortion
        )[0, 0]
        point_camera = np.asarray(
            [normalized[0] * depth_m, normalized[1] * depth_m, depth_m, 1.0]
        )
        point_base_m = [float(value) for value in (base_from_camera @ point_camera)[:3]]

    return {
        "pixel_uv": [u, v],
        "bbox_xywh": [int(x), int(y), int(width), int(height)],
        "area_px": area,
        "depth_m": depth_m,
        "point_base_m": point_base_m,
    }


def find_downward_motion(records: list[dict]) -> tuple[float, int, float] | None:
    baseline_z = [
        record["cube"]["point_base_m"][2]
        for record in records
        if record["time_from_command_s"] < 0.0
        and record["cube"] is not None
        and record["cube"]["point_base_m"] is not None
    ]
    if not baseline_z:
        return None
    stable_z = float(np.median(baseline_z[-12:]))

    for index in range(len(records) - 1):
        current = records[index]
        following = records[index + 1]
        if current["time_from_command_s"] < 0.0:
            continue
        if current["cube"] is None or following["cube"] is None:
            continue
        current_point = current["cube"]["point_base_m"]
        following_point = following["cube"]["point_base_m"]
        if current_point is None or following_point is None:
            continue
        current_drop = stable_z - current_point[2]
        following_drop = stable_z - following_point[2]
        if current_drop >= DROP_THRESHOLD_M and following_drop >= DROP_THRESHOLD_M:
            return stable_z, index, current_drop
    return None


def annotate_frame(frame: np.ndarray, record: dict, event: bool) -> np.ndarray:
    image = frame.copy()
    cube = record["cube"]
    if cube is not None:
        x, y, width, height = cube["bbox_xywh"]
        cv2.rectangle(image, (x, y), (x + width, y + height), (0, 0, 255), 2)
        if cube["point_base_m"] is not None:
            z = cube["point_base_m"][2]
            cv2.putText(
                image,
                f"base z={z:.3f} m",
                (max(0, x - 20), max(20, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 255),
                1,
                cv2.LINE_AA,
            )
    t = record["time_from_command_s"]
    label = f"t={t:+.3f} s from G1 command"
    color = (0, 0, 255) if event else (255, 255, 255)
    cv2.putText(
        image, label, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2, cv2.LINE_AA
    )
    return image


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--held-position", type=float, default=370.0)
    parser.add_argument("--release-position", type=float, default=520.0)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    hardware = load_hardware()
    handoff = load_handoff_plan()
    global_camera = next(
        camera for camera in hardware.cameras if camera.role == "global"
    )
    print(f"global camera: {global_camera.serial}, {WIDTH}x{HEIGHT} at {FPS} Hz")
    print(
        f"sequence: handoff -> hold at {args.held_position:g} -> "
        f"record -> release to {args.release_position:g}"
    )
    if not args.execute:
        print("dry-run only; pass --execute and place a soft mat below the cube")
        return

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = DEMO_ROOT / "outputs" / "detach_trials" / stamp
    output_dir.mkdir(parents=True, exist_ok=True)

    intrinsic_yaml = yaml.safe_load(global_camera.intrinsics_path.read_text())
    extrinsic_yaml = yaml.safe_load(global_camera.extrinsics_path.read_text())
    intrinsic = np.asarray(intrinsic_yaml["K"], dtype=float)
    distortion = np.asarray(intrinsic_yaml["dist"], dtype=float)
    base_from_camera = np.asarray(extrinsic_yaml["X_CammountCam"], dtype=float)

    pipeline = rs.pipeline()
    camera_config = rs.config()
    camera_config.enable_device(global_camera.serial)
    camera_config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
    camera_config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)

    frames: list[dict] = []
    command_host_s = None
    command_return_s = None
    final_gripper_position = None
    with PickPlaceRobot(hardware) as robot:
        robot.prepare_motion()
        try:
            robot.move_joints(handoff.handoff_joint_rad, "detach-test handoff")
            robot.open_gripper()
            input("Hold the cube between the fingers, then press Enter to close: ")
            robot.set_gripper_position(args.held_position)
            input(
                "Remove your hand and put a soft mat directly below the cube, "
                "then press Enter: "
            )

            profile = pipeline.start(camera_config)
            align = rs.align(rs.stream.color)
            depth_scale_m = float(
                profile.get_device().first_depth_sensor().get_depth_scale()
            )
            try:
                for _ in range(30):
                    align.process(pipeline.wait_for_frames())

                print("recording baseline, then releasing automatically")
                for _ in range(PRE_FRAMES):
                    frame_set = align.process(pipeline.wait_for_frames())
                    host_received_s = time.monotonic()
                    color_frame = frame_set.get_color_frame()
                    depth_frame = frame_set.get_depth_frame()
                    frames.append(
                        {
                            "host_received_s": host_received_s,
                            "camera_timestamp_s": color_frame.get_timestamp() / 1000.0,
                            "frame_number": int(color_frame.get_frame_number()),
                            "color": np.asanyarray(color_frame.get_data()).copy(),
                            "depth": np.asanyarray(depth_frame.get_data()).copy(),
                        }
                    )

                command_host_s = time.monotonic()
                robot.command_gripper_position(args.release_position)
                command_return_s = time.monotonic() - command_host_s

                for _ in range(POST_FRAMES):
                    frame_set = align.process(pipeline.wait_for_frames())
                    host_received_s = time.monotonic()
                    color_frame = frame_set.get_color_frame()
                    depth_frame = frame_set.get_depth_frame()
                    frames.append(
                        {
                            "host_received_s": host_received_s,
                            "camera_timestamp_s": color_frame.get_timestamp() / 1000.0,
                            "frame_number": int(color_frame.get_frame_number()),
                            "color": np.asanyarray(color_frame.get_data()).copy(),
                            "depth": np.asanyarray(depth_frame.get_data()).copy(),
                        }
                    )
            finally:
                pipeline.stop()

            final_gripper_position = robot.gripper_position(check_baud=False)
            robot.open_gripper()
        except Exception:
            robot.stop()
            raise

    clock_offset_s = float(
        np.median(
            [
                frame["host_received_s"] - frame["camera_timestamp_s"]
                for frame in frames
            ]
        )
    )
    records = []
    for frame in frames:
        record = {
            "frame_number": frame["frame_number"],
            "camera_timestamp_s": frame["camera_timestamp_s"],
            "time_from_command_s": (
                frame["camera_timestamp_s"] + clock_offset_s - command_host_s
            ),
            "cube": observe_cube(
                frame["color"],
                frame["depth"],
                depth_scale_m=depth_scale_m,
                intrinsic=intrinsic,
                distortion=distortion,
                base_from_camera=base_from_camera,
            ),
        }
        records.append(record)

    event = find_downward_motion(records)
    event_index = event[1] if event is not None else None
    observed_motion_s = records[event_index]["time_from_command_s"] if event else None
    release_estimate_s = None
    baseline_z_m = None
    observed_drop_m = None
    if event is not None:
        baseline_z_m, _, observed_drop_m = event
        release_estimate_s = observed_motion_s - np.sqrt(2.0 * observed_drop_m / 9.81)
        release_estimate_s = float(release_estimate_s)

    video_path = output_dir / "detach_tracking.avi"
    writer = cv2.VideoWriter(
        str(video_path), cv2.VideoWriter_fourcc(*"MJPG"), FPS, (WIDTH, HEIGHT)
    )
    if not writer.isOpened():
        raise RuntimeError("could not open the annotated video writer")
    annotated_frames = []
    try:
        for index, (frame, record) in enumerate(zip(frames, records)):
            annotated = annotate_frame(frame["color"], record, index == event_index)
            annotated_frames.append(annotated)
            writer.write(annotated)
    finally:
        writer.release()

    strip_path = None
    if event_index is not None:
        indices = range(max(0, event_index - 2), min(len(frames), event_index + 4))
        strip = cv2.hconcat([annotated_frames[index] for index in indices])
        strip_path = output_dir / "detach_event_strip.png"
        cv2.imwrite(str(strip_path), strip)

    camera_periods = np.diff([record["camera_timestamp_s"] for record in records])
    result = {
        "global_camera_serial": global_camera.serial,
        "resolution": [WIDTH, HEIGHT],
        "requested_fps": FPS,
        "measured_fps": float(1.0 / np.median(camera_periods)),
        "held_position": args.held_position,
        "release_position": args.release_position,
        "command_return_s": command_return_s,
        "final_gripper_position": final_gripper_position,
        "drop_threshold_m": DROP_THRESHOLD_M,
        "baseline_z_m": baseline_z_m,
        "first_observed_downward_motion_s": observed_motion_s,
        "gravity_corrected_release_estimate_s": release_estimate_s,
        "frames": records,
    }
    result_path = output_dir / "detach_result.json"
    result_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")

    print(f"measured camera rate: {result['measured_fps']:.1f} Hz")
    print(f"G1 command return: {command_return_s:.3f} s")
    if event is None:
        print("no sustained 5 mm downward cube motion was detected; inspect the video")
    else:
        print(f"first observed 5 mm downward motion: {observed_motion_s:.3f} s")
        print(f"gravity-corrected release estimate: {release_estimate_s:.3f} s")
    print(f"saved result: {result_path}")
    print(f"saved video: {video_path}")
    if strip_path is not None:
        print(f"saved event strip: {strip_path}")


if __name__ == "__main__":
    main()
