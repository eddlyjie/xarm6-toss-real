#!/usr/bin/env python3
"""Generate a cube marker or measure its offline image-plane rotation."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from xarm6_toss.video_angle import (  # noqa: E402
    marker_edge_angle_deg,
    summarize_measurements,
)


def aruco_dictionary(name: str):
    if not name.startswith("DICT_") or not hasattr(cv2.aruco, name):
        raise ValueError(f"unsupported ArUco dictionary: {name}")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def generate_marker(path: Path, dictionary_name: str, marker_id: int, size_px: int) -> None:
    dictionary = aruco_dictionary(dictionary_name)
    if marker_id < 0 or marker_id >= int(dictionary.bytesList.shape[0]):
        raise ValueError("marker id is outside the selected dictionary")
    if size_px < 100:
        raise ValueError("marker size must be at least 100 pixels")
    image = cv2.aruco.generateImageMarker(dictionary, marker_id, size_px)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), image):
        raise RuntimeError(f"failed to write marker: {path}")


def measure_video(args: argparse.Namespace) -> dict:
    capture = cv2.VideoCapture(str(args.video))
    if not capture.isOpened():
        raise RuntimeError(f"cannot open video: {args.video}")
    fps = float(capture.get(cv2.CAP_PROP_FPS))
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if fps <= 0.0 or width <= 0 or height <= 0:
        capture.release()
        raise RuntimeError("video has invalid fps or dimensions")

    dictionary = aruco_dictionary(args.dictionary)
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    args.output_dir.mkdir(parents=True, exist_ok=True)
    annotated_path = args.output_dir / "annotated.mp4"
    writer = cv2.VideoWriter(
        str(annotated_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        (width, height),
    )
    if not writer.isOpened():
        capture.release()
        raise RuntimeError(f"cannot create video: {annotated_path}")

    rows = []
    frame_index = 0
    detected_in_interval = 0
    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            time_s = frame_index / fps
            in_interval = time_s >= args.start_s and (
                args.end_s is None or time_s <= args.end_s
            )
            if in_interval:
                corners, ids, _ = detector.detectMarkers(frame)
                if ids is not None:
                    ids_flat = ids.reshape(-1).tolist()
                    if args.marker_id in ids_flat:
                        marker_index = ids_flat.index(args.marker_id)
                        points = np.asarray(corners[marker_index], dtype=float).reshape(4, 2)
                        angle = marker_edge_angle_deg(points)
                        rows.append(
                            {
                                "frame_index": frame_index,
                                "time_s": time_s,
                                "raw_angle_deg": angle,
                            }
                        )
                        detected_in_interval += 1
                        cv2.aruco.drawDetectedMarkers(
                            frame,
                            [corners[marker_index]],
                            np.asarray([[args.marker_id]], dtype=np.int32),
                        )
                        cv2.putText(
                            frame,
                            f"marker {args.marker_id}: {angle:+.1f} deg",
                            (20, 36),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.8,
                            (0, 255, 0),
                            2,
                            cv2.LINE_AA,
                        )
            writer.write(frame)
            frame_index += 1
    finally:
        capture.release()
        writer.release()

    summary = summarize_measurements(rows)
    summary.update(
        {
            "schema": "xarm6_offline_marker_rotation_v1",
            "video": str(args.video),
            "dictionary": args.dictionary,
            "marker_id": args.marker_id,
            "fps": fps,
            "video_frame_count": frame_index,
            "requested_start_s": args.start_s,
            "requested_end_s": args.end_s,
            "measurement_scope": "2d_image_plane_rotation",
            "annotated_video": str(annotated_path),
        }
    )
    csv_path = args.output_dir / "angle_measurements.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as stream:
        writer_csv = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer_csv.writeheader()
        writer_csv.writerows(rows)
    summary_path = args.output_dir / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)
    marker = subparsers.add_parser("generate-marker")
    marker.add_argument("--output", type=Path, required=True)
    marker.add_argument("--dictionary", default="DICT_4X4_50")
    marker.add_argument("--marker-id", type=int, default=0)
    marker.add_argument("--size-px", type=int, default=600)
    measure = subparsers.add_parser("measure")
    measure.add_argument("--video", type=Path, required=True)
    measure.add_argument("--output-dir", type=Path, required=True)
    measure.add_argument("--dictionary", default="DICT_4X4_50")
    measure.add_argument("--marker-id", type=int, default=0)
    measure.add_argument("--start-s", type=float, default=0.0)
    measure.add_argument("--end-s", type=float, default=None)
    args = parser.parse_args()

    if args.command == "generate-marker":
        generate_marker(args.output, args.dictionary, args.marker_id, args.size_px)
        print(f"wrote marker: {args.output}")
        return 0
    summary = measure_video(args)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
