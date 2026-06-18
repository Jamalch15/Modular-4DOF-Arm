from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from time import sleep

import cv2


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from app.apriltag_calibration import (
    AprilTagCalibrationSession,
    annotate_apriltag_frame,
    configured_tag_ids,
    detect_apriltags,
)
from app.config import ensure_local_config, load_config, save_calibration_updates
from app.demo_settings import camera_settings


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Collect and verify the fixed workspace AprilTag camera pose.")
    parser.add_argument("--config", type=Path, help="Optional robot YAML path. Defaults to the normal local config.")
    parser.add_argument("--source", type=int, help="Override camera source index.")
    parser.add_argument("--frames", type=int, default=20, help="Frames to accumulate.")
    parser.add_argument("--interval-ms", type=int, default=80, help="Delay between samples.")
    parser.add_argument("--image", type=Path, help="Use one still image instead of the webcam.")
    parser.add_argument("--show", action="store_true", help="Show the final annotated frame.")
    parser.add_argument("--save", action="store_true", help="Save an accepted all-tag pose into robot.local.yaml.")
    parser.add_argument("--json", action="store_true", help="Print full JSON result.")
    return parser.parse_args()


def open_camera(camera: dict) -> cv2.VideoCapture:
    capture = cv2.VideoCapture(int(camera.get("source_index", 0)))
    resolution = camera.get("resolution") or {}
    if int(resolution.get("width", 0) or 0) > 0:
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, int(resolution["width"]))
    if int(resolution.get("height", 0) or 0) > 0:
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, int(resolution["height"]))
    if not capture.isOpened():
        capture.release()
        raise RuntimeError("could not open camera")
    return capture


def main() -> int:
    args = parse_args()
    config = load_config(args.config) if args.config else load_config()
    camera = camera_settings(config)
    if args.source is not None:
        camera["source_index"] = args.source
    session = AprilTagCalibrationSession(camera)
    latest_image = None
    latest_detections = []

    if args.image:
        latest_image = cv2.imread(str(args.image))
        if latest_image is None:
            raise RuntimeError(f"could not read image: {args.image}")
        latest_detections = detect_apriltags(latest_image, session.settings)
        session.add(latest_detections)
    else:
        capture = open_camera(camera)
        try:
            for index in range(max(1, args.frames)):
                ok, image = capture.read()
                if not ok:
                    raise RuntimeError(f"camera read failed at sample {index + 1}")
                detections = detect_apriltags(image, session.settings)
                session.add(detections)
                latest_image = image
                latest_detections = detections
                print(
                    f"\rsample {index + 1}/{max(1, args.frames)} "
                    f"visible={[detection.tag_id for detection in detections]} "
                    f"session={session.summary()['tag_ids']}",
                    end="",
                    flush=True,
                )
                if index + 1 < args.frames:
                    sleep(max(0, args.interval_ms) / 1000.0)
            print()
        finally:
            capture.release()

    result = session.solve()
    metrics = result.get("metrics") or {}
    position = (result.get("camera_to_robot") or {}).get("position_mm")
    print(f"accepted: {result.get('accepted', False)}")
    print(f"tags used: {result.get('tags_used', [])}")
    print(f"frames used: {result.get('frames_used', 0)}")
    print(f"camera position mm: {position or '-'}")
    print(f"reprojection RMSE px: {metrics.get('reprojection_rmse_px', '-')}")
    print(f"confidence: {metrics.get('confidence', '-')}")
    if result.get("error"):
        print(f"message: {result['error']}")
    if args.json:
        print(json.dumps(result, indent=2))

    if latest_image is not None:
        annotated = annotate_apriltag_frame(latest_image, latest_detections, camera, result)
        if args.show:
            cv2.imshow("AprilTag workspace calibration", annotated)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

    if args.save:
        required = {int(value) for value in session.settings.get("required_ids", [])}
        missing = sorted(required - set(result.get("tags_used", [])))
        if missing:
            print(f"not saved: missing required tags {missing}", file=sys.stderr)
            return 2
        if not result.get("minimum_samples_met"):
            print("not saved: minimum frame count not met", file=sys.stderr)
            return 2
        if not result.get("required_tag_samples_met"):
            counts = result.get("tag_observation_counts", {})
            minimum = session.summary().get("minimum_samples", 0)
            print(
                f"not saved: each required tag needs {minimum} observations; counts={counts}",
                file=sys.stderr,
            )
            return 2
        if not result.get("accepted"):
            print("not saved: pose did not pass quality checks", file=sys.stderr)
            return 2
        calibration = camera.setdefault("calibration", {})
        april_tag = calibration.setdefault("apriltag", {})
        april_tag["result"] = result
        target = args.config or ensure_local_config()
        save_calibration_updates(target, {"camera": camera})
        print(f"saved: {target}")

    unknown = sorted(
        detection.tag_id
        for detection in latest_detections
        if detection.tag_id not in configured_tag_ids(session.settings)
    )
    if unknown:
        print(f"warning: unknown tags visible {unknown}")
    return 0 if result.get("accepted") else 1


if __name__ == "__main__":
    raise SystemExit(main())
