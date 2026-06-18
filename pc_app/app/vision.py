from __future__ import annotations

import base64
from typing import Any

import cv2
import numpy as np

from .apriltag_calibration import project_image_point_to_plane


def planar_transform_from_points(
    image_points: list[list[float]],
    robot_points: list[list[float]],
) -> np.ndarray:
    if len(image_points) != 4 or len(robot_points) != 4:
        raise ValueError("planar calibration requires exactly four image and robot points")
    src = np.array(image_points, dtype=np.float32)
    dst = np.array(robot_points, dtype=np.float32)
    return cv2.getPerspectiveTransform(src, dst)


def apply_planar_transform(transform: np.ndarray, image_point: list[float]) -> dict[str, float]:
    point = np.array([[[float(image_point[0]), float(image_point[1])]]], dtype=np.float32)
    mapped = cv2.perspectiveTransform(point, transform)[0][0]
    return {"x_mm": float(mapped[0]), "y_mm": float(mapped[1])}


def decode_image_b64(image_b64: str) -> np.ndarray:
    payload = image_b64.split(",", 1)[-1]
    raw = base64.b64decode(payload)
    image = cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("could not decode image")
    return image


def encode_image_b64(image_bgr: np.ndarray, ext: str = ".jpg") -> str:
    ok, encoded = cv2.imencode(ext, image_bgr)
    if not ok:
        raise ValueError("could not encode image")
    mime = "image/png" if ext.lower() == ".png" else "image/jpeg"
    payload = base64.b64encode(encoded.tobytes()).decode("ascii")
    return f"data:{mime};base64,{payload}"


def detect_color_blob(image_bgr: np.ndarray, profile: dict[str, Any]) -> dict[str, Any]:
    hsv_min = np.array(profile.get("hsv_min", [0, 0, 0]), dtype=np.uint8)
    hsv_max = np.array(profile.get("hsv_max", [179, 255, 255]), dtype=np.uint8)
    min_area = float(profile.get("min_area_px", 200.0))
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, hsv_min, hsv_max)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return {"ok": False, "reason": "no contour", "area_px": 0.0}
    contour = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(contour))
    if area < min_area:
        return {"ok": False, "reason": "below min area", "area_px": area}
    moments = cv2.moments(contour)
    if abs(moments["m00"]) < 1e-9:
        return {"ok": False, "reason": "empty contour moment", "area_px": area}
    cx = float(moments["m10"] / moments["m00"])
    cy = float(moments["m01"] / moments["m00"])
    x, y, w, h = cv2.boundingRect(contour)
    return {
        "ok": True,
        "center_px": {"x": cx, "y": cy},
        "area_px": area,
        "bbox_px": {"x": int(x), "y": int(y), "w": int(w), "h": int(h)},
    }


def detect_configured_colors(
    image_bgr: np.ndarray,
    profiles: dict[str, dict[str, Any]],
    calibration: dict[str, Any] | None = None,
) -> list[dict[str, Any]]:
    transform = None
    april_tag_result = None
    if calibration:
        image_points = calibration.get("image_points") or []
        robot_points = calibration.get("robot_points") or []
        if len(image_points) == 4 and len(robot_points) == 4:
            transform = planar_transform_from_points(image_points, robot_points)
        april_tag = calibration.get("apriltag") if isinstance(calibration.get("apriltag"), dict) else {}
        saved_result = april_tag.get("result") if isinstance(april_tag.get("result"), dict) else {}
        if saved_result.get("accepted"):
            april_tag_result = saved_result
        elif transform is None:
            planar = saved_result.get("planar") if isinstance(saved_result.get("planar"), dict) else {}
            homography = planar.get("homography_image_to_robot")
            if homography:
                parsed = np.asarray(homography, dtype=np.float64)
                if parsed.shape == (3, 3) and np.all(np.isfinite(parsed)):
                    transform = parsed

    detections: list[dict[str, Any]] = []
    for name, profile in profiles.items():
        if not bool(profile.get("enabled", True)):
            continue
        result = detect_color_blob(image_bgr, profile)
        result["color"] = name
        result["drop_zone"] = profile.get("drop_zone")
        if result.get("ok") and april_tag_result is not None:
            center = result["center_px"]
            try:
                result["robot"] = project_image_point_to_plane(
                    [center["x"], center["y"]],
                    april_tag_result,
                )
                result["coordinate_source"] = "apriltag_camera_pose"
                result["camera_pose_id"] = april_tag_result.get("id")
                result["camera_pose_timestamp"] = april_tag_result.get("timestamp")
                result["projection_quality"] = {
                    "confidence": (april_tag_result.get("metrics") or {}).get("confidence"),
                    "reprojection_rmse_px": (april_tag_result.get("metrics") or {}).get(
                        "reprojection_rmse_px"
                    ),
                }
            except ValueError as exc:
                result["projection_error"] = str(exc)
        elif result.get("ok") and transform is not None:
            center = result["center_px"]
            robot_xy = apply_planar_transform(transform, [center["x"], center["y"]])
            result["robot"] = robot_xy
            result["coordinate_source"] = "planar_homography"
        detections.append(result)
    return detections


def annotated_detection_frame(
    image_bgr: np.ndarray,
    detections: list[dict[str, Any]],
) -> np.ndarray:
    annotated = image_bgr.copy()
    palette = {
        "red": (30, 30, 230),
        "blue": (230, 90, 30),
        "green": (70, 200, 70),
        "yellow": (0, 220, 230),
    }
    for detection in detections:
        if not detection.get("ok"):
            continue
        color_name = str(detection.get("color", "object"))
        color = palette.get(color_name, (230, 230, 230))
        bbox = detection.get("bbox_px") or {}
        center = detection.get("center_px") or {}
        x = int(bbox.get("x", center.get("x", 0)))
        y = int(bbox.get("y", center.get("y", 0)))
        w = int(bbox.get("w", 0))
        h = int(bbox.get("h", 0))
        cx = int(center.get("x", x + w / 2))
        cy = int(center.get("y", y + h / 2))
        if w > 0 and h > 0:
            cv2.rectangle(annotated, (x, y), (x + w, y + h), color, 2)
        cv2.circle(annotated, (cx, cy), 4, color, -1)
        robot = detection.get("robot") or {}
        label = f"{color_name} px({cx},{cy})"
        if robot:
            label += f" r({robot.get('x_mm', 0):.0f},{robot.get('y_mm', 0):.0f})"
        cv2.putText(
            annotated,
            label,
            (max(0, x), max(16, y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            color,
            1,
            cv2.LINE_AA,
        )
    return annotated
