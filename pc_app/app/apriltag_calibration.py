from __future__ import annotations

from dataclasses import dataclass
import json
from math import acos, atan2, degrees, hypot
from time import time
from typing import Any
from uuid import uuid4

import cv2
import numpy as np


APRILTAG_DICTIONARIES = {
    "DICT_APRILTAG_16H5": cv2.aruco.DICT_APRILTAG_16H5,
    "DICT_APRILTAG_25H9": cv2.aruco.DICT_APRILTAG_25H9,
    "DICT_APRILTAG_36H10": cv2.aruco.DICT_APRILTAG_36H10,
    "DICT_APRILTAG_36H11": cv2.aruco.DICT_APRILTAG_36H11,
}

TAG_LOCAL_CORNERS = {
    "top_left": np.array([-1.0, 1.0], dtype=np.float64),
    "top_right": np.array([1.0, 1.0], dtype=np.float64),
    "bottom_right": np.array([1.0, -1.0], dtype=np.float64),
    "bottom_left": np.array([-1.0, -1.0], dtype=np.float64),
}


@dataclass(frozen=True)
class AprilTagDetection:
    tag_id: int
    corners_px: np.ndarray

    @property
    def center_px(self) -> np.ndarray:
        return np.mean(self.corners_px, axis=0)

    def to_dict(self, configured: bool = False) -> dict[str, Any]:
        center = self.center_px
        return {
            "id": self.tag_id,
            "configured": configured,
            "corners_px": [[float(x), float(y)] for x, y in self.corners_px],
            "center_px": {"x": float(center[0]), "y": float(center[1])},
            "area_px": float(abs(cv2.contourArea(self.corners_px.astype(np.float32)))),
            "perimeter_px": float(cv2.arcLength(self.corners_px.astype(np.float32), True)),
        }


def april_tag_settings(camera: dict[str, Any]) -> dict[str, Any]:
    calibration = camera.get("calibration") if isinstance(camera.get("calibration"), dict) else {}
    raw = calibration.get("apriltag") if isinstance(calibration.get("apriltag"), dict) else {}
    defaults = {
        "enabled": True,
        "dictionary": "DICT_APRILTAG_36H11",
        "tag_size_mm": 40.0,
        "required_ids": [0, 1, 2, 3],
        "min_tags_for_pose": 2,
        "min_samples": 12,
        "max_samples": 120,
        "max_reprojection_error_px": 2.5,
        "max_tilt_from_down_deg": 45.0,
        "tags": {
            "0": {
                "workspace_corner_mm": [-239.0, 86.5, 0.0],
                "aligned_tag_corner": "bottom_left",
                "yaw_deg": 0.0,
            },
            "1": {
                "workspace_corner_mm": [239.0, 86.5, 0.0],
                "aligned_tag_corner": "bottom_right",
                "yaw_deg": 0.0,
            },
            "2": {
                "workspace_corner_mm": [239.0, 401.5, 0.0],
                "aligned_tag_corner": "top_right",
                "yaw_deg": 0.0,
            },
            "3": {
                "workspace_corner_mm": [-239.0, 401.5, 0.0],
                "aligned_tag_corner": "top_left",
                "yaw_deg": 0.0,
            },
        },
        "result": None,
    }
    defaults.update(raw)
    return defaults


def camera_intrinsics(camera: dict[str, Any]) -> tuple[np.ndarray | None, np.ndarray, list[str]]:
    raw = camera.get("intrinsics") if isinstance(camera.get("intrinsics"), dict) else {}
    matrix = raw.get("camera_matrix")
    errors: list[str] = []
    camera_matrix: np.ndarray | None = None
    if isinstance(matrix, list):
        parsed = np.asarray(matrix, dtype=np.float64)
        if parsed.shape == (3, 3) and np.all(np.isfinite(parsed)) and parsed[0, 0] > 0 and parsed[1, 1] > 0:
            camera_matrix = parsed
        elif matrix:
            errors.append("camera.intrinsics.camera_matrix must be a valid 3x3 matrix")
    if camera_matrix is None:
        fx = raw.get("fx_px")
        fy = raw.get("fy_px")
        cx = raw.get("cx_px")
        cy = raw.get("cy_px")
        values = [fx, fy, cx, cy]
        if all(isinstance(value, (int, float)) for value in values):
            camera_matrix = np.array(
                [[float(fx), 0.0, float(cx)], [0.0, float(fy), float(cy)], [0.0, 0.0, 1.0]],
                dtype=np.float64,
            )
            if camera_matrix[0, 0] <= 0 or camera_matrix[1, 1] <= 0:
                camera_matrix = None
                errors.append("camera focal lengths must be positive")
    if camera_matrix is None and not errors:
        errors.append("camera intrinsics are missing; configure fx, fy, cx, and cy before solving 6-DoF pose")

    distortion = raw.get("distortion_coefficients", [0.0, 0.0, 0.0, 0.0, 0.0])
    try:
        dist_coeffs = np.asarray(distortion, dtype=np.float64).reshape(-1, 1)
    except (TypeError, ValueError):
        dist_coeffs = np.zeros((5, 1), dtype=np.float64)
        errors.append("camera distortion coefficients are invalid")
    if dist_coeffs.size not in {4, 5, 8, 12, 14} or not np.all(np.isfinite(dist_coeffs)):
        dist_coeffs = np.zeros((5, 1), dtype=np.float64)
        errors.append("camera distortion coefficients must contain 4, 5, 8, 12, or 14 finite values")
    return camera_matrix, dist_coeffs, errors


def detect_apriltags(image_bgr: np.ndarray, settings: dict[str, Any]) -> list[AprilTagDetection]:
    dictionary_name = str(settings.get("dictionary", "DICT_APRILTAG_36H11")).upper()
    dictionary_id = APRILTAG_DICTIONARIES.get(dictionary_name)
    if dictionary_id is None:
        raise ValueError(f"unsupported AprilTag dictionary: {dictionary_name}")
    parameters = cv2.aruco.DetectorParameters()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(dictionary_id), parameters)
    corners, ids, _rejected = detector.detectMarkers(image_bgr)
    if ids is None:
        return []
    return [
        AprilTagDetection(int(tag_id), np.asarray(tag_corners, dtype=np.float64).reshape(4, 2))
        for tag_corners, tag_id in zip(corners, ids.reshape(-1), strict=True)
    ]


def tag_world_center(tag_id: int, settings: dict[str, Any]) -> np.ndarray:
    tags = settings.get("tags") if isinstance(settings.get("tags"), dict) else {}
    tag = tags.get(str(tag_id), tags.get(tag_id))
    if not isinstance(tag, dict):
        raise KeyError(tag_id)
    size = float(tag.get("size_mm", settings.get("tag_size_mm", 40.0)))
    if size <= 0:
        raise ValueError(f"AprilTag {tag_id} size must be positive")
    yaw = np.radians(float(tag.get("yaw_deg", 0.0)))
    rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]], dtype=np.float64)
    center = np.asarray(tag.get("center_mm", []), dtype=np.float64)
    if center.shape == (3,) and np.all(np.isfinite(center)):
        return center

    workspace_corner = np.asarray(tag.get("workspace_corner_mm", []), dtype=np.float64)
    if workspace_corner.shape != (3,) or not np.all(np.isfinite(workspace_corner)):
        raise ValueError(
            f"AprilTag {tag_id} must define center_mm or workspace_corner_mm with three finite values"
        )
    aligned_corner_name = str(tag.get("aligned_tag_corner", "")).lower()
    aligned_corner = TAG_LOCAL_CORNERS.get(aligned_corner_name)
    if aligned_corner is None:
        raise ValueError(
            f"AprilTag {tag_id} aligned_tag_corner must be one of "
            f"{', '.join(TAG_LOCAL_CORNERS)}"
        )
    local_offset = aligned_corner * (size / 2.0)
    center = workspace_corner.copy()
    center[:2] -= rotation @ local_offset
    return center


def tag_world_corners(tag_id: int, settings: dict[str, Any]) -> np.ndarray:
    tags = settings.get("tags") if isinstance(settings.get("tags"), dict) else {}
    tag = tags.get(str(tag_id), tags.get(tag_id))
    if not isinstance(tag, dict):
        raise KeyError(tag_id)
    center = tag_world_center(tag_id, settings)
    size = float(tag.get("size_mm", settings.get("tag_size_mm", 40.0)))
    yaw = np.radians(float(tag.get("yaw_deg", 0.0)))
    rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]], dtype=np.float64)
    # OpenCV corner order: printed top-left, top-right, bottom-right, bottom-left.
    local_xy = np.array(
        [[-size / 2, size / 2], [size / 2, size / 2], [size / 2, -size / 2], [-size / 2, -size / 2]],
        dtype=np.float64,
    )
    world = np.zeros((4, 3), dtype=np.float64)
    world[:, :2] = local_xy @ rotation.T + center[:2]
    world[:, 2] = center[2]
    return world


def configured_tag_ids(settings: dict[str, Any]) -> set[int]:
    tags = settings.get("tags") if isinstance(settings.get("tags"), dict) else {}
    return {int(tag_id) for tag_id in tags}


def _point_correspondences(
    detections: list[AprilTagDetection],
    settings: dict[str, Any],
) -> tuple[np.ndarray, np.ndarray, list[int]]:
    object_points: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    tags_used: list[int] = []
    for detection in detections:
        try:
            world = tag_world_corners(detection.tag_id, settings)
        except KeyError:
            continue
        object_points.append(world)
        image_points.append(detection.corners_px)
        tags_used.append(detection.tag_id)
    if not object_points:
        return np.empty((0, 3), np.float64), np.empty((0, 2), np.float64), []
    return np.vstack(object_points), np.vstack(image_points), sorted(set(tags_used))


def estimate_planar_homography(
    detections: list[AprilTagDetection],
    settings: dict[str, Any],
) -> dict[str, Any]:
    object_points, image_points, tags_used = _point_correspondences(detections, settings)
    if len(tags_used) < 1 or len(object_points) < 4:
        return {"ok": False, "error": "no configured AprilTags are visible"}
    homography, mask = cv2.findHomography(image_points, object_points[:, :2], cv2.RANSAC, 2.5)
    if homography is None:
        return {"ok": False, "error": "could not solve planar image-to-robot homography"}
    mapped = cv2.perspectiveTransform(image_points.reshape(-1, 1, 2).astype(np.float64), homography).reshape(-1, 2)
    errors = np.linalg.norm(mapped - object_points[:, :2], axis=1)
    inliers = int(np.count_nonzero(mask)) if mask is not None else len(errors)
    return {
        "ok": True,
        "tags_used": tags_used,
        "homography_image_to_robot": homography.tolist(),
        "rmse_mm": float(np.sqrt(np.mean(errors**2))),
        "max_error_mm": float(np.max(errors)),
        "inlier_count": inliers,
        "point_count": int(len(errors)),
    }


def _rotation_matrix_to_euler_xyz_deg(rotation: np.ndarray) -> list[float]:
    sy = hypot(float(rotation[0, 0]), float(rotation[1, 0]))
    singular = sy < 1e-8
    if not singular:
        x = atan2(float(rotation[2, 1]), float(rotation[2, 2]))
        y = atan2(float(-rotation[2, 0]), sy)
        z = atan2(float(rotation[1, 0]), float(rotation[0, 0]))
    else:
        x = atan2(float(-rotation[1, 2]), float(rotation[1, 1]))
        y = atan2(float(-rotation[2, 0]), sy)
        z = 0.0
    return [degrees(x), degrees(y), degrees(z)]


def _pose_confidence(
    tag_count: int,
    required_count: int,
    reprojection_rmse_px: float,
    maximum_rmse_px: float,
    inlier_ratio: float,
    tilt_from_down_deg: float,
    maximum_tilt_deg: float,
) -> float:
    tag_score = min(1.0, tag_count / max(required_count, 1))
    error_score = max(0.0, 1.0 - reprojection_rmse_px / max(maximum_rmse_px * 2.0, 1e-6))
    tilt_score = max(0.0, 1.0 - tilt_from_down_deg / max(maximum_tilt_deg * 1.5, 1e-6))
    return float(max(0.0, min(1.0, 0.35 * tag_score + 0.35 * error_score + 0.2 * inlier_ratio + 0.1 * tilt_score)))


def estimate_camera_pose(
    detections: list[AprilTagDetection],
    camera: dict[str, Any],
    settings: dict[str, Any] | None = None,
) -> dict[str, Any]:
    settings = settings or april_tag_settings(camera)
    camera_matrix, dist_coeffs, intrinsic_errors = camera_intrinsics(camera)
    planar = estimate_planar_homography(detections, settings)
    object_points, image_points, tags_used = _point_correspondences(detections, settings)
    base: dict[str, Any] = {
        "ok": False,
        "accepted": False,
        "id": str(uuid4()),
        "timestamp": time(),
        "dictionary": str(settings.get("dictionary", "DICT_APRILTAG_36H11")),
        "tag_size_mm": float(settings.get("tag_size_mm", 40.0)),
        "tags_used": tags_used,
        "point_count": int(len(object_points)),
        "planar": planar,
    }
    if camera_matrix is None:
        base["error"] = "; ".join(intrinsic_errors)
        return base
    min_tags = max(1, int(settings.get("min_tags_for_pose", 2)))
    if len(tags_used) < min_tags:
        base["error"] = f"pose needs at least {min_tags} configured visible tags"
        return base

    success, rvec, tvec, inliers = cv2.solvePnPRansac(
        object_points,
        image_points,
        camera_matrix,
        dist_coeffs,
        iterationsCount=150,
        reprojectionError=float(settings.get("max_reprojection_error_px", 2.5)) * 1.5,
        confidence=0.999,
        flags=cv2.SOLVEPNP_SQPNP,
    )
    if not success:
        base["error"] = "OpenCV could not solve a camera pose from the visible tags"
        return base
    inlier_indices = inliers.reshape(-1) if inliers is not None and len(inliers) >= 4 else np.arange(len(object_points))
    success, rvec, tvec = cv2.solvePnP(
        object_points[inlier_indices],
        image_points[inlier_indices],
        camera_matrix,
        dist_coeffs,
        rvec,
        tvec,
        useExtrinsicGuess=True,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not success:
        base["error"] = "OpenCV could not refine the robust camera pose"
        return base
    refined_rvec, refined_tvec = cv2.solvePnPRefineLM(
        object_points[inlier_indices],
        image_points[inlier_indices],
        camera_matrix,
        dist_coeffs,
        rvec,
        tvec,
    )
    rotation_world_to_camera, _ = cv2.Rodrigues(refined_rvec)
    rotation_camera_to_robot = rotation_world_to_camera.T
    camera_position = (-rotation_camera_to_robot @ refined_tvec.reshape(3, 1)).reshape(3)
    optical_axis_robot = rotation_camera_to_robot @ np.array([0.0, 0.0, 1.0])
    down_alignment = float(np.clip(np.dot(optical_axis_robot, np.array([0.0, 0.0, -1.0])), -1.0, 1.0))
    tilt_from_down_deg = degrees(acos(down_alignment))

    projected, _ = cv2.projectPoints(object_points, refined_rvec, refined_tvec, camera_matrix, dist_coeffs)
    reprojection_errors = np.linalg.norm(projected.reshape(-1, 2) - image_points, axis=1)
    rmse_px = float(np.sqrt(np.mean(reprojection_errors**2)))
    max_error_px = float(np.max(reprojection_errors))
    inlier_count = int(len(inlier_indices))
    inlier_ratio = float(inlier_count / max(len(object_points), 1))
    maximum_rmse = float(settings.get("max_reprojection_error_px", 2.5))
    maximum_tilt = float(settings.get("max_tilt_from_down_deg", 45.0))
    required_count = len(settings.get("required_ids", [])) or len(configured_tag_ids(settings))
    confidence = _pose_confidence(
        len(tags_used),
        required_count,
        rmse_px,
        maximum_rmse,
        inlier_ratio,
        tilt_from_down_deg,
        maximum_tilt,
    )
    rejection_reasons: list[str] = []
    if camera_position[2] <= 0:
        rejection_reasons.append("camera solution is not above the workspace plane")
    if down_alignment <= 0:
        rejection_reasons.append("camera optical axis points away from the workspace plane")
    if tilt_from_down_deg > maximum_tilt:
        rejection_reasons.append(f"camera tilt {tilt_from_down_deg:.1f} deg exceeds {maximum_tilt:.1f} deg")
    if rmse_px > maximum_rmse:
        rejection_reasons.append(f"reprojection RMSE {rmse_px:.2f} px exceeds {maximum_rmse:.2f} px")
    if inlier_ratio < 0.75:
        rejection_reasons.append(f"PnP inlier ratio {inlier_ratio:.2f} is below 0.75")

    image_size = camera.get("resolution") if isinstance(camera.get("resolution"), dict) else {}
    base.update(
        {
            "ok": True,
            "accepted": not rejection_reasons,
            "error": "; ".join(rejection_reasons),
            "camera_matrix": camera_matrix.tolist(),
            "distortion_coefficients": dist_coeffs.reshape(-1).tolist(),
            "image_size_px": {
                "width": int(image_size.get("width", 0)),
                "height": int(image_size.get("height", 0)),
            },
            "world_to_camera": {
                "rvec": refined_rvec.reshape(-1).tolist(),
                "tvec_mm": refined_tvec.reshape(-1).tolist(),
                "rotation_matrix": rotation_world_to_camera.tolist(),
            },
            "camera_to_robot": {
                "position_mm": camera_position.tolist(),
                "rotation_matrix": rotation_camera_to_robot.tolist(),
                "euler_xyz_deg": _rotation_matrix_to_euler_xyz_deg(rotation_camera_to_robot),
                "optical_axis": optical_axis_robot.tolist(),
            },
            "metrics": {
                "reprojection_rmse_px": rmse_px,
                "reprojection_max_px": max_error_px,
                "inlier_count": inlier_count,
                "inlier_ratio": inlier_ratio,
                "tilt_from_down_deg": tilt_from_down_deg,
                "confidence": confidence,
            },
        }
    )
    return base


class AprilTagCalibrationSession:
    def __init__(self, camera: dict[str, Any]):
        self.camera = camera
        self.settings = april_tag_settings(camera)
        self._observation_signature = self._camera_observation_signature(camera, self.settings)
        self.frames: list[dict[int, np.ndarray]] = []

    @staticmethod
    def _camera_observation_signature(
        camera: dict[str, Any],
        settings: dict[str, Any],
    ) -> tuple[Any, ...]:
        resolution = camera.get("resolution") if isinstance(camera.get("resolution"), dict) else {}
        return (
            int(camera.get("source_index", 0)),
            int(resolution.get("width", 0) or 0),
            int(resolution.get("height", 0) or 0),
            str(settings.get("dictionary", "DICT_APRILTAG_36H11")).upper(),
            float(settings.get("tag_size_mm", 40.0)),
            json.dumps(settings.get("tags", {}), sort_keys=True, separators=(",", ":")),
        )

    def configure(self, camera: dict[str, Any], preserve_frames: bool = True) -> None:
        settings = april_tag_settings(camera)
        observation_signature = self._camera_observation_signature(camera, settings)
        reset_frames = not preserve_frames or observation_signature != self._observation_signature
        self.camera = camera
        self.settings = settings
        self._observation_signature = observation_signature
        if reset_frames:
            self.reset()

    def reset(self) -> None:
        self.frames.clear()

    def add(self, detections: list[AprilTagDetection]) -> None:
        configured = configured_tag_ids(self.settings)
        frame = {
            detection.tag_id: detection.corners_px.copy()
            for detection in detections
            if detection.tag_id in configured
        }
        if frame:
            self.frames.append(frame)
            maximum = max(1, int(self.settings.get("max_samples", 120)))
            if len(self.frames) > maximum:
                self.frames = self.frames[-maximum:]

    def aggregated_detections(self) -> list[AprilTagDetection]:
        observations: dict[int, list[np.ndarray]] = {}
        for frame in self.frames:
            for tag_id, corners in frame.items():
                observations.setdefault(tag_id, []).append(corners)
        return [
            AprilTagDetection(tag_id, np.median(np.stack(corners), axis=0))
            for tag_id, corners in sorted(observations.items())
        ]

    def _jitter_metrics(self) -> dict[str, float]:
        metrics: dict[str, float] = {}
        aggregated = {detection.tag_id: detection.corners_px for detection in self.aggregated_detections()}
        for tag_id, median in aggregated.items():
            errors = [
                float(np.sqrt(np.mean((frame[tag_id] - median) ** 2)))
                for frame in self.frames
                if tag_id in frame
            ]
            if errors:
                metrics[str(tag_id)] = float(np.mean(errors))
        return metrics

    def tag_observation_counts(self) -> dict[str, int]:
        return {
            str(tag_id): sum(1 for frame in self.frames if tag_id in frame)
            for tag_id in sorted(configured_tag_ids(self.settings))
        }

    def solve(self) -> dict[str, Any]:
        aggregated = self.aggregated_detections()
        result = estimate_camera_pose(aggregated, self.camera, self.settings)
        minimum_samples = int(self.settings.get("min_samples", 12))
        required_ids = {
            int(value)
            for value in self.settings.get("required_ids", configured_tag_ids(self.settings))
        }
        observation_counts = self.tag_observation_counts()
        result["frames_used"] = len(self.frames)
        result["tag_observation_counts"] = observation_counts
        result["corner_jitter_rms_px"] = self._jitter_metrics()
        result["minimum_samples_met"] = len(self.frames) >= minimum_samples
        result["required_tag_samples_met"] = all(
            observation_counts.get(str(tag_id), 0) >= minimum_samples
            for tag_id in required_ids
        )
        return result

    def summary(self) -> dict[str, Any]:
        aggregated = self.aggregated_detections()
        minimum_samples = int(self.settings.get("min_samples", 12))
        required_ids = {
            int(value)
            for value in self.settings.get("required_ids", configured_tag_ids(self.settings))
        }
        observation_counts = self.tag_observation_counts()
        return {
            "frame_count": len(self.frames),
            "tag_ids": [detection.tag_id for detection in aggregated],
            "tag_count": len(aggregated),
            "minimum_samples": minimum_samples,
            "tag_observation_counts": observation_counts,
            "required_tag_samples_met": all(
                observation_counts.get(str(tag_id), 0) >= minimum_samples
                for tag_id in required_ids
            ),
            "corner_jitter_rms_px": self._jitter_metrics(),
        }


def project_image_point_to_plane(
    image_point: list[float],
    calibration_result: dict[str, Any],
    plane_z_mm: float = 0.0,
) -> dict[str, float]:
    camera_matrix = np.asarray(calibration_result.get("camera_matrix"), dtype=np.float64)
    dist_coeffs = np.asarray(
        calibration_result.get("distortion_coefficients", [0.0, 0.0, 0.0, 0.0, 0.0]),
        dtype=np.float64,
    ).reshape(-1, 1)
    pose = calibration_result.get("camera_to_robot") or {}
    rotation = np.asarray(pose.get("rotation_matrix"), dtype=np.float64)
    position = np.asarray(pose.get("position_mm"), dtype=np.float64)
    if (
        camera_matrix.shape != (3, 3)
        or rotation.shape != (3, 3)
        or position.shape != (3,)
        or dist_coeffs.size not in {4, 5, 8, 12, 14}
        or not np.all(np.isfinite(camera_matrix))
        or not np.all(np.isfinite(rotation))
        or not np.all(np.isfinite(position))
        or not np.all(np.isfinite(dist_coeffs))
    ):
        raise ValueError("saved AprilTag camera pose is incomplete")
    pixel = np.asarray(image_point, dtype=np.float64)
    if pixel.shape != (2,) or not np.all(np.isfinite(pixel)):
        raise ValueError("image point must contain two finite pixel coordinates")
    normalized = cv2.undistortPoints(
        pixel.reshape(1, 1, 2),
        camera_matrix,
        dist_coeffs,
    ).reshape(2)
    ray_camera = np.array([normalized[0], normalized[1], 1.0], dtype=np.float64)
    ray_robot = rotation @ ray_camera
    if abs(float(ray_robot[2])) < 1e-9:
        raise ValueError("camera ray is parallel to the workspace plane")
    distance = (float(plane_z_mm) - float(position[2])) / float(ray_robot[2])
    if distance <= 0:
        raise ValueError("camera ray does not intersect the workspace plane in front of the camera")
    point = position + distance * ray_robot
    return {"x_mm": float(point[0]), "y_mm": float(point[1]), "z_mm": float(point[2])}


def annotate_apriltag_frame(
    image_bgr: np.ndarray,
    detections: list[AprilTagDetection],
    camera: dict[str, Any],
    result: dict[str, Any] | None = None,
) -> np.ndarray:
    annotated = image_bgr.copy()
    settings = april_tag_settings(camera)
    configured = configured_tag_ids(settings)
    if detections:
        cv2.aruco.drawDetectedMarkers(
            annotated,
            [detection.corners_px.astype(np.float32).reshape(1, 4, 2) for detection in detections],
            np.asarray([[detection.tag_id] for detection in detections], dtype=np.int32),
        )
    for detection in detections:
        center = tuple(np.round(detection.center_px).astype(int))
        color = (60, 220, 80) if detection.tag_id in configured else (40, 80, 240)
        cv2.circle(annotated, center, 5, color, -1)
        cv2.putText(
            annotated,
            f"Tag {detection.tag_id}{'' if detection.tag_id in configured else ' unknown'}",
            (center[0] + 8, center[1] - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )
    if result and result.get("ok"):
        camera_matrix = np.asarray(result["camera_matrix"], dtype=np.float64)
        dist_coeffs = np.asarray(result["distortion_coefficients"], dtype=np.float64)
        rvec = np.asarray(result["world_to_camera"]["rvec"], dtype=np.float64)
        tvec = np.asarray(result["world_to_camera"]["tvec_mm"], dtype=np.float64)
        cv2.drawFrameAxes(annotated, camera_matrix, dist_coeffs, rvec, tvec, 80.0, 2)
        workspace_points = [
            np.asarray(tag.get("workspace_corner_mm"), dtype=np.float64)
            for tag in settings.get("tags", {}).values()
            if isinstance(tag, dict)
        ]
        valid_workspace_points = [
            point
            for point in workspace_points
            if point.shape == (3,) and np.all(np.isfinite(point))
        ]
        if len(valid_workspace_points) >= 3:
            centers = np.asarray(valid_workspace_points, dtype=np.float64)
            hull_indices = cv2.convexHull(
                centers[:, :2].astype(np.float32),
                returnPoints=False,
            ).reshape(-1)
            workspace = centers[hull_indices]
            projected, _ = cv2.projectPoints(workspace, rvec, tvec, camera_matrix, dist_coeffs)
            cv2.polylines(
                annotated,
                [np.round(projected.reshape(-1, 2)).astype(np.int32)],
                True,
                (220, 180, 60),
                2,
            )
    status = "No pose"
    if result:
        metrics = result.get("metrics") or {}
        status = (
            f"{'ACCEPTED' if result.get('accepted') else 'REJECTED'} "
            f"tags={len(result.get('tags_used', []))} "
            f"rmse={metrics.get('reprojection_rmse_px', 0):.2f}px "
            f"conf={metrics.get('confidence', 0):.2f}"
        )
    cv2.rectangle(annotated, (8, 8), (min(annotated.shape[1] - 8, 620), 40), (12, 16, 24), -1)
    cv2.putText(annotated, status, (16, 31), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (235, 240, 248), 2, cv2.LINE_AA)
    return annotated
